# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Module that scores a pose-graph SLAM module's loop closures against KITTI groundtruth.

Subscribes to two outputs that any pose-graph SLAM module exposes:

* ``pose_graph_edges: In[NavPath]`` — pose-graph edges where loop closures
  are tagged with ``orientation.w == 0.4`` (odometry edges use ``1.0``).
  Each endpoint's ``orientation.x`` carries a per-detection confidence
  score in [0, 1]. A score of 0 is "no score provided" (back-compat with
  producers like PGO that don't publish per-edge confidence) — the scorer
  falls back to single-threshold event-counting in that case.
* ``loop_closure: In[NavPath]`` — one event per loop-closure update with
  per-keyframe deltas.

The scoring module needs to know, for each edge endpoint, which input scan
produced that keyframe. The producer publishes a timestamp on each endpoint's
``PoseStamped`` header — we keep a (timestamp → frame_id) cache built from
the playback module's send schedule so we can map back unambiguously even
after iSAM2 has shifted the optimized keyframe positions.

In addition to the single-threshold TP/FP/FN/precision/recall/F1 numbers,
``get_results()`` returns a precision-recall curve (``pr_curve``) computed
by sweeping the score threshold from high to low, and the ``max_f1`` /
``max_f1_threshold`` / ``average_precision`` derived from that curve. These
are the metric conventions used in the lidar-loop-closure literature
(OverlapNet, Scan Context, M2DP) and let our numbers be directly compared
to published F1 scores.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any

from pydantic import Field
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.msgs.nav_msgs.Path import Path as NavPath

# Default tag value used by the PGO publisher to mark loop-closure edges in
# the orientation.w field of pose_graph_edges PoseStamped pairs (odometry
# edges use 1.0). Both knobs are exposed on PoseGraphScoringConfig so any
# other pose-graph producer can dial in its own marker.
DEFAULT_LOOP_CLOSURE_TRAVERSABILITY = 0.4
DEFAULT_TRAVERSABILITY_TOLERANCE = 0.05


@dataclass
class LoopMetrics:
    true_positive: int
    false_positive: int
    false_negative: int

    @property
    def precision(self) -> float:
        denom = self.true_positive + self.false_positive
        return self.true_positive / denom if denom > 0 else float("nan")

    @property
    def recall(self) -> float:
        denom = self.true_positive + self.false_negative
        return self.true_positive / denom if denom > 0 else float("nan")

    @property
    def f1(self) -> float:
        precision, recall = self.precision, self.recall
        if not (precision > 0 and recall > 0):
            return 0.0
        return 2.0 * precision * recall / (precision + recall)


class PoseGraphScoringConfig(ModuleConfig):
    # ``ModuleConfig`` inherits from ``pydantic.BaseModel``, so default
    # factories must come from ``pydantic.Field`` — ``dataclasses.field``
    # would be stored as the literal default value and break validation
    # (greptile c5 on PR #2099).
    frame_ids: list[int] = Field(default_factory=list)
    send_timestamps: list[float] = Field(default_factory=list)
    # JSON-friendly form of LoopGroundtruth.valid_loops_per_query:
    # frame_id → list of frame_ids that form valid loop pairs.
    valid_loops_per_query: dict[int, list[int]] = Field(default_factory=dict)
    # Tag value the publisher writes into orientation.w to mark a
    # pose_graph_edges PoseStamped pair as a loop closure (vs the
    # odometry-edge default of 1.0). Both fields are config-driven so
    # different pose-graph SLAM producers can plug in their own marker.
    loop_closure_traversability: float = DEFAULT_LOOP_CLOSURE_TRAVERSABILITY
    traversability_tolerance: float = DEFAULT_TRAVERSABILITY_TOLERANCE


class PoseGraphScoringModule(Module):
    """Accumulates loop-closure detections and scores them against KITTI groundtruth."""

    config: PoseGraphScoringConfig

    pose_graph_edges: In[NavPath]
    loop_closure: In[NavPath]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        # (source_frame_id, target_frame_id, score) per detection. Score
        # comes from orientation.x of the edge's PoseStamped endpoints;
        # 0 means "producer didn't publish a score, treat as unscored."
        self._detected_pairs: list[tuple[int, int, float]] = []
        self._loop_closure_events: int = 0
        self._timestamp_ms_to_frame_id: dict[int, int] = {
            round(send_timestamp * 1e3): frame_id
            for frame_id, send_timestamp in zip(
                self.config.frame_ids, self.config.send_timestamps, strict=True
            )
        }

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.loop_closure.subscribe(self._on_loop_closure)))
        self.register_disposable(
            Disposable(self.pose_graph_edges.subscribe(self._on_pose_graph_edges))
        )

    def _on_loop_closure(self, message: NavPath) -> None:
        del message
        self._loop_closure_events += 1

    def _on_pose_graph_edges(self, message: NavPath) -> None:
        pose_index = 0
        while pose_index + 1 < len(message.poses):
            start_pose = message.poses[pose_index]
            end_pose = message.poses[pose_index + 1]
            traversability = float(start_pose.orientation.w)
            if (
                abs(traversability - self.config.loop_closure_traversability)
                < self.config.traversability_tolerance
            ):
                start_frame_id = self._timestamp_to_frame(start_pose.ts)
                end_frame_id = self._timestamp_to_frame(end_pose.ts)
                if start_frame_id is not None and end_frame_id is not None:
                    # Confidence score lives in orientation.x; 0 means
                    # "producer didn't publish one." Take the max of
                    # both endpoints in case one was left unset.
                    score = max(
                        float(start_pose.orientation.x),
                        float(end_pose.orientation.x),
                    )
                    detection = (start_frame_id, end_frame_id, score)
                    # Dedupe by frame-id pair; keep the highest score
                    # seen for any duplicate pair.
                    existing_index = next(
                        (
                            i
                            for i, existing in enumerate(self._detected_pairs)
                            if existing[0] == start_frame_id and existing[1] == end_frame_id
                        ),
                        None,
                    )
                    if existing_index is None:
                        self._detected_pairs.append(detection)
                    elif self._detected_pairs[existing_index][2] < score:
                        self._detected_pairs[existing_index] = detection
            pose_index += 2

    def _timestamp_to_frame(self, timestamp_sec: float) -> int | None:
        timestamp_ms = round(timestamp_sec * 1e3)
        # ±1 ms slop: PoseStamped.ts round-trips through (int32 sec, uint32 nsec).
        for slop_ms in (0, -1, 1):
            frame_id = self._timestamp_ms_to_frame_id.get(timestamp_ms + slop_ms)
            if frame_id is not None:
                return frame_id
        return None

    @rpc
    def get_results(self) -> dict[str, Any]:
        valid_loops_per_query: dict[int, set[int]] = {
            frame_id: set(loops) for frame_id, loops in self.config.valid_loops_per_query.items()
        }
        queries_with_loop = sum(1 for valid in valid_loops_per_query.values() if valid)
        total_pairs = sum(len(valid) for valid in valid_loops_per_query.values())

        # Single-threshold metrics (count every published detection).
        # Matches the historical scorer contract — kept for back-compat.
        flat_pairs = [(src, dst) for src, dst, _score in self._detected_pairs]
        metrics = _score_pairs(flat_pairs, valid_loops_per_query)

        # Precision-recall sweep over score thresholds. Treats each
        # detection as ranked by score (orientation.x from the C++
        # binary). Producers without per-edge confidence publish score=0;
        # in that degenerate case the sweep collapses to a single point
        # (threshold = 0 = include all) and pr_curve / max_f1 carry no
        # new information beyond the single-threshold numbers.
        pr_curve, max_f1, max_f1_threshold, average_precision = _precision_recall_curve(
            self._detected_pairs, valid_loops_per_query
        )

        return {
            "scans_played": len(self.config.frame_ids),
            "groundtruth_queries_with_loop": queries_with_loop,
            "groundtruth_total_loop_pairs": total_pairs,
            "detected_loop_edges": len(self._detected_pairs),
            "loop_closure_events": self._loop_closure_events,
            "true_positive": metrics.true_positive,
            "false_positive": metrics.false_positive,
            "false_negative": metrics.false_negative,
            "precision": (metrics.precision if math.isfinite(metrics.precision) else None),
            "recall": metrics.recall if math.isfinite(metrics.recall) else None,
            "f1": metrics.f1,
            "max_f1": max_f1,
            "max_f1_threshold": max_f1_threshold,
            "average_precision": average_precision,
            "pr_curve": pr_curve,
        }


def _score_pairs(
    detected_pairs: list[tuple[int, int]],
    valid_loops_per_query: dict[int, set[int]],
) -> LoopMetrics:
    true_positives = 0
    false_positives = 0
    seen_queries_with_hit: set[int] = set()
    queries_with_any_groundtruth = {
        frame_id for frame_id, valid in valid_loops_per_query.items() if valid
    }
    for source_frame_id, target_frame_id in detected_pairs:
        source_valid = valid_loops_per_query.get(source_frame_id, set())
        target_valid = valid_loops_per_query.get(target_frame_id, set())
        if target_frame_id in source_valid or source_frame_id in target_valid:
            true_positives += 1
            seen_queries_with_hit.add(max(source_frame_id, target_frame_id))
        else:
            false_positives += 1
    false_negatives = len(queries_with_any_groundtruth - seen_queries_with_hit)
    return LoopMetrics(
        true_positive=true_positives,
        false_positive=false_positives,
        false_negative=false_negatives,
    )


def _precision_recall_curve(
    detected_pairs: list[tuple[int, int, float]],
    valid_loops_per_query: dict[int, set[int]],
) -> tuple[list[dict[str, float]], float, float, float]:
    """Sweep score threshold high→low; return (curve, max_f1, max_f1_thr, AP).

    Each curve point is ``{threshold, precision, recall, f1, true_positive,
    false_positive, false_negative}``. The maximum-F1 point and its threshold
    are returned separately, and `average_precision` is computed as the
    standard PR-AUC under stepwise interpolation (sum over n of
    (R_n - R_(n-1)) * P_n).

    Detections are sorted by score descending. At rank k we treat the
    top-k detections as "above threshold" and compute precision/recall
    against the full ground-truth set. Recall denominator is queries-with-
    any-GT (matches the single-threshold scorer's FN convention).
    """
    queries_with_any_groundtruth = {
        frame_id for frame_id, valid in valid_loops_per_query.items() if valid
    }
    total_gt = len(queries_with_any_groundtruth)
    if not detected_pairs or total_gt == 0:
        return [], 0.0, 0.0, 0.0

    # Sort by score descending; ties broken arbitrarily but stable.
    ranked = sorted(detected_pairs, key=lambda triple: -triple[2])

    curve: list[dict[str, float]] = []
    seen_queries_with_hit: set[int] = set()
    true_positives = 0
    false_positives = 0
    max_f1 = 0.0
    max_f1_threshold = 0.0
    # Stepwise PR-AUC running sum: ∑ ΔRecall · Precision_at_step.
    average_precision = 0.0
    previous_recall = 0.0

    for source_frame_id, target_frame_id, score in ranked:
        source_valid = valid_loops_per_query.get(source_frame_id, set())
        target_valid = valid_loops_per_query.get(target_frame_id, set())
        if target_frame_id in source_valid or source_frame_id in target_valid:
            true_positives += 1
            seen_queries_with_hit.add(max(source_frame_id, target_frame_id))
        else:
            false_positives += 1

        denom_p = true_positives + false_positives
        precision = true_positives / denom_p if denom_p > 0 else 0.0
        recall = len(seen_queries_with_hit) / total_gt
        denom_f = precision + recall
        f1 = (2.0 * precision * recall / denom_f) if denom_f > 0 else 0.0

        false_negatives = total_gt - len(seen_queries_with_hit)
        curve.append(
            {
                "threshold": float(score),
                "precision": precision,
                "recall": recall,
                "f1": f1,
                "true_positive": float(true_positives),
                "false_positive": float(false_positives),
                "false_negative": float(false_negatives),
            }
        )

        if f1 > max_f1:
            max_f1 = f1
            max_f1_threshold = float(score)

        if recall > previous_recall:
            average_precision += (recall - previous_recall) * precision
            previous_recall = recall

    return curve, max_f1, max_f1_threshold, average_precision
