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

"""Unit tests for the precision-recall scoring helpers."""

from __future__ import annotations

import pytest

from dimos.navigation.nav_stack.benchmarks.pose_graph_kitti360.scoring import (
    _precision_recall_curve,
    _score_pairs,
)


def test_score_pairs_counts_tp_fp_fn() -> None:
    """Single-threshold scorer counts each detection against GT.

    GT is bidirectional in the real benchmark (KITTI loop pair (i,j)
    implies query i has target j *and* query j has target i), so we
    mirror that here.
    """
    gt = {
        100: {200},
        200: {100},
        150: {250},
        250: {150},
        300: {400},
        400: {300},
    }
    detections = [(100, 200), (150, 250), (170, 999)]
    metrics = _score_pairs(detections, gt)
    assert metrics.true_positive == 2
    assert metrics.false_positive == 1
    # Three "places" have GT loops (each represented twice in `gt`);
    # detection hits 200 (covers query 200, and 100 by symmetry) and 250
    # but misses the 300↔400 pair. The scorer keys hits by max(src,dst)
    # so seen_queries = {200, 250}; queries_with_any_groundtruth has 6
    # keys; FN = 6 - 2 = 4.
    assert metrics.false_negative == 4


def test_pr_curve_empty_inputs_return_zeros() -> None:
    """No detections or no GT yields zero metrics, no exceptions."""
    curve, max_f1, max_f1_thr, ap = _precision_recall_curve([], {100: {200}})
    assert curve == []
    assert max_f1 == 0.0
    assert max_f1_thr == 0.0
    assert ap == 0.0

    curve, max_f1, max_f1_thr, ap = _precision_recall_curve([(1, 2, 0.9)], {})
    assert curve == []


def test_pr_curve_back_compat_unscored_detections() -> None:
    """PGO (and other producers without per-edge confidence) publish score=0.

    The PR sweep should still produce a meaningful curve at threshold=0,
    monotonically including each detection in the order it was published.
    """
    gt = {100: {200}, 150: {250}, 300: {400}}
    # Two TPs followed by an FP — order matters in tied-score sweeps.
    detections = [(100, 200, 0.0), (150, 250, 0.0), (170, 999, 0.0)]
    curve, max_f1, max_f1_thr, ap = _precision_recall_curve(detections, gt)
    assert len(curve) == 3
    # Max-F1 lands at the 2-TP-0-FP step.
    assert max_f1 == pytest.approx(0.8)
    assert max_f1_thr == pytest.approx(0.0)
    # AP = (1/3 - 0) * 1.0 + (2/3 - 1/3) * 1.0 + 0 (recall doesn't grow on the FP step)
    assert ap == pytest.approx(2 / 3)


def test_pr_curve_score_aware_separates_tp_from_fp() -> None:
    """With per-detection scores, the sweep should yield max-F1 = 1.0 when
    the score correctly orders TPs above FPs."""
    gt = {100: {200}, 150: {250}, 300: {400}}
    detections = [
        (100, 200, 0.95),
        (150, 250, 0.85),
        (300, 400, 0.60),
        (170, 999, 0.50),  # FP, lowest score
    ]
    curve, max_f1, max_f1_thr, ap = _precision_recall_curve(detections, gt)
    assert len(curve) == 4
    assert max_f1 == pytest.approx(1.0)
    # Max-F1 sits at the threshold of the lowest-scoring TP.
    assert max_f1_thr == pytest.approx(0.6)
    # Perfect AP because TPs all rank above FPs.
    assert ap == pytest.approx(1.0)


def test_pr_curve_falling_precision_on_late_fps() -> None:
    """Adding FPs below the top TP block degrades precision; max-F1 stays
    pinned to the cleanest TP-only prefix."""
    gt = {100: {200}, 150: {250}}
    detections = [
        (100, 200, 0.9),
        (170, 999, 0.7),  # FP
        (150, 250, 0.5),  # TP
        (999, 998, 0.3),  # FP
    ]
    curve, max_f1, max_f1_thr, _ap = _precision_recall_curve(detections, gt)
    # Sweep order: thr 0.9 → P=1.0 R=0.5 F1=0.667
    #              thr 0.7 → P=0.5 R=0.5 F1=0.5 (FP)
    #              thr 0.5 → P=2/3 R=1.0 F1=0.8 (TP)
    #              thr 0.3 → P=0.5 R=1.0 F1=0.667 (FP)
    assert max_f1 == pytest.approx(0.8)
    assert max_f1_thr == pytest.approx(0.5)
    # Curve has all four points in score-descending order.
    assert [round(p["threshold"], 1) for p in curve] == [0.9, 0.7, 0.5, 0.3]
