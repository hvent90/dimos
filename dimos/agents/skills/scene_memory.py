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

"""Skills answering spatio-temporal queries from recorded robot memory.

The pose trail comes from a memory2 recording (the replay DB by default):
the ``odom`` stream's timestamped poses answer "when was the robot in
region R" and "where was the robot at time t". Regions are 2D polygons in
world coordinates. All timestamps are recording-time epoch seconds — the
same time base the recording's streams carry.
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import threading
from typing import Any

import numpy as np
from numpy.typing import NDArray

from dimos.agents.annotation import skill
from dimos.agents.skill_result import CommonSkillError, SkillResult
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.mapping.occupancy.polygons import points_in_polygon, polygon_from_flat
from dimos.memory2.replay import resolve_db_path
from dimos.memory2.store.sqlite import SqliteStore
from dimos.memory2.type.observation import Observation
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Bridge gaps between in-region odom samples up to this long (sensor dropouts
# and doorway-straddling flicker), so one visit doesn't splinter into many.
DEFAULT_VISIT_GAP_S = 2.0


def visit_intervals(
    ts: NDArray[np.float64], inside: NDArray[np.bool_], max_gap_s: float = DEFAULT_VISIT_GAP_S
) -> list[tuple[float, float]]:
    """Group in-region samples into visit intervals ``[enter_ts, exit_ts]``.

    Consecutive inside samples whose timestamps differ by at most
    ``max_gap_s`` belong to the same visit; larger gaps split visits.

    Args:
        ts: (N,) sample timestamps, ascending.
        inside: (N,) whether each sample is inside the region.
        max_gap_s: max time gap bridged within one visit.
    """
    idx = np.nonzero(inside)[0]
    if idx.size == 0:
        return []
    inside_ts = ts[idx]
    splits = np.nonzero(np.diff(inside_ts) > max_gap_s)[0]
    starts = np.concatenate(([0], splits + 1))
    ends = np.concatenate((splits, [inside_ts.size - 1]))
    return [(float(inside_ts[s]), float(inside_ts[e])) for s, e in zip(starts, ends, strict=True)]


def _iso(ts: float) -> str:
    return datetime.fromtimestamp(ts, tz=timezone.utc).strftime("%Y-%m-%d %H:%M:%S")


@dataclass(frozen=True)
class PoseTrail:
    """Robot trajectory loaded from a recording: (N,) timestamps + (N, 2) xy."""

    ts: NDArray[np.float64]
    xy: NDArray[np.float64]

    def time_range(self) -> tuple[float, float]:
        return float(self.ts[0]), float(self.ts[-1])


def load_pose_trail(db_path: str, stream_names: list[str]) -> PoseTrail:
    """Load the robot pose trail from a memory2 SQLite recording.

    Uses the first stream in ``stream_names`` present in the store. Positions
    come from each observation's pose columns when stamped, else from the
    payload (expected to expose ``.position`` like ``PoseStamped``).

    Raises:
        LookupError: no candidate stream exists or the stream has no poses.
    """
    with SqliteStore(path=db_path, must_exist=True) as store:
        available = store.list_streams()
        name = next((s for s in stream_names if s in available), None)
        if name is None:
            raise LookupError(f"None of streams {stream_names} in {db_path}; found {available}")
        ts_list: list[float] = []
        xy_list: list[tuple[float, float]] = []
        obs: Observation[Any]
        for obs in store.stream(name).order_by("ts"):
            if obs.pose_tuple is not None:
                x, y = obs.pose_tuple[0], obs.pose_tuple[1]
            else:
                position: Any = obs.data.position
                x, y = float(position.x), float(position.y)
            ts_list.append(float(obs.ts))
            xy_list.append((x, y))
    if not ts_list:
        raise LookupError(f"Stream {name!r} in {db_path} has no observations")
    return PoseTrail(ts=np.asarray(ts_list), xy=np.asarray(xy_list))


class SceneMemoryConfig(ModuleConfig):
    # Recording holding the robot's pose trail: a dataset name or .db path,
    # resolved like the replay DB. Defaults to the session's replay DB.
    trail_db: str = ""
    # Candidate pose streams, first match wins (naming varies by recording rig).
    trail_streams: list[str] = ["go2_odom", "odom"]


class SceneMemorySkillContainer(Module):
    """Agent skills over recorded spatio-temporal memory (pose trail queries)."""

    config: SceneMemoryConfig

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._trail: PoseTrail | None = None
        self._trail_lock = threading.Lock()

    @rpc
    def start(self) -> None:
        super().start()

    @rpc
    def stop(self) -> None:
        super().stop()

    def _trail_db(self) -> str:
        return self.config.trail_db or self.config.g.replay_db

    def _get_trail(self) -> PoseTrail:
        with self._trail_lock:
            if self._trail is None:
                path = resolve_db_path(self._trail_db())
                self._trail = load_pose_trail(str(path), self.config.trail_streams)
                logger.info(
                    "Loaded pose trail",
                    db=str(path),
                    samples=len(self._trail.ts),
                )
            return self._trail

    @skill
    def robot_trail_info(self) -> SkillResult[CommonSkillError]:
        """Describe the robot's recorded pose trail: time range and coverage.

        Call this first to learn the recording's absolute time range (epoch
        seconds, UTC) so you can convert relative times like "30 seconds in"
        into the timestamps other trail skills expect.
        """
        try:
            trail = self._get_trail()
        except (FileNotFoundError, LookupError) as e:
            return SkillResult.fail("NOT_CONFIGURED", f"No pose trail available: {e}")
        t0, t1 = trail.time_range()
        bounds_min = trail.xy.min(axis=0)
        bounds_max = trail.xy.max(axis=0)
        return SkillResult.ok(
            f"Pose trail: {len(trail.ts)} samples from {_iso(t0)} to {_iso(t1)} UTC "
            f"({t1 - t0:.1f} s).",
            start_ts=round(t0, 3),
            end_ts=round(t1, 3),
            duration_s=round(t1 - t0, 1),
            samples=len(trail.ts),
            xy_bounds=[
                [round(float(bounds_min[0]), 2), round(float(bounds_min[1]), 2)],
                [round(float(bounds_max[0]), 2), round(float(bounds_max[1]), 2)],
            ],
        )

    @skill
    def robot_position_at(self, t: float, tolerance: float = 2.0) -> SkillResult[CommonSkillError]:
        """Where was the robot at time t? Returns the nearest recorded pose.

        Args:
            t: Timestamp in recording-time epoch seconds (see robot_trail_info
                for the recording's time range).
            tolerance: Max seconds between t and the nearest pose sample.
        """
        try:
            trail = self._get_trail()
        except (FileNotFoundError, LookupError) as e:
            return SkillResult.fail("NOT_CONFIGURED", f"No pose trail available: {e}")
        t0, t1 = trail.time_range()
        i = int(np.argmin(np.abs(trail.ts - t)))
        dt = float(abs(trail.ts[i] - t))
        if dt > tolerance:
            return SkillResult.fail(
                "INVALID_INPUT",
                f"No pose within {tolerance} s of t={t}; trail covers "
                f"{t0:.3f}..{t1:.3f} ({_iso(t0)}..{_iso(t1)} UTC).",
            )
        x, y = float(trail.xy[i, 0]), float(trail.xy[i, 1])
        return SkillResult.ok(
            f"At {_iso(float(trail.ts[i]))} UTC the robot was at "
            f"({x:.2f}, {y:.2f}) in the world frame.",
            ts=round(float(trail.ts[i]), 3),
            x=round(x, 3),
            y=round(y, 3),
        )

    @skill
    def robot_visits_to_region(self, region: list[float]) -> SkillResult[CommonSkillError]:
        """When was the robot inside a region? Lists visits, most recent last.

        Answers "when were you last in <region>": the last visit's end time is
        when the robot was last there.

        Args:
            region: Region polygon in world coordinates, flattened
                [x1, y1, x2, y2, x3, y3, ...] (at least 3 vertices).
        """
        try:
            polygon = polygon_from_flat(region)
        except ValueError as e:
            return SkillResult.fail("INVALID_INPUT", str(e))
        try:
            trail = self._get_trail()
        except (FileNotFoundError, LookupError) as e:
            return SkillResult.fail("NOT_CONFIGURED", f"No pose trail available: {e}")

        t0, t1 = trail.time_range()
        inside = points_in_polygon(trail.xy, polygon)
        intervals = visit_intervals(trail.ts, inside)
        if not intervals:
            return SkillResult.ok(
                f"The robot was never inside that region during the recorded trail "
                f"({_iso(t0)}..{_iso(t1)} UTC, {t1 - t0:.1f} s of full pose coverage).",
                visits=[],
                trail_start_ts=round(t0, 3),
                trail_end_ts=round(t1, 3),
            )
        enter, exit_ = intervals[-1]
        return SkillResult.ok(
            f"The robot was last in the region from {_iso(enter)} to {_iso(exit_)} UTC "
            f"(left {t1 - exit_:.1f} s before the end of the trail). "
            f"{len(intervals)} visit(s) total.",
            visits=[[round(a, 3), round(b, 3)] for a, b in intervals],
            last_enter_ts=round(enter, 3),
            last_exit_ts=round(exit_, 3),
            trail_start_ts=round(t0, 3),
            trail_end_ts=round(t1, 3),
        )
