# Copyright 2025-2026 Dimensional Inc.
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

"""Unit tests for the trajectory generator: the profile never exceeds the
planning margins, the reference is C1 (velocity-continuous), and the
endpoint holds."""

from __future__ import annotations

import math

import numpy as np
import pytest

from dimos.control.tasks.trajectory_tracking_task.constants import PLAN_MAX_ACC, PLAN_MAX_VEL
from dimos.control.tasks.trajectory_tracking_task.trajectory_generator import (
    TimedTrajectory,
    TrapezoidalProfile,
)
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Path import Path

_DT = 0.01  # dense sampling at the coordinator tick rate
_PLAN_V = min(PLAN_MAX_VEL.x, PLAN_MAX_VEL.y)
_PLAN_A = min(PLAN_MAX_ACC.x, PLAN_MAX_ACC.y)


def _pose(x: float, y: float, yaw: float = 0.0) -> PoseStamped:
    return PoseStamped(
        position=Vector3(x, y, 0.0),
        orientation=Quaternion.from_euler(Vector3(0.0, 0.0, yaw)),
    )


def _line_path(length: float = 2.0, step: float = 0.05) -> Path:
    n = int(length / step) + 1
    return Path(frame_id="world", poses=[_pose(i * step, 0.0) for i in range(n)])


def _l_path(leg: float = 1.0, step: float = 0.05) -> Path:
    n = int(leg / step)
    poses = [_pose(i * step, 0.0) for i in range(n + 1)]
    poses += [_pose(leg, i * step, math.pi / 2) for i in range(1, n + 1)]
    return Path(frame_id="world", poses=poses)


def _speeds(trajectory: TimedTrajectory) -> np.ndarray:
    ts = np.arange(0.0, trajectory.duration + _DT, _DT)
    return np.array(
        [math.hypot(s.vx_world, s.vy_world) for s in (trajectory.sample(t) for t in ts)]
    )


def test_trapezoid_reaches_and_respects_cruise() -> None:
    profile = TrapezoidalProfile.plan(distance=4.0, v_max=0.5, a_max=0.5)
    assert profile.v_cruise == pytest.approx(0.5)
    s_end, v_end = profile.sample(profile.duration)
    assert s_end == pytest.approx(4.0)
    assert v_end == pytest.approx(0.0)


def test_trapezoid_degenerates_to_triangle() -> None:
    # Too short to reach cruise: peak = sqrt(d * a) < v_max.
    profile = TrapezoidalProfile.plan(distance=0.1, v_max=0.5, a_max=0.5)
    assert profile.t_cruise == pytest.approx(0.0)
    assert profile.v_cruise == pytest.approx(math.sqrt(0.1 * 0.5))


def test_profile_never_exceeds_planning_margins() -> None:
    trajectory = TimedTrajectory.from_path(_line_path(4.0))
    speeds = _speeds(trajectory)
    assert float(speeds.max()) <= _PLAN_V + 1e-9
    accel = np.abs(np.diff(speeds)) / _DT
    assert float(accel.max()) <= _PLAN_A + 1e-6


def test_max_speed_caps_but_never_raises() -> None:
    slow = TimedTrajectory.from_path(_line_path(4.0), max_speed=0.2)
    assert slow.max_speed == pytest.approx(0.2)
    fast = TimedTrajectory.from_path(_line_path(4.0), max_speed=10.0)
    assert fast.max_speed <= _PLAN_V


def test_velocity_is_c1_continuous() -> None:
    trajectory = TimedTrajectory.from_path(_line_path(2.0), max_speed=0.5)
    speeds = _speeds(trajectory)
    # C1: no velocity jump anywhere exceeds one accel-limited tick.
    assert float(np.abs(np.diff(speeds)).max()) <= _PLAN_A * _DT + 1e-9


def test_endpoint_holds_past_duration() -> None:
    trajectory = TimedTrajectory.from_path(_line_path(2.0), max_speed=0.5)
    end = trajectory.sample(trajectory.duration + 100.0)
    assert end.x == pytest.approx(2.0)
    assert end.vx_world == pytest.approx(0.0)
    assert end.vy_world == pytest.approx(0.0)


def test_tangent_heading_follows_path() -> None:
    trajectory = TimedTrajectory.from_path(_l_path(), max_speed=0.3)
    start = trajectory.sample(0.0)
    end = trajectory.end_sample()
    assert start.yaw == pytest.approx(0.0, abs=1e-6)
    assert end.yaw == pytest.approx(math.pi / 2, abs=1e-6)


def test_fixed_heading_mode() -> None:
    heading = 0.7
    trajectory = TimedTrajectory.from_path(
        _l_path(), max_speed=0.3, heading_mode="fixed", fixed_heading=heading
    )
    for t in np.arange(0.0, trajectory.duration, 0.1):
        sample = trajectory.sample(float(t))
        assert sample.yaw == pytest.approx(heading)
        assert sample.omega == pytest.approx(0.0)


def _interior_min_speed(trajectory: TimedTrajectory, margin_s: float = 1.5) -> float:
    """Min speed away from the rest-to-rest end ramps."""
    ts = np.arange(margin_s, trajectory.duration - margin_s, _DT)
    return min(math.hypot(s.vx_world, s.vy_world) for s in (trajectory.sample(t) for t in ts))


def test_straight_holds_cruise_corner_slows() -> None:
    """A straight path holds cruise in the interior; a path with a sharp
    corner dips well below cruise where the curvature spikes."""
    straight = TimedTrajectory.from_path(_line_path(2.0), max_speed=0.5)
    corner = TimedTrajectory.from_path(_l_path(1.0), max_speed=0.5)
    assert _interior_min_speed(straight) == pytest.approx(0.5, abs=0.02)
    assert _interior_min_speed(corner) < 0.3


def test_straight_profile_is_unimodal_trapezoid() -> None:
    """No curvature -> the curvature-aware profile reduces to a trapezoid:
    speed rises to cruise, holds, falls — never dips in the middle."""
    speeds = _speeds(TimedTrajectory.from_path(_line_path(4.0), max_speed=0.5))
    peak = int(np.argmax(speeds))
    assert np.all(np.diff(speeds[: peak + 1]) >= -1e-9)  # non-decreasing up to peak
    assert np.all(np.diff(speeds[peak:]) <= 1e-9)  # non-increasing after


def test_sharp_corner_slows_more_than_rounded() -> None:
    """The benchmark's sharp 90° corner forces a deeper speed dip than its
    filleted counterpart — the curvature-aware basis of the sharp-vs-curved
    benchmark comparison."""
    from dimos.utils.benchmarking.paths import single_corner, smooth_corner

    sharp = TimedTrajectory.from_path(single_corner(leg_length=2.0, angle_deg=90.0), max_speed=0.5)
    rounded = TimedTrajectory.from_path(
        smooth_corner(leg_length=2.0, angle_deg=90.0, arc_radius=0.5), max_speed=0.5
    )
    assert _interior_min_speed(sharp) < _interior_min_speed(rounded)


def test_rounded_square_is_closed_and_bounded() -> None:
    """rounded_square is a closed loop fitting inside the side-by-side box,
    with filleted (not sharp) corners."""
    from dimos.utils.benchmarking.paths import rounded_square

    path = rounded_square(side=2.0, arc_radius=0.5)
    pts = np.array([[p.position.x, p.position.y] for p in path.poses])
    assert np.hypot(*(pts[0] - pts[-1])) < 1e-6  # closed
    assert pts[:, 0].min() >= -1e-9 and pts[:, 0].max() <= 2.0 + 1e-9
    assert pts[:, 1].min() >= -1e-9 and pts[:, 1].max() <= 2.0 + 1e-9


def test_rejects_empty_path_and_bad_mode() -> None:
    with pytest.raises(ValueError):
        TimedTrajectory.from_path(Path(frame_id="world", poses=[]))
    with pytest.raises(ValueError):
        TimedTrajectory.from_path(_line_path(), heading_mode="spiral")
