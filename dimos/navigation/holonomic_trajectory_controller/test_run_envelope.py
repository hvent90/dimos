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

"""Run-profile speed application in ``_HolonomicPathFollower``.

Relocated from ``dannav/test_local_planner_run_envelope.py``. The session
profile is resolved from ``DanHolonomicTCConfig.run_profile`` at construction and
re-resolved live by ``set_run_profile``; the per-goal ``run_profile_name``
override is gone, so the leak test is recast to session-profile switching.
"""

from __future__ import annotations

import math
import time

import numpy as np
import pytest

from dimos.core.global_config import GlobalConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Path import Path
from dimos.navigation.holonomic_trajectory_controller.module import (
    DanHolonomicTCConfig,
    _HolonomicPathFollower,
)
from dimos.navigation.holonomic_trajectory_controller.path_distancer import PathDistancer


def _yaw_quaternion(yaw_rad: float) -> Quaternion:
    return Quaternion(0.0, 0.0, math.sin(yaw_rad / 2.0), math.cos(yaw_rad / 2.0))


def _pose_stamped(x: float, y: float, yaw_rad: float, *, ts: float = 1.0) -> PoseStamped:
    return PoseStamped(
        ts=ts,
        frame_id="map",
        position=[x, y, 0.0],
        orientation=_yaw_quaternion(yaw_rad),
    )


def _path_from_points(points: list[tuple[float, float]]) -> Path:
    poses: list[PoseStamped] = []
    for index, point in enumerate(points):
        if index + 1 < len(points):
            next_point = points[index + 1]
            yaw = math.atan2(next_point[1] - point[1], next_point[0] - point[0])
        else:
            prev_point = points[index - 1]
            yaw = math.atan2(point[1] - prev_point[1], point[0] - prev_point[0])
        poses.append(_pose_stamped(point[0], point[1], yaw))
    return Path(frame_id="map", poses=poses)


def _straight_points(length_m: float, spacing_m: float = 0.1) -> list[tuple[float, float]]:
    n = round(length_m / spacing_m)
    return [(i * spacing_m, 0.0) for i in range(n + 1)]


def _make_follower(**overrides: object) -> _HolonomicPathFollower:
    return _HolonomicPathFollower(DanHolonomicTCConfig(**overrides))


def _path_speed_at(core: _HolonomicPathFollower, path: Path, pos: tuple[float, float]) -> float:
    distancer = PathDistancer(path)
    current_pos = np.array(pos, dtype=np.float64)
    return core._path_speed_for_index(
        distancer, distancer.find_closest_point_index(current_pos), current_pos
    )


def _integrate_holonomic_pose(
    x_m: float,
    y_m: float,
    yaw_rad: float,
    cmd_body: Twist,
    dt_s: float,
) -> tuple[float, float, float]:
    """Integrate body-frame cmd_vel in the world frame (test harness only)."""
    vx = float(cmd_body.linear.x)
    vy = float(cmd_body.linear.y)
    wz = float(cmd_body.angular.z)
    c = math.cos(yaw_rad)
    s = math.sin(yaw_rad)
    x_m += (c * vx - s * vy) * dt_s
    y_m += (s * vx + c * vy) * dt_s
    yaw_rad += wz * dt_s
    yaw_rad = math.atan2(math.sin(yaw_rad), math.cos(yaw_rad))
    return x_m, y_m, yaw_rad


def test_walk_default_profile_speed() -> None:
    """No override: the path speed is the walk profile cruise (0.55 m/s)."""
    core = _make_follower()
    speed = _path_speed_at(core, _path_from_points(_straight_points(6.0)), (3.0, 0.0))
    assert speed == pytest.approx(0.55)


def test_cruise_override_drives_path_speed() -> None:
    """``speed_m_s`` overrides the profile cruise (the old planner_robot_speed)."""
    core = _make_follower(speed_m_s=1.0)
    speed = _path_speed_at(core, _path_from_points(_straight_points(6.0)), (3.0, 0.0))
    assert speed == pytest.approx(1.0)


def test_run_conservative_drives_planner_speed_and_command_caps() -> None:
    core = _make_follower(run_profile="run_conservative")
    path = _path_from_points(_straight_points(6.0))

    mid_speed = _path_speed_at(core, path, (3.0, 0.0))
    near_goal_speed = _path_speed_at(core, path, (5.7, 0.0))

    assert mid_speed == pytest.approx(1.5)
    assert near_goal_speed == pytest.approx(math.sqrt(2.0 * 1.5 * 0.3))


def test_run_profile_speed_respects_global_nerf() -> None:
    core = _make_follower(run_profile="run_conservative", g=GlobalConfig(nerf_speed=0.5))
    speed = _path_speed_at(core, _path_from_points(_straight_points(6.0)), (3.0, 0.0))
    assert speed == pytest.approx(0.75)


def test_set_run_profile_switches_session_speed() -> None:
    """``set_run_profile`` replaces the dropped per-goal override: switching the
    session profile re-resolves the envelope live, and switching back restores
    the previous speed (no leak)."""
    core = _make_follower()
    path = _path_from_points(_straight_points(6.0))

    assert core.set_run_profile("trot") is True
    trot_speed = _path_speed_at(core, path, (3.0, 0.0))

    assert core.set_run_profile("walk") is True
    walk_speed = _path_speed_at(core, path, (3.0, 0.0))

    assert trot_speed == pytest.approx(1.0)
    assert walk_speed == pytest.approx(0.55)


def test_set_run_profile_rejects_unknown_profile() -> None:
    core = _make_follower()
    assert core.set_run_profile("does-not-exist") is False
    # The rejected name does not poison the live envelope.
    speed = _path_speed_at(core, _path_from_points(_straight_points(6.0)), (3.0, 0.0))
    assert speed == pytest.approx(0.55)


def test_run_profile_commands_faster_than_walk_in_closed_loop() -> None:
    rate_hz = 60.0
    dt_s = 1.0 / rate_hz
    core = _make_follower(
        run_profile="run_conservative", control_frequency=rate_hz, goal_tolerance=0.1
    )
    plant_x_m, plant_y_m, plant_yaw_rad = 0.0, 0.0, 0.0
    latest_cmd = Twist()
    commanded_speeds: list[float] = []
    stops: list[str] = []

    def _on_cmd_vel(cmd: Twist) -> None:
        nonlocal latest_cmd
        latest_cmd = Twist(cmd)
        commanded_speeds.append(math.hypot(float(cmd.linear.x), float(cmd.linear.y)))

    cmd_sub = core.cmd_vel.subscribe(_on_cmd_vel)
    stop_sub = core.stopped_navigating.subscribe(stops.append)
    sim_time_s = 1.0

    try:
        core.handle_odom(_pose_stamped(plant_x_m, plant_y_m, plant_yaw_rad, ts=sim_time_s))
        core.start_planning(_path_from_points([(0.1, 0.0), (3.1, 0.0)]))
        for _ in range(420):
            if "arrived" in stops:
                break
            time.sleep(dt_s * 1.1)
            plant_x_m, plant_y_m, plant_yaw_rad = _integrate_holonomic_pose(
                plant_x_m, plant_y_m, plant_yaw_rad, latest_cmd, dt_s
            )
            sim_time_s += dt_s
            core.handle_odom(
                _pose_stamped(plant_x_m, plant_y_m, plant_yaw_rad, ts=sim_time_s)
            )
    finally:
        core.close()
        cmd_sub.dispose()
        stop_sub.dispose()

    assert "arrived" in stops
    assert commanded_speeds
    assert max(commanded_speeds) > 1.0
    assert max(commanded_speeds) <= 1.5 + 1e-6
