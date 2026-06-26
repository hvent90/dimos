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

"""``_HolonomicPathFollower`` tracking law, path-speed caps, and closed loop.

Relocated from ``dannav/test_local_planner_path_controller.py`` for the
``DanHolonomicTC`` control core. The costmap/yaw-lock and differential cases are
gone with the planner; the holonomic tracking law, path-speed envelope, and
arrival behavior remain. Movement envelopes that the old tests expressed through
``GlobalConfig`` accel fields now come from the run-profile registry, so
``_install_envelope`` drives the core with an explicit ``ActiveRunEnvelope`` for
the geometry-cap unit tests (the one seam that needs arbitrary accel values).
"""

from __future__ import annotations

import math
import time

import numpy as np
import pytest

from dimos.core.global_config import GlobalConfig
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Path import Path
from dimos.navigation.holonomic_trajectory_controller.holonomic_path_controller import (
    CommandEnvelopeOverrides,
    HolonomicPathController,
)
from dimos.navigation.holonomic_trajectory_controller.module import (
    ActiveRunEnvelope,
    DanHolonomicTCConfig,
    _HolonomicPathFollower,
)
from dimos.navigation.holonomic_trajectory_controller.path_distancer import PathDistancer
from dimos.navigation.holonomic_trajectory_controller.trajectory_holonomic_tracking_controller import (
    HolonomicTrackingController,
)
from dimos.navigation.holonomic_trajectory_controller.trajectory_path_speed_profile import (
    PathSpeedProfileLimits,
)
from dimos.navigation.holonomic_trajectory_controller.trajectory_types import (
    TrajectoryMeasuredSample,
    TrajectoryReferenceSample,
)
from dimos.utils.trigonometry import angle_diff


def _planar_speed_m_s(cmd: Twist) -> float:
    return math.hypot(float(cmd.linear.x), float(cmd.linear.y))


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


def _make_follower(**overrides: object) -> _HolonomicPathFollower:
    return _HolonomicPathFollower(DanHolonomicTCConfig(**overrides))


def _install_envelope(
    core: _HolonomicPathFollower,
    *,
    speed_m_s: float,
    max_tangent_accel_m_s2: float,
    max_normal_accel_m_s2: float,
    goal_decel_m_s2: float,
    max_planar_cmd_accel_m_s2: float = 8.0,
    max_yaw_accel_rad_s2: float = 8.0,
    max_yaw_rate_rad_s: float | None = None,
) -> None:
    """Drive the core with an explicit movement envelope.

    The run-profile registry is the only production envelope source, so this is
    the test seam for the arbitrary accel values the old ``GlobalConfig`` accel
    fields used to provide. ``_apply_run_envelope`` re-seats the controller and
    invalidates the cached path-speed profile, exactly as ``set_run_profile``.
    """
    core._apply_run_envelope(
        ActiveRunEnvelope(
            profile_name="test",
            speed_m_s=speed_m_s,
            path_limits=PathSpeedProfileLimits(
                max_speed_m_s=speed_m_s,
                max_tangent_accel_m_s2=max_tangent_accel_m_s2,
                max_normal_accel_m_s2=max_normal_accel_m_s2,
            ),
            goal_decel_m_s2=goal_decel_m_s2,
            command_overrides=CommandEnvelopeOverrides(
                max_yaw_rate_rad_s=speed_m_s if max_yaw_rate_rad_s is None else max_yaw_rate_rad_s,
                max_planar_cmd_accel_m_s2=max_planar_cmd_accel_m_s2,
                max_yaw_accel_rad_s2=max_yaw_accel_rad_s2,
            ),
        )
    )


def _follower_with_envelope(
    *,
    speed_m_s: float,
    max_tangent_accel_m_s2: float,
    max_normal_accel_m_s2: float,
    goal_decel_m_s2: float,
    goal_tolerance: float = 0.01,
) -> _HolonomicPathFollower:
    core = _make_follower(goal_tolerance=goal_tolerance)
    _install_envelope(
        core,
        speed_m_s=speed_m_s,
        max_tangent_accel_m_s2=max_tangent_accel_m_s2,
        max_normal_accel_m_s2=max_normal_accel_m_s2,
        goal_decel_m_s2=goal_decel_m_s2,
    )
    return core


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


class _HarnessResult:
    def __init__(
        self,
        *,
        command_history: list[Twist],
        stop_messages: list[str],
        final_x_m: float,
        final_y_m: float,
        final_yaw_rad: float,
    ) -> None:
        self.command_history = command_history
        self.stop_messages = stop_messages
        self.final_x_m = final_x_m
        self.final_y_m = final_y_m
        self.final_yaw_rad = final_yaw_rad


class _RecordingHolonomicTrackingController(HolonomicTrackingController):
    def __init__(self) -> None:
        super().__init__(k_position_per_s=0.0, k_yaw_per_s=0.0)
        self.measurements: list[TrajectoryMeasuredSample] = []

    def control(
        self,
        reference: TrajectoryReferenceSample,
        measurement: TrajectoryMeasuredSample,
    ) -> Twist:
        self.measurements.append(measurement)
        return super().control(reference, measurement)


def _follower_with_recording_holonomic_controller() -> tuple[
    _HolonomicPathFollower, _RecordingHolonomicTrackingController
]:
    core = _make_follower(goal_tolerance=0.01)
    path = _path_from_points([(0.0, 0.0), (2.0, 0.0)])
    core._path = path
    core._path_distancer = PathDistancer(path)
    controller = core._controller
    assert isinstance(controller, HolonomicPathController)
    recorder = _RecordingHolonomicTrackingController()
    controller._inner = recorder
    return core, recorder


def _run_follower_harness(
    core: _HolonomicPathFollower,
    *,
    points: list[tuple[float, float]],
    initial_yaw_rad: float = 0.0,
    max_ticks: int = 260,
    rate_hz: float = 60.0,
) -> _HarnessResult:
    dt_s = 1.0 / rate_hz
    plant_x_m, plant_y_m, plant_yaw_rad = 0.0, 0.0, initial_yaw_rad
    latest_cmd = Twist()
    command_history: list[Twist] = []
    stop_messages: list[str] = []

    def _on_cmd_vel(cmd: Twist) -> None:
        nonlocal latest_cmd
        latest_cmd = Twist(cmd)
        command_history.append(Twist(cmd))

    cmd_sub = core.cmd_vel.subscribe(_on_cmd_vel)
    stop_sub = core.stopped_navigating.subscribe(stop_messages.append)
    sim_time_s = 1.0

    try:
        core.handle_odom(_pose_stamped(plant_x_m, plant_y_m, plant_yaw_rad, ts=sim_time_s))
        core.start_planning(_path_from_points(points))
        for _ in range(max_ticks):
            if "arrived" in stop_messages:
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

    return _HarnessResult(
        command_history=command_history,
        stop_messages=stop_messages,
        final_x_m=plant_x_m,
        final_y_m=plant_y_m,
        final_yaw_rad=plant_yaw_rad,
    )


def test_holonomic_path_controller_slews_first_command_from_rest() -> None:
    ctrl = HolonomicPathController(
        GlobalConfig(),
        speed=0.55,
        control_frequency=10.0,
        k_position_per_s=2.0,
        k_yaw_per_s=1.5,
    )
    odom = PoseStamped(frame_id="map", position=[0.0, 0.0, 0.0], orientation=Quaternion(0, 0, 0, 1))
    out = ctrl.advance(np.array([0.5, 0.0], dtype=np.float64), odom)
    assert math.hypot(float(out.linear.x), float(out.linear.y)) == pytest.approx(0.5)


def test_path_following_passes_estimated_measured_body_twist() -> None:
    core, recorder = _follower_with_recording_holonomic_controller()
    core._current_odom = _pose_stamped(0.0, 0.0, 0.0, ts=1.0)
    core._compute_path_following()
    core._current_odom = _pose_stamped(0.3, -0.1, 0.2, ts=1.5)

    core._compute_path_following()

    measured = recorder.measurements[-1].twist_body
    vx_w = 0.3 / 0.5
    vy_w = -0.1 / 0.5
    assert measured.linear.x == pytest.approx(math.cos(0.2) * vx_w + math.sin(0.2) * vy_w)
    assert measured.linear.y == pytest.approx(-math.sin(0.2) * vx_w + math.cos(0.2) * vy_w)
    assert measured.angular.z == pytest.approx(0.2 / 0.5)


def test_rotation_passes_estimated_measured_body_twist() -> None:
    core, recorder = _follower_with_recording_holonomic_controller()
    core._current_odom = _pose_stamped(0.0, 0.0, 1.0, ts=1.0)
    core._compute_initial_rotation()
    core._current_odom = _pose_stamped(0.2, 0.1, 0.8, ts=1.4)

    core._compute_initial_rotation()

    measured = recorder.measurements[-1].twist_body
    vx_w = 0.2 / 0.4
    vy_w = 0.1 / 0.4
    assert measured.linear.x == pytest.approx(math.cos(0.8) * vx_w + math.sin(0.8) * vy_w)
    assert measured.linear.y == pytest.approx(-math.sin(0.8) * vx_w + math.cos(0.8) * vy_w)
    assert measured.angular.z == pytest.approx(angle_diff(0.8, 1.0) / 0.4)


def test_invalid_dt_passes_zero_measured_body_twist() -> None:
    core, recorder = _follower_with_recording_holonomic_controller()
    core._current_odom = _pose_stamped(0.0, 0.0, 0.0, ts=5.0)
    core._compute_path_following()
    core._current_odom = _pose_stamped(0.2, 0.0, 0.0, ts=5.0)

    core._compute_path_following()

    measured = recorder.measurements[-1].twist_body
    assert measured.is_zero()


def test_lookahead_reference_uses_path_end_progress() -> None:
    core = _make_follower(speed_m_s=0.5, goal_tolerance=0.01)
    path = _path_from_points([(0.0, 0.0), (2.0, 0.0)])
    distancer = PathDistancer(path)
    odom = _pose_stamped(0.2, 0.1, 0.0, ts=10.0)
    current_pos = np.array([odom.position.x, odom.position.y], dtype=np.float64)
    path_speed = core._path_speed_for_index(distancer, 0, current_pos)

    reference = core._lookahead_reference_sample(
        distancer,
        odom,
        current_pos,
        path_speed,
    )

    assert reference.pose_plan.position.x == pytest.approx(0.7)
    assert reference.time_s == pytest.approx(11.0)


def test_uses_curvature_speed_cap_for_holonomic_path() -> None:
    core = _follower_with_envelope(
        speed_m_s=1.2,
        max_tangent_accel_m_s2=1.0,
        max_normal_accel_m_s2=0.6,
        goal_decel_m_s2=1.0,
    )
    path = Path(
        frame_id="map",
        poses=[
            PoseStamped(frame_id="map", position=[0.0, 0.0, 0.0], orientation=Quaternion(0, 0, 0, 1)),
            PoseStamped(frame_id="map", position=[0.5, 0.0, 0.0], orientation=Quaternion(0, 0, 0, 1)),
            PoseStamped(frame_id="map", position=[0.5, 0.5, 0.0], orientation=Quaternion(0, 0, 0, 1)),
        ],
    )
    core._path = path
    core._path_distancer = PathDistancer(path)
    core._current_odom = PoseStamped(
        frame_id="map",
        position=[0.5, 0.0, 0.0],
        orientation=Quaternion(0, 0, 0, 1),
    )

    core._compute_path_following()

    assert core._controller._speed < 1.2


def test_uses_configured_goal_decel_for_path_speed_cap() -> None:
    core = _follower_with_envelope(
        speed_m_s=2.0,
        max_tangent_accel_m_s2=4.0,
        max_normal_accel_m_s2=0.6,
        goal_decel_m_s2=0.5,
    )
    distancer = PathDistancer(_path_from_points([(0.0, 0.0), (1.0, 0.0)]))
    current_pos = np.array([0.75, 0.0], dtype=np.float64)

    speed = core._path_speed_for_index(distancer, 0, current_pos)

    assert speed == pytest.approx(0.5, abs=1e-3)


def test_path_speed_uses_geometry_and_near_goal_decel_caps() -> None:
    max_normal_accel_m_s2 = 0.5
    goal_decel_m_s2 = 1.0
    core = _follower_with_envelope(
        speed_m_s=2.0,
        max_tangent_accel_m_s2=1.0,
        max_normal_accel_m_s2=max_normal_accel_m_s2,
        goal_decel_m_s2=goal_decel_m_s2,
    )
    distancer = PathDistancer(
        _path_from_points([(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (2.0, 1.0)])
    )

    corner_pos = np.array([1.0, 0.0], dtype=np.float64)
    corner_index = distancer.find_closest_point_index(corner_pos)
    corner_speed = core._path_speed_for_index(distancer, corner_index, corner_pos)

    near_goal_pos = np.array([1.95, 1.0], dtype=np.float64)
    near_goal_index = distancer.find_closest_point_index(near_goal_pos)
    near_goal_speed = core._path_speed_for_index(distancer, near_goal_index, near_goal_pos)

    right_angle_radius_m = math.sqrt(0.5)
    geometry_cap_m_s = math.sqrt(max_normal_accel_m_s2 * right_angle_radius_m)
    goal_decel_cap_m_s = math.sqrt(
        2.0 * goal_decel_m_s2 * distancer.distance_to_goal(near_goal_pos)
    )
    assert corner_speed == pytest.approx(geometry_cap_m_s)
    assert near_goal_speed == pytest.approx(goal_decel_cap_m_s, abs=1e-3)
    assert near_goal_speed < corner_speed


def test_path_speed_anticipates_corner_tangent_decel() -> None:
    speed_m_s = 2.0
    core = _follower_with_envelope(
        speed_m_s=speed_m_s,
        max_tangent_accel_m_s2=0.5,
        max_normal_accel_m_s2=0.1,
        goal_decel_m_s2=1.0,
    )
    distancer = PathDistancer(_path_from_points([(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]))

    before_corner_pos = np.array([0.85, 0.0], dtype=np.float64)
    before_corner_speed = core._path_speed_for_index(
        distancer,
        distancer.find_closest_point_index(before_corner_pos),
        before_corner_pos,
    )

    assert before_corner_speed < speed_m_s
    assert before_corner_speed < 1.0


def test_in_the_loop_harness_reaches_arrival_on_straight_line() -> None:
    result = _run_follower_harness(
        _make_follower(speed_m_s=1.0, control_frequency=60.0, goal_tolerance=0.08),
        points=[(0.1, 0.0), (1.2, 0.0)],
    )

    assert "arrived" in result.stop_messages
    assert result.command_history
    assert math.hypot(result.final_x_m - 1.2, result.final_y_m) < 0.15


def test_in_the_loop_harness_exercises_curvature_speed_cap() -> None:
    result = _run_follower_harness(
        _make_follower(speed_m_s=1.2, control_frequency=60.0, goal_tolerance=0.08),
        points=[(0.1, 0.0), (0.45, 0.0), (0.45, 0.45), (0.8, 0.45)],
        max_ticks=320,
    )

    assert "arrived" in result.stop_messages


def test_in_the_loop_harness_supports_2mps_right_angle_with_conservative_profile() -> None:
    core = _make_follower(control_frequency=60.0, goal_tolerance=0.08)
    _install_envelope(
        core,
        speed_m_s=2.0,
        max_tangent_accel_m_s2=0.5,
        max_normal_accel_m_s2=0.1,
        goal_decel_m_s2=0.5,
    )
    result = _run_follower_harness(
        core,
        points=[(0.1, 0.0), (0.55, 0.0), (0.55, 0.55), (0.9, 0.55)],
        max_ticks=420,
    )
    commanded_speeds = [_planar_speed_m_s(cmd) for cmd in result.command_history]

    assert "arrived" in result.stop_messages
    assert commanded_speeds
    assert max(commanded_speeds) < 0.75


def test_in_the_loop_harness_decelerates_near_goal() -> None:
    result = _run_follower_harness(
        _make_follower(speed_m_s=1.2, control_frequency=60.0, goal_tolerance=0.08),
        points=[(0.1, 0.0), (1.0, 0.0)],
        max_ticks=300,
    )
    commanded_speeds = [_planar_speed_m_s(cmd) for cmd in result.command_history]
    moving_speeds = [speed for speed in commanded_speeds if speed > 0.05]

    assert "arrived" in result.stop_messages
    assert moving_speeds
    assert max(moving_speeds[: len(moving_speeds) // 2]) > 0.7
    assert min(moving_speeds[-10:]) < 0.55
    assert min(moving_speeds[-10:]) < 0.6 * max(moving_speeds)
    assert math.hypot(result.final_x_m - 1.0, result.final_y_m) < 0.15


def test_in_the_loop_harness_exercises_initial_rotation() -> None:
    # Rotate-first is off by default for the holonomic base; opt in to exercise
    # the initial_rotation state machine branch.
    result = _run_follower_harness(
        _make_follower(
            speed_m_s=0.9,
            control_frequency=60.0,
            goal_tolerance=0.08,
            align_heading_before_move=True,
        ),
        points=[(0.1, 0.0), (1.0, 0.0)],
        initial_yaw_rad=0.8,
        max_ticks=320,
    )

    assert "arrived" in result.stop_messages
    assert any(
        abs(float(cmd.angular.z)) > 0.05 and _planar_speed_m_s(cmd) < 0.05
        for cmd in result.command_history[:20]
    )
    assert abs(result.final_yaw_rad) < 0.35


def test_holonomic_velocity_damping_reduces_planar_command() -> None:
    # Aligned pose so k_position/k_yaw contribute nothing; the only difference is
    # the velocity-damping gain wired into the inner law via the controller ctor.
    reference = TrajectoryReferenceSample(
        time_s=1.0,
        pose_plan=Pose(position=[0.0, 0.0, 0.0], orientation=_yaw_quaternion(0.0)),
        twist_body=Twist(linear=[0.5, 0.2, 0.0]),
    )
    odom = _pose_stamped(0.0, 0.0, 0.0, ts=1.0)
    measured_body_twist = Twist(linear=[1.1, 0.6, 0.0])

    def _factory(k_velocity_per_s: float) -> HolonomicPathController:
        return HolonomicPathController(
            # Generous slew headroom so the first-tick accel clamp does not mask the gain.
            GlobalConfig(local_planner_max_planar_cmd_accel_m_s2=8.0),
            speed=1.0,
            control_frequency=10.0,
            k_position_per_s=0.0,
            k_yaw_per_s=0.0,
            k_velocity_per_s=k_velocity_per_s,
            k_yaw_rate_per_s=0.0,
        )

    undamped = _factory(0.0).advance_reference(reference, odom, measured_body_twist)
    damped = _factory(0.5).advance_reference(reference, odom, measured_body_twist)

    assert undamped.linear.x == pytest.approx(0.5)
    assert undamped.linear.y == pytest.approx(0.2)
    # 0.5 * (ref - measured) damping pulls the command down to (0.2, 0.0).
    assert damped.linear.x == pytest.approx(0.2)
    assert damped.linear.y == pytest.approx(0.0)
    assert math.hypot(damped.linear.x, damped.linear.y) < math.hypot(
        undamped.linear.x, undamped.linear.y
    )
