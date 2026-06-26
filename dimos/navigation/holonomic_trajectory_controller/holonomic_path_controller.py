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

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from dimos.core.global_config import GlobalConfig
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.navigation.holonomic_trajectory_controller.trajectory_command_limits import (
    HolonomicCommandLimits,
    clamp_holonomic_cmd_vel,
)
from dimos.navigation.holonomic_trajectory_controller.trajectory_holonomic_tracking_controller import HolonomicTrackingController
from dimos.navigation.holonomic_trajectory_controller.trajectory_run_profiles import RunProfile
from dimos.navigation.holonomic_trajectory_controller.trajectory_types import TrajectoryMeasuredSample, TrajectoryReferenceSample


def _pose_from_xy_yaw(x: float, y: float, yaw: float) -> Pose:
    return Pose(
        position=Vector3(x, y, 0.0),
        orientation=Quaternion.from_euler(Vector3(0.0, 0.0, float(yaw))),
    )


def _pose_from_pose_stamped(odom: PoseStamped) -> Pose:
    return Pose(odom.position, odom.orientation)


@dataclass(frozen=True)
class CommandEnvelopeOverrides:
    """Run-profile command caps that replace the GlobalConfig defaults.

    The planar speed cap keeps tracking ``set_speed`` (the geometry-capped
    path speed); these are the remaining saturation limits a movement
    envelope owns.
    """

    max_yaw_rate_rad_s: float
    max_planar_cmd_accel_m_s2: float
    max_yaw_accel_rad_s2: float


def command_envelope_overrides_for_profile(profile: RunProfile) -> CommandEnvelopeOverrides:
    """Map a :class:`RunProfile` to planner command caps (planar speed excluded)."""
    limits = profile.command_limits()
    return CommandEnvelopeOverrides(
        max_yaw_rate_rad_s=limits.max_yaw_rate_rad_s,
        max_planar_cmd_accel_m_s2=limits.max_planar_linear_accel_m_s2,
        max_yaw_accel_rad_s2=limits.max_yaw_accel_rad_s2,
    )


class HolonomicPathController:
    """Follow path segments using the holonomic tracking law (P3-3, issue 921).

    Wraps :class:`HolonomicTrackingController` in the :class:`Controller` seam
    (lookahead + odom). Rotations in place use the same law with a fixed
    position reference. Not a car-style or Pure Pursuit path law.
    """

    def __init__(
        self,
        global_config: GlobalConfig,
        speed: float,
        control_frequency: float,
        k_position_per_s: float,
        k_yaw_per_s: float,
        k_velocity_per_s: float = 0.0,
        k_yaw_rate_per_s: float = 0.0,
    ) -> None:
        self._global_config = global_config
        self._speed = float(speed)
        self._control_frequency = float(control_frequency)
        self._inner = HolonomicTrackingController(
            k_position_per_s=k_position_per_s,
            k_yaw_per_s=k_yaw_per_s,
            k_velocity_per_s=k_velocity_per_s,
            k_yaw_rate_per_s=k_yaw_rate_per_s,
        )
        self._envelope_overrides: CommandEnvelopeOverrides | None = None
        self._limits = self._make_limits()
        self._inner.configure(self._limits)
        self._previous_cmd = Twist()

    def set_speed(self, speed_m_s: float) -> None:
        self._speed = float(speed_m_s)
        self._limits = self._make_limits()
        self._inner.configure(self._limits)

    def set_command_envelope(self, overrides: CommandEnvelopeOverrides | None) -> None:
        """Apply (or clear, with ``None``) a run profile's command caps."""
        self._envelope_overrides = overrides
        self._limits = self._make_limits()
        self._inner.configure(self._limits)

    def _make_limits(self) -> HolonomicCommandLimits:
        overrides = self._envelope_overrides
        if overrides is not None:
            return HolonomicCommandLimits(
                max_planar_speed_m_s=self._speed,
                max_yaw_rate_rad_s=overrides.max_yaw_rate_rad_s,
                max_planar_linear_accel_m_s2=overrides.max_planar_cmd_accel_m_s2,
                max_yaw_accel_rad_s2=overrides.max_yaw_accel_rad_s2,
            )
        max_yaw_rate = self._global_config.local_planner_max_yaw_rate_rad_s
        return HolonomicCommandLimits(
            max_planar_speed_m_s=self._speed,
            max_yaw_rate_rad_s=self._speed if max_yaw_rate is None else float(max_yaw_rate),
            max_planar_linear_accel_m_s2=self._global_config.local_planner_max_planar_cmd_accel_m_s2,
            max_yaw_accel_rad_s2=self._global_config.local_planner_max_yaw_accel_rad_s2,
        )

    def advance(
        self,
        lookahead_point: NDArray[np.float64],
        current_odom: PoseStamped,
        measured_body_twist: Twist | None = None,
    ) -> Twist:
        current_pos = np.array([float(current_odom.position.x), float(current_odom.position.y)])
        direction = np.asarray(lookahead_point, dtype=np.float64) - current_pos
        distance = float(np.linalg.norm(direction))

        if distance < 1e-6 or not np.isfinite(distance):
            return Twist()

        ref_yaw = float(np.arctan2(direction[1], direction[0]))
        ref_pose = _pose_from_xy_yaw(float(lookahead_point[0]), float(lookahead_point[1]), ref_yaw)
        # Feedforward along the reference heading in the body frame of the target pose.
        ref_ff = Twist(
            linear=Vector3(self._speed, 0.0, 0.0),
            angular=Vector3(0.0, 0.0, 0.0),
        )
        ref = TrajectoryReferenceSample(0.0, ref_pose, ref_ff)
        return self.advance_reference(ref, current_odom, measured_body_twist)

    def advance_reference(
        self,
        reference: TrajectoryReferenceSample,
        current_odom: PoseStamped,
        measured_body_twist: Twist | None = None,
    ) -> Twist:
        twist = Twist() if measured_body_twist is None else measured_body_twist
        meas = TrajectoryMeasuredSample(0.0, _pose_from_pose_stamped(current_odom), twist)
        return self._limit_output(self._inner.control(reference, meas))

    def rotate(
        self,
        yaw_error: float,
        current_odom: PoseStamped | None = None,
        measured_body_twist: Twist | None = None,
    ) -> Twist:
        if current_odom is None:
            # ``LocalPlanner`` should always pass odom; keep a safe fallback.
            wz = float(0.5 * yaw_error)
            wz = float(np.clip(wz, -self._speed, self._speed))
            if wz != 0.0 and abs(wz) < 0.2:
                wz = 0.2 * (1.0 if wz > 0 else -1.0)
            t = Twist(
                linear=Vector3(0.0, 0.0, 0.0),
                angular=Vector3(0.0, 0.0, wz),
            )
            return self._limit_output(self._apply_sim_angular(t))

        robot_yaw = float(current_odom.orientation.euler[2])
        target_yaw = float(np.arctan2(np.sin(robot_yaw + yaw_error), np.cos(robot_yaw + yaw_error)))
        p = _pose_from_xy_yaw(
            float(current_odom.position.x),
            float(current_odom.position.y),
            target_yaw,
        )
        ref = TrajectoryReferenceSample(0.0, p, Twist())
        twist = Twist() if measured_body_twist is None else measured_body_twist
        meas = TrajectoryMeasuredSample(0.0, _pose_from_pose_stamped(current_odom), twist)
        out = self._inner.control(ref, meas)
        return self._limit_output(self._apply_sim_angular(out))

    def reset_errors(self) -> None:
        self._inner.reset()
        self._previous_cmd = Twist()

    def reset_yaw_error(self, value: float) -> None:
        del value

    def _apply_sim_angular(self, t: Twist) -> Twist:
        wz = float(t.angular.z)
        if self._global_config.simulation and 1e-9 < abs(wz) < 0.8:
            wz = 0.8 * (1.0 if wz > 0 else -1.0)
        return Twist(
            linear=Vector3(float(t.linear.x), float(t.linear.y), float(t.linear.z)),
            angular=Vector3(0.0, 0.0, wz),
        )

    def _limit_output(self, raw: Twist) -> Twist:
        out = clamp_holonomic_cmd_vel(
            self._previous_cmd,
            raw,
            self._limits,
            1.0 / self._control_frequency,
        )
        self._previous_cmd = Twist(out)
        return out
