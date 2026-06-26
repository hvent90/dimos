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

import math
from typing import Protocol

import numpy as np
from numpy.typing import NDArray

from dimos.core.global_config import GlobalConfig
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.navigation.holonomic_trajectory_controller.holonomic_path_controller import HolonomicPathController
from dimos.navigation.holonomic_trajectory_controller.trajectory_types import TrajectoryReferenceSample
from dimos.utils.trigonometry import angle_diff


def _pose_from_xy_yaw(x: float, y: float, yaw: float) -> Pose:
    return Pose(
        position=Vector3(x, y, 0.0),
        orientation=Quaternion.from_euler(Vector3(0.0, 0.0, float(yaw))),
    )


class Controller(Protocol):
    def advance(
        self,
        lookahead_point: NDArray[np.float64],
        current_odom: PoseStamped,
        measured_body_twist: Twist | None = None,
    ) -> Twist: ...

    def advance_reference(
        self,
        reference: TrajectoryReferenceSample,
        current_odom: PoseStamped,
        measured_body_twist: Twist | None = None,
    ) -> Twist: ...

    def rotate(
        self,
        yaw_error: float,
        current_odom: PoseStamped | None = None,
        measured_body_twist: Twist | None = None,
    ) -> Twist: ...

    def set_speed(self, speed_m_s: float) -> None: ...

    def reset_errors(self) -> None: ...

    def reset_yaw_error(self, value: float) -> None: ...


class PController:
    _global_config: GlobalConfig
    _speed: float
    _control_frequency: float

    _min_linear_velocity: float = 0.2
    _min_angular_velocity: float = 0.2
    _k_angular: float = 0.5
    _max_angular_accel: float = 2.0
    _rotation_threshold: float = 90 * (math.pi / 180)

    def __init__(self, global_config: GlobalConfig, speed: float, control_frequency: float) -> None:
        self._global_config = global_config
        self._speed = speed
        self._control_frequency = control_frequency

    def set_speed(self, speed_m_s: float) -> None:
        self._speed = float(speed_m_s)

    def advance(
        self,
        lookahead_point: NDArray[np.float64],
        current_odom: PoseStamped,
        measured_body_twist: Twist | None = None,
    ) -> Twist:
        reference = TrajectoryReferenceSample(
            time_s=float(current_odom.ts),
            pose_plan=_pose_from_xy_yaw(
                float(lookahead_point[0]),
                float(lookahead_point[1]),
                0.0,
            ),
            twist_body=Twist(
                linear=Vector3(self._speed, 0.0, 0.0),
                angular=Vector3(0.0, 0.0, 0.0),
            ),
        )
        return self.advance_reference(reference, current_odom, measured_body_twist)

    def advance_reference(
        self,
        reference: TrajectoryReferenceSample,
        current_odom: PoseStamped,
        measured_body_twist: Twist | None = None,
    ) -> Twist:
        del measured_body_twist
        current_pos = np.array([current_odom.position.x, current_odom.position.y])
        lookahead_point = np.array(
            [
                float(reference.pose_plan.position.x),
                float(reference.pose_plan.position.y),
            ],
            dtype=np.float64,
        )
        direction = lookahead_point - current_pos
        distance = np.linalg.norm(direction)

        if distance < 1e-6:
            # Robot is coincidentally at the lookahead point; skip this cycle.
            return Twist()

        robot_yaw = current_odom.orientation.euler[2]
        desired_yaw = np.arctan2(direction[1], direction[0])
        yaw_error = angle_diff(desired_yaw, robot_yaw)

        angular_velocity = self._compute_angular_velocity(yaw_error)

        # Rotate-then-drive: if heading error is large, rotate in place first
        if abs(yaw_error) > self._rotation_threshold:
            return self._angular_twist(angular_velocity)

        # When aligned, drive forward with proportional angular correction
        linear_velocity = self._speed * (1.0 - abs(yaw_error) / self._rotation_threshold)
        linear_velocity = self._apply_min_velocity(linear_velocity, self._min_linear_velocity)

        return Twist(
            linear=Vector3(linear_velocity, 0.0, 0.0),
            angular=Vector3(0.0, 0.0, angular_velocity),
        )

    def rotate(
        self,
        yaw_error: float,
        current_odom: PoseStamped | None = None,
        measured_body_twist: Twist | None = None,
    ) -> Twist:
        del measured_body_twist
        del current_odom
        angular_velocity = self._compute_angular_velocity(yaw_error)
        return self._angular_twist(angular_velocity)

    def _compute_angular_velocity(self, yaw_error: float) -> float:
        angular_velocity = self._k_angular * yaw_error
        angular_velocity = np.clip(angular_velocity, -self._speed, self._speed)
        angular_velocity = self._apply_min_velocity(angular_velocity, self._min_angular_velocity)
        return float(angular_velocity)

    def reset_errors(self) -> None:
        pass

    def reset_yaw_error(self, value: float) -> None:
        pass

    def _apply_min_velocity(self, velocity: float, min_velocity: float) -> float:
        """Apply minimum velocity threshold, preserving sign. Returns 0 if velocity is 0."""
        if velocity == 0.0:
            return 0.0
        if abs(velocity) < min_velocity:
            return min_velocity if velocity > 0 else -min_velocity
        return velocity

    def _angular_twist(self, angular_velocity: float) -> Twist:
        # In simulation, we need stroger values
        if self._global_config.simulation and abs(angular_velocity) < 0.8:
            angular_velocity = 0.8 * np.sign(angular_velocity)

        return Twist(
            linear=Vector3(0.0, 0.0, 0.0),
            angular=Vector3(0.0, 0.0, angular_velocity),
        )


class PdController(PController):
    _k_derivative: float = 0.15

    _prev_yaw_error: float
    _prev_angular_velocity: float

    def __init__(self, global_config: GlobalConfig, speed: float, control_frequency: float) -> None:
        super().__init__(global_config, speed, control_frequency)

        self._prev_yaw_error = 0.0
        self._prev_angular_velocity = 0.0

    def reset_errors(self) -> None:
        self._prev_yaw_error = 0.0
        self._prev_angular_velocity = 0.0

    def reset_yaw_error(self, value: float) -> None:
        self._prev_yaw_error = value

    def _compute_angular_velocity(self, yaw_error: float) -> float:
        dt = 1.0 / self._control_frequency

        # PD control: proportional + derivative damping
        yaw_error_derivative = (yaw_error - self._prev_yaw_error) / dt
        angular_velocity = self._k_angular * yaw_error - self._k_derivative * yaw_error_derivative

        # Rate limiting: limit angular acceleration to prevent jerky corrections
        max_delta = self._max_angular_accel * dt
        angular_velocity = np.clip(
            angular_velocity,
            self._prev_angular_velocity - max_delta,
            self._prev_angular_velocity + max_delta,
        )

        angular_velocity = np.clip(angular_velocity, -self._speed, self._speed)
        angular_velocity = self._apply_min_velocity(angular_velocity, self._min_angular_velocity)

        self._prev_yaw_error = yaw_error
        self._prev_angular_velocity = angular_velocity

        return float(angular_velocity)


def make_local_path_controller(
    global_config: GlobalConfig,
    speed: float,
    control_frequency: float,
) -> Controller:
    if global_config.local_planner_path_controller == "holonomic":
        return HolonomicPathController(
            global_config,
            speed,
            control_frequency,
            k_position_per_s=global_config.local_planner_holonomic_kp,
            k_yaw_per_s=global_config.local_planner_holonomic_ky,
            k_velocity_per_s=global_config.local_planner_holonomic_kv,
            k_yaw_rate_per_s=global_config.local_planner_holonomic_kw,
        )
    return PController(global_config, speed, control_frequency)
