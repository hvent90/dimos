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

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import threading
from typing import TYPE_CHECKING, Any

import numpy as np

from dimos.control.task import (
    BaseControlTask,
    ControlMode,
    CoordinatorState,
    JointCommandOutput,
    ResourceClaim,
)
from dimos.manipulation.planning.kinematics.pinocchio_ik import (
    PinocchioIK,
    check_joint_delta,
    get_worst_joint_delta,
)
from dimos.protocol.service.spec import BaseConfig
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from numpy.typing import NDArray
    import pinocchio  # type: ignore[import-not-found]

    from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped

logger = setup_logger()


@dataclass
class EEFTwistTaskConfig:
    joint_names: list[str]
    model_path: str | Path
    ee_joint_id: int
    priority: int = 10
    timeout: float = 0.3
    max_joint_delta_deg: float = 15.0
    max_dt: float = 0.05
    max_linear_step: float = 0.02
    max_angular_step: float = 0.2
    max_final_error: float = 0.05
    accept_unconverged: bool = False


class EEFTwistTask(BaseControlTask):
    """Spatial EEF twist task using twist-integrated pose IK."""

    def __init__(self, name: str, config: EEFTwistTaskConfig) -> None:
        if not config.joint_names:
            raise ValueError(f"EEFTwistTask '{name}' requires at least one joint")
        if not config.model_path:
            raise ValueError(f"EEFTwistTask '{name}' requires model_path for IK solver")
        self._name = name
        self._config = config
        self._joint_names = frozenset(config.joint_names)
        self._joint_names_list = list(config.joint_names)
        self._ik = PinocchioIK.from_model_path(config.model_path, config.ee_joint_id)
        self._lock = threading.Lock()
        self._latest_twist: TwistStamped | None = None
        self._last_update_time = 0.0
        self._last_integrate_time: float | None = None
        self._target_pose: pinocchio.SE3 | None = None
        self._last_q_solution: NDArray[np.floating[Any]] | None = None

    @property
    def name(self) -> str:
        return self._name

    def claim(self) -> ResourceClaim:
        return ResourceClaim(self._joint_names, self._config.priority, ControlMode.SERVO_POSITION)

    def is_active(self) -> bool:
        with self._lock:
            return self._latest_twist is not None

    def on_ee_twist_command(self, twist: TwistStamped, t_now: float) -> bool:
        values = self._twist_values(twist)
        if not np.all(np.isfinite(values)):
            logger.warning(f"EEFTwistTask {self._name}: rejecting non-finite twist")
            return False
        with self._lock:
            if np.allclose(values, 0.0):
                self._clear_locked()
                self._last_update_time = t_now
                return True
            self._latest_twist = twist
            self._last_update_time = t_now
        return True

    def compute(self, state: CoordinatorState) -> JointCommandOutput | None:
        with self._lock:
            twist = self._latest_twist
            if twist is None:
                return None
            if (
                self._config.timeout > 0
                and state.t_now - self._last_update_time > self._config.timeout
            ):
                self._clear_locked()
                return None
            target_pose = self._target_pose.copy() if self._target_pose is not None else None
            last_integrate_time = self._last_integrate_time

        q_current = self._get_current_joints(state)
        if q_current is None or not np.all(np.isfinite(q_current)):
            return None
        if target_pose is None:
            target_pose = self._ik.forward_kinematics(q_current)
            last_integrate_time = state.t_now - min(max(state.dt, 0.0), self._config.max_dt)
        if not self._pose_is_finite(target_pose):
            return None

        dt = min(max(state.t_now - (last_integrate_time or state.t_now), 0.0), self._config.max_dt)
        candidate = self._integrate_twist(target_pose, twist, dt)
        if not self._pose_is_finite(candidate):
            return None

        q_solution, converged, final_error = self._ik.solve(candidate, q_current)
        if not np.all(np.isfinite(q_solution)) or not np.isfinite(final_error):
            return None
        if (
            not converged and not self._config.accept_unconverged
        ) or final_error > self._config.max_final_error:
            logger.debug(f"EEFTwistTask {self._name}: IK rejected error={final_error:.4f}")
            return None
        if not check_joint_delta(q_solution, q_current, self._config.max_joint_delta_deg):
            worst_idx, worst_deg = get_worst_joint_delta(q_solution, q_current)
            logger.warning(
                f"EEFTwistTask {self._name}: rejecting joint {self._joint_names_list[worst_idx]} "
                f"delta {worst_deg:.1f}° exceeds {self._config.max_joint_delta_deg}°"
            )
            return None

        with self._lock:
            self._target_pose = candidate.copy()
            self._last_integrate_time = state.t_now
            self._last_q_solution = q_solution.copy()
        return JointCommandOutput(
            joint_names=self._joint_names_list,
            positions=q_solution.flatten().tolist(),
            mode=ControlMode.SERVO_POSITION,
        )

    def on_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        if joints & self._joint_names:
            logger.warning(f"EEFTwistTask {self._name} preempted by {by_task} on joints {joints}")

    def _get_current_joints(self, state: CoordinatorState) -> NDArray[np.floating[Any]] | None:
        positions = []
        for joint_name in self._joint_names_list:
            pos = state.joints.get_position(joint_name)
            if pos is None:
                return None
            positions.append(pos)
        return np.array(positions, dtype=np.float64)

    def _clear_locked(self) -> None:
        self._latest_twist = None
        self._target_pose = None
        self._last_integrate_time = None

    def _twist_values(self, twist: TwistStamped) -> NDArray[np.float64]:
        return np.array(
            [
                twist.linear.x,
                twist.linear.y,
                twist.linear.z,
                twist.angular.x,
                twist.angular.y,
                twist.angular.z,
            ],
            dtype=np.float64,
        )

    def _integrate_twist(
        self, pose: pinocchio.SE3, twist: TwistStamped, dt: float
    ) -> pinocchio.SE3:
        candidate = pose.copy()
        linear_step = (
            np.array([twist.linear.x, twist.linear.y, twist.linear.z], dtype=np.float64) * dt
        )
        angular_step = (
            np.array([twist.angular.x, twist.angular.y, twist.angular.z], dtype=np.float64) * dt
        )
        linear_norm = float(np.linalg.norm(linear_step))
        angular_norm = float(np.linalg.norm(angular_step))
        if linear_norm > self._config.max_linear_step:
            linear_step *= self._config.max_linear_step / linear_norm
        if angular_norm > self._config.max_angular_step:
            angular_step *= self._config.max_angular_step / angular_norm
            angular_norm = self._config.max_angular_step
        candidate.translation = candidate.translation + linear_step
        if angular_norm > 0.0:
            candidate.rotation = (
                self._rotation_matrix(angular_step, angular_norm) @ candidate.rotation
            )
        return candidate

    def _rotation_matrix(
        self, axis_angle: NDArray[np.float64], angle: float
    ) -> NDArray[np.float64]:
        axis = axis_angle / angle
        x, y, z = axis
        skew = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]], dtype=np.float64)
        result: NDArray[np.float64] = (
            np.eye(3, dtype=np.float64)
            + np.sin(angle) * skew
            + (1.0 - np.cos(angle)) * (skew @ skew)
        )
        return result

    def _pose_is_finite(self, pose: pinocchio.SE3) -> bool:
        return bool(np.all(np.isfinite(pose.translation)) and np.all(np.isfinite(pose.rotation)))


class EEFTwistTaskParams(BaseConfig):
    model_path: str | Path
    ee_joint_id: int = 6
    timeout: float = 0.3
    max_joint_delta_deg: float = 15.0
    max_dt: float = 0.05
    max_linear_step: float = 0.02
    max_angular_step: float = 0.2
    max_final_error: float = 0.05
    accept_unconverged: bool = False


def create_task(cfg: Any, hardware: Any) -> EEFTwistTask:
    params = EEFTwistTaskParams.model_validate(cfg.params)
    return EEFTwistTask(
        cfg.name,
        EEFTwistTaskConfig(
            joint_names=cfg.joint_names,
            model_path=params.model_path,
            ee_joint_id=params.ee_joint_id,
            priority=cfg.priority,
            timeout=params.timeout,
            max_joint_delta_deg=params.max_joint_delta_deg,
            max_dt=params.max_dt,
            max_linear_step=params.max_linear_step,
            max_angular_step=params.max_angular_step,
            max_final_error=params.max_final_error,
            accept_unconverged=params.accept_unconverged,
        ),
    )
