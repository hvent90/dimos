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

"""Joint-space Quest teleop for a small joint group (e.g. the Go2 FR leg).

Maps Quest controller pose deltas directly to N joint deltas, without IK.
Receives pose via the coordinator's `cartesian_command` routing (the Quest
module stamps `frame_id = task_name`; only deltas while the right primary
button is held - that's the base `QuestTeleopModule` engage behavior).

Use case: Go2 tripod - operator holds A on right controller, then waves
the controller to wiggle the held-up FR paw. Joint-space avoids IK
singularities and feels direct for a 3-DOF leg.

Default 3-joint mapping (override `axis_map` for other configs):
    controller delta x -> j0  (hip abduction)
    controller delta y -> j1  (thigh pitch, scaled negative so push-fwd = lift)
    controller delta z -> j2  (calf curl)
"""

from __future__ import annotations

import threading
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any

import numpy as np
from pydantic import Field

from dimos.control.task import (
    BaseControlTask,
    ControlMode,
    CoordinatorState,
    JointCommandOutput,
    ResourceClaim,
)
from dimos.protocol.service.spec import BaseConfig
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped

logger = setup_logger()


@dataclass
class QuestJointTaskConfig:
    """Per-instance config built from TaskConfig.params."""

    joint_names: list[str]
    rest_pose: list[float]
    # 3xN row-major mapping: rows = controller (x, y, z), cols = joints.
    # Default fits Go2 FR leg: dx->hip, -dy->thigh (push fwd = lift), dz->calf.
    axis_map: list[float] = field(default_factory=lambda: [
        2.0, 0.0, 0.0,   # x -> j0
        0.0, -2.0, 0.0,  # y -> j1
        0.0, 0.0, 3.0,   # z -> j2
    ])
    joint_limits: list[tuple[float, float]] | None = None
    priority: int = 20
    command_timeout: float = 0.3


class QuestJointTask(BaseControlTask):
    """Quest right-controller delta -> joint targets, gated by Quest engage upstream."""

    def __init__(self, name: str, config: QuestJointTaskConfig) -> None:
        self._name = name
        self._config = config
        self._n = len(config.joint_names)
        if len(config.rest_pose) != self._n:
            raise ValueError(
                f"rest_pose len {len(config.rest_pose)} != joint_names len {self._n}"
            )
        axis = np.array(config.axis_map, dtype=np.float32)
        if axis.size != 3 * self._n:
            raise ValueError(
                f"axis_map len {axis.size} != 3*{self._n} for {self._n} joints"
            )
        # Stored as (3, N) so axis_map.T @ delta_xyz gives joint deltas.
        self._axis_map = axis.reshape(3, self._n)
        self._rest_pose = np.array(config.rest_pose, dtype=np.float32)
        self._joint_set = frozenset(config.joint_names)

        self._last_delta = np.zeros(3, dtype=np.float32)
        self._last_command_t = -1.0
        self._lock = threading.Lock()

        logger.info(
            f"QuestJointTask {name} initialized for {self._n} joints "
            f"(priority={config.priority}, joints={config.joint_names})"
        )

    @property
    def name(self) -> str:
        return self._name

    def claim(self) -> ResourceClaim:
        return ResourceClaim(
            joints=self._joint_set,
            priority=self._config.priority,
            mode=ControlMode.SERVO_POSITION,
        )

    def is_active(self) -> bool:
        with self._lock:
            return self._last_command_t > 0.0

    def compute(self, state: CoordinatorState) -> JointCommandOutput | None:
        with self._lock:
            if self._last_command_t < 0.0:
                return None
            # Timeout: release joints if no fresh pose (operator dropped A).
            if state.t_now - self._last_command_t > self._config.command_timeout:
                self._last_command_t = -1.0
                self._last_delta[:] = 0.0
                return None
            delta = self._last_delta.copy()

        joint_deltas = delta @ self._axis_map  # (3,) @ (3,N) -> (N,)
        target = self._rest_pose + joint_deltas

        if self._config.joint_limits is not None:
            for i, (lo, hi) in enumerate(self._config.joint_limits):
                target[i] = float(np.clip(target[i], lo, hi))

        return JointCommandOutput(
            joint_names=list(self._config.joint_names),
            positions=[float(x) for x in target],
            mode=ControlMode.SERVO_POSITION,
        )

    def on_cartesian_command(self, msg: PoseStamped, t_now: float) -> None:
        """Coordinator delivers pose here when `msg.frame_id == self.name`."""
        with self._lock:
            self._last_delta[0] = float(msg.position.x)
            self._last_delta[1] = float(msg.position.y)
            self._last_delta[2] = float(msg.position.z)
            self._last_command_t = t_now


class QuestJointTaskParams(BaseConfig):
    """TaskConfig.params schema."""

    rest_pose: list[float] = Field(...)
    axis_map: list[float] | None = None
    joint_limits: list[tuple[float, float]] | None = None
    command_timeout: float = 0.3


def create_task(cfg: Any, hardware: Any) -> QuestJointTask:
    params = QuestJointTaskParams.model_validate(cfg.params)
    kwargs: dict[str, Any] = {
        "joint_names": cfg.joint_names,
        "rest_pose": params.rest_pose,
        "priority": cfg.priority,
        "command_timeout": params.command_timeout,
    }
    if params.axis_map is not None:
        kwargs["axis_map"] = params.axis_map
    if params.joint_limits is not None:
        kwargs["joint_limits"] = params.joint_limits
    return QuestJointTask(cfg.name, QuestJointTaskConfig(**kwargs))


__all__ = ["QuestJointTask", "QuestJointTaskConfig", "create_task"]
