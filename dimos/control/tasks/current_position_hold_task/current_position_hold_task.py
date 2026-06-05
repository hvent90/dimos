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

"""Current-position hold task for hardware write-path bring-up.

This task continuously outputs the current measured joint positions as
SERVO_POSITION commands. It is intended for controlled hardware tests where the
adapter should actively write hold frames without requiring an external
trajectory command.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from dimos.control.task import (
    BaseControlTask,
    ControlMode,
    CoordinatorState,
    JointCommandOutput,
    ResourceClaim,
)
from dimos.protocol.service.spec import BaseConfig
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


@dataclass
class CurrentPositionHoldTaskConfig:
    joint_names: list[str]
    priority: int = 5


class CurrentPositionHoldTask(BaseControlTask):
    """Hold the current measured joint positions every coordinator tick."""

    def __init__(self, name: str, config: CurrentPositionHoldTaskConfig) -> None:
        if not config.joint_names:
            raise ValueError(f"CurrentPositionHoldTask '{name}' requires at least one joint")
        self._name = name
        self._config = config
        self._joint_names = frozenset(config.joint_names)
        self._joint_names_list = list(config.joint_names)
        self._active = False
        logger.info(
            f"CurrentPositionHoldTask {name} initialized for joints: {config.joint_names}"
        )

    @property
    def name(self) -> str:
        return self._name

    def claim(self) -> ResourceClaim:
        return ResourceClaim(
            joints=self._joint_names,
            priority=self._config.priority,
            mode=ControlMode.SERVO_POSITION,
        )

    def is_active(self) -> bool:
        return self._active

    def compute(self, state: CoordinatorState) -> JointCommandOutput | None:
        if not self._active:
            return None
        positions: list[float] = []
        for joint_name in self._joint_names_list:
            position = state.joints.get_position(joint_name)
            if position is None:
                return None
            positions.append(position)
        return JointCommandOutput(
            joint_names=self._joint_names_list,
            positions=positions,
            mode=ControlMode.SERVO_POSITION,
        )

    def on_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        if joints & self._joint_names:
            logger.warning(
                f"CurrentPositionHoldTask {self._name} preempted by {by_task} on joints {joints}"
            )

    def start(self) -> None:
        self._active = True
        logger.info(f"CurrentPositionHoldTask {self._name} started")

    def stop(self) -> None:
        self._active = False
        logger.info(f"CurrentPositionHoldTask {self._name} stopped")


class CurrentPositionHoldTaskParams(BaseConfig):
    pass


def create_task(cfg: Any, hardware: Any) -> CurrentPositionHoldTask:
    CurrentPositionHoldTaskParams.model_validate(cfg.params)
    return CurrentPositionHoldTask(
        cfg.name,
        CurrentPositionHoldTaskConfig(
            joint_names=cfg.joint_names,
            priority=cfg.priority,
        ),
    )


__all__ = [
    "CurrentPositionHoldTask",
    "CurrentPositionHoldTaskConfig",
]
