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

from __future__ import annotations

import math
from pathlib import Path
from typing import Any

import numpy as np

from dimos.hardware.manipulators.spec import (
    ControlMode,
    JointLimits,
    ManipulatorInfo,
)
from dimos.sim2.control.client import SimRobotClient
from dimos.sim2.ipc.registry import SimRegistry
from dimos.sim2.spec import ControlInterface

_MODE_POSITION = 0
_MODE_VELOCITY = 1
_MODE_EFFORT = 2


class SimManipulatorAdapter:
    def __init__(
        self,
        dof: int,
        hardware_id: str,
        sim_id: str = "main",
        robot_id: str | None = None,
        address: str | Path | None = None,
        registry: SimRegistry | None = None,
        **_: Any,
    ) -> None:
        del address
        self._dof = dof
        self._mode = ControlMode.POSITION
        self._enabled = True
        self._gripper_target = 0.0
        self._command_mode = _MODE_POSITION
        self._position_target = [0.0] * dof
        self._velocity_target = [0.0] * dof
        self._effort_target = [0.0] * dof
        self._velocity_scale = 1.0
        self._client = SimRobotClient(
            sim_id=sim_id,
            robot_id=robot_id or hardware_id,
            expected_interface=ControlInterface.MANIPULATOR,
            dof=dof,
            registry=registry,
        )

    def connect(self) -> bool:
        connected = self._client.connect()
        if connected:
            self._position_target = self.read_joint_positions()
        return connected

    def disconnect(self) -> None:
        self._client.disconnect()

    def is_connected(self) -> bool:
        return self._client.is_connected()

    def activate(self) -> bool:
        return self.write_enable(True)

    def deactivate(self) -> bool:
        stopped = self.write_stop()
        disabled = self.write_enable(False)
        return stopped and disabled

    def get_info(self) -> ManipulatorInfo:
        return ManipulatorInfo(vendor="DimOS", model="sim2", dof=self._dof)

    def get_dof(self) -> int:
        return self._dof

    def get_limits(self) -> JointLimits:
        return JointLimits(
            position_lower=[-math.pi] * self._dof,
            position_upper=[math.pi] * self._dof,
            velocity_max=[math.pi] * self._dof,
        )

    def set_control_mode(self, mode: ControlMode) -> bool:
        if mode not in (
            ControlMode.POSITION,
            ControlMode.SERVO_POSITION,
            ControlMode.VELOCITY,
            ControlMode.TORQUE,
        ):
            return False
        self._mode = mode
        return True

    def get_control_mode(self) -> ControlMode:
        return self._mode

    def read_joint_positions(self) -> list[float]:
        return self._read_vector("position")

    def read_joint_velocities(self) -> list[float]:
        return self._read_vector("velocity")

    def read_joint_efforts(self) -> list[float]:
        return self._read_vector("effort")

    def read_state(self) -> dict[str, int]:
        moving = any(abs(value) > 1e-4 for value in self.read_joint_velocities())
        return {"state": int(moving), "mode": list(ControlMode).index(self._mode)}

    def read_error(self) -> tuple[int, str]:
        observation = self._client.observation()
        code = int(observation.values["error_code"][0]) if observation is not None else -1
        return code, "" if code == 0 else "simulation backend error"

    def write_joint_positions(self, positions: list[float], velocity: float = 1.0) -> bool:
        if len(positions) != self._dof:
            return False
        self._mode = ControlMode.POSITION
        self._command_mode = _MODE_POSITION
        self._position_target = list(positions)
        self._velocity_scale = velocity
        return self._publish()

    def write_joint_velocities(self, velocities: list[float]) -> bool:
        if len(velocities) != self._dof:
            return False
        self._mode = ControlMode.VELOCITY
        self._command_mode = _MODE_VELOCITY
        self._velocity_target = list(velocities)
        return self._publish()

    def write_joint_efforts(self, efforts: list[float]) -> bool:
        if len(efforts) != self._dof:
            return False
        self._mode = ControlMode.TORQUE
        self._command_mode = _MODE_EFFORT
        self._effort_target = list(efforts)
        return self._publish()

    def write_stop(self) -> bool:
        self._command_mode = _MODE_POSITION
        self._position_target = self.read_joint_positions()
        return self._publish()

    def write_enable(self, enable: bool) -> bool:
        self._enabled = enable
        return self._publish()

    def read_enabled(self) -> bool:
        observation = self._client.observation()
        return bool(observation.values["enabled"][0]) if observation is not None else False

    def write_clear_errors(self) -> bool:
        return self.is_connected()

    def read_cartesian_position(self) -> dict[str, float] | None:
        return None

    def write_cartesian_position(
        self,
        pose: dict[str, float],
        velocity: float = 1.0,
    ) -> bool:
        del pose, velocity
        return False

    def read_gripper_position(self) -> float | None:
        if "gripper" not in self._client.capabilities:
            return None
        observation = self._client.observation()
        return float(observation.values["gripper"][0]) if observation is not None else None

    def write_gripper_position(self, position: float) -> bool:
        if "gripper" not in self._client.capabilities:
            return False
        self._gripper_target = position
        return self._publish()

    def read_force_torque(self) -> list[float] | None:
        return None

    def _read_vector(self, name: str) -> list[float]:
        observation = self._client.observation()
        if observation is None:
            return [0.0] * self._dof
        return [float(value) for value in observation.values[name]]

    def _publish(self) -> bool:
        return self._client.publish(
            {
                "command_mode": np.array([self._command_mode], dtype=np.int32),
                "enabled": np.array([self._enabled], dtype=np.uint8),
                "position": np.asarray(self._position_target, dtype=np.float64),
                "velocity": np.asarray(self._velocity_target, dtype=np.float64),
                "effort": np.asarray(self._effort_target, dtype=np.float64),
                "velocity_scale": np.array([self._velocity_scale], dtype=np.float64),
                "gripper": np.array([self._gripper_target], dtype=np.float64),
            }
        )
