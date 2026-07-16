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

from pathlib import Path
from typing import Any

import numpy as np

from dimos.hardware.whole_body.spec import (
    POS_STOP,
    VEL_STOP,
    IMUState,
    MotorCommand,
    MotorState,
)
from dimos.sim2.control.client import SimRobotClient
from dimos.sim2.ipc.registry import SimRegistry
from dimos.sim2.spec import ControlInterface


class SimWholeBodyAdapter:
    def __init__(
        self,
        dof: int,
        hardware_id: str,
        sim_id: str = "main",
        robot_id: str | None = None,
        address: str | Path | None = None,
        domain_id: int = 0,
        registry: SimRegistry | None = None,
        **_: Any,
    ) -> None:
        del address, domain_id
        self._dof = dof
        self._client = SimRobotClient(
            sim_id=sim_id,
            robot_id=robot_id or hardware_id,
            expected_interface=ControlInterface.WHOLE_BODY,
            dof=dof,
            registry=registry,
        )

    def connect(self) -> bool:
        return self._client.connect()

    def disconnect(self) -> None:
        self._client.disconnect()

    def is_connected(self) -> bool:
        return self._client.is_connected()

    def has_motor_states(self) -> bool:
        return self._client.observation() is not None

    def read_motor_states(self) -> list[MotorState]:
        observation = self._client.observation()
        if observation is None:
            return [MotorState() for _ in range(self._dof)]
        return [
            MotorState(q=float(q), dq=float(dq), tau=float(tau))
            for q, dq, tau in zip(
                observation.values["position"],
                observation.values["velocity"],
                observation.values["effort"],
                strict=True,
            )
        ]

    def read_imu(self) -> IMUState:
        observation = self._client.observation()
        if observation is None:
            return IMUState()
        return IMUState(
            quaternion=tuple(observation.values["imu_quaternion"].tolist()),
            gyroscope=tuple(observation.values["imu_gyroscope"].tolist()),
            accelerometer=tuple(observation.values["imu_accelerometer"].tolist()),
            rpy=tuple(observation.values["imu_rpy"].tolist()),
        )

    def write_motor_commands(self, commands: list[MotorCommand]) -> bool:
        if len(commands) != self._dof:
            return False
        return self._client.publish(
            {
                "enabled": np.array([1], dtype=np.uint8),
                "position": np.array(
                    [0.0 if command.q == POS_STOP else command.q for command in commands]
                ),
                "velocity": np.array(
                    [0.0 if command.dq == VEL_STOP else command.dq for command in commands]
                ),
                "kp": np.array([command.kp for command in commands]),
                "kd": np.array([command.kd for command in commands]),
                "effort": np.array([command.tau for command in commands]),
            }
        )
