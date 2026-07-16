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

from dimos.sim2.control.client import SimRobotClient
from dimos.sim2.ipc.registry import SimRegistry
from dimos.sim2.spec import ControlInterface


class SimTwistBaseAdapter:
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
        self._enabled = True
        self._client = SimRobotClient(
            sim_id=sim_id,
            robot_id=robot_id or hardware_id,
            expected_interface=ControlInterface.TWIST_BASE,
            dof=dof,
            registry=registry,
        )

    def connect(self) -> bool:
        return self._client.connect()

    def disconnect(self) -> None:
        self._client.disconnect()

    def is_connected(self) -> bool:
        return self._client.is_connected()

    def get_dof(self) -> int:
        return self._dof

    def read_velocities(self) -> list[float]:
        observation = self._client.observation()
        if observation is None:
            return [0.0] * self._dof
        return [float(value) for value in observation.values["velocities"]]

    def read_odometry(self) -> list[float] | None:
        observation = self._client.observation()
        if observation is None:
            return None
        return [float(value) for value in observation.values["odometry"]]

    def write_velocities(self, velocities: list[float]) -> bool:
        if len(velocities) != self._dof:
            return False
        return self._client.publish(
            {
                "enabled": np.array([self._enabled], dtype=np.uint8),
                "velocities": np.asarray(velocities, dtype=np.float64),
            }
        )

    def write_stop(self) -> bool:
        return self.write_velocities([0.0] * self._dof)

    def write_enable(self, enable: bool) -> bool:
        self._enabled = enable
        return self.write_velocities([0.0] * self._dof)

    def read_enabled(self) -> bool:
        observation = self._client.observation()
        if observation is None:
            return False
        return bool(observation.values["enabled"][0])
