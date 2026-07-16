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

"""Backend-blind client for one sim2 robot channel."""

from __future__ import annotations

import threading
import time
from typing import Any

from dimos.sim2.ipc.channel import ChannelFrame, FrameMetadata, RobotChannel
from dimos.sim2.ipc.registry import SimRegistry
from dimos.sim2.spec import ControlInterface


class SimRobotClient:
    def __init__(
        self,
        *,
        sim_id: str,
        robot_id: str,
        expected_interface: ControlInterface,
        dof: int,
        connect_timeout: float = 30.0,
        registry: SimRegistry | None = None,
    ) -> None:
        self.sim_id = sim_id
        self.robot_id = robot_id
        self.expected_interface = expected_interface
        self.dof = dof
        self.connect_timeout = connect_timeout
        self.registry = registry or SimRegistry()
        self.channel: RobotChannel | None = None
        self._connected = False
        self._wait_event = threading.Event()

    def connect(self) -> bool:
        deadline = time.monotonic() + self.connect_timeout
        while time.monotonic() < deadline:
            try:
                descriptor = self.registry.resolve(self.sim_id, self.robot_id)
                if descriptor.control_interface != self.expected_interface:
                    raise ValueError(
                        f"robot '{self.robot_id}' exposes {descriptor.control_interface.value}, "
                        f"expected {self.expected_interface.value}"
                    )
                if descriptor.dof != self.dof:
                    raise ValueError(
                        f"robot '{self.robot_id}' exposes {descriptor.dof} DOF, expected {self.dof}"
                    )
                channel = RobotChannel.attach(descriptor)
                if channel.lifecycle == "ready" and channel.read_observation() is not None:
                    self.channel = channel
                    self._connected = True
                    return True
                channel.close()
            except FileNotFoundError:
                pass
            self._wait_event.wait(min(0.05, max(0.0, deadline - time.monotonic())))
        return False

    @property
    def capabilities(self) -> frozenset[str]:
        if self.channel is None:
            return frozenset()
        return frozenset(self.channel.descriptor.capabilities)

    def disconnect(self) -> None:
        if self.channel is not None:
            self.channel.close()
        self.channel = None
        self._connected = False

    def is_connected(self) -> bool:
        return self._connected and self.channel is not None and self.channel.lifecycle == "ready"

    def observation(self) -> ChannelFrame | None:
        if not self.is_connected() or self.channel is None:
            return None
        return self.channel.read_observation()

    def publish(self, values: dict[str, Any]) -> bool:
        if not self.is_connected() or self.channel is None:
            return False
        observation = self.channel.read_observation()
        if observation is None:
            return False
        self.channel.publish_action(
            values,
            FrameMetadata(
                sequence=0,
                episode_id=observation.metadata.episode_id,
                physics_tick=observation.metadata.physics_tick,
                control_tick=observation.metadata.control_tick,
                sim_time=observation.metadata.sim_time,
            ),
        )
        return True
