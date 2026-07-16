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

"""World-oriented simulation backend interface."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Protocol, runtime_checkable

from dimos.sim2.spec import (
    ControlInterface,
    EntityDescriptor,
    SimRobotSpec,
    SpawnPose,
    WorldSpec,
)


@dataclass(frozen=True)
class RobotHandle:
    robot_id: str
    control_interface: ControlInterface
    dof: int
    backend_data: Any = None


@dataclass(frozen=True)
class SensorSample:
    sensor_id: str
    robot_id: str
    frame_id: str
    payload: Any


@runtime_checkable
class SimBackend(Protocol):
    @property
    def capabilities(self) -> frozenset[ControlInterface]: ...

    def load(
        self,
        world: WorldSpec,
        robots: tuple[SimRobotSpec, ...],
        physics_dt: float,
    ) -> dict[str, RobotHandle]: ...

    def reset(self, seed: int | None = None) -> None: ...

    def apply_action(self, handle: RobotHandle, action: dict[str, Any]) -> None: ...

    def step(self, dt: float) -> None: ...

    def observe(self, handle: RobotHandle) -> dict[str, Any]: ...

    def entity_states(self) -> tuple[Any, ...]: ...

    def sensor_samples(self, sim_time: float) -> tuple[SensorSample, ...]: ...

    def close(self) -> None: ...


@runtime_checkable
class RobotPoseBackend(Protocol):
    def set_robot_pose(self, handle: RobotHandle, pose: SpawnPose) -> None: ...


@runtime_checkable
class SceneAuthoringBackend(Protocol):
    def add_wall(
        self,
        x1: float,
        y1: float,
        x2: float,
        y2: float,
        height: float,
        thickness: float,
    ) -> EntityDescriptor: ...
