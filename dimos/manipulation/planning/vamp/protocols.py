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

"""Protocols for the optional VAMP Python bindings."""

from __future__ import annotations

from collections.abc import Callable, Sequence
from typing import Protocol

import numpy as np
from numpy.typing import NDArray


class VampPathProtocol(Protocol):
    """Path object returned by VAMP bindings."""

    def numpy(self) -> NDArray[np.float64]:
        """Return path waypoints as an array."""
        ...


class VampPlanningResultProtocol(Protocol):
    """Planning or simplification result returned by VAMP bindings."""

    solved: bool
    path: object
    iterations: int


class VampEnvironmentProtocol(Protocol):
    """VAMP collision environment."""

    def add_sphere(self, sphere: object) -> None: ...

    def add_cuboid(self, cuboid: object) -> None: ...

    def add_capsule(self, capsule: object) -> None: ...


VampPlannerFunction = Callable[
    [Sequence[float], Sequence[float], VampEnvironmentProtocol, object, object],
    VampPlanningResultProtocol,
]


class VampRobotModuleProtocol(Protocol):
    """Robot-specific VAMP module such as ``vamp.panda``."""

    __name__: str

    def halton(self) -> object: ...

    def validate(
        self,
        configuration: Sequence[float],
        environment: VampEnvironmentProtocol,
        check_bounds: bool,
    ) -> bool: ...

    def validate_motion(
        self,
        configuration_in: Sequence[float],
        configuration_out: Sequence[float],
        environment: VampEnvironmentProtocol,
        check_bounds: bool,
    ) -> bool: ...

    def eefk(self, configuration: Sequence[float]) -> NDArray[np.float64]: ...

    def simplify(
        self,
        path: object,
        environment: VampEnvironmentProtocol,
        settings: object,
        sampler: object,
    ) -> VampPlanningResultProtocol: ...


class VampModuleProtocol(Protocol):
    """Top-level VAMP module."""

    def Environment(self) -> VampEnvironmentProtocol: ...

    def Sphere(self, center: Sequence[float], radius: float) -> object: ...

    def Cuboid(
        self, center: Sequence[float], euler_xyz: Sequence[float], half_extents: Sequence[float]
    ) -> object: ...

    def Cylinder(
        self, center: Sequence[float], euler_xyz: Sequence[float], radius: float, length: float
    ) -> object: ...

    def configure_robot_and_planner_with_kwargs(
        self, robot_name: str, planner_name: str, max_iterations: int
    ) -> tuple[VampRobotModuleProtocol, VampPlannerFunction, object, object]: ...
