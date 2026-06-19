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
    """Path value returned by VAMP bindings."""

    def numpy(self) -> NDArray[np.float64]:
        """Return path waypoints as an array."""
        ...


VampPathSource = VampPathProtocol | Sequence[Sequence[float]] | NDArray[np.float64]


class VampSphereProtocol(Protocol):
    """Sphere primitive handle accepted by VAMP environments."""


class VampCuboidProtocol(Protocol):
    """Cuboid primitive handle accepted by VAMP environments."""


class VampCylinderProtocol(Protocol):
    """Cylinder primitive handle accepted by VAMP environments."""


class VampPlannerSettingsProtocol(Protocol):
    """Planner settings handle returned by VAMP."""


class VampSimplifySettingsProtocol(Protocol):
    """Simplification settings handle returned by VAMP."""


class VampSamplerProtocol(Protocol):
    """Sampler handle returned by robot-specific VAMP modules."""


class VampPlanningResultProtocol(Protocol):
    """Planning or simplification result returned by VAMP bindings."""

    solved: bool
    path: VampPathSource
    iterations: int


class VampEnvironmentProtocol(Protocol):
    """VAMP collision environment."""

    def add_sphere(self, sphere: VampSphereProtocol) -> None: ...

    def add_cuboid(self, cuboid: VampCuboidProtocol) -> None: ...

    def add_capsule(self, capsule: VampCylinderProtocol) -> None: ...


VampPlannerFunction = Callable[
    [
        Sequence[float],
        Sequence[float],
        VampEnvironmentProtocol,
        VampPlannerSettingsProtocol,
        VampSamplerProtocol,
    ],
    VampPlanningResultProtocol,
]


class VampRobotModuleProtocol(Protocol):
    """Robot-specific VAMP module such as ``vamp.panda``."""

    __name__: str

    def halton(self) -> VampSamplerProtocol: ...

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
        path: VampPathSource,
        environment: VampEnvironmentProtocol,
        settings: VampSimplifySettingsProtocol,
        sampler: VampSamplerProtocol,
    ) -> VampPlanningResultProtocol: ...


class VampModuleProtocol(Protocol):
    """Top-level VAMP module."""

    def Environment(self) -> VampEnvironmentProtocol: ...

    def Sphere(self, center: Sequence[float], radius: float) -> VampSphereProtocol: ...

    def Cuboid(
        self, center: Sequence[float], euler_xyz: Sequence[float], half_extents: Sequence[float]
    ) -> VampCuboidProtocol: ...

    def Cylinder(
        self, center: Sequence[float], euler_xyz: Sequence[float], radius: float, length: float
    ) -> VampCylinderProtocol: ...

    def configure_robot_and_planner_with_kwargs(
        self, robot_name: str, planner_name: str, max_iterations: int
    ) -> tuple[
        VampRobotModuleProtocol,
        VampPlannerFunction,
        VampPlannerSettingsProtocol,
        VampSimplifySettingsProtocol,
    ]: ...
