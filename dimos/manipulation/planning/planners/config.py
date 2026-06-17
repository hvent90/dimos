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

"""Configuration models for manipulation planner backends."""

from __future__ import annotations

from typing import Annotated, Literal

from pydantic import Field

from dimos.protocol.service.spec import BaseConfig


class RRTConnectPlannerConfig(BaseConfig):
    """Configuration for the backend-agnostic RRT-Connect planner."""

    backend: Literal["rrt_connect"] = "rrt_connect"
    step_size: float = 0.1
    connect_step_size: float = 0.05
    goal_tolerance: float = 0.1
    collision_step_size: float = 0.02


class VampPlannerConfig(BaseConfig):
    """Configuration for the VAMP-native joint-space planner adapter."""

    backend: Literal["vamp"] = "vamp"
    algorithm: Literal["rrtc", "prm", "fcit", "aorrtc"] = "rrtc"
    simplify: bool = True
    validate_path: bool = True


ManipulationPlannerConfig = Annotated[
    RRTConnectPlannerConfig | VampPlannerConfig,
    Field(discriminator="backend"),
]


def planner_config_from_name(name: str) -> ManipulationPlannerConfig:
    """Create a default planner config from a legacy planner name."""
    if name == "rrt_connect":
        return RRTConnectPlannerConfig()
    if name == "vamp":
        return VampPlannerConfig()
    raise ValueError(f"Unknown planner: {name}. Available: ['rrt_connect', 'vamp']")


__all__ = [
    "ManipulationPlannerConfig",
    "RRTConnectPlannerConfig",
    "VampPlannerConfig",
    "planner_config_from_name",
]
