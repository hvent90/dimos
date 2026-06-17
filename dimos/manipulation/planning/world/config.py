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

"""Configuration models for manipulation world backends."""

from __future__ import annotations

from pathlib import Path
from typing import Annotated, Literal

from pydantic import Field

from dimos.protocol.service.spec import BaseConfig


class DrakeWorldConfig(BaseConfig):
    """Configuration for the default Drake world backend."""

    backend: Literal["drake"] = "drake"


class OfficialVampArtifactConfig(BaseConfig):
    """Load a robot artifact exposed by the installed VAMP package."""

    mode: Literal["official"] = "official"
    robot: str = "panda"


class CustomVampArtifactConfig(BaseConfig):
    """Load a user-prepared VAMP robot artifact from an explicit local path."""

    mode: Literal["custom"] = "custom"
    path: Path


VampArtifactConfig = Annotated[
    OfficialVampArtifactConfig | CustomVampArtifactConfig,
    Field(discriminator="mode"),
]


class VampWorldConfig(BaseConfig):
    """Configuration for the VAMP-native world backend."""

    backend: Literal["vamp"] = "vamp"
    artifact: VampArtifactConfig = Field(default_factory=OfficialVampArtifactConfig)


ManipulationWorldConfig = Annotated[
    DrakeWorldConfig | VampWorldConfig,
    Field(discriminator="backend"),
]


def world_config_from_name(name: str) -> ManipulationWorldConfig:
    """Create a default world config from a backend name."""
    if name == "drake":
        return DrakeWorldConfig()
    if name == "vamp":
        return VampWorldConfig()
    raise ValueError(f"Unknown backend: {name}. Available: ['drake', 'vamp']")


__all__ = [
    "CustomVampArtifactConfig",
    "DrakeWorldConfig",
    "ManipulationWorldConfig",
    "OfficialVampArtifactConfig",
    "VampArtifactConfig",
    "VampWorldConfig",
    "world_config_from_name",
]
