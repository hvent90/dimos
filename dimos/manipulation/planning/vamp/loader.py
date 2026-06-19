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

"""Lazy loaders for VAMP robot artifacts."""

from __future__ import annotations

import importlib
import importlib.util
from pathlib import Path
import sys
from typing import cast

from dimos.manipulation.planning.vamp.errors import VampDependencyError
from dimos.manipulation.planning.vamp.protocols import VampModuleProtocol, VampRobotModuleProtocol
from dimos.manipulation.planning.world.config import (
    CustomVampArtifactConfig,
    OfficialVampArtifactConfig,
    VampArtifactConfig,
)

try:
    import vamp as _vamp_module
except ImportError:
    _vamp_module = None


def load_vamp_robot_module(
    artifact: VampArtifactConfig,
) -> tuple[VampModuleProtocol, VampRobotModuleProtocol]:
    """Load the VAMP package and configured robot module."""
    if _vamp_module is None:
        raise VampDependencyError()
    vamp_module = cast("VampModuleProtocol", _vamp_module)
    if isinstance(artifact, OfficialVampArtifactConfig):
        return vamp_module, _load_official_robot_module(vamp_module, artifact.robot)
    if isinstance(artifact, CustomVampArtifactConfig):
        return vamp_module, _load_custom_robot_module(artifact.path)
    raise TypeError(f"Unsupported VAMP artifact config: {type(artifact).__name__}")


def _load_official_robot_module(
    vamp_module: VampModuleProtocol, robot: str
) -> VampRobotModuleProtocol:
    del vamp_module
    try:
        imported = importlib.import_module(f"vamp.{robot}")
    except ImportError as exc:
        raise ValueError(
            f"Installed VAMP package does not expose robot artifact '{robot}'"
        ) from exc
    return cast("VampRobotModuleProtocol", imported)


def _load_custom_robot_module(path: Path) -> VampRobotModuleProtocol:
    artifact_path = path.expanduser().resolve()
    if not artifact_path.exists():
        raise FileNotFoundError(f"VAMP custom artifact path does not exist: {artifact_path}")

    if artifact_path.is_dir():
        parent = str(artifact_path.parent)
        if parent not in sys.path:
            sys.path.insert(0, parent)
        return cast("VampRobotModuleProtocol", importlib.import_module(artifact_path.name))

    module_name = artifact_path.stem
    spec = importlib.util.spec_from_file_location(module_name, artifact_path)
    if spec is None or spec.loader is None:
        raise ImportError(f"Could not load VAMP custom artifact module: {artifact_path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return cast("VampRobotModuleProtocol", module)
