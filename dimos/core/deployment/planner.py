# Copyright 2026 Dimensional Inc.
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

import subprocess

from dimos.core.deployment.models import (
    DeploymentPlan,
    DeploymentSpec,
    ExternalModule,
    LaunchEnvelope,
)


def plan_deployment(spec: DeploymentSpec) -> DeploymentPlan:
    python_modules = []
    external_modules = []
    for atom in spec.blueprint.active_blueprints:
        if issubclass(atom.module, ExternalModule):
            metadata = spec.external.get(atom.module)
            if metadata is None:
                raise ValueError(
                    f"External module {atom.module.__name__} is missing deployment metadata"
                )
            external_modules.append(LaunchEnvelope(atom.module, metadata, dict(atom.kwargs)))
        else:
            python_modules.append(atom.module)
    return DeploymentPlan(tuple(python_modules), tuple(external_modules))


def launch_command_for_package(envelope: LaunchEnvelope) -> tuple[str, ...]:
    python_dir = envelope.metadata.python_dir
    pyproject = python_dir / "pyproject.toml"
    if not pyproject.exists():
        raise FileNotFoundError(f"Missing required packaged Python project file: {pyproject}")
    if (python_dir / "pixi.toml").exists():
        return ("pixi", "run", "uv", "run", "python")
    return ("uv", "run", "python")


def prepare_local_package(envelope: LaunchEnvelope) -> tuple[str, ...]:
    python_dir = envelope.metadata.python_dir
    launch_command = launch_command_for_package(envelope)
    sync_command = ("pixi", "run", "uv", "sync") if launch_command[0] == "pixi" else ("uv", "sync")
    subprocess.run(sync_command, cwd=python_dir, check=True)
    return launch_command
