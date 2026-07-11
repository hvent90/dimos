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

from dataclasses import dataclass, field
from pathlib import Path
from typing import ClassVar, NewType

from dimos.core.coordination.blueprints import Blueprint
from dimos.core.core import rpc
from dimos.core.module import Deployment, ModuleBase
from dimos.core.stream import In, Out, Transport

RuntimeSessionId = NewType("RuntimeSessionId", str)


class ExternalModule(ModuleBase):
    """Coordinator-visible declaration for a packaged external Module.

    Subclasses declare streams, config type, @rpc/@skill methods, and module refs.
    The coordinator imports this declaration only; the packaged runtime class
    subclasses it plus the real Module implementation surface.
    """

    deployment: ClassVar[Deployment] = "external-python"
    __external_metadata__: ClassVar[LocalPythonPackage | None] = None

    @rpc
    def dimos_ready(self) -> str:
        """Side-effect-free readiness endpoint for external process startup."""
        return "ready"

    @rpc
    def set_transport(self, stream_name: str, transport: Transport[object]) -> bool:
        """Attach a coordinator-selected transport to a declared stream.

        External runtime classes commonly inherit the declaration before
        `Module`. Providing the real implementation here prevents the
        declaration method from shadowing `Module.set_transport`.
        """
        stream = getattr(self, stream_name, None)
        if not stream:
            raise ValueError(f"{stream_name} not found in {self.__class__.__name__}")

        if not isinstance(stream, Out) and not isinstance(stream, In):
            raise TypeError(f"Output {stream_name} is not a valid stream")

        stream._transport = transport
        return True


@dataclass(frozen=True)
class LocalPythonPackage:
    package_root: Path
    declaration: type[ExternalModule]
    runtime_ref: str
    readiness_timeout_s: float = 10.0

    @property
    def python_dir(self) -> Path:
        return self.package_root / "python"


@dataclass(frozen=True)
class LaunchEnvelope:
    module_class: type[ExternalModule]
    metadata: LocalPythonPackage
    kwargs: dict[str, object] = field(default_factory=dict)


@dataclass(frozen=True)
class DeploymentPlan:
    python_modules: tuple[type[ModuleBase], ...]
    external_modules: tuple[LaunchEnvelope, ...]


@dataclass(frozen=True)
class PrepareResult:
    envelope: LaunchEnvelope
    command_prefix: tuple[str, ...]


@dataclass(frozen=True)
class DeploymentSpec:
    blueprint: Blueprint
    external: dict[type[ExternalModule], LocalPythonPackage] = field(default_factory=dict)

    def __post_init__(self) -> None:
        for declaration, metadata in self.external.items():
            if metadata.declaration is not declaration:
                raise ValueError("LocalPythonPackage declaration must match metadata key")
            declaration.deployment = "external-python"
            declaration.__external_metadata__ = metadata
