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

import inspect
from pathlib import Path
from typing import TYPE_CHECKING

from dimos.core.deployment.models import (
    DeploymentPlan,
    DeploymentSpec,
    ExternalModule,
    ExternalModulePlan,
    LocalPythonPackage,
    ModuleDeployment,
)
from dimos.core.module import ModuleBase

if TYPE_CHECKING:
    from dimos.core.coordination.blueprints import Blueprint


def import_ref_for(obj: type[object]) -> str:
    return f"{obj.__module__}:{obj.__qualname__}"


def plan_deployment(spec: DeploymentSpec) -> DeploymentPlan:
    python_modules: list[type[ModuleBase]] = []
    external_modules: list[ExternalModulePlan] = []
    seen_external: set[type[ExternalModule]] = set()

    for atom in spec.blueprint.active_blueprints:
        if issubclass(atom.module, ExternalModule):
            if atom.module in seen_external:
                raise ValueError(
                    f"Duplicate external declaration instances are not supported yet: "
                    f"{atom.module.__name__}"
                )
            seen_external.add(atom.module)
            policy = _policy_for_external(spec, atom.module)
            package = discover_local_python_package(atom.module, policy)
            external_modules.append(
                ExternalModulePlan(
                    module_class=atom.module,
                    module_id=atom.module.__name__,
                    module_name=atom.module.__name__,
                    rpc_name=atom.module.__name__,
                    package=package,
                    policy=policy,
                    kwargs=dict(atom.kwargs),
                )
            )
        else:
            python_modules.append(atom.module)

    return DeploymentPlan(tuple(python_modules), tuple(external_modules))


def reject_external_modules_without_deployment_spec(blueprint: Blueprint) -> None:
    modules = _external_module_names(blueprint)
    if not modules:
        return
    joined = ", ".join(modules)
    raise ValueError(
        f"Blueprint {getattr(blueprint, 'name', 'blueprint')} contains "
        f"ExternalModule declarations ({joined}). "
        "External modules require ModuleCoordinator.build_deployment(DeploymentSpec(...))."
    )


def _external_module_names(blueprint: Blueprint) -> list[str]:
    return [
        atom.module.__name__
        for atom in blueprint.active_blueprints
        if issubclass(atom.module, ExternalModule)
    ]


def discover_local_python_package(
    declaration: type[ExternalModule], policy: ModuleDeployment
) -> LocalPythonPackage:
    if policy.execution_target != "local":
        raise NotImplementedError("Only local external module execution is implemented in this PR")

    package_root = Path(inspect.getfile(declaration)).resolve().parent
    conventions = _known_conventions(package_root)
    if len(conventions) == 0:
        raise FileNotFoundError(
            f"No supported implementation convention found beside {declaration.__name__} "
            f"at {package_root}. Expected python/pyproject.toml."
        )
    if len(conventions) > 1:
        names = ", ".join(name for name, _path in conventions)
        raise ValueError(
            f"Multiple implementation conventions found beside {declaration.__name__}: {names}"
        )

    convention, _path = conventions[0]
    if convention != "python":
        raise NotImplementedError(
            f"External module convention {convention!r} is not implemented in this PR"
        )

    implementation = declaration.implementation
    if not isinstance(implementation, str) or not implementation:
        raise ValueError(
            f"Python external module {declaration.__name__} must declare "
            "implementation = 'module.path:RuntimeClass'"
        )

    return LocalPythonPackage(
        package_root=package_root,
        declaration=declaration,
        declaration_ref=import_ref_for(declaration),
        implementation_ref=implementation,
        uses_pixi=(package_root / "python" / "pixi.toml").exists(),
        readiness_timeout_s=policy.readiness_timeout_s,
    )


def launch_command_for_package(module: ExternalModulePlan) -> tuple[str, ...]:
    pyproject = module.package.python_dir / "pyproject.toml"
    if not pyproject.exists():
        raise FileNotFoundError(f"Missing required packaged Python project file: {pyproject}")
    if module.package.uses_pixi:
        return ("pixi", "run", "uv", "run", "python")
    return ("uv", "run", "python")


def prepare_command_for_package(module: ExternalModulePlan) -> tuple[str, ...]:
    launch_command = launch_command_for_package(module)
    if launch_command[0] == "pixi":
        return ("pixi", "run", "uv", "sync")
    return ("uv", "sync")


def _policy_for_external(
    spec: DeploymentSpec, declaration: type[ExternalModule]
) -> ModuleDeployment:
    policy = spec.modules.get(declaration)
    if policy is None:
        return ModuleDeployment()
    return policy


def _known_conventions(package_root: Path) -> list[tuple[str, Path]]:
    candidates = [
        ("python", package_root / "python" / "pyproject.toml"),
        ("rust", package_root / "rust" / "Cargo.toml"),
        ("cpp", package_root / "cpp" / "CMakeLists.txt"),
    ]
    return [(name, path) for name, path in candidates if path.exists()]
