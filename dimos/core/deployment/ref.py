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

from dimos.core.deployment.models import DeploymentSpec


def resolve_deployment_ref(ref: str) -> DeploymentSpec:
    if ":" not in ref:
        raise ValueError("Deployment reference must be 'module.path:variable_name'")
    module_name, variable_name = ref.split(":", 1)
    if not module_name or not variable_name or "." in variable_name:
        raise ValueError(
            "Deployment reference must point to a module-level deployment spec variable"
        )
    try:
        module = __import__(module_name, fromlist=[variable_name])
    except Exception as exc:
        raise ValueError(
            f"Could not import deployment reference module {module_name!r}: {exc}"
        ) from exc
    if not hasattr(module, variable_name):
        raise ValueError(f"Deployment reference variable {variable_name!r} was not found")
    obj = getattr(module, variable_name)
    if not isinstance(obj, DeploymentSpec):
        raise ValueError(
            "Deployment reference must point to a module-level DeploymentSpec variable"
        )
    return obj
