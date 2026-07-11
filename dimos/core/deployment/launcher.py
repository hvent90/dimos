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

import json
import signal
import time

import typer

from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.coordination.worker_manager_external import prepare_deployment
from dimos.core.deployment.planner import plan_deployment
from dimos.core.deployment.ref import resolve_deployment_ref
from dimos.core.global_config import global_config

app = typer.Typer(help="Temporary local deployment integration launcher")


def _plan_dict(ref: str) -> dict[str, list[str]]:
    spec = resolve_deployment_ref(ref)
    plan = plan_deployment(spec)
    return {
        "python_modules": [cls.__name__ for cls in plan.python_modules],
        "external_modules": [env.module_class.__name__ for env in plan.external_modules],
        "external_worker_modules": [env.module_id for env in plan.external_modules],
    }


@app.command()
def plan(reference: str) -> None:
    print(json.dumps(_plan_dict(reference), indent=2))


@app.command()
def prepare(reference: str) -> None:
    spec = resolve_deployment_ref(reference)
    deployment_plan = plan_deployment(spec)
    prepared = prepare_deployment(deployment_plan, global_config)
    print(json.dumps({"prepared_external_modules": prepared}, indent=2))


@app.command()
def run(reference: str) -> None:
    spec = resolve_deployment_ref(reference)
    coordinator = ModuleCoordinator.build_deployment(spec)
    stop_requested = False

    def _request_stop(_signum: int, _frame: object) -> None:
        nonlocal stop_requested
        stop_requested = True

    previous_sigterm = signal.signal(signal.SIGTERM, _request_stop)
    previous_sigint = signal.signal(signal.SIGINT, _request_stop)
    try:
        print(json.dumps(_plan_dict(reference), indent=2))
        while not stop_requested:
            time.sleep(0.2)
    finally:
        signal.signal(signal.SIGTERM, previous_sigterm)
        signal.signal(signal.SIGINT, previous_sigint)
        coordinator.stop()


def main() -> None:
    app()


if __name__ == "__main__":
    main()
