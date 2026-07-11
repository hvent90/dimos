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
import os
from pathlib import Path
import shutil
import subprocess
import sys

import pytest

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.coordination.worker_manager_external import WorkerManagerExternal
from dimos.core.core import rpc
from dimos.core.deployment.models import DeploymentSpec, ExternalModule, LocalPythonPackage
from dimos.core.deployment.planner import plan_deployment, prepare_local_package
from dimos.core.deployment.ref import resolve_deployment_ref
from dimos.core.global_config import global_config
from dimos.core.module import Module
from dimos.core.rpc_client import RPCClient


class NormalTestModule(Module):
    pass


class ExternalTestDeclaration(ExternalModule):
    @rpc
    def ping(self) -> str: ...


deployment_spec_for_test = DeploymentSpec(
    ExternalTestDeclaration.blueprint(),
    {ExternalTestDeclaration: LocalPythonPackage(Path("."), ExternalTestDeclaration, "x:Y")},
)


def test_resolve_example_deployment_ref() -> None:
    spec = resolve_deployment_ref("dimos.core.deployment.test_deployment:deployment_spec_for_test")
    assert isinstance(spec, DeploymentSpec)


def test_invalid_deployment_ref_rejected() -> None:
    with pytest.raises(ValueError, match="module-level DeploymentSpec"):
        resolve_deployment_ref("dimos.core.deployment.test_deployment:NormalTestModule")


def test_mixed_planning_does_not_mutate_package(tmp_path: Path) -> None:
    package = tmp_path / "pkg"
    package.mkdir()
    spec = DeploymentSpec(
        autoconnect(NormalTestModule.blueprint(), ExternalTestDeclaration.blueprint()),
        {ExternalTestDeclaration: LocalPythonPackage(package, ExternalTestDeclaration, "x:Y")},
    )
    before = sorted(package.iterdir())
    plan = plan_deployment(spec)
    after = sorted(package.iterdir())
    assert plan.python_modules == (NormalTestModule,)
    assert [env.module_class for env in plan.external_modules] == [ExternalTestDeclaration]
    assert before == after


def test_prepare_command_selection_and_missing_pyproject(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    sync_commands: list[tuple[str, ...]] = []

    def fake_run(
        command: tuple[str, ...], *, cwd: Path, check: bool
    ) -> subprocess.CompletedProcess[tuple[str, ...]]:
        assert cwd == python_dir
        assert check is True
        sync_commands.append(command)
        return subprocess.CompletedProcess(command, 0)

    package = tmp_path / "pkg"
    python_dir = package / "python"
    python_dir.mkdir(parents=True)
    monkeypatch.setattr("dimos.core.deployment.planner.subprocess.run", fake_run)
    spec = DeploymentSpec(
        ExternalTestDeclaration.blueprint(),
        {ExternalTestDeclaration: LocalPythonPackage(package, ExternalTestDeclaration, "x:Y")},
    )
    env = plan_deployment(spec).external_modules[0]
    with pytest.raises(FileNotFoundError, match="pyproject.toml"):
        prepare_local_package(env)
    (python_dir / "pyproject.toml").write_text("[project]\nname='x'\nversion='0'\n")
    assert prepare_local_package(env) == ("uv", "run", "python")
    assert sync_commands[-1] == ("uv", "sync")
    (python_dir / "pixi.toml").write_text("[project]\nname='x'\nchannels=[]\nplatforms=[]\n")
    assert prepare_local_package(env) == ("pixi", "run", "uv", "run", "python")
    assert sync_commands[-1] == ("pixi", "run", "uv", "sync")


def test_external_proxy_declared_rpc_and_undeclared_attr() -> None:
    proxy = RPCClient.remote(ExternalTestDeclaration)
    try:
        assert callable(proxy.ping)
        with pytest.raises(AttributeError, match="non-@rpc attribute access"):
            attr_name = "not_declared"
            getattr(proxy, attr_name)
    finally:
        proxy.stop_rpc_client()


def test_external_readiness_timeout_is_fast(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    class FakeProcess:
        pid = 123456

        def poll(self) -> int | None:
            return None

        def wait(self, timeout: float | None = None) -> int:
            return 0

    class FakeRpc:
        def call_sync(
            self, method: str, payload: tuple[list[object], dict[str, object]], rpc_timeout: float
        ) -> tuple[object, object]:
            raise TimeoutError(method)

    class FakeProxy:
        rpc = FakeRpc()

    package = tmp_path / "pkg"
    python_dir = package / "python"
    python_dir.mkdir(parents=True)
    (python_dir / "pyproject.toml").write_text("[project]\nname='x'\nversion='0'\n")
    metadata = LocalPythonPackage(
        package_root=package,
        declaration=ExternalTestDeclaration,
        runtime_ref="x:Y",
        readiness_timeout_s=0.01,
    )
    ExternalTestDeclaration.__external_metadata__ = metadata
    monkeypatch.setattr(
        "dimos.core.coordination.worker_manager_external.subprocess.Popen",
        lambda *args, **kwargs: FakeProcess(),
    )
    monkeypatch.setattr(
        "dimos.core.coordination.worker_manager_external.RPCClient.remote",
        lambda module_class: FakeProxy(),
    )
    monkeypatch.setattr(
        "dimos.core.coordination.worker_manager_external.prepare_local_package",
        lambda envelope: ("uv", "run", "python"),
    )
    monkeypatch.setattr(
        "dimos.core.coordination.worker_manager_external.os.killpg", lambda pid, sig: None
    )
    manager = WorkerManagerExternal(global_config)
    with pytest.raises(TimeoutError, match="RPC readiness"):
        manager.deploy_parallel([(ExternalTestDeclaration, global_config, {})], {})


def test_mixed_normal_and_external_example_e2e() -> None:
    if shutil.which("uv") is None:
        pytest.skip("uv is required for packaged external module smoke test")
    if shutil.which("pixi") is None:
        pytest.skip("pixi is required for the packaged external module example")
    example_root = Path(__file__).parents[3] / "examples" / "external-python-module"
    sys.path.insert(0, str(example_root))
    old_pythonpath = os.environ.get("PYTHONPATH")
    os.environ["PYTHONPATH"] = os.pathsep.join([str(example_root), old_pythonpath or ""])
    old_deployment_module = sys.modules.pop("deployment", None)
    try:
        deployment_module = __import__(
            "deployment",
            fromlist=["deployment_spec", "ExampleClient", "ExampleExternalDeclaration"],
        )
        coordinator = ModuleCoordinator.build_deployment(deployment_module.deployment_spec)
        try:
            external = coordinator.get_instance(deployment_module.ExampleExternalDeclaration)
            client = coordinator.get_instance(deployment_module.ExampleClient)
            assert external.greet("qa") == "hi, qa from external runtime"
            assert external.greet_with_helper("qa") == "hi, qa from external runtime; helper saw qa"
            assert (
                client.call_external_dependency("qa")
                == "external-only humanize formatted 1,234,567 for qa"
            )
            assert (
                client.roundtrip_stream("stream-qa")
                == "external-only humanize formatted 1,234,567 for stream-qa"
            )
        finally:
            coordinator.stop()
    finally:
        sys.modules.pop("deployment", None)
        if old_deployment_module is not None:
            sys.modules["deployment"] = old_deployment_module
        if old_pythonpath is None:
            os.environ.pop("PYTHONPATH", None)
        else:
            os.environ["PYTHONPATH"] = old_pythonpath
        sys.path.remove(str(example_root))
        shutil.rmtree(example_root / "python" / ".venv", ignore_errors=True)
        shutil.rmtree(example_root / "python" / ".pixi", ignore_errors=True)
        for generated in (
            example_root / "python" / "uv.lock",
            example_root / "python" / "pixi.lock",
        ):
            generated.unlink(missing_ok=True)
        for pycache in example_root.rglob("__pycache__"):
            shutil.rmtree(pycache, ignore_errors=True)
