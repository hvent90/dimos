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

from collections.abc import Mapping, Sequence
import contextlib
import os
from pathlib import Path
import pickle
import signal
import subprocess
import tempfile
import time
from typing import Any

from dimos.core.deployment.models import ExternalModule, LaunchEnvelope, LocalPythonPackage
from dimos.core.deployment.planner import prepare_local_package
from dimos.core.global_config import GlobalConfig
from dimos.core.module import ModuleBase, ModuleSpec
from dimos.core.rpc_client import ModuleProxyProtocol, RPCClient


class WorkerManagerExternal:
    deployment_identifier = "external-python"

    def __init__(self, g: GlobalConfig) -> None:
        self.g = g
        self._processes: list[subprocess.Popen[bytes]] = []
        self._launch_envelopes: list[Path] = []

    def start(self) -> None:
        return None

    def deploy(
        self, module_class: type[ModuleBase], global_config: GlobalConfig, kwargs: dict[str, Any]
    ) -> ModuleProxyProtocol:
        return self.deploy_parallel([(module_class, global_config, kwargs)], {})[0]

    def deploy_parallel(
        self, specs: Sequence[ModuleSpec], blueprint_args: Mapping[str, Mapping[str, Any]]
    ) -> list[ModuleProxyProtocol]:
        proxies: list[ModuleProxyProtocol] = []
        for module_class, _global_config, kwargs in specs:
            if not issubclass(module_class, ExternalModule):
                raise TypeError(f"{module_class.__name__} is not an ExternalModule declaration")
            external_class = module_class
            metadata = getattr(module_class, "__external_metadata__", None)
            if not isinstance(metadata, LocalPythonPackage):
                raise ValueError(f"External module {module_class.__name__} is missing metadata")
            envelope = LaunchEnvelope(external_class, metadata, dict(kwargs))
            prefix = prepare_local_package(envelope)
            envelope_path = self._write_launch_envelope(envelope)
            cmd = [
                *prefix,
                "-m",
                "dimos.core.deployment.runtime",
                "--launch-envelope",
                str(envelope_path),
            ]
            env = os.environ.copy()
            repo_root = Path(__file__).parents[3]
            env["PYTHONPATH"] = os.pathsep.join(
                [str(repo_root), str(metadata.package_root), env.get("PYTHONPATH", "")]
            )
            proc = subprocess.Popen(
                cmd,
                cwd=metadata.python_dir,
                env=env,
                start_new_session=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
            self._processes.append(proc)
            proxy = RPCClient.remote(module_class)
            deadline = time.monotonic() + metadata.readiness_timeout_s
            while True:
                if proc.poll() is not None:
                    stdout, stderr = proc.communicate(timeout=1)
                    raise RuntimeError(
                        f"External module {module_class.__name__} exited during startup. "
                        f"stdout={stdout.decode(errors='replace')!r} "
                        f"stderr={stderr.decode(errors='replace')!r}"
                    )
                try:
                    if proxy.rpc is None:
                        raise RuntimeError("External readiness RPC client is closed")
                    _result, unsubscribe = proxy.rpc.call_sync(
                        f"{module_class.__name__}/dimos_ready", ([], {}), rpc_timeout=0.2
                    )
                    unsubscribe()
                    break
                except Exception as exc:
                    if time.monotonic() >= deadline:
                        self._terminate_process(proc)
                        raise TimeoutError(
                            f"Timed out waiting for external module {module_class.__name__} RPC readiness"
                        ) from exc
                    time.sleep(0.1)
            proxies.append(proxy)  # type: ignore[arg-type]
        return proxies

    def _write_launch_envelope(self, envelope: LaunchEnvelope) -> Path:
        tmp = tempfile.NamedTemporaryFile("wb", prefix="dimos-external-launch-", delete=False)
        with tmp:
            pickle.dump(envelope, tmp)
        path = Path(tmp.name)
        self._launch_envelopes.append(path)
        return path

    def _terminate_process(self, proc: subprocess.Popen[bytes]) -> None:
        if proc.poll() is not None:
            return
        with contextlib.suppress(ProcessLookupError):
            os.killpg(proc.pid, signal.SIGTERM)
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            with contextlib.suppress(ProcessLookupError):
                os.killpg(proc.pid, signal.SIGKILL)
            proc.wait(timeout=5)

    def stop(self) -> None:
        for proc in self._processes:
            self._terminate_process(proc)
        for path in self._launch_envelopes:
            with contextlib.suppress(FileNotFoundError):
                path.unlink()
        self._launch_envelopes = []

    def health_check(self) -> bool:
        return all(proc.poll() is None for proc in self._processes)

    def suppress_console(self) -> None:
        return None
