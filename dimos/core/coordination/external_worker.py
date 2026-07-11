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

import contextlib
from dataclasses import dataclass, field
import json
from multiprocessing.connection import Connection
from multiprocessing.process import BaseProcess
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import threading
import time
import traceback

from dimos.core.coordination.python_worker import get_forkserver_context
from dimos.core.deployment.models import ExternalModule, JsonValue, ModuleLaunchEnvelope
from dimos.core.library_config import apply_library_config
from dimos.core.rpc_client import RPCClient
from dimos.utils.logging_config import setup_logger
from dimos.utils.sequential_ids import SequentialIds

logger = setup_logger()


@dataclass(frozen=True)
class LaunchRuntimeRequest:
    envelope: dict[str, JsonValue]
    command_prefix: tuple[str, ...]
    environment: dict[str, str]


@dataclass(frozen=True)
class StopRuntimeRequest:
    module_id: str


@dataclass(frozen=True)
class StatusRequest:
    pass


@dataclass(frozen=True)
class ShutdownExternalWorkerRequest:
    pass


@dataclass(frozen=True)
class ExternalWorkerResponse:
    result: JsonValue = None
    error: str | None = None


ExternalWorkerRequest = (
    LaunchRuntimeRequest | StopRuntimeRequest | StatusRequest | ShutdownExternalWorkerRequest
)


@dataclass
class _RuntimeHandle:
    process: subprocess.Popen[bytes]
    envelope_path: Path


@dataclass
class _ExternalWorkerState:
    worker_id: int
    handles: dict[str, _RuntimeHandle] = field(default_factory=dict)
    should_stop: bool = False


_external_worker_ids = SequentialIds()


class ExternalWorkerClient:
    def __init__(self) -> None:
        self._worker_id = _external_worker_ids.next()
        self._lock = threading.Lock()
        self._conn: Connection | None = None
        self._process: BaseProcess | None = None

    def start_process(self) -> None:
        ctx = get_forkserver_context()
        parent_conn, child_conn = ctx.Pipe()
        self._conn = parent_conn
        process = ctx.Process(
            target=_external_worker_entrypoint,
            args=(child_conn, self._worker_id),
            daemon=True,
        )
        process.start()
        self._process = process

    def launch_runtime(
        self,
        envelope: ModuleLaunchEnvelope | dict[str, JsonValue],
        command_prefix: tuple[str, ...],
        environment: dict[str, str],
    ) -> None:
        envelope_data = (
            envelope.to_json() if isinstance(envelope, ModuleLaunchEnvelope) else envelope
        )
        self._send(
            LaunchRuntimeRequest(
                envelope=envelope_data,
                command_prefix=command_prefix,
                environment=environment,
            )
        )

    def stop_runtime(self, module_id: str) -> None:
        self._send(StopRuntimeRequest(module_id=module_id))

    def status(self) -> dict[str, JsonValue]:
        result = self._send(StatusRequest())
        if not isinstance(result, dict):
            raise RuntimeError("ExternalWorker status response was not an object")
        return result

    def shutdown(self) -> None:
        if self._conn is not None:
            try:
                self._send(ShutdownExternalWorkerRequest(), timeout_s=5.0)
            except (BrokenPipeError, EOFError, ConnectionResetError, RuntimeError):
                pass
            finally:
                self._conn.close()
                self._conn = None

        process = self._process
        if process is not None:
            process.join(timeout=5)
            if process.is_alive():
                process.terminate()
                process.join(timeout=1)
        self._process = None

    def _send(self, request: ExternalWorkerRequest, timeout_s: float | None = None) -> JsonValue:
        if self._conn is None:
            raise RuntimeError("ExternalWorker process not started")
        with self._lock:
            self._conn.send(request)
            if timeout_s is not None and not self._conn.poll(timeout=timeout_s):
                raise RuntimeError("ExternalWorker did not respond before timeout")
            response: ExternalWorkerResponse = self._conn.recv()
        if response.error is not None:
            raise RuntimeError(response.error)
        return response.result


def _external_worker_entrypoint(conn: Connection, worker_id: int) -> None:
    apply_library_config()
    state = _ExternalWorkerState(worker_id=worker_id)

    def _shutdown_from_signal(_signum: int, _frame: object) -> None:
        for module_id in list(state.handles):
            _stop_runtime(state, module_id)
        state.should_stop = True
        raise SystemExit(0)

    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, _shutdown_from_signal)
    try:
        _external_worker_loop(conn, state)
    except Exception as exc:
        traceback.print_exception(exc, file=sys.stderr)
        sys.stderr.flush()
        logger.error(
            "External worker process error",
            worker_id=worker_id,
            error_type=type(exc).__name__,
            error_repr=repr(exc),
            exc_info=True,
        )
    finally:
        for module_id in list(state.handles):
            _stop_runtime(state, module_id)


def _external_worker_loop(conn: Connection, state: _ExternalWorkerState) -> None:
    while True:
        try:
            if not conn.poll(timeout=0.1):
                continue
            request: ExternalWorkerRequest = conn.recv()
        except EOFError:
            break

        try:
            response = _handle_external_request(request, state)
        except Exception as exc:
            response = ExternalWorkerResponse(
                error=f"{exc.__class__.__name__}: {exc}\n{traceback.format_exc()}"
            )

        try:
            conn.send(response)
        except (BrokenPipeError, EOFError):
            break

        if state.should_stop:
            break


def _handle_external_request(
    request: ExternalWorkerRequest, state: _ExternalWorkerState
) -> ExternalWorkerResponse:
    match request:
        case LaunchRuntimeRequest(
            envelope=envelope_data, command_prefix=command_prefix, environment=env
        ):
            envelope = ModuleLaunchEnvelope.model_validate(envelope_data)
            _launch_runtime(state, envelope, command_prefix, env)
            return ExternalWorkerResponse(result=True)
        case StopRuntimeRequest(module_id=module_id):
            _stop_runtime(state, module_id)
            return ExternalWorkerResponse(result=True)
        case StatusRequest():
            return ExternalWorkerResponse(
                result={
                    "worker_id": state.worker_id,
                    "module_ids": list(state.handles),
                }
            )
        case ShutdownExternalWorkerRequest():
            for module_id in list(state.handles):
                _stop_runtime(state, module_id)
            state.should_stop = True
            return ExternalWorkerResponse(result=True)


def _launch_runtime(
    state: _ExternalWorkerState,
    envelope: ModuleLaunchEnvelope,
    command_prefix: tuple[str, ...],
    env: dict[str, str],
) -> None:
    if envelope.module_id in state.handles:
        raise RuntimeError(f"Runtime handle already exists for {envelope.module_id}")
    envelope_path = _write_envelope(envelope)
    cmd = [
        *command_prefix,
        "-m",
        "dimos.core.deployment.runtime",
        "--launch-envelope-json",
        str(envelope_path),
    ]
    process = subprocess.Popen(
        cmd,
        cwd=envelope.runtime_workdir,
        env=env,
        start_new_session=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    state.handles[envelope.module_id] = _RuntimeHandle(process=process, envelope_path=envelope_path)
    logger.info("External worker launched runtime", module_id=envelope.module_id, pid=process.pid)
    try:
        _wait_for_readiness(process, envelope)
    except Exception:
        _stop_runtime(state, envelope.module_id)
        raise


def _wait_for_readiness(proc: subprocess.Popen[bytes], envelope: ModuleLaunchEnvelope) -> None:
    declaration_class = _resolve_class(envelope.declaration_ref)
    if not issubclass(declaration_class, ExternalModule):
        raise TypeError(f"{envelope.declaration_ref} is not an ExternalModule declaration")
    proxy = RPCClient.remote(declaration_class)
    deadline = time.monotonic() + envelope.readiness_timeout_s
    try:
        while True:
            if proc.poll() is not None:
                stdout, stderr = proc.communicate(timeout=1)
                raise RuntimeError(
                    f"External module {envelope.module_name} exited during startup. "
                    f"stdout={stdout.decode(errors='replace')!r} "
                    f"stderr={stderr.decode(errors='replace')!r}"
                )
            try:
                if proxy.rpc is None:
                    raise RuntimeError("External readiness RPC client is closed")
                _result, unsubscribe = proxy.rpc.call_sync(
                    f"{envelope.rpc_name}/{envelope.readiness_method}", ([], {}), rpc_timeout=0.2
                )
                unsubscribe()
                return
            except Exception as exc:
                if time.monotonic() >= deadline:
                    timeout_stdout, timeout_stderr = _terminate_and_collect_output(proc)
                    raise TimeoutError(
                        f"Timed out waiting for external module {envelope.module_name} RPC readiness. "
                        f"stdout={timeout_stdout!r} stderr={timeout_stderr!r}"
                    ) from exc
                time.sleep(0.1)
    finally:
        proxy.stop_rpc_client()


def _stop_runtime(state: _ExternalWorkerState, module_id: str) -> None:
    handle = state.handles.pop(module_id, None)
    if handle is None:
        return
    _terminate_process(handle.process)
    with contextlib.suppress(FileNotFoundError):
        handle.envelope_path.unlink()
    logger.info("External worker stopped runtime", module_id=module_id)


def _terminate_process(proc: subprocess.Popen[bytes]) -> None:
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


def _terminate_and_collect_output(proc: subprocess.Popen[bytes]) -> tuple[str, str]:
    _terminate_process(proc)
    try:
        stdout, stderr = proc.communicate(timeout=1)
    except subprocess.TimeoutExpired:
        return "", ""
    return stdout.decode(errors="replace"), stderr.decode(errors="replace")


def _write_envelope(envelope: ModuleLaunchEnvelope) -> Path:
    tmp = tempfile.NamedTemporaryFile(
        "w", prefix="dimos-external-launch-", suffix=".json", delete=False
    )
    with tmp:
        json.dump(envelope.model_dump(mode="json"), tmp)
    return Path(tmp.name)


def _resolve_class(ref: str) -> type[object]:
    module_name, name = ref.split(":", 1)
    module = __import__(module_name, fromlist=[name])
    resolved = getattr(module, name)
    if not isinstance(resolved, type):
        raise TypeError(f"{ref} did not resolve to a class")
    return resolved
