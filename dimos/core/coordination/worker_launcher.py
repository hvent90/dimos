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

from abc import ABC, abstractmethod
from multiprocessing.connection import Connection, Listener
from multiprocessing.process import BaseProcess
import os
from pathlib import Path
import queue
import secrets
import signal
import subprocess
import tempfile
import threading

from dimos.core.runtime_environment import PythonProjectLaunchMaterial


class WorkerLaunchError(RuntimeError):
    pass


class WorkerProcessHandle(ABC):
    connection: Connection
    supports_parent_actor_ref: bool = True

    @property
    @abstractmethod
    def pid(self) -> int | None: ...

    @abstractmethod
    def join(self, timeout: float | None = None) -> None: ...

    @abstractmethod
    def is_alive(self) -> bool: ...

    @abstractmethod
    def terminate(self) -> None: ...


class WorkerLauncher(ABC):
    @abstractmethod
    def launch(self, worker_id: int) -> WorkerProcessHandle: ...


class ForkserverWorkerProcessHandle(WorkerProcessHandle):
    def __init__(self, process: BaseProcess, connection: Connection) -> None:
        self._process = process
        self.connection = connection

    @property
    def pid(self) -> int | None:
        return self._process.pid

    def join(self, timeout: float | None = None) -> None:
        self._process.join(timeout=timeout)

    def is_alive(self) -> bool:
        return self._process.is_alive()

    def terminate(self) -> None:
        self._process.terminate()


class ForkserverWorkerLauncher(WorkerLauncher):
    def launch(self, worker_id: int) -> WorkerProcessHandle:
        # Imported lazily to avoid the python_worker <-> worker_launcher import cycle.
        from dimos.core.coordination.python_worker import get_forkserver_context, worker_entrypoint

        ctx = get_forkserver_context()
        parent_conn, child_conn = ctx.Pipe()
        process = ctx.Process(target=worker_entrypoint, args=(child_conn, worker_id), daemon=True)
        process.start()
        return ForkserverWorkerProcessHandle(process, parent_conn)


class SubprocessWorkerProcessHandle(WorkerProcessHandle):
    supports_parent_actor_ref = False

    def __init__(
        self,
        process: subprocess.Popen[bytes],
        connection: Connection,
        *,
        terminate_process_group: bool = False,
    ) -> None:
        self._process = process
        self.connection = connection
        self._terminate_process_group = terminate_process_group

    @property
    def pid(self) -> int | None:
        return self._process.pid

    def join(self, timeout: float | None = None) -> None:
        try:
            self._process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            return

    def is_alive(self) -> bool:
        return self._process.poll() is None

    def terminate(self) -> None:
        if self._terminate_process_group and self._process.pid is not None:
            _terminate_process_group(self._process.pid)
            return
        self._process.terminate()


class CommandWorkerLauncher(WorkerLauncher):
    def __init__(
        self,
        material: PythonProjectLaunchMaterial,
        *,
        startup_timeout: float = 10.0,
    ) -> None:
        self._material = material
        self._startup_timeout = startup_timeout

    def launch(self, worker_id: int) -> WorkerProcessHandle:
        return _launch_subprocess_worker(
            argv=(
                *self._material.argv_prefix,
                "-m",
                "dimos.core.coordination.venv_worker_entrypoint",
            ),
            env=dict(self._material.env),
            cwd=self._material.cwd,
            worker_id=worker_id,
            runtime_name=self._material.runtime_name,
            startup_timeout=self._startup_timeout,
            terminate_process_group=True,
        )


def _launch_subprocess_worker(
    *,
    argv: tuple[str, ...],
    env: dict[str, str],
    cwd: Path | None,
    worker_id: int,
    runtime_name: str,
    startup_timeout: float,
    terminate_process_group: bool,
) -> WorkerProcessHandle:
    with tempfile.TemporaryDirectory(prefix="dimos-runtime-worker-") as tmpdir:
        address = str(Path(tmpdir) / "worker.sock")
        authkey = secrets.token_bytes(32)
        listener = Listener(address, family="AF_UNIX", authkey=authkey)
        process_env = {**os.environ, **env}
        full_argv = (
            *argv,
            "--address",
            address,
            "--authkey-hex",
            authkey.hex(),
            "--worker-id",
            str(worker_id),
        )
        process: subprocess.Popen[bytes] | None = None
        try:
            process = subprocess.Popen(
                full_argv,
                cwd=cwd,
                env=process_env,
                start_new_session=terminate_process_group,
            )
            connection = _accept_worker_connection(listener, startup_timeout)
            if connection is None:
                _terminate_subprocess(process, terminate_process_group=terminate_process_group)
                raise WorkerLaunchError(
                    f"Runtime {runtime_name!r} worker did not connect within {startup_timeout}s"
                )
            return SubprocessWorkerProcessHandle(
                process,
                connection,
                terminate_process_group=terminate_process_group,
            )
        except Exception:
            if process is not None and process.poll() is None:
                _terminate_subprocess(process, terminate_process_group=terminate_process_group)
            raise
        finally:
            listener.close()


def _accept_worker_connection(listener: Listener, timeout: float) -> Connection | None:
    results: queue.Queue[Connection | Exception] = queue.Queue(maxsize=1)

    def _accept() -> None:
        try:
            results.put(listener.accept())
        except Exception as error:
            results.put(error)

    threading.Thread(target=_accept, daemon=True).start()
    try:
        result = results.get(timeout=timeout)
    except queue.Empty:
        return None
    if isinstance(result, Exception):
        raise result
    return result


def _terminate_subprocess(
    process: subprocess.Popen[bytes],
    *,
    terminate_process_group: bool,
    timeout: float = 2.0,
) -> None:
    if process.poll() is not None:
        return
    if terminate_process_group and process.pid is not None:
        _signal_process_group(process.pid, signal.SIGTERM)
    else:
        process.terminate()
    try:
        process.wait(timeout=timeout)
        return
    except subprocess.TimeoutExpired:
        pass

    if terminate_process_group and process.pid is not None:
        _signal_process_group(process.pid, signal.SIGKILL)
    else:
        process.kill()
    try:
        process.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        logger_pid = process.pid
        raise WorkerLaunchError(f"Worker subprocess {logger_pid} did not exit after SIGKILL")


def _terminate_process_group(pid: int) -> None:
    _signal_process_group(pid, signal.SIGTERM)


def _signal_process_group(pid: int, sig: signal.Signals) -> None:
    try:
        os.killpg(pid, sig)
    except ProcessLookupError:
        return
