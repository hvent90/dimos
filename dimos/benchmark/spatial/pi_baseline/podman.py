# Copyright 2026 Dimensional Inc.
# Licensed under the Apache License, Version 2.0 (the "License").

"""Small rootless Podman adapter for the PI baseline with policy-only egress."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import re
import subprocess
from threading import Event
import time

from .scheduler_executor import ExecutionInterrupted
from .topology import PinnedRuntimeTopology


class PodmanSecurityError(RuntimeError):
    """Raised when the local Podman security preconditions are not met."""


class ContainerCleanupError(RuntimeError):
    """Raised when a container cannot be removed and verified safely."""

    reason = "container_cleanup_failed"


_DIGEST = re.compile(r"^[a-z0-9][a-z0-9./_-]*@sha256:[0-9a-f]{64}$")
_IDENTIFIER = re.compile(r"^[a-z0-9][a-z0-9_.-]{0,62}$")
_CLEANUP_TIMEOUT_SECONDS = 30.0
_CLEANUP_COMMAND_TIMEOUT_SECONDS = 10.0


@dataclass(frozen=True)
class PodmanLimits:
    """Hard limits applied to one baseline invocation."""

    timeout_seconds: float = 300.0
    memory: str = "4g"
    cpus: str = "2"
    pids: int = 512
    output_bytes: int = 1_048_576


@dataclass(frozen=True)
class PodmanRun:
    """Inputs needed to construct a constrained baseline container invocation."""

    image: str
    run_id: str
    topology: PinnedRuntimeTopology
    args: tuple[str, ...] = ()
    limits: PodmanLimits = PodmanLimits()

    @property
    def input_dir(self) -> Path:
        return self.topology.input.path

    @property
    def workspace_dir(self) -> Path:
        return self.topology.workspace.path

    @property
    def pass_fds(self) -> tuple[int, ...]:
        return self.topology.fds


class RootlessPodman:
    """Run constrained one-shot or persistent rootless Podman cases."""

    def __init__(self, executable: str = "podman") -> None:
        self.executable = executable

    def is_rootless(self, cancel_requested: Event) -> bool:
        result = _run_command(
            [self.executable, "info", "--format", "{{.Host.Security.Rootless}}"],
            check=True,
            timeout=10.0,
            cancel_requested=cancel_requested,
        )
        return result.stdout.strip().lower() == "true"

    def command(self, request: PodmanRun) -> list[str]:
        self._validate(request)
        request.topology.verify()
        name = f"pi-baseline-{request.run_id}"
        limits = request.limits
        return [
            self.executable,
            "run",
            "--rm",
            "--name",
            name,
            "--read-only",
            "--userns=keep-id",
            "--ipc=private",
            "--uts=private",
            "--pid=private",
            "--cap-drop=ALL",
            "--security-opt=no-new-privileges",
            f"--memory={limits.memory}",
            f"--cpus={limits.cpus}",
            f"--pids-limit={limits.pids}",
            "--tmpfs",
            "/tmp:rw,size=64m,mode=1777",
            "--volume",
            f"/proc/self/fd/{request.topology.input.fd}:/input:ro",
            "--volume",
            f"/proc/self/fd/{request.topology.workspace.fd}:/work:rw",
            request.image,
            *request.args,
        ]

    def persistent(
        self, request: PodmanRun, cancel_requested: Event
    ) -> PersistentPodmanCase:
        """Return the preferred per-case persistent container lifecycle."""

        self._validate(request)
        return PersistentPodmanCase(self, request, cancel_requested)

    def run(self, request: PodmanRun, cancel_requested: Event) -> subprocess.CompletedProcess[str]:
        if not self.is_rootless(cancel_requested):
            raise PodmanSecurityError("Podman did not report rootless operation")
        command = self.command(request)
        name = command[command.index("--name") + 1]
        try:
            result = _run_command(
                command,
                check=True,
                timeout=request.limits.timeout_seconds,
                pass_fds=request.pass_fds,
                cancel_requested=cancel_requested,
            )
            return self._bounded(result, request.limits.output_bytes)
        finally:
            # --rm is not sufficient for interrupted or failed runtime setup.
            subprocess.run(
                [self.executable, "rm", "--force", name],
                check=False,
                capture_output=True,
                text=True,
                timeout=10.0,
            )

    def verify_removed(self, run_id: str, timeout_seconds: float = 10.0) -> bool:
        """Return whether the named case no longer exists.

        A non-zero exit status is Podman's documented result for a missing
        container.  Other failures are surfaced so callers do not mistake an
        unavailable Podman service for successful cleanup.
        """

        name = f"pi-baseline-{run_id}"
        result = subprocess.run(
            [self.executable, "container", "exists", name],
            check=False,
            capture_output=True,
            text=True,
            timeout=timeout_seconds,
        )
        if result.returncode == 0:
            return False
        if result.returncode == 1:
            return True
        raise RuntimeError(
            f"could not verify removal of container {name!r} "
            f"(podman container exists exited {result.returncode})"
        )

    @staticmethod
    def _bounded(
        result: subprocess.CompletedProcess[str], limit: int
    ) -> subprocess.CompletedProcess[str]:
        return subprocess.CompletedProcess(
            result.args,
            result.returncode,
            result.stdout[:limit],
            result.stderr[:limit],
        )

    @staticmethod
    def _validate(request: PodmanRun) -> None:
        if not _DIGEST.fullmatch(request.image):
            raise PodmanSecurityError("image must be pinned to an immutable sha256 digest")
        if not _IDENTIFIER.fullmatch(request.run_id):
            raise PodmanSecurityError("run_id must be a unique safe container identifier")
        try:
            request.topology.verify()
        except Exception as error:
            raise PodmanSecurityError("pinned runtime topology is invalid") from error
        if request.limits.timeout_seconds <= 0 or request.limits.output_bytes <= 0:
            raise PodmanSecurityError("timeout and output bound must be positive")
        if request.limits.pids <= 0:
            raise PodmanSecurityError("pids limit must be positive")


class PersistentPodmanCase:
    """One container reused for bounded analysis commands for one case."""

    def __init__(
        self, adapter: RootlessPodman, request: PodmanRun, cancel_requested: Event
    ) -> None:
        self.adapter = adapter
        self.request = request
        self.name = f"pi-baseline-{request.run_id}"
        self._started = False
        self._closed = False
        self.cancel_requested = cancel_requested

    def __enter__(self) -> PersistentPodmanCase:
        try:
            _check_cancel(self.cancel_requested)
            if not self.adapter.is_rootless(self.cancel_requested):
                raise PodmanSecurityError("Podman did not report rootless operation")
            _check_cancel(self.cancel_requested)
            # This is deliberately immediately adjacent to create().  The
            # descriptors, not their original pathnames, are the mount source.
            self.request.topology.verify()
            _run_command(
                self.create_command(),
                check=True, timeout=self.request.limits.timeout_seconds,
                pass_fds=self.request.pass_fds,
                cancel_requested=self.cancel_requested,
            )
            _check_cancel(self.cancel_requested)
            _run_command(
                [self.adapter.executable, "start", self.name],
                check=True, timeout=self.request.limits.timeout_seconds,
                cancel_requested=self.cancel_requested,
            )
            self._started = True
            return self
        except BaseException:
            self.close()
            raise

    def __exit__(self, exc_type: object, exc_value: object, traceback: object) -> None:
        self.close()

    def create_command(self) -> list[str]:
        """Construct the single detached container creation command."""

        request = self.request
        limits = request.limits
        return [
            self.adapter.executable,
            "create",
            "--name",
            self.name,
            "--read-only",
            "--userns=keep-id",
            "--ipc=private",
            "--uts=private",
            "--pid=private",
            "--cap-drop=ALL",
            "--security-opt=no-new-privileges",
            f"--memory={limits.memory}",
            f"--cpus={limits.cpus}",
            f"--pids-limit={limits.pids}",
            "--tmpfs",
            "/tmp:rw,size=64m,mode=1777",
            "--volume",
            f"/proc/self/fd/{request.topology.input.fd}:/input:ro",
            "--volume",
            f"/proc/self/fd/{request.topology.workspace.fd}:/work:rw",
            request.image,
            "sleep",
            "infinity",
        ]

    def exec(self, command: str) -> subprocess.CompletedProcess[str]:
        """Execute one shell command inside this case's existing container."""

        if not self._started or self._closed:
            raise RuntimeError("persistent Podman case is not running")
        result = _run_command(
            [
                self.adapter.executable,
                "exec",
                "--workdir",
                "/work",
                self.name,
                "sh",
                "-lc",
                command,
            ],
            check=False, timeout=self.request.limits.timeout_seconds,
            cancel_requested=self.cancel_requested,
        )
        return self.adapter._bounded(result, self.request.limits.output_bytes)

    def logs(self) -> subprocess.CompletedProcess[str]:
        """Collect bounded container logs before the case is closed."""

        result = _run_command(
            [self.adapter.executable, "logs", self.name],
            check=True, timeout=self.request.limits.timeout_seconds,
            cancel_requested=self.cancel_requested,
        )
        return self.adapter._bounded(result, self.request.limits.output_bytes)

    def close(self) -> None:
        """Immediately remove the case, even when creation or execution failed."""

        if self._closed:
            return
        deadline = time.monotonic() + _CLEANUP_TIMEOUT_SECONDS
        errors: list[str] = []
        removed = False
        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            command_timeout = min(_CLEANUP_COMMAND_TIMEOUT_SECONDS, remaining)
            try:
                subprocess.run(
                    [self.adapter.executable, "rm", "--force", "--time", "0", self.name],
                    check=False,
                    capture_output=True,
                    text=True,
                    timeout=command_timeout,
                )
            except BaseException as error:
                errors.append(f"force removal failed: {error}")

            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            try:
                removed = self.adapter.verify_removed(
                    self.request.run_id,
                    timeout_seconds=min(_CLEANUP_COMMAND_TIMEOUT_SECONDS, remaining),
                )
            except BaseException as error:
                errors.append(f"removal verification failed: {error}")
            if removed:
                self._closed = True
                break
            time.sleep(min(0.1, max(0.0, deadline - time.monotonic())))

        if not removed:
            raise ContainerCleanupError("failed to remove container (container_cleanup_failed)") from None


def _check_cancel(cancel_requested: Event) -> None:
    if cancel_requested.is_set():
        raise ExecutionInterrupted


def _run_command(
    command: list[str],
    *,
    check: bool,
    timeout: float,
    pass_fds: tuple[int, ...] = (),
    cancel_requested: Event,
) -> subprocess.CompletedProcess[str]:
    """Run Podman with short polling when cooperative cancellation is enabled."""
    _check_cancel(cancel_requested)
    process = subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        pass_fds=pass_fds,
    )
    deadline = time.monotonic() + timeout
    while process.poll() is None:
        _check_cancel_process(process, cancel_requested)
        if time.monotonic() >= deadline:
            _terminate_process(process)
            raise subprocess.TimeoutExpired(command, timeout)
        cancel_requested.wait(min(0.1, max(0.0, deadline - time.monotonic())))
    stdout, stderr = process.communicate()
    result = subprocess.CompletedProcess(command, process.returncode, stdout, stderr)
    if check and result.returncode:
        raise subprocess.CalledProcessError(
            result.returncode, command, output=stdout, stderr=stderr
        )
    return result


def _check_cancel_process(process: subprocess.Popen[str], cancel_requested: Event) -> None:
    if cancel_requested.is_set():
        _terminate_process(process)
        raise ExecutionInterrupted


def _terminate_process(process: subprocess.Popen[str]) -> None:
    if process.poll() is None:
        process.terminate()
        try:
            process.wait(timeout=1.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=1.0)


# Short name useful to callers, while keeping the security properties explicit.
PodmanAdapter = RootlessPodman
