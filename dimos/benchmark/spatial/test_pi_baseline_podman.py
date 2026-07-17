from pathlib import Path
import subprocess
from threading import Event
from unittest.mock import Mock, patch

import pytest

import dimos.benchmark.spatial.pi_baseline.podman as podman_module
from dimos.benchmark.spatial.pi_baseline.podman import (
    PodmanRun,
    PodmanSecurityError,
    RootlessPodman,
)
from dimos.benchmark.spatial.pi_baseline.topology import pin_runtime_topology


@pytest.fixture(autouse=True)
def _mock_cancellable_command(
    monkeypatch: pytest.MonkeyPatch, request: pytest.FixtureRequest
) -> None:
    if request.node.name.startswith("test_cancellation_during_each_runtime_command"):
        return
    def run(command: list[str], *, check: bool, timeout: float, pass_fds: tuple[int, ...] = (), cancel_requested: Event) -> subprocess.CompletedProcess[str]:
        if cancel_requested.is_set():
            raise podman_module.ExecutionInterrupted
        result = subprocess.run(
            command,
            check=False,
            capture_output=True,
            text=True,
            timeout=timeout,
            pass_fds=pass_fds,
        )
        if check and isinstance(result.returncode, int) and result.returncode:
            raise subprocess.CalledProcessError(result.returncode, command, output=result.stdout, stderr=result.stderr)
        return result

    monkeypatch.setattr(podman_module, "_run_command", run)


def request(tmp_path: Path) -> PodmanRun:
    paths = [tmp_path / name for name in ("in", "work", "out", "private")]
    for path in paths:
        path.mkdir()
    return PodmanRun(
        "registry.example/pi@sha256:" + "a" * 64,
        "run-1",
        pin_runtime_topology(
            input_dir=paths[0], workspace_dir=paths[1], output_dir=paths[2], private_dir=paths[3]
        ),
    )


def test_command_has_sandbox_and_only_expected_mounts(tmp_path: Path) -> None:
    command = RootlessPodman().command(request(tmp_path))
    rendered = " ".join(command)
    assert "--cap-drop=ALL" in command
    assert "--security-opt=no-new-privileges" in command
    assert not any(argument.startswith("--network") for argument in command)
    assert "--tmpfs" in command
    assert "/tmp:rw,size=64m,mode=1777" in command
    assert ":/input:ro" in rendered and ":/work:rw" in rendered
    assert "docker.sock" not in rendered
    assert "sha256:" in rendered


def test_requires_digest_and_distinct_workspace(tmp_path: Path) -> None:
    with pytest.raises(PodmanSecurityError):
        RootlessPodman().command(PodmanRun("image:latest", "run-1", request(tmp_path).topology))


@pytest.mark.parametrize("stage", ["info", "create", "start", "exec", "logs"])
def test_cancellation_during_each_runtime_command_terminates_client(
    monkeypatch: pytest.MonkeyPatch, stage: str, tmp_path: Path
) -> None:
    cancel_requested = Event()
    started: list[str] = []
    cleanup: list[list[str]] = []

    class Client:
        returncode = None
        stdout = None
        stderr = None

        def __init__(self, command: list[str], **__: object) -> None:
            self.command = command
            self.stage = command[1]
            self.returncode = 0
            started.append(self.stage)
            if self.stage == stage:
                self.returncode = None
                cancel_requested.set()

        def poll(self) -> int | None:
            return self.returncode

        def terminate(self) -> None:
            self.returncode = -15

        def wait(self, timeout: float) -> int:
            return self.returncode or 0

        def communicate(self) -> tuple[str, str]:
            return ("true\n" if self.stage == "info" else "", "")

    monkeypatch.setattr(podman_module.subprocess, "Popen", Client)
    def cleanup_run(command: list[str], **_: object) -> subprocess.CompletedProcess[str]:
        cleanup.append(command)
        if command[1:3] == ["container", "exists"]:
            return subprocess.CompletedProcess(command, 1, "", "")
        return subprocess.CompletedProcess(command, 0, "", "")

    monkeypatch.setattr(podman_module.subprocess, "run", cleanup_run)
    adapter = RootlessPodman()
    case = adapter.persistent(request(tmp_path), cancel_requested)
    if stage in {"info", "create", "start"}:
        with pytest.raises(podman_module.ExecutionInterrupted):
            with case:
                pass
    else:
        with case as active:
            with pytest.raises(podman_module.ExecutionInterrupted):
                if stage == "exec":
                    active.exec("sleep infinity")
                else:
                    active.logs()

    assert stage in started
    assert any(command[1:5] == ["rm", "--force", "--time", "0"] for command in cleanup)
    assert any(command[1:3] == ["container", "exists"] for command in cleanup)


@patch("dimos.benchmark.spatial.pi_baseline.podman.subprocess.run")
def test_failure_unconditionally_removes_container(mock_run: Mock, tmp_path: Path) -> None:
    mock_run.side_effect = [Mock(stdout="true\n"), RuntimeError("failed"), Mock()]
    with pytest.raises(RuntimeError):
        RootlessPodman().run(request(tmp_path), Event())
    assert mock_run.call_args_list[-1].args[0][1:3] == ["rm", "--force"]


@patch("dimos.benchmark.spatial.pi_baseline.podman.subprocess.run")
def test_persistent_case_reuses_one_container_and_collects_logs(
    mock_run: Mock, tmp_path: Path
) -> None:
    def result(command: list[str], **_: object) -> subprocess.CompletedProcess[str]:
        if command[1:3] == ["container", "exists"]:
            return subprocess.CompletedProcess(command, 1, "", "")
        stdout = "true\n" if command[1:2] == ["info"] else "output\n"
        return subprocess.CompletedProcess(command, 0, stdout, "")

    mock_run.side_effect = result
    adapter = RootlessPodman()
    with adapter.persistent(request(tmp_path), Event()) as case:
        case.exec("python analysis.py")
        case.exec("uv pip install package")
        logs = case.logs()

    commands = [call.args[0] for call in mock_run.call_args_list]
    assert sum(command[1:2] == ["create"] for command in commands) == 1
    assert sum(command[1:2] == ["start"] for command in commands) == 1
    exec_commands = [command for command in commands if command[1:2] == ["exec"]]
    assert len(exec_commands) == 2
    assert all(command[-3:-1] == ["sh", "-lc"] for command in exec_commands)
    create_command = next(command for command in commands if command[1:2] == ["create"])
    assert "/tmp:rw,size=64m,mode=1777" in create_command
    assert sum(argument == "--volume" for argument in create_command) == 2
    assert "docker.sock" not in " ".join(create_command)
    assert all(command[0] == "podman" for command in commands)
    assert logs.stdout == "output\n"
    assert commands[-2][1:2] == ["rm"]
    assert commands[-2][2:5] == ["--force", "--time", "0"]
    assert commands[-1][1:3] == ["container", "exists"]


@patch("dimos.benchmark.spatial.pi_baseline.podman.subprocess.run")
def test_persistent_timeout_immediately_removes_container(
    mock_run: Mock, tmp_path: Path
) -> None:
    mock_run.side_effect = [
        Mock(stdout="true\n"),
        Mock(stdout="container\n"),
        Mock(stdout="started\n"),
        TimeoutError("bounded command timed out"),
        Mock(stdout=""),
        subprocess.CompletedProcess([], 1, "", ""),
    ]
    with pytest.raises(TimeoutError):
        with RootlessPodman().persistent(request(tmp_path), Event()) as case:
            case.exec("long analysis")
    assert mock_run.call_args_list[-2].args[0][1:2] == ["rm"]
    assert mock_run.call_args_list[-2].args[0][2:5] == ["--force", "--time", "0"]
    assert mock_run.call_args_list[-1].args[0][1:3] == ["container", "exists"]


@patch("dimos.benchmark.spatial.pi_baseline.podman.subprocess.run")
@patch("dimos.benchmark.spatial.pi_baseline.podman._CLEANUP_TIMEOUT_SECONDS", 0.01)
def test_persistent_cleanup_reports_unremovable_container(
    mock_run: Mock, tmp_path: Path
) -> None:
    def responder(command: list[str], **_: object) -> subprocess.CompletedProcess[str]:
        if command[1:2] == ["info"]:
            return subprocess.CompletedProcess(command, 0, "true\n", "")
        if command[1:3] == ["container", "exists"]:
            return subprocess.CompletedProcess(command, 0, "", "")
        return subprocess.CompletedProcess(command, 0, "", "")

    mock_run.side_effect = responder
    with pytest.raises(RuntimeError, match="failed to remove container"):
        with RootlessPodman().persistent(request(tmp_path), Event()):
            pass


@patch("dimos.benchmark.spatial.pi_baseline.podman.subprocess.run")
def test_persistent_exec_propagates_nonzero_result_without_host_exception(
    mock_run: Mock, tmp_path: Path
) -> None:
    def responder(command: list[str], **_: object) -> subprocess.CompletedProcess[str]:
        if command[1:2] == ["info"]:
            return subprocess.CompletedProcess(command, 0, "true\n", "")
        if command[1:2] == ["exec"]:
            return subprocess.CompletedProcess(command, 17, "partial output\n", "analysis failed\n")
        if command[1:3] == ["container", "exists"]:
            return subprocess.CompletedProcess(command, 1, "", "")
        return subprocess.CompletedProcess(command, 0, "", "")

    mock_run.side_effect = responder
    with RootlessPodman().persistent(request(tmp_path), Event()) as case:
        completed = case.exec("false")

    assert completed.returncode == 17
    assert completed.stdout == "partial output\n"
    assert completed.stderr == "analysis failed\n"
    exec_call = next(call for call in mock_run.call_args_list if call.args[0][1:2] == ["exec"])
    assert exec_call.kwargs["check"] is False
