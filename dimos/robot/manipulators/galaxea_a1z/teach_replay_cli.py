# Copyright 2025-2026 Dimensional Inc.
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

"""Foreground A1Z hand-teach and replay commands."""

from __future__ import annotations

from datetime import datetime
from pathlib import Path
import time
from typing import Any

import typer

from dimos.constants import STATE_DIR

app = typer.Typer(help="Record and replay Galaxea A1Z hand-taught episodes")

_TEACH_HARDWARE_ID = "arm"
# Matches the adapter's default G1Z max opening; the adapter clamps to the
# configured range, so a full-open command stays correct if that changes.
_GRIPPER_OPEN_M = 0.1
_GRIPPER_CLOSED_M = 0.0


def _default_recording_path() -> Path:
    return STATE_DIR / "recordings" / f"a1z_teach_{datetime.now():%Y%m%d_%H%M%S}.db"


def _press_enter(message: str) -> None:
    typer.prompt(message, default="", show_default=False)


def _read_key(message: str) -> str:
    """Read one keypress without waiting for ENTER.

    Returns the lowercased character; ENTER is normalized to "". Falls back
    to line input when stdin is not an interactive terminal.
    """
    import sys

    typer.echo(message)
    if not sys.stdin.isatty():
        line = sys.stdin.readline()
        if not line:
            raise EOFError
        return line.strip().lower()[:1]

    import termios
    import tty

    fd = sys.stdin.fileno()
    saved = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, saved)
    if key == "\x03":  # Ctrl-C arrives as a literal byte in cbreak mode
        raise KeyboardInterrupt
    if key in ("\r", "\n"):
        return ""
    return key.lower()


@app.command()
def teach(
    output: Path | None = typer.Argument(
        None,
        help="Memory2 .db output (default: timestamped file under the DimOS state directory)",
    ),
    task: str | None = typer.Option(None, "--task", help="Task label stored with each episode"),
    camera_index: int = typer.Option(
        0,
        "--camera-index",
        min=0,
        help="Linux camera index N for /dev/videoN",
    ),
    gripper_free_drive: bool = typer.Option(
        False,
        "--gripper-free-drive",
        help="Zero-torque gripper you pinch by hand (legacy); default keeps the "
        "gripper powered and toggled with g so your hand stays out of the camera",
    ),
) -> None:
    """Hand-teach episodes into one Memory2 recording."""
    from dimos.control.coordinator import ControlCoordinator
    from dimos.core.coordination.module_coordinator import ModuleCoordinator
    from dimos.learning.collection.episode_monitor import EpisodeMonitorModule
    from dimos.robot.manipulators.galaxea_a1z.blueprints.basic import (
        make_a1z_teach_blueprint,
    )

    db_path = (output or _default_recording_path()).expanduser().resolve()
    if db_path.exists():
        typer.echo(f"error: refusing to overwrite existing recording: {db_path}", err=True)
        raise typer.Exit(2)

    typer.echo("A1Z hand-teach mode")
    typer.echo(f"Recording: {db_path}")
    typer.echo(f"Camera: /dev/video{camera_index} (640x480 at 15 FPS)")
    typer.echo("The arm will become hand-drivable after startup.")
    if gripper_free_drive:
        typer.echo("Gripper: free drive (open and close it by hand).")
    else:
        typer.echo("Gripper: powered; type g then ENTER to toggle open/closed.")
    typer.echo("Keep the arm supported: it has no brakes and can fall when motors disable.\n")

    coordinator: ModuleCoordinator | None = None
    recording = False
    gripper_open: bool | None = None

    try:
        coordinator = ModuleCoordinator.build(
            make_a1z_teach_blueprint(
                db_path,
                task_label=task,
                camera_index=camera_index,
                gripper_free_drive=gripper_free_drive,
            ),
            {},
        )
        monitor: Any = coordinator.get_instance(EpisodeMonitorModule)
        control: Any = coordinator.get_instance(ControlCoordinator)
        if not gripper_free_drive:
            measured = control.get_gripper_position(_TEACH_HARDWARE_ID)
            gripper_open = measured is not None and measured > _GRIPPER_OPEN_M / 2
        typer.echo("Ready. Move only after starting an episode.")

        def _toggle_gripper() -> None:
            nonlocal gripper_open
            if gripper_free_drive:
                typer.echo("Gripper is in free drive; open and close it by hand.")
                return
            target_open = not gripper_open
            target = _GRIPPER_OPEN_M if target_open else _GRIPPER_CLOSED_M
            if control.set_gripper_position(_TEACH_HARDWARE_ID, target):
                gripper_open = target_open
                typer.echo(f"Gripper {'opening' if target_open else 'closing'}.")
            else:
                typer.echo("Gripper command rejected; check hardware state.", err=True)

        while True:
            if not recording:
                command = _read_key(
                    "Press ENTER to start an episode, g to toggle the gripper, or q to finish"
                )
                if command == "q":
                    break
                if command == "g":
                    _toggle_gripper()
                    continue
                if command:
                    typer.echo("Use ENTER to start, g for the gripper, or q to finish.")
                    continue
                monitor.start_episode()
                recording = True
                typer.echo("RECORDING — move the arm by hand; g toggles the gripper.")
                continue

            command = _read_key(
                "Press ENTER to save, g to toggle the gripper, d to discard, "
                "or q to discard and finish"
            )
            if command == "g":
                _toggle_gripper()
            elif command == "d":
                monitor.discard_episode()
                recording = False
                typer.echo("Episode discarded.")
            elif command == "q":
                monitor.discard_episode()
                recording = False
                typer.echo("Active episode discarded.")
                break
            elif not command:
                status = monitor.save_episode()
                recording = False
                typer.echo(f"Episode saved ({status.episodes_saved} total).")
            else:
                typer.echo("Use ENTER to save, g for the gripper, d to discard, or q to finish.")
    except KeyboardInterrupt:
        if coordinator is not None and recording:
            monitor = coordinator.get_instance(EpisodeMonitorModule)
            monitor.discard_episode()
            typer.echo("\nActive episode discarded.")
    except Exception as exc:
        typer.echo(f"A1Z teach failed: {exc}", err=True)
        raise typer.Exit(1)
    finally:
        if coordinator is not None:
            typer.echo("\nSupport the arm before the recording is flushed and motors disable.")
            try:
                _press_enter("Press ENTER when the arm is supported")
            except (KeyboardInterrupt, EOFError):
                pass
            coordinator.stop()

    typer.echo(f"Saved Memory2 recording: {db_path}")


@app.command()
def replay(
    source: Path = typer.Argument(..., help="Memory2 recording .db"),
    episode: int = typer.Option(-1, "--episode", "-e", help="Saved episode index; -1 is latest"),
    speed: float = typer.Option(1.0, "--speed", min=0.01, help="Requested playback speed"),
) -> None:
    """Validate and replay one saved A1Z episode through ControlCoordinator."""
    from dimos.control.coordinator import ControlCoordinator
    from dimos.core.coordination.module_coordinator import ModuleCoordinator
    from dimos.msgs.trajectory_msgs.TrajectoryStatus import TrajectoryState
    from dimos.robot.manipulators.galaxea_a1z.blueprints.basic import (
        A1Z_REPLAY_TASK_NAME,
        make_a1z_replay_blueprint,
    )
    from dimos.robot.manipulators.galaxea_a1z.teach_replay import (
        build_execution_trajectory,
        load_recorded_episode,
        prepare_episode,
    )

    source = source.expanduser().resolve()
    try:
        recorded = load_recorded_episode(source, episode)
        prepared = prepare_episode(recorded, speed=speed)
    except Exception as exc:
        typer.echo(f"A1Z replay preflight failed: {exc}", err=True)
        raise typer.Exit(1)

    typer.echo(f"Recording: {source}")
    typer.echo(
        f"Episode: {recorded.episode_index} ({len(recorded.timestamps)} measured samples, "
        f"{recorded.timestamps[-1]:.2f}s)"
    )
    if prepared.effective_speed < prepared.requested_speed * 0.999:
        typer.echo(
            f"Safety time-scaling: requested {prepared.requested_speed:.2f}x, "
            f"using {prepared.effective_speed:.2f}x"
        )
    else:
        typer.echo(f"Playback speed: {prepared.effective_speed:.2f}x")
    typer.echo("Raw recorded values passed command-limit validation; nothing was clipped.")
    typer.echo("Support the arm during startup. It has no brakes.\n")

    coordinator: ModuleCoordinator | None = None
    started = False
    try:
        coordinator = ModuleCoordinator.build(make_a1z_replay_blueprint(), {})
        control: Any = coordinator.get_instance(ControlCoordinator)
        current_positions = control.get_joint_positions()
        trajectory = build_execution_trajectory(current_positions, prepared)

        typer.echo(
            f"The robot will approach the recorded start pose, then replay for "
            f"{prepared.duration:.2f}s. Total controlled motion: {trajectory.duration:.2f}s."
        )
        if not typer.confirm("Execute this motion now?", default=False):
            typer.echo("Replay cancelled before motion.")
            return

        accepted = control.task_invoke(
            A1Z_REPLAY_TASK_NAME,
            "execute",
            {"trajectory": trajectory},
        )
        if not accepted:
            raise RuntimeError("ControlCoordinator rejected the replay trajectory")
        started = True

        deadline = time.monotonic() + trajectory.duration + 5.0
        while time.monotonic() < deadline:
            state = TrajectoryState(control.task_invoke(A1Z_REPLAY_TASK_NAME, "get_state", {}))
            if state == TrajectoryState.COMPLETED:
                typer.echo("Replay complete. The arm is holding the final pose.")
                break
            if state in (TrajectoryState.ABORTED, TrajectoryState.FAULT):
                raise RuntimeError(f"Replay ended in state {state.name}")
            time.sleep(0.05)
        else:
            control.task_invoke(A1Z_REPLAY_TASK_NAME, "cancel", {})
            raise TimeoutError("Replay did not complete before its safety timeout")
    except KeyboardInterrupt:
        typer.echo("\nReplay interrupted.", err=True)
        if coordinator is not None and started:
            control = coordinator.get_instance(ControlCoordinator)
            control.task_invoke(A1Z_REPLAY_TASK_NAME, "cancel", {})
    except Exception as exc:
        typer.echo(f"A1Z replay failed: {exc}", err=True)
        raise typer.Exit(1)
    finally:
        if coordinator is not None:
            typer.echo("Support the arm before disabling its motors.")
            try:
                _press_enter("Press ENTER when the arm is supported")
            except (KeyboardInterrupt, EOFError):
                pass
            coordinator.stop()
