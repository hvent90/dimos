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

"""Record a teleop delta trajectory while a button is held, replay it on demand.

A pure interceptor that sits between a teleop source (e.g. the Quest
``ArmTeleopModule``) and the :class:`ControlCoordinator`. It forwards live
teleop through untouched, so normal press-and-hold teleop keeps working. On top
of that it adds a *record + replay* gesture:

- Hold the **draw** button (default: right trigger) to record the incoming delta
  ``PoseStamped`` stream into a buffer. While drawing, the deltas are NOT
  forwarded to the arm — the arm stays put while you sketch a path in the air.
- Press the **play** button (default: right secondary / B) to replay the buffer:
  a background thread synthesizes one engage edge (so the ``TeleopIKTask``
  re-captures a fresh EE reference), then re-publishes the recorded deltas at
  ``replay_speed`` × record rate. The coordinator routes them to the task exactly
  like live teleop, so all task safety limits still apply.

The recorded poses are the same engage-relative deltas the teleop source already
produces; replaying them makes the arm retrace the drawn shape. Live teleop
input is gated off for the duration of a replay so the two don't fight.
"""

from __future__ import annotations

import threading
import time
from typing import TYPE_CHECKING, Any

from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.std_msgs.UInt32 import UInt32
from dimos.teleop.quest.quest_types import BUTTON_ALIASES, Buttons
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from collections.abc import Callable

logger = setup_logger()


class TrajectoryReplayConfig(ModuleConfig):
    """Configuration for :class:`TrajectoryReplayModule`.

    Attributes:
        draw_button: Button held to record the delta trajectory. Face-label
            alias ("LG"/"RG"/"X"/…) or raw ``Buttons`` attribute ("right_grip").
        play_button: Button whose rising edge triggers replay.
        engage_button: The ``Buttons`` attribute the target ``TeleopIKTask``
            watches for engage (must match the task's ``hand``). For a
            right-handed task this is ``"right_primary"``.
        hand: Which controller's trigger analog drives the recorded gripper
            ("right" or "left"); must match the task's hand.
        replay_speed: Playback rate as a fraction of the recorded rate.
            0.5 = half speed. Clamped to (0, 1].
        min_points: Minimum recorded deltas required before a replay will run.
    """

    draw_button: str = "right_grip"
    play_button: str = "right_secondary"
    engage_button: str = "right_primary"
    hand: str = "right"
    replay_speed: float = 0.5
    min_points: int = 2


class TrajectoryReplayModule(Module):
    """Intercept teleop streams to record a delta trajectory and replay it.

    Wire the teleop source's controller-pose output into ``controller_in`` and
    its button output into ``buttons_in``; wire ``cartesian_out`` to the
    coordinator's cartesian command input and ``buttons_out`` to its button
    input. Live teleop passes through unchanged; the draw/play buttons add the
    record/replay gesture on top.

    Inputs:
        - controller_in: PoseStamped (engage-relative delta from the teleop source)
        - buttons_in: Buttons (controller button state)

    Outputs:
        - cartesian_out: PoseStamped (forwarded live deltas, or replayed deltas)
        - buttons_out: Buttons (forwarded live buttons, or synthesized engage edge)
    """

    config: TrajectoryReplayConfig

    controller_in: In[PoseStamped]
    buttons_in: In[Buttons]

    cartesian_out: Out[PoseStamped]
    buttons_out: Out[Buttons]
    # Replay progress as permille [0,1000]; a headset overlay consumes the
    # drawn line up to this fraction as the arm retraces it.
    replay_progress: Out[UInt32]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)

        # Resolve button aliases to raw Buttons attribute names once.
        self._draw_attr = BUTTON_ALIASES.get(self.config.draw_button, self.config.draw_button)
        self._play_attr = BUTTON_ALIASES.get(self.config.play_button, self.config.play_button)
        self._engage_attr = BUTTON_ALIASES.get(
            self.config.engage_button, self.config.engage_button
        )
        self._is_left = self.config.hand == "left"
        self._speed = max(1e-3, min(1.0, self.config.replay_speed))

        self._lock = threading.Lock()
        self._recording = False
        self._replaying = False
        # Recorded (ts, delta, trigger) samples for the current/last draw window.
        # trigger is the analog gripper value [0,1] at that sample, so replay
        # reproduces gripper open/close alongside the arm motion.
        self._buffer: list[tuple[float, PoseStamped, float]] = []
        # Rising-edge detection state for draw/play/engage buttons.
        self._prev_draw = False
        self._prev_play = False
        self._prev_primary = False
        # Set when a replay is cancelled by an engage (A) press, so the worker
        # skips its own disengage edge and lets live teleop own the buttons.
        self._cancelled = False
        # Whether the live-teleop engage (X/A) is currently held. Only then may a
        # pose be forwarded to the arm — so draw poses can never leak to the arm
        # even if a pose arrives a tick before the recording latch flips.
        self._primary_held = False
        # Latest trigger analog seen on the buttons stream, sampled per pose.
        self._last_trigger = 0.0

        self._replay_thread: threading.Thread | None = None
        self._stop_replay = threading.Event()

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.controller_in.subscribe(self._on_controller)))
        self.register_disposable(Disposable(self.buttons_in.subscribe(self._on_buttons)))
        logger.info(
            "TrajectoryReplayModule started (draw=%s, play=%s, engage=%s, speed=%.2f)",
            self._draw_attr,
            self._play_attr,
            self._engage_attr,
            self._speed,
        )

    @rpc
    def stop(self) -> None:
        self._stop_replay.set()
        thread = self._replay_thread
        if thread is not None:
            thread.join(timeout=2.0)
            self._replay_thread = None
        super().stop()

    # ── port handlers ────────────────────────────────────────────────────────

    def _on_controller(self, msg: PoseStamped) -> None:
        """Forward live deltas, or record them while drawing.

        During a replay, live input is dropped so it doesn't fight the playback.
        While recording (draw held), deltas are buffered but NOT forwarded, so
        the arm stays still while the path is sketched.
        """
        with self._lock:
            if self._replaying:
                return
            if self._recording:
                self._buffer.append((time.perf_counter(), msg, self._last_trigger))
                return
            # Only forward to the arm during genuine live teleop (X/A engaged).
            # A pose arriving during a draw before the recording latch flips is
            # dropped, not sent to the arm.
            if not self._primary_held:
                return
        # Live teleop: pass the delta straight through.
        self.cartesian_out.publish(msg)

    def _on_buttons(self, msg: Buttons) -> None:
        """Pass buttons through, and edge-detect the draw/play gestures.

        During a replay we suppress live buttons so the synthesized engage edge
        isn't overridden by a stale controller state — except a press of the
        engage button (A), which cancels the replay and hands control back to
        live teleop.
        """
        draw = bool(getattr(msg, self._draw_attr, False))
        play = bool(getattr(msg, self._play_attr, False))
        primary = bool(getattr(msg, self._engage_attr, False))
        trigger = msg.left_trigger_analog if self._is_left else msg.right_trigger_analog

        # Pressing engage (A) during a replay cancels it and lets live teleop
        # take over. Done before the lock so the worker can observe the flags.
        with self._lock:
            cancel = self._replaying and primary and not self._prev_primary
        if cancel:
            self._cancel_replay()

        start_replay = False
        with self._lock:
            replaying = self._replaying
            # Latest gripper trigger, sampled by _on_controller into the buffer.
            self._last_trigger = trigger
            self._prev_primary = primary
            # Track live-teleop engage so _on_controller only forwards X/A poses.
            # Suppressed during replay (we drive the engage edge ourselves).
            if not replaying:
                self._primary_held = primary

            if not replaying:
                # Draw button edges: rising → start recording, falling → freeze.
                if draw and not self._prev_draw:
                    self._buffer = []
                    self._recording = True
                    logger.info("TrajectoryReplayModule: collection STARTED (grip pressed)")
                elif not draw and self._prev_draw:
                    self._recording = False
                    logger.info(
                        "TrajectoryReplayModule: collection ENDED (grip released) — %d points",
                        len(self._buffer),
                    )

                # Play button rising edge → replay (only when not drawing).
                if play and not self._prev_play and not self._recording:
                    if len(self._buffer) >= self.config.min_points:
                        start_replay = True
                    else:
                        logger.warning(
                            "TrajectoryReplayModule: nothing to replay (%d < %d points)",
                            len(self._buffer),
                            self.config.min_points,
                        )

            self._prev_draw = draw
            self._prev_play = play

        if start_replay:
            self._begin_replay()
            return

        # Forward live buttons only when not replaying (replay drives its own).
        with self._lock:
            replaying = self._replaying
        if not replaying:
            self.buttons_out.publish(msg)

    # ── replay ───────────────────────────────────────────────────────────────

    def _begin_replay(self) -> None:
        """Spawn a background thread to replay the recorded buffer."""
        with self._lock:
            if self._replaying:
                return
            self._replaying = True
            self._cancelled = False
            buffer = list(self._buffer)

        self._stop_replay.clear()
        self._replay_thread = threading.Thread(
            target=self._replay_worker,
            args=(buffer,),
            daemon=True,
            name="TrajectoryReplay",
        )
        self._replay_thread.start()

    def _cancel_replay(self) -> None:
        """Abort an in-flight replay so live teleop can take over.

        Marks the run cancelled (worker skips its disengage edge) and stops the
        thread. Runs on the buttons callback thread, so we join briefly; the
        worker exits promptly on the stop event.
        """
        with self._lock:
            if not self._replaying:
                return
            self._cancelled = True
        self._stop_replay.set()
        thread = self._replay_thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=1.0)
        with self._lock:
            self._replaying = False
        logger.info("TrajectoryReplayModule: replay CANCELLED by engage (A) — live teleop resumes")

    def _replay_worker(self, buffer: list[tuple[float, PoseStamped, float]]) -> None:
        """Emit an engage edge, replay deltas + gripper at replay_speed, disengage.

        Timing between deltas preserves the recorded cadence divided by
        ``replay_speed`` (so 0.5 → twice as slow). The frame_id on each delta is
        left untouched so the coordinator routes it to the same task the live
        stream targeted. Each step also republishes a Buttons carrying the
        engage bit (to keep the task engaged) plus the recorded trigger analog
        (so the gripper tracks). A per-step progress float [0,1] is published on
        ``replay_progress`` so a headset overlay can consume the drawn line.
        """
        total = len(buffer)
        try:
            logger.info("TrajectoryReplayModule: replay started (%d points)", total)

            # 1) Rising engage edge → task re-captures its EE reference.
            self._publish_buttons(engaged=True, trigger=buffer[0][2])
            # Give the coordinator a tick to process the engage before deltas.
            time.sleep(0.05)

            # 2) Stream the recorded deltas + gripper at the scaled cadence.
            prev_ts = buffer[0][0]
            for i, (rec_ts, delta, trigger) in enumerate(buffer):
                if self._stop_replay.is_set():
                    break
                gap = (rec_ts - prev_ts) / self._speed
                prev_ts = rec_ts
                if gap > 0:
                    if self._stop_replay.wait(gap):
                        break
                # Keep the task engaged and drive the gripper for this sample.
                self._publish_buttons(engaged=True, trigger=trigger)
                # Restamp so the task's timeout logic sees a fresh command.
                out = PoseStamped(
                    position=delta.position,
                    orientation=delta.orientation,
                    ts=time.time(),
                    frame_id=delta.frame_id,
                )
                self.cartesian_out.publish(out)
                self._publish_progress((i + 1) / total)
        except Exception:
            logger.exception("TrajectoryReplayModule: replay failed")
        finally:
            with self._lock:
                cancelled = self._cancelled
            if cancelled:
                # Cancelled by a live engage (A): live teleop now owns the
                # buttons, so don't publish our disengage edge (it would fight
                # the operator's engage). Clear the line and leave.
                try:
                    self._publish_progress(1.0)
                except Exception:
                    logger.exception("TrajectoryReplayModule: progress publish failed")
            else:
                # 3) Falling engage edge → task disengages and stops holding.
                try:
                    self._publish_buttons(engaged=False, trigger=0.0)
                    self._publish_progress(1.0)  # ensure the line is fully consumed
                except Exception:
                    logger.exception("TrajectoryReplayModule: failed to publish disengage")
            with self._lock:
                self._replaying = False
            logger.info("TrajectoryReplayModule: replay finished (cancelled=%s)", cancelled)

    def _publish_buttons(self, *, engaged: bool, trigger: float) -> None:
        """Publish a Buttons with the engage bit and the gripper trigger analog."""
        buttons = Buttons()
        buttons.set_attribute(self._engage_attr, engaged)
        if self._is_left:
            buttons.pack_analog_triggers(left=trigger, right=0.0)
        else:
            buttons.pack_analog_triggers(left=0.0, right=trigger)
        self.buttons_out.publish(buttons)

    def _publish_progress(self, fraction: float) -> None:
        """Publish replay progress as permille [0,1000] for a headset overlay.

        Permille in a UInt32 avoids adding a float scalar msg type; the JS side
        divides by 1000 to trim the drawn line up to that fraction.
        """
        permille = int(round(max(0.0, min(1.0, fraction)) * 1000))
        self.replay_progress.publish(UInt32(permille))
