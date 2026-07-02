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

"""Hosted-teleop connection for coordinator-driven arms (transport-swap pattern).

The arm sibling of ``Go2HostedConnection``: ONE module colocating every
broker-bound stream so all hosted transports share the per-process
BrokerProvider session. There is no robot connection to subclass — actuation
runs through the ControlCoordinator, which this module feeds over LCM:

    operator (WebXR) ──cmd_unreliable──▶ cmd_raw ─┐
                                                  ├─ engage/delta-pose loop
    coordinator ◀── /coordinator/cartesian_command┘   (frame_id = task name)
                ◀── /teleop/buttons                    (analog triggers → gripper)

    cam1_in/cam2_in (RealSense over LCM) ──▶ mux ──▶ mux_image ──▶ CF video

Teleop logic (engage gating, delta poses, task routing) comes from
``ArmTeleopModule``; the hosted plane from ``HostedConnectionMixin``.
Operator bytes arrive on In-streams instead of the quest module's local
WebSocket, so the embedded web server is never started.
"""

from __future__ import annotations

from typing import Any

from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.Image import Image
from dimos.teleop.quest.quest_extensions import ArmTeleopConfig, ArmTeleopModule
from dimos.teleop.quest.quest_types import Hand
from dimos.teleop.quest_hosted.hosted_base import HostedConnectionMixin
from dimos.teleop.utils.teleop_transforms import webxr_to_robot
from dimos.teleop.utils.video_stats import VideoStats
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class ArmHostedConnectionConfig(ArmTeleopConfig):
    # task_names ("left"/"right" → coordinator task name) and control_loop_hz
    # come from ArmTeleopConfig; server_port is unused (no local web server).
    telemetry_hz: float = 3.0  # robot → operator HUD telemetry push rate
    # Publish-side video caps (0 = source rate/resolution), applied at the mux.
    video_max_width: int = 0
    video_max_fps: float = 0.0
    latency_stamp: bool = False  # benchmark: append capture-time strip


class ArmHostedConnection(ArmTeleopModule, HostedConnectionMixin):
    """Operator ⇄ coordinator bridge + camera mux, one broker session."""

    config: ArmHostedConnectionConfig

    # Broker-bound (bind CF/LiveKit transports to these in the blueprint).
    cmd_raw: In[bytes]  # LCM PoseStamped/Joy from the operator, fingerprint-dispatched
    state_json: In[bytes]  # JSON control plane: camera_select / estop / video_stats
    telemetry_out: Out[bytes]  # robot_telemetry + cmd_ack, state_reliable_back
    mux_image: Out[Image]

    # Local (LCM) side. left/right_controller_output and buttons are inherited.
    cam1_in: In[Image]
    cam2_in: In[Image]
    video_stats: Out[VideoStats]  # operator-side getStats() relay, recorder tap

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._hosted_init(["cam1", "cam2"])
        # While the E-STOP latch (from _hosted_init) is set, hands are held
        # disengaged and no poses are published — the coordinator's
        # TeleopIKTask keeps its last target, so the arm freezes in place.

    # No local WebSocket server — the operator connects through the broker.
    def _start_server(self) -> None:
        pass

    def _stop_server(self) -> None:
        pass

    @rpc
    def start(self) -> None:
        super().start()  # Module start + control loop (web server no-op'd)
        # Sync subscribes (not async handle_*): keep-latest would drop bursts.
        for stream, cb in (
            (self.cmd_raw, self._on_cmd_raw),
            (self.state_json, self._on_state_json),
        ):
            self.register_disposable(Disposable(stream.subscribe(cb)))
        self.register_disposable(
            Disposable(self.cam1_in.subscribe(lambda i: self._on_cam("cam1", i)))
        )
        self.register_disposable(
            Disposable(self.cam2_in.subscribe(lambda i: self._on_cam("cam2", i)))
        )
        self._start_telemetry()

    @rpc
    def stop(self) -> None:
        super().stop()  # stops the control loop (sets _stop_event) + Module teardown
        self._stop_telemetry()

    # ─── Inbound command plane (operator → robot) ─────────────────────

    def _on_cmd_raw(self, data: Any) -> None:
        """Fingerprint-dispatch LCM bytes from cmd_unreliable via the decoder
        table inherited from QuestTeleopModule (PoseStamped, Joy)."""
        if isinstance(data, str):
            data = data.encode()
        decoder = self._decoders.get(data[:8])
        if decoder is None:
            return  # foreign / undecodable frame — skip
        try:
            decoder(data)
        except Exception:
            logger.debug("cmd_raw decode failed", exc_info=True)

    def _on_pose_bytes(self, data: bytes) -> None:
        """Controller pose → robot frame. Overrides the quest version to drop
        (not raise on) unexpected frame_ids from the wire, and to feed the
        command-plane stats pushed to the operator HUD."""
        msg = PoseStamped.lcm_decode(data)
        try:
            hand = self._resolve_hand(msg.frame_id)
        except ValueError:
            return
        self._cmd_stats.record(msg.ts, nbytes=len(data))
        robot_pose = webxr_to_robot(msg, is_left_controller=(hand == Hand.LEFT))
        with self._lock:
            self._current_poses[hand] = robot_pose

    # ─── E-STOP gating over the inherited control loop ────────────────

    def _handle_engage(self) -> None:
        """While E-STOP is latched, refuse engagement (and drop any lingering
        engage) so clearing the latch never resumes motion by itself — the
        operator must release and re-press the engage button."""
        if self._estopped:
            for hand in Hand:
                if self._is_engaged[hand]:
                    self._disengage(hand)
            return
        super()._handle_engage()

    def _should_publish(self, hand: Hand) -> bool:
        # Belt to _handle_engage's braces: the latch can flip mid-iteration
        # (subscriber thread), so gate the publish path too.
        return not self._estopped and super()._should_publish(hand)

    # ─── E-STOP / operator-loss hooks (dispatched by the mixin) ───────

    def _handle_estop(self, nonce: Any) -> None:
        """Latch FIRST (gates the control loop immediately), then disengage."""
        self._estopped = True
        logger.warning("E-STOP latched by operator")
        with self._lock:
            self._disengage()
        self._send_ack(nonce, True)

    def _handle_estop_clear(self, nonce: Any) -> None:
        """Re-arm. Deliberately does NOT resume motion — hands stay disengaged
        until the operator releases and re-presses the engage button (which
        recaptures the initial pose, so the delta restarts from zero)."""
        self._estopped = False
        logger.warning("E-STOP cleared by operator")
        self._send_ack(nonce, True)

    def _on_operator_lost(self) -> None:
        """Command plane gone: disengage so a stale engage can't keep streaming
        the last delta into the coordinator when the operator reconnects."""
        logger.warning("operator link lost — disengaging")
        with self._lock:
            self._disengage()

    # ─── Telemetry (robot → operator) ─────────────────────────────────

    def _telemetry_state(self) -> dict[str, Any]:
        """Per-hand engage state (cams + estopped are merged in by the mixin)."""
        with self._lock:
            return {
                "engaged": {
                    "left": self._is_engaged[Hand.LEFT],
                    "right": self._is_engaged[Hand.RIGHT],
                }
            }


__all__ = ["ArmHostedConnection", "ArmHostedConnectionConfig"]
