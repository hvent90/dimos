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

"""Go2 driver + hosted-teleop control plane in ONE module.

The broker provider is a per-process singleton, and ``GO2Connection`` is
``dedicated_worker=True`` (its own process), so all hosted broker transports
(cmd, video, state, state_back) must live on this one module to share a single
CF session — a separate bridge module lands in another worker = a 2nd session
the operator can't see. Opt-in subclass; plain ``GO2Connection`` is unchanged.
"""

from __future__ import annotations

import json
import threading
import time
from typing import Any

import numpy as np
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.Image import Image
from dimos.robot.unitree.go2.connection import ConnectionConfig, GO2Connection
from dimos.teleop.utils.stream_stats import LiveStreamStats
from dimos.teleop.utils.video_stats import VideoStats
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Operator-allowed sport commands → SPORT_CMD api_id (robot-side allow-list).
ALLOWED_SPORT_CMDS: dict[str, int] = {
    "StandDown": 1005,
    "RecoveryStand": 1006,
    "Sit": 1009,
    "Hello": 1016,
    "Stretch": 1017,
    "Damp": 1001,
    "FrontPounce": 1032,  # acrobatic — leaps
    "FrontJump": 1031,  # acrobatic — leaps
}


class Go2HostedConnectionConfig(ConnectionConfig):
    telemetry_hz: float = 3.0  # robot → operator HUD telemetry push rate


class Go2HostedConnection(GO2Connection):
    """GO2Connection + the hosted-teleop state plane, colocated (one session)."""

    config: Go2HostedConnectionConfig

    state_json: In[bytes]  # operator → robot control JSON (state_reliable)
    cmd_raw: In[bytes]  # operator → robot command bytes (stats tap)
    video_stats: Out[VideoStats]  # operator video health, for recorders
    telemetry_out: Out[bytes]  # robot → operator telemetry + acks (state_reliable_back)
    cam2_in: In[Image]  # extra camera (RealSense) for the mux
    mux_image: Out[Image]  # composited cam1(Go2)+cam2 → video transport

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._cmd_stats = LiveStreamStats()
        self._telemetry_thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._rage_active = False  # tracks firmware Rage Mode (speed bar)
        self._cam_lock = threading.Lock()
        self._cam_frames: dict[str, Image] = {}  # "cam1"/"cam2" → latest frame
        self._cam_selected = ["cam1"]  # operator tab selection

    @rpc
    def start(self) -> None:
        super().start()
        self._stop_event.clear()
        # Sync subscribes (not async handle_*): keep-latest would drop bursts.
        for stream, cb in (
            (self.state_json, self._on_state_json),
            (self.cmd_raw, self._on_cmd_raw),
        ):
            self.register_disposable(Disposable(stream.subscribe(cb)))
        # Mux: tap the base's color_image as cam1, RealSense as cam2 → mux_image.
        self.register_disposable(
            Disposable(self.color_image.subscribe(lambda i: self._on_cam("cam1", i)))
        )
        self.register_disposable(
            Disposable(self.cam2_in.subscribe(lambda i: self._on_cam("cam2", i)))
        )
        self._start_telemetry()

    # ─── Camera mux ──────────────────────────────────────────────────
    def _on_cam(self, cam: str, img: Image) -> None:
        with self._cam_lock:
            self._cam_frames[cam] = img
            shown = cam in self._cam_selected
        if shown:
            out = self._composite()
            if out is not None:
                self.mux_image.publish(out)

    def _composite(self) -> Image | None:
        with self._cam_lock:
            order = [c for c in ("cam1", "cam2") if c in self._cam_selected]
            imgs = [self._cam_frames[c] for c in order if c in self._cam_frames]
        if not imgs:
            return None
        if len(imgs) == 1:
            return imgs[0]
        import cv2

        target_h = min(im.data.shape[0] for im in imgs)
        tiles = []
        for im in imgs:
            h, w = im.data.shape[:2]
            tiles.append(
                cv2.resize(im.data, (int(w * target_h / h), target_h)) if h != target_h else im.data
            )
        return Image(data=np.hstack(tiles), format=imgs[0].format, frame_id="camera_mux")

    def _set_cam_selection(self, cams: list[str]) -> None:
        sel = [c for c in cams if c in ("cam1", "cam2")] or ["cam1"]
        with self._cam_lock:
            self._cam_selected = sel
        logger.info("camera selection → %s", sel)
        out = self._composite()
        if out is not None:
            self.mux_image.publish(out)

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        if self._telemetry_thread is not None:
            self._telemetry_thread.join(timeout=2.0)
            self._telemetry_thread = None
        super().stop()

    # ─── Inbound state plane (operator → robot) ──────────────────────

    def _on_state_json(self, data: Any) -> None:
        if isinstance(data, str):
            data = data.encode()
        if not data.startswith(b"{"):
            return  # not JSON
        try:
            msg = json.loads(data)
        except ValueError:
            logger.warning("state_reliable: malformed JSON: %r", data[:80])
            return

        kind = msg.get("type")
        if kind == "sport_cmd":
            self._handle_sport_cmd(msg)
        elif kind == "set_mode":
            self._handle_set_mode(msg)
        elif kind == "camera_select":
            self._set_cam_selection(msg.get("cams", []))
        elif kind == "video_stats":
            self.video_stats.publish(VideoStats.from_dict(msg))
        elif kind == "clock_report":
            logger.info(
                "clock-sync: operator rtt=%s offset=%s",
                msg.get("rtt_ms"),
                msg.get("offset_ms"),
            )
        # ping answered by BrokerProvider; unknown types ignored.

    def _handle_sport_cmd(self, msg: dict[str, Any]) -> None:
        """Operator button → allow-listed SPORT_MOD request, ack on cmd_ack."""
        name = msg.get("name")
        nonce = msg.get("nonce")

        # StandReady is the standup+balance combo, never the two separately.
        if name == "StandReady":
            self._stand_ready(nonce)
            return

        api_id = ALLOWED_SPORT_CMDS.get(name) if isinstance(name, str) else None
        if api_id is None:
            logger.warning("sport_cmd: disallowed/unknown name %r", name)
            self._send_ack(nonce, False)
            return

        # sport_command blocks the WebRTC loop (= the video loop) — run off the
        # callback so a gesture like Hello doesn't stall video.
        def runner() -> None:
            ok = False
            try:
                ok = bool(self.connection.sport_command(api_id))
            except Exception:
                logger.exception("sport_cmd %s failed", name)
            self._send_ack(nonce, ok)

        threading.Thread(target=runner, daemon=True, name=f"Go2SportCmd-{name}").start()

    def _stand_ready(self, nonce: Any) -> None:
        """Standup → settle → BalanceStand (drive-ready). Acks when balanced."""

        def runner() -> None:
            ok = False
            try:
                self.connection.standup()
                time.sleep(3.0)  # standup must finish before balance_stand
                self.connection.balance_stand()
                ok = True
            except Exception:
                logger.exception("StandReady failed")
            self._send_ack(nonce, ok)

        threading.Thread(target=runner, daemon=True, name="Go2StandReady").start()

    def _handle_set_mode(self, msg: dict[str, Any]) -> None:
        """Speed-mode select. normal/high differ only by browser-side scale;
        only the rage on/off boundary toggles the firmware (set_rage_mode)."""
        mode = msg.get("mode")
        nonce = msg.get("nonce")
        if mode not in ("normal", "high", "rage"):
            logger.warning("set_mode: unknown mode %r", mode)
            self._send_ack(nonce, False)
            return
        want_rage = mode == "rage"
        if want_rage == self._rage_active:
            self._send_ack(nonce, True)  # already in the right FSM
            return

        def runner() -> None:
            ok = False
            try:
                ok = bool(self.connection.set_rage_mode(want_rage))
                if ok:
                    self._rage_active = want_rage
                logger.info("set_mode: rage=%s ok=%s", want_rage, ok)
            except Exception:
                logger.exception("set_mode rage=%s failed", want_rage)
            self._send_ack(nonce, ok)

        threading.Thread(target=runner, daemon=True, name="Go2SetMode").start()

    def _send_ack(self, nonce: Any, ok: bool) -> None:
        try:
            self.telemetry_out.publish(
                json.dumps({"type": "cmd_ack", "nonce": nonce, "ok": ok}).encode()
            )
        except Exception:
            logger.debug("cmd_ack publish failed", exc_info=True)

    # ─── Command-plane health (robot → operator) ─────────────────────

    def _on_cmd_raw(self, data: Any) -> None:
        """Read the send-stamp off the header for one-way latency stats."""
        if isinstance(data, str):
            data = data.encode()
        try:
            from dimos_lcm.geometry_msgs import TwistStamped as LCMTwistStamped

            lcm = LCMTwistStamped.lcm_decode(data)
            ts = lcm.header.stamp.sec + lcm.header.stamp.nsec / 1_000_000_000
        except Exception:
            return  # foreign / undecodable frame — skip
        self._cmd_stats.record(ts, nbytes=len(data))

    def _start_telemetry(self) -> None:
        def runner() -> None:
            interval = 1.0 / max(self.config.telemetry_hz, 0.1)
            while not self._stop_event.is_set():
                snap = self._cmd_stats.snapshot()
                soc = getattr(self, "_latest_soc", None)  # cached by GO2Connection
                if snap is not None or soc is not None:
                    payload = json.dumps(
                        {
                            "type": "robot_telemetry",
                            "cmd": snap,
                            "soc": soc,
                            "robot_ts": time.time(),
                        }
                    )
                    try:
                        self.telemetry_out.publish(payload.encode())
                    except Exception:
                        logger.debug("telemetry publish failed", exc_info=True)
                self._stop_event.wait(interval)

        self._telemetry_thread = threading.Thread(
            target=runner, daemon=True, name="Go2HostedTelemetry"
        )
        self._telemetry_thread.start()


__all__ = ["Go2HostedConnection", "Go2HostedConnectionConfig"]
