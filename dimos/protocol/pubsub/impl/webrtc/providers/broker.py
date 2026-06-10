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

"""Broker-mediated Cloudflare Realtime provider (hosted teleop).

The robot dials out to the ``dimensional-teleop`` broker, which owns CF
session creation and operator lifecycle. The SCTP id of the operator's
command channel arrives via heartbeat acks; we open/close the negotiated
channel to track the broker's view (operator join / leave / rejoin).
The aiortc/CF quirks this inherits (MAX_BUNDLE, the id=0 throwaway channel)
are documented in ``dimos/teleop/quest_hosted/README.md``.

Env vars (fallback when config fields are unset):
    TELEOP_BROKER_URL — default https://teleop.dimensionalos.com
    TELEOP_API_KEY    — robot API key (dtk_live_*)
    TELEOP_ROBOT_ID   — robot identifier
    TELEOP_ROBOT_NAME — human-readable robot name
"""

from __future__ import annotations

import asyncio
from collections import defaultdict
from collections.abc import Callable
import contextlib
from dataclasses import dataclass
import os
import threading
from typing import Any

from dimos.protocol.pubsub.impl.webrtc.providers.spec import (
    WEBRTC_AVAILABLE,
    AsyncProviderBase,
    ProviderConfig,
    wait_connected,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

if WEBRTC_AVAILABLE:
    from aiortc import (
        RTCBundlePolicy,
        RTCConfiguration,
        RTCDataChannel,
        RTCIceServer,
        RTCPeerConnection,
        RTCSessionDescription,
    )
    import httpx
else:
    RTCDataChannel = Any  # type: ignore[misc,assignment]


@dataclass(frozen=True)
class BrokerConfig(ProviderConfig):
    """Hosted teleop broker access. Credentials default from TELEOP_* env."""

    broker_url: str | None = None
    api_key: str | None = None
    robot_id: str | None = None
    robot_name: str | None = None
    stun_url: str = "stun:stun.cloudflare.com:3478"
    heartbeat_hz: float = 1.0
    ordered: bool = False
    max_retransmits: int | None = 0

    def _create(self) -> BrokerProvider:
        return BrokerProvider(self)


class BrokerProvider(AsyncProviderBase):
    """Receive-only provider: operator commands arrive on one multiplexed
    DataChannel; demux by LCM fingerprint happens at the transport layer.

    Robot → operator channels are not bridged by the broker yet, so
    ``publish()`` raises. Telemetry/video stay on ``HostedTeleopModule``
    until those move into this provider.
    """

    def __init__(self, config: BrokerConfig | None = None) -> None:
        if not WEBRTC_AVAILABLE:
            raise RuntimeError("aiortc and httpx required: pip install dimos[webrtc]")
        super().__init__()
        config = config or BrokerConfig()
        self._broker_url = (
            config.broker_url
            or os.environ.get("TELEOP_BROKER_URL", "https://teleop.dimensionalos.com")
        ).rstrip("/")
        self._api_key = config.api_key or os.environ.get("TELEOP_API_KEY", "")
        self._robot_id = config.robot_id or os.environ.get("TELEOP_ROBOT_ID", "")
        self._robot_name = config.robot_name or os.environ.get("TELEOP_ROBOT_NAME", "robot")
        if not self._api_key:
            raise RuntimeError("TELEOP_API_KEY or BrokerConfig.api_key required")
        if not self._robot_id:
            raise RuntimeError("TELEOP_ROBOT_ID or BrokerConfig.robot_id required")
        self._config = config

        self._http: httpx.AsyncClient | None = None
        self._pc: RTCPeerConnection | None = None
        self.session_id: str | None = None
        self._hb_task: asyncio.Task[None] | None = None
        self._cmd_dc: RTCDataChannel | None = None
        self._cmd_dc_id: int | None = None
        self._has_operator = threading.Event()

        # Guarded by self._lock (from the base).
        self._callbacks: dict[str, list[Callable[[bytes, str], None]]] = defaultdict(list)

    @property
    def has_operator(self) -> bool:
        """Whether an operator's command channel is currently open."""
        return self._has_operator.is_set()

    def wait_for_operator(self, timeout: float | None = None) -> bool:
        """Block until an operator connects. Returns True if one did."""
        return self._has_operator.wait(timeout=timeout)

    @property
    def _headers(self) -> dict[str, str]:
        return {"X-Robot-API-Key": self._api_key, "Content-Type": "application/json"}

    # ─── Connect / Disconnect (loop thread) ──────────────────────────

    async def _connect(self) -> None:
        self._http = httpx.AsyncClient(timeout=30.0)
        # MAX_BUNDLE + the id=0 throwaway channel are CF/aiortc workarounds —
        # see dimos/teleop/quest_hosted/README.md before changing.
        self._pc = RTCPeerConnection(
            RTCConfiguration(
                iceServers=[RTCIceServer(urls=[self._config.stun_url])],
                bundlePolicy=RTCBundlePolicy.MAX_BUNDLE,
            )
        )
        self._pc.createDataChannel("_sctp_init", negotiated=True, id=0)

        offer = await self._pc.createOffer()
        await self._pc.setLocalDescription(offer)
        if self._pc.iceGatheringState != "complete":
            ev = asyncio.Event()
            pc = self._pc

            @pc.on("icegatheringstatechange")
            def _on_gathering() -> None:
                if pc.iceGatheringState == "complete":
                    ev.set()

            await asyncio.wait_for(ev.wait(), 10.0)

        r = await self._http.post(
            f"{self._broker_url}/api/v1/sessions",
            headers=self._headers,
            json={
                "robot_id": self._robot_id,
                "robot_name": self._robot_name,
                "sdp_offer": self._pc.localDescription.sdp,
            },
        )
        if r.status_code not in (200, 201):
            raise RuntimeError(f"Broker session create failed: {r.status_code} {r.text[:500]}")
        data = r.json()
        self.session_id = data["session_id"]
        await self._pc.setRemoteDescription(
            RTCSessionDescription(sdp=data["sdp_answer"], type="answer")
        )
        await wait_connected(self._pc)
        logger.info(
            "Broker provider connected: session=%s robot=%s", self.session_id, self._robot_id
        )
        self._hb_task = asyncio.get_running_loop().create_task(self._heartbeat_loop())

    async def _disconnect(self) -> None:
        if self._hb_task is not None:
            self._hb_task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await self._hb_task
            self._hb_task = None
        if self._http and self.session_id:
            with contextlib.suppress(Exception):  # best-effort deregistration
                await self._http.delete(
                    f"{self._broker_url}/api/v1/sessions/{self.session_id}",
                    headers=self._headers,
                )
        self._close_cmd_channel()
        if self._pc:
            await self._pc.close()
            self._pc = None
        if self._http:
            await self._http.aclose()
            self._http = None
        self.session_id = None

    # ─── Heartbeat (loop thread; cancelled in _disconnect) ───────────

    async def _heartbeat_loop(self) -> None:
        interval = 1.0 / max(self._config.heartbeat_hz, 0.1)
        while True:
            try:
                await self._heartbeat_once()
            except Exception:
                logger.exception("Broker heartbeat failed")
            await asyncio.sleep(interval)

    async def _heartbeat_once(self) -> None:
        if self._http is None or self.session_id is None:
            return
        r = await self._http.post(
            f"{self._broker_url}/api/v1/sessions/{self.session_id}/heartbeat",
            headers=self._headers,
            json={},
        )
        if r.status_code != 200:
            logger.warning("Heartbeat failed: %d %s", r.status_code, r.text[:200])
            return
        sub_id = r.json().get("cmd_channel_subscriber_id")
        sub_id = int(sub_id) if sub_id is not None else None
        # Track the broker's view: open on join, close on leave, re-open on
        # rejoin (the broker assigns a fresh SCTP id per operator session).
        if sub_id != self._cmd_dc_id:
            self._close_cmd_channel()
            self._cmd_dc_id = sub_id
            if sub_id is not None:
                self._open_cmd_channel(sub_id)

    def _open_cmd_channel(self, sctp_id: int) -> None:
        assert self._pc is not None
        logger.info("Opening negotiated cmd_unreliable on SCTP id %d", sctp_id)
        ch = self._pc.createDataChannel(
            "cmd_unreliable",
            negotiated=True,
            id=sctp_id,
            ordered=self._config.ordered,
            maxRetransmits=self._config.max_retransmits,
        )

        @ch.on("open")
        def _on_open() -> None:
            self._has_operator.set()

        @ch.on("message")
        def _on_msg(payload: Any) -> None:
            if isinstance(payload, str):
                payload = payload.encode()
            with self._lock:
                callbacks = [cb for cbs in self._callbacks.values() for cb in cbs]
            for cb in callbacks:
                try:
                    cb(payload, "cmd_unreliable")
                except Exception:
                    logger.exception("Broker subscriber callback error")

        self._cmd_dc = ch

    def _close_cmd_channel(self) -> None:
        self._has_operator.clear()
        if self._cmd_dc is not None:
            with contextlib.suppress(Exception):
                self._cmd_dc.close()
            self._cmd_dc = None

    # ─── Public API (Provider) ───────────────────────────────────────

    def publish(self, topic: str, data: bytes) -> None:
        raise NotImplementedError(
            "The teleop broker does not bridge robot→operator DataChannels yet; "
            "broker-backed transports are receive-only (direction in)."
        )

    def subscribe(self, topic: str, callback: Callable[[bytes, str], None]) -> Callable[[], None]:
        """All subscribers receive all inbound bytes from the multiplexed
        command channel; the transport layer filters by LCM fingerprint."""
        if not self.is_connected:
            self.start()
        with self._lock:
            self._callbacks[topic].append(callback)

        def _unsub() -> None:
            with self._lock:
                try:
                    self._callbacks[topic].remove(callback)
                except ValueError:
                    pass

        return _unsub


__all__ = ["BrokerConfig", "BrokerProvider"]
