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

"""Cloudflare Realtime integration smoke tests.

These talk to the live CF SFU and are skipped unless ``CF_TELEOP_APP_ID``
and ``CF_TELEOP_APP_SECRET`` are set. Throughput/latency numbers come from
the standard harness instead::

    pytest -m tool dimos/protocol/pubsub/benchmark/test_benchmark.py -k webrtc

Run with: ``pytest -m tool dimos/protocol/pubsub/impl/webrtc/test_webrtcpubsub.py``
"""

from __future__ import annotations

from collections.abc import Iterator
import os
import threading
import time

import pytest

from dimos.protocol.pubsub.impl.webrtc.providers.spec import WEBRTC_AVAILABLE
from dimos.protocol.pubsub.impl.webrtc.webrtcpubsub import WebRTCPubSub

CF_CREDS_PRESENT = bool(os.environ.get("CF_TELEOP_APP_ID")) and bool(
    os.environ.get("CF_TELEOP_APP_SECRET")
)

skip_unless_cf = pytest.mark.skipif(
    not (WEBRTC_AVAILABLE and CF_CREDS_PRESENT),
    reason="Requires aiortc + CF_TELEOP_APP_ID/CF_TELEOP_APP_SECRET",
)


@pytest.fixture
def pubsub() -> Iterator[WebRTCPubSub]:
    from dimos.protocol.pubsub.impl.webrtc.providers.cloudflare import (
        CloudflareConfig,
        CloudflareProvider,
    )

    ps = WebRTCPubSub(provider=CloudflareProvider(CloudflareConfig()))
    ps.start()
    try:
        yield ps
    finally:
        ps.stop()


@pytest.mark.tool
@skip_unless_cf
@pytest.mark.timeout(60)
def test_basic_pub_sub(pubsub: WebRTCPubSub) -> None:
    """Send a single message through the CF SFU and verify it is received."""
    received: list[tuple[bytes, str]] = []
    done = threading.Event()

    def cb(msg: bytes, topic: str) -> None:
        received.append((msg, topic))
        done.set()

    unsub = pubsub.subscribe("test_basic", cb)
    try:
        time.sleep(0.2)  # let the subscribe-side DataChannel settle
        pubsub.publish("test_basic", b"hello world")
        assert done.wait(timeout=10.0), "Did not receive published message"
        assert received[0] == (b"hello world", "test_basic")
    finally:
        unsub()


@pytest.mark.tool
@skip_unless_cf
@pytest.mark.timeout(60)
def test_loopback_rtt(pubsub: WebRTCPubSub) -> None:
    """Same-host loopback through the nearest CF PoP.

    Measures transport-stack overhead (pub PC → CF edge → sub PC, both
    local), NOT operator→robot teleop latency — that depends on the
    operator's network. Sanity bound only; real numbers come from the
    benchmark harness.
    """
    n = 30
    durations: list[float] = []
    received = threading.Event()
    sent_at = [0.0]

    def cb(_msg: bytes, _topic: str) -> None:
        durations.append(time.perf_counter() - sent_at[0])
        received.set()

    unsub = pubsub.subscribe("test_rtt", cb)
    try:
        time.sleep(0.3)
        for i in range(n):
            received.clear()
            sent_at[0] = time.perf_counter()
            pubsub.publish("test_rtt", f"ping-{i}".encode())
            assert received.wait(timeout=5.0), f"Timed out on ping {i}"

        med = sorted(durations)[n // 2]
        assert med < 1.0, f"Median loopback RTT too high: {med * 1000:.0f} ms"
        print(f"\n  WebRTC loopback RTT via CF edge: median {med * 1000:.1f} ms (n={n})")
    finally:
        unsub()
