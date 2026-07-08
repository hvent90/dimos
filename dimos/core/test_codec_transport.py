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

from __future__ import annotations

import pickle
import statistics
import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest

from dimos.core.transport import CodecTransport, LCMTransport, ZenohTransport
from dimos.msgs.sensor_msgs.CompressedImage import CompressedImage
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.protocol.service.zenohservice import ZenohSessionPool


def make_image(width: int = 1280, height: int = 720) -> Image:
    """720p-ish frame: gradient + noise, roughly what a real camera compresses like."""
    rng = np.random.RandomState(7)
    gradient = np.broadcast_to(np.linspace(0, 255, width, dtype=np.uint8), (height, width))
    noise = rng.randint(0, 60, (height, width), dtype=np.uint8)
    data = np.stack([gradient, np.minimum(gradient, 128) + noise // 2, noise], axis=-1)
    return Image(data=data, format=ImageFormat.RGB, frame_id="cam", ts=42.125)


@pytest.fixture()
def collector():
    received = []
    event = threading.Event()

    def callback(msg):
        received.append(msg)
        event.set()

    return SimpleNamespace(received=received, event=event, callback=callback)


@pytest.fixture()
def session_pool():
    pool = ZenohSessionPool()
    yield pool
    pool.close_all()


def test_roundtrip_over_lcm(retry_until, collector) -> None:
    t = CodecTransport(LCMTransport("dimos/test/codec_lcm", CompressedImage))
    t.subscribe(collector.callback)
    src = make_image(320, 240)
    retry_until(collector.event, lambda: t.broadcast(None, src))
    img = collector.received[0]
    assert isinstance(img, Image)
    assert img.frame_id == "cam"
    assert abs(img.ts - src.ts) < 1e-6
    assert img.shape == src.shape
    t.stop()


def test_roundtrip_over_zenoh(retry_until, collector, session_pool) -> None:
    t = CodecTransport(
        ZenohTransport("dimos/test/codec_zenoh", CompressedImage, session_pool=session_pool)
    )
    t.subscribe(collector.callback)
    src = make_image(320, 240)
    retry_until(collector.event, lambda: t.broadcast(None, src))
    img = collector.received[0]
    assert isinstance(img, Image)
    assert abs(img.ts - src.ts) < 1e-6
    t.stop()


def test_decode_false_delivers_compressed(retry_until, collector) -> None:
    t = CodecTransport(LCMTransport("dimos/test/codec_raw", CompressedImage), decode=False)
    t.subscribe(collector.callback)
    retry_until(collector.event, lambda: t.broadcast(None, make_image(320, 240)))
    msg = collector.received[0]
    assert isinstance(msg, CompressedImage)
    assert msg.format == "jpeg"
    t.stop()


def test_compressed_passthrough_no_recompression(retry_until, collector) -> None:
    t = CodecTransport(LCMTransport("dimos/test/codec_pass", CompressedImage), decode=False)
    t.subscribe(collector.callback)
    ci = CompressedImage.from_image(make_image(320, 240), quality=30)
    retry_until(collector.event, lambda: t.broadcast(None, ci))
    assert collector.received[0].data == ci.data
    t.stop()


def test_double_wrap_raises() -> None:
    inner = CodecTransport(LCMTransport("dimos/test/codec_dw", CompressedImage))
    with pytest.raises(ValueError, match="cannot wrap"):
        CodecTransport(inner)


def test_pickle_roundtrip() -> None:
    t = CodecTransport(
        LCMTransport("dimos/test/codec_pickle", CompressedImage), quality=50, max_width=640
    )
    t2 = pickle.loads(pickle.dumps(t))
    assert isinstance(t2, CodecTransport)
    assert t2.quality == 50
    assert t2.max_width == 640
    assert t2.topic.topic == t.topic.topic


def _bench(transport, frame: Image, n: int, wire_bytes: int) -> dict | None:
    """Round-trip n frames one at a time; per-frame latency includes encode+wire+decode.

    Large raw frames fragment over UDP and can genuinely drop — drops are
    counted and reported, not failed on. Returns None when nothing ever
    arrives (raw 720p over LCM can be entirely undeliverable).
    """
    received = []
    event = threading.Event()

    def callback(msg):
        received.append(msg)
        event.set()

    latencies = []
    drops = 0
    transport.subscribe(callback)
    try:
        # warmup (first delivery also proves subscription is live)
        deadline = time.monotonic() + 10
        while not event.is_set() and time.monotonic() < deadline:
            transport.broadcast(None, frame)
            event.wait(0.2)
        if not event.is_set():
            return None

        for _ in range(n):
            count = len(received)
            start = time.perf_counter()
            transport.broadcast(None, frame)
            deadline = time.monotonic() + 2
            while len(received) <= count and time.monotonic() < deadline:
                time.sleep(0.0005)
            if len(received) > count:
                latencies.append(time.perf_counter() - start)
            else:
                drops += 1
    finally:
        transport.stop()
    if not latencies:
        return None
    lat = statistics.median(latencies)
    return {"latency_ms": lat * 1000, "wire_bytes": wire_bytes, "fps": 1 / lat, "drops": drops}


@pytest.mark.parametrize("proto", ["lcm", "zenoh"])
def test_benchmark_image_vs_compressed(proto, session_pool) -> None:
    """Old path (raw Image on the wire) vs CodecTransport(CompressedImage), same transport."""
    frame = make_image()  # 720p, 2.76 MB raw
    n = 15

    def inner(topic: str, typ: type):
        if proto == "lcm":
            return LCMTransport(topic, typ)
        return ZenohTransport(topic, typ, session_pool=session_pool)

    raw_wire = len(frame.lcm_encode())
    jpeg_wire = len(CompressedImage.from_image(frame).lcm_encode())

    raw = _bench(inner(f"dimos/bench/{proto}_raw", Image), frame, n, raw_wire)
    codec = _bench(
        CodecTransport(inner(f"dimos/bench/{proto}_jpeg", CompressedImage)), frame, n, jpeg_wire
    )

    print(f"\n{proto} 720p Image, median of {n} round-trips (encode+wire+decode):")
    for name, r in (("raw Image", raw), ("CodecTransport jpeg", codec)):
        if r is None:
            print(f"  {name:<20} UNDELIVERABLE (every frame lost)")
        else:
            print(
                f"  {name:<20} {r['wire_bytes'] / 1e6:7.2f} MB/frame"
                f"  {r['latency_ms']:7.2f} ms/frame  {r['fps']:6.1f} fps"
                f"  drops {r['drops']}/{n}"
            )
    print(f"  wire reduction: {raw_wire / jpeg_wire:.1f}x")

    assert codec is not None and codec["drops"] == 0, "codec path must deliver every frame"
    assert jpeg_wire < raw_wire * 0.15, "JPEG should cut wire size by >85%"
