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

"""Cross-host camera receiver: pulls JPEG-over-TCP frames from a remote sender.

Pairs with ``dimos.hardware.sensors.camera.tcp_jpeg_sender`` (e.g. running
on the robot). The sender captures a v4l2 device, JPEG-encodes, and ships
``[u32 length][JPEG bytes]`` frames over a TCP socket. This module connects,
decodes, and publishes a regular dimos ``Image`` on ``video``.

Why this instead of GStreamer: zero system deps on either host (no
python-gi / gst-python). Just OpenCV + stdlib socket. Trades a few percent
CPU on the receiver vs. hardware-accelerated decode, but it's the same
proven pattern the X2 camera bridge uses.
"""

from __future__ import annotations

import socket
import struct
import threading
import time
from typing import Any

import cv2
import numpy as np
from pydantic import Field

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class TcpJpegCameraConfig(ModuleConfig):
    host: str = Field(default="127.0.0.1", description="Remote host serving JPEG frames")
    port: int = Field(default=5000, description="Remote port")
    reconnect_interval: float = Field(
        default=2.0, description="Seconds to wait before reconnecting after a drop"
    )
    frame_id: str = Field(default="camera_optical", description="Image header frame_id")


class TcpJpegCameraModule(Module):
    """Connect to a JPEG-over-TCP sender and republish frames as ``Image``.

    Ports:
        video (Out[Image]): decoded BGR frames at whatever the sender ships.
    """

    config: TcpJpegCameraConfig
    video: Out[Image]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._sock: socket.socket | None = None
        self._frame_count = 0

    @rpc
    def start(self) -> None:
        super().start()
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="tcp-jpeg-camera", daemon=True)
        self._thread.start()
        logger.info(
            "TcpJpegCameraModule connecting to %s:%d",
            self.config.host,
            self.config.port,
        )

    @rpc
    def stop(self) -> None:
        self._stop.set()
        sock = self._sock
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                sock.close()
            except OSError:
                pass
            self._sock = None
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        super().stop()

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                self._connect_and_stream()
            except Exception as exc:
                logger.warning(
                    "TcpJpegCameraModule: stream error: %s (reconnect in %.1fs)",
                    exc,
                    self.config.reconnect_interval,
                )
            if self._stop.wait(self.config.reconnect_interval):
                break

    def _connect_and_stream(self) -> None:
        sock = socket.create_connection((self.config.host, self.config.port), timeout=5.0)
        sock.settimeout(None)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._sock = sock
        logger.info(
            "TcpJpegCameraModule connected to %s:%d",
            self.config.host,
            self.config.port,
        )

        try:
            while not self._stop.is_set():
                header = self._recv_exact(sock, 4)
                if header is None:
                    return
                (length,) = struct.unpack("<I", header)
                if length == 0 or length > 50 * 1024 * 1024:
                    raise RuntimeError(f"bad frame length: {length}")
                payload = self._recv_exact(sock, length)
                if payload is None:
                    return

                arr = np.frombuffer(payload, dtype=np.uint8)
                frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                if frame is None:
                    logger.warning("TcpJpegCameraModule: imdecode failed")
                    continue

                image = Image(
                    data=frame,
                    format=ImageFormat.BGR,
                    frame_id=self.config.frame_id,
                    ts=time.time(),
                )
                self.video.publish(image)

                self._frame_count += 1
                if self._frame_count == 1 or self._frame_count % 120 == 0:
                    logger.info(
                        "TcpJpegCameraModule: frame %d (%d KB)",
                        self._frame_count,
                        len(payload) // 1024,
                    )
        finally:
            try:
                sock.close()
            except OSError:
                pass
            self._sock = None

    @staticmethod
    def _recv_exact(sock: socket.socket, n: int) -> bytes | None:
        buf = bytearray()
        while len(buf) < n:
            chunk = sock.recv(n - len(buf))
            if not chunk:
                return None
            buf.extend(chunk)
        return bytes(buf)


class WorkspaceTcpJpegCameraModule(TcpJpegCameraModule):
    """Same behaviour as ``TcpJpegCameraModule``; a distinct class lets the
    module coordinator keep two camera receivers in the same blueprint
    (it keys deployed modules by class, so two of the same class get
    silently collapsed into one)."""


tcp_jpeg_camera = TcpJpegCameraModule.blueprint

__all__ = [
    "TcpJpegCameraConfig",
    "TcpJpegCameraModule",
    "WorkspaceTcpJpegCameraModule",
    "tcp_jpeg_camera",
]
