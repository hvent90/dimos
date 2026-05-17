#!/usr/bin/env python3

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

"""Minimal v4l2 → JPEG → TCP sender for cross-host camera streaming.

Listens on a TCP port, accepts one client at a time, and streams JPEG-encoded
frames as ``[u32 little-endian length][JPEG bytes]``. The receiver side is
``TcpJpegCameraModule`` in dimos which decodes back to a regular ``Image``.

Why this exists instead of the GStreamer sender: zero system-package
dependencies (no python-gi). Just OpenCV + stdlib. Easier to run on a
fresh robot image that might not have the GStreamer Python bindings.

Usage::

    python -m dimos.hardware.sensors.camera.tcp_jpeg_sender \\
        --device /dev/video6 --host 0.0.0.0 --port 5000 \\
        --width 1280 --height 720 --fps 30 --quality 80
"""

from __future__ import annotations

import argparse
import logging
import signal
import socket
import struct
import sys
import threading
import time

import cv2

logger = logging.getLogger("tcp_jpeg_sender")


def serve(
    device: str,
    host: str,
    port: int,
    width: int,
    height: int,
    fps: float,
    quality: int,
) -> None:
    cap_index: int | str = device
    if device.isdigit():
        cap_index = int(device)

    # Force the v4l2 backend. The autobackend tries GStreamer first, which
    # often fails on robot images without the gst plugins for the camera,
    # and then falls through to a v4l2 capture that opens but never reads.
    cap = cv2.VideoCapture(cap_index, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise RuntimeError(f"Failed to open {device} with v4l2 backend")
    # Request MJPG fourcc before resolution/fps. Most USB webcams (C920e,
    # Logitech B-series, etc.) only deliver high resolutions / framerates
    # in MJPG; YUYV is bandwidth-limited and may negotiate down to 10 fps.
    fourcc = cv2.VideoWriter_fourcc(*"MJPG")
    cap.set(cv2.CAP_PROP_FOURCC, fourcc)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    actual_fps = cap.get(cv2.CAP_PROP_FPS)
    logger.info(
        "camera %s opened: %dx%d @ %.1f fps (requested %dx%d@%g)",
        device, actual_w, actual_h, actual_fps, width, height, fps,
    )

    encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)]
    period = 0.0 if fps <= 0 else 1.0 / fps

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((host, port))
    server.listen(1)
    logger.info("listening on %s:%d", host, port)

    stop = threading.Event()

    def _on_signal(_sig: int, _frm: object) -> None:
        logger.info("shutting down")
        stop.set()
        try:
            server.close()
        except Exception:
            pass

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    while not stop.is_set():
        try:
            client, addr = server.accept()
        except OSError:
            break
        logger.info("client connected: %s", addr)
        client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

        frame_count = 0
        next_t = time.monotonic()
        try:
            while not stop.is_set():
                ok, frame = cap.read()
                if not ok:
                    logger.warning("camera read failed; retrying")
                    time.sleep(0.05)
                    continue
                ok, buf = cv2.imencode(".jpg", frame, encode_params)
                if not ok:
                    continue
                payload = buf.tobytes()
                try:
                    client.sendall(struct.pack("<I", len(payload)) + payload)
                except (BrokenPipeError, ConnectionResetError, OSError) as exc:
                    logger.info("client disconnected: %s", exc)
                    break
                frame_count += 1
                if frame_count == 1 or frame_count % 100 == 0:
                    logger.info("sent %d frames (%d KB last)", frame_count, len(payload) // 1024)
                if period > 0:
                    next_t += period
                    delay = next_t - time.monotonic()
                    if delay > 0:
                        time.sleep(delay)
                    else:
                        next_t = time.monotonic()
        finally:
            try:
                client.close()
            except Exception:
                pass

    cap.release()
    try:
        server.close()
    except Exception:
        pass


def main() -> None:
    parser = argparse.ArgumentParser(description="JPEG-over-TCP camera sender")
    parser.add_argument("--device", default="/dev/video0", help="v4l2 device path or numeric index")
    parser.add_argument("--host", default="0.0.0.0", help="bind host (default: 0.0.0.0)")
    parser.add_argument("--port", type=int, default=5000, help="bind port (default: 5000)")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--quality", type=int, default=80, help="JPEG quality 1..100")
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
    )

    try:
        serve(
            device=args.device,
            host=args.host,
            port=args.port,
            width=args.width,
            height=args.height,
            fps=args.fps,
            quality=args.quality,
        )
    except Exception as exc:
        logger.error("fatal: %s", exc)
        sys.exit(1)


if __name__ == "__main__":
    main()
