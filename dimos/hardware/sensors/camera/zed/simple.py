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

import asyncio
from collections.abc import AsyncIterator
import glob
import math
import os
import struct
import time

import cv2

from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# ZED-M IMU over USB HID — protocol from Stereolabs' open-source zed-open-capture.
_ZED_USB_VENDOR = 0x2B03
_ZED_M_MCU_PID = 0xF681
_REP_ID_SENSOR_STREAM_STATUS = 0x32
# Packed RawData struct (sensorcapture_def.hpp); we only read the IMU fields.
_RAW_DATA_FMT = "<BBQhhhhhhBBIhBhhhBIBIBhIIhh"
_RAW_DATA_SIZE = struct.calcsize(_RAW_DATA_FMT)
_DEFAULT_GRAVITY = 9.8189
_ACC_SCALE = _DEFAULT_GRAVITY * (8.0 / 32768.0)  # raw -> m/s^2 (+-8g range)
_GYRO_SCALE = (1000.0 / 32768.0) * (math.pi / 180.0)  # raw -> rad/s (+-1000 deg/s)


def autodetect_zed_device() -> str | None:
    """Resolve the ZED's UVC ``/dev/video*`` node via its v4l by-id symlink."""
    for link in sorted(glob.glob("/dev/v4l/by-id/*")):
        if "ZED" in os.path.basename(link).upper():
            return os.path.realpath(link)
    return None


class ZedSimpleConfig(ModuleConfig):
    device: str | None = None
    # Full side-by-side frame; the ZED is YUYV-only. 2560x720 => 1280x720 per eye.
    width: int = 2560
    height: int = 720
    fps: int = 15
    side: str = "left"
    fourcc: str = "YUYV"
    enable_imu: bool = True
    imu_pid: int = _ZED_M_MCU_PID
    camera_name: str = "zed"


class ZedSimple(Module):
    """SDK-free ZED color + IMU capture.

    Fallback for when the ZED SDK / ``pyzed`` is not installed. Color comes from
    the UVC stereo stream (one side-by-side frame; we slice out one eye and
    publish ``color_image``). The IMU is read straight off the camera's USB-HID
    device using Stereolabs' open ``zed-open-capture`` protocol and published as
    ``imu``. No depth or pointcloud — use the SDK-backed ``ZEDCamera`` for those.

    Color (UVC) and IMU (HID) are independent USB interfaces, so each runs and
    fails independently: a missing camera does not stop the IMU and vice versa.
    """

    config: ZedSimpleConfig
    color_image: Out[Image]
    imu: Out[Imu]

    def __init__(self, *args, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(*args, **kwargs)
        self._capture: cv2.VideoCapture | None = None
        self._running = False
        self._video_task: asyncio.Task[None] | None = None
        self._imu_task: asyncio.Task[None] | None = None

    @property
    def _optical_frame(self) -> str:
        return f"{self.config.camera_name}_{self.config.side}_color_optical_frame"

    @property
    def _imu_frame(self) -> str:
        return f"{self.config.camera_name}_imu_link"

    def _open(self) -> cv2.VideoCapture:
        device = self.config.device or autodetect_zed_device()
        if device is None:
            raise RuntimeError(
                "No ZED UVC video device found under /dev/v4l/by-id. Confirm the "
                "camera is on a USB-3 port, or set ZedSimpleConfig.device explicitly."
            )
        capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not capture.isOpened():
            raise RuntimeError(f"Failed to open ZED video device: {device}")
        if self.config.fourcc:
            capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc(*self.config.fourcc))
        if self.config.width:
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.config.width)
        if self.config.height:
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config.height)
        if self.config.fps:
            capture.set(cv2.CAP_PROP_FPS, self.config.fps)
        return capture

    async def main(self) -> AsyncIterator[None]:
        self._running = True

        # IMU (USB-HID) runs independently of the camera (UVC).
        if self.config.enable_imu:
            self._imu_task = asyncio.create_task(asyncio.to_thread(self._imu_loop))

        # Fail soft: a missing/flaky camera must not abort a multi-sensor
        # recording (nor stop the IMU). Log loudly and run without frames.
        try:
            self._capture = await asyncio.to_thread(self._open)
        except RuntimeError as error:
            logger.error(f"ZedSimple: camera unavailable, no color_image will publish: {error}")
        if self._capture is not None:
            width = int(self._capture.get(cv2.CAP_PROP_FRAME_WIDTH))
            height = int(self._capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
            logger.info(
                f"ZedSimple: streaming {width}x{height} side-by-side, publishing {self.config.side} eye"
            )
            self._video_task = asyncio.create_task(asyncio.to_thread(self._capture_loop))

        yield

        self._running = False
        for task in (self._video_task, self._imu_task):
            if task is not None:
                await task
        if self._capture is not None:
            self._capture.release()
            self._capture = None

    def _capture_loop(self) -> None:
        if self._capture is None:
            return
        while self._running:
            ok, frame = self._capture.read()
            if not ok:
                time.sleep(0.01)
                continue
            half_width = frame.shape[1] // 2
            eye = frame[:, :half_width] if self.config.side == "left" else frame[:, half_width:]
            self.color_image.publish(
                Image(
                    data=cv2.cvtColor(eye, cv2.COLOR_BGR2RGB),
                    format=ImageFormat.RGB,
                    frame_id=self._optical_frame,
                    ts=time.time(),
                )
            )

    def _imu_loop(self) -> None:
        try:
            import hid
        except ImportError:
            logger.error("ZedSimple: `hid` not installed, no IMU will publish (pip install hid)")
            return

        try:
            device = hid.Device(_ZED_USB_VENDOR, self.config.imu_pid)
        except Exception as error:
            logger.error(f"ZedSimple: IMU HID device unavailable, no imu will publish: {error}")
            return

        try:
            device.send_feature_report(bytes([_REP_ID_SENSOR_STREAM_STATUS, 0x01]))
            # ROS convention: orientation unknown (raw IMU, no fusion) -> covariance[0] = -1.
            unknown_orientation_covariance = [-1.0] + [0.0] * 8
            while self._running:
                report = device.read(_RAW_DATA_SIZE, timeout=200)
                if not report or len(report) < _RAW_DATA_SIZE:
                    continue
                fields = struct.unpack(_RAW_DATA_FMT, bytes(report[:_RAW_DATA_SIZE]))
                imu_not_valid = fields[1]
                if imu_not_valid:
                    continue
                # fields: struct_id, imu_not_valid, timestamp, gX,gY,gZ, aX,aY,aZ, ...
                gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z = fields[3:9]
                self.imu.publish(
                    Imu(
                        angular_velocity=Vector3(
                            gyro_x * _GYRO_SCALE, gyro_y * _GYRO_SCALE, gyro_z * _GYRO_SCALE
                        ),
                        linear_acceleration=Vector3(
                            acc_x * _ACC_SCALE, acc_y * _ACC_SCALE, acc_z * _ACC_SCALE
                        ),
                        orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
                        orientation_covariance=unknown_orientation_covariance,
                        frame_id=self._imu_frame,
                        ts=time.time(),
                    )
                )
        finally:
            try:
                device.send_feature_report(bytes([_REP_ID_SENSOR_STREAM_STATUS, 0x00]))
            except Exception:
                pass
            device.close()
