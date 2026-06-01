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

import math
import os
import time

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.module import Module
from dimos.core.stream import In
from dimos.hardware.sensors.lidar.fastlio2.module import FastLio2
from dimos.hardware.sensors.lidar.fastlio2.recorder import FastLio2Recorder
from dimos.hardware.sensors.lidar.livox.module import Mid360
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.robot.unitree.go2.connection import GO2Connection
from dimos.robot.unitree.keyboard_teleop import KeyboardTeleop
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


MPH_PER_MPS = 2.23694
SPEED_LIMIT_MPH = 30.0
_SPEED_STATUS_PRINT_INTERVAL_SEC = 1.0


class SpeedWarner(Module):
    """Watches fastlio_odometry; once speed ever exceeds the limit (impossible for the Go2,
    so it indicates the FastLio2 estimate has diverged / sensor is about to crash),
    latches and spams an error on every subsequent odom message until restart.

    FastLio2's C++ publisher hardcodes twist to zero (cpp/main.cpp), so msg.vx/vy/vz
    are always 0. Speed is derived from pose deltas instead.
    """

    fastlio_odometry: In[Odometry]

    _tripped: bool = False
    _max_mph: float = 0.0
    _last_pos: tuple[float, float, float] | None = None
    _last_ts: float | None = None
    _last_print_ts: float = 0.0

    async def handle_fastlio_odometry(self, msg: Odometry) -> None:
        ts = msg.ts or time.time()
        pos = (msg.pose.x, msg.pose.y, msg.pose.z)
        last_pos, last_ts = self._last_pos, self._last_ts
        self._last_pos, self._last_ts = pos, ts
        if last_pos is None or last_ts is None:
            return
        dt = ts - last_ts
        if dt <= 0:
            return
        dx, dy, dz = pos[0] - last_pos[0], pos[1] - last_pos[1], pos[2] - last_pos[2]
        speed_mph = math.sqrt(dx * dx + dy * dy + dz * dz) / dt * MPH_PER_MPS
        if speed_mph > self._max_mph:
            self._max_mph = speed_mph
        if ts - self._last_print_ts >= _SPEED_STATUS_PRINT_INTERVAL_SEC:
            self._last_print_ts = ts
            print(
                f"\rspeed: {speed_mph:6.2f} mph  max: {self._max_mph:6.2f} mph ",
                end="",
                flush=True,
            )
        if not self._tripped and speed_mph > SPEED_LIMIT_MPH:
            self._tripped = True
            logger.error(
                f"!!! FASTLIO ODOMETRY DIVERGED !!! reported {speed_mph:.1f} mph "
                f"(limit {SPEED_LIMIT_MPH:.1f} mph). Latching warnings."
            )


_LIDAR_IP = os.getenv("LIDAR_IP", "192.168.1.107")


unitree_go2_record = autoconnect(
    GO2Connection.blueprint(),
    KeyboardTeleop.blueprint(),
    MovementManager.blueprint(),
    Mid360.blueprint(
        lidar_ip=_LIDAR_IP,
    ).remappings(
        [
            (Mid360, "lidar", "livox_lidar"),
            (Mid360, "imu", "livox_imu"),
        ]
    ),
    FastLio2.blueprint(
        frame_id="world",
        map_freq=-1,
        lidar_ip=_LIDAR_IP,
        max_velocity_norm_ms=3.1,
    ).remappings(
        [
            (FastLio2, "lidar", "fastlio_lidar"),
            (FastLio2, "odometry", "fastlio_odometry"),
        ]
    ),
    FastLio2Recorder.blueprint(lidar_ip=_LIDAR_IP, record_pcap=True),
    SpeedWarner.blueprint(),
).global_config(n_workers=10, robot_model="unitree_go2")
