#!/usr/bin/env python3
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

"""Record blueprint for the RealSense D435i + Mid-360 rig.

Point-LIO odom+lidar and the RealSense color/depth/pointcloud streams are recorded into
a memory2 db, with the rig's mount frames published continuously onto tf. Raw Livox
capture is opt-in: set ``RECORD_PCAP=1`` to also record a .pcap of the Mid-360 UDP
stream. Mirrors the Go2 record blueprint, minus the dog and teleop.

Run it for a timestamped ``recordings/`` folder::

    export LIDAR_IP=192.168.1.107
    uv run python dimos/hardware/sensors/lidar/mid360_realsense_30/mid360_realsense_record.py
"""

from datetime import datetime
import os
from pathlib import Path

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.global_config import global_config
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.hardware.sensors.lidar.livox.module import Mid360
from dimos.hardware.sensors.lidar.mid360_realsense_30.recorder import Mid360RealsenseRecorder
from dimos.hardware.sensors.lidar.mid360_realsense_30.static_transforms import (
    Mid360RealsenseStaticTf,
)
from dimos.hardware.sensors.lidar.pointlio.module import PointLio
from dimos.hardware.sensors.lidar.virtual_mid360.recorder import Mid360PcapRecorder
from dimos.utils.logging_config import set_run_log_dir, setup_logger

logger = setup_logger()

_LIDAR_IP = os.getenv("LIDAR_IP", "192.168.1.107")
# Opt-in raw-Livox pcap capture (default off). Set RECORD_PCAP=1 to include it.
_RECORD_PCAP = os.getenv("RECORD_PCAP", "").lower() in ("1", "true", "yes", "on")

_N_WORKERS = 8


def _default_recording_dir() -> Path:
    # Local time, with the machine's actual zone abbreviation (not a hardcoded PST).
    now = datetime.now().astimezone()
    stamp = (
        now.strftime("%Y-%m-%d") + "_" + now.strftime("%I-%M%p").lower() + "-" + now.strftime("%Z")
    )
    return Path("recordings") / stamp


_modules = [
    RealSenseCamera.blueprint().remappings(
        [
            (RealSenseCamera, "depth_image", "realsense_depth_image"),
            (RealSenseCamera, "pointcloud", "realsense_pointcloud"),
            (RealSenseCamera, "camera_info", "realsense_camera_info"),
            (RealSenseCamera, "depth_camera_info", "realsense_depth_camera_info"),
        ]
    ),
    Mid360.blueprint(lidar_ip=_LIDAR_IP).remappings(
        [
            (Mid360, "lidar", "livox_lidar"),
            (Mid360, "imu", "livox_imu"),
        ]
    ),
    PointLio.blueprint(frame_id="world", lidar_ip=_LIDAR_IP).remappings(
        [
            (PointLio, "lidar", "pointlio_lidar"),
            (PointLio, "odometry", "pointlio_odometry"),
        ]
    ),
    Mid360RealsenseRecorder.blueprint(),
    # Continuously republishes the rig's mount frames onto tf (no latched static tf).
    Mid360RealsenseStaticTf.blueprint(),
]

if _RECORD_PCAP:
    _modules.append(Mid360PcapRecorder.blueprint(lidar_ip=_LIDAR_IP))

mid360_realsense_record = autoconnect(*_modules).global_config(n_workers=_N_WORKERS)


if __name__ == "__main__":
    recording_dir = _default_recording_dir().resolve()
    recording_dir.mkdir(parents=True, exist_ok=True)
    set_run_log_dir(recording_dir)
    global_config.obstacle_avoidance = False
    coordinator = ModuleCoordinator.build(
        mid360_realsense_record,
        {Mid360RealsenseRecorder.name: {"db_path": str(recording_dir / "mem2.db")}},
    )
    coordinator.loop()
