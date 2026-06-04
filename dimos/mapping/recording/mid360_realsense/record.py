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

import os
import time
from typing import Any

from reactivex.disposable import Disposable

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.global_config import global_config
from dimos.core.stream import In
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.hardware.sensors.lidar.fastlio2.module import FastLio2
from dimos.hardware.sensors.lidar.fastlio2.recorder import FastLio2Recorder, _default_recording_dir
from dimos.hardware.sensors.lidar.livox.module import Mid360
from dimos.mapping.recording.mid360_realsense.static_transforms import (
    MID360_TO_WORLD,
    REALSENSE_COLOR_OPTICAL_FRAME_TO_MID360_IMU_FRAME,
)
from dimos.memory2.stream import Stream
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.logging_config import set_run_log_dir, setup_logger

logger = setup_logger()

# FAST-LIO odom is the Mid-360 IMU frame; map it to the RealSense color optical
# frame. See static_transforms.py for the geometry and sources.
MID360_TO_CAMERA_OPTICAL = REALSENSE_COLOR_OPTICAL_FRAME_TO_MID360_IMU_FRAME.inverse()

# FAST-LIO reports the Mid-360 pose in its own start frame. Re-anchor that onto a flat world
# frame whose origin is the camera screw and whose axes are level (the camera is angled up).
# Assumes FAST-LIO initializes at the rig's starting pose.
WORLD_TO_MID360 = MID360_TO_WORLD.inverse()


_LIDAR_IP = os.getenv("LIDAR_IP", "192.168.1.107")


class TfHackRecorder(FastLio2Recorder):
    """Records with statically-applied transforms instead of querying tf.

    FastLio2 tracks the Mid-360 (``mid360_link``) and reports its pose in the
    ``world`` frame as ``fastlio_odometry``; its registered cloud is likewise
    already in that world frame. Recorded observations are anchored from the
    latest fastlio odom and the fixed camera mount:

    - ``fastlio_lidar`` / ``fastlio_odometry`` -> ``mid360_link`` pose in world
    - ``color_image`` / ``realsense_depth_image`` / ``realsense_pointcloud``
      -> ``camera_optical`` pose in world
    - everything else (odom, camera_info, imu) -> no pose
    """

    fastlio_lidar: In[PointCloud2]
    fastlio_odometry: In[Odometry]
    color_image: In[Image]
    realsense_depth_image: In[Image]
    realsense_pointcloud: In[PointCloud2]
    realsense_camera_info: In[CameraInfo]
    realsense_depth_camera_info: In[CameraInfo]
    realsense_imu: In[Imu]
    livox_lidar: In[PointCloud2]
    livox_imu: In[Imu]
    # Shadow the parent's generic companion ports so they're not recorded as
    # empty `lidar`/`odom` streams; the prefixed ports above take their place.
    lidar: None = None  # type: ignore[assignment]
    odom: None = None  # type: ignore[assignment]

    _latest_fastlio_odom: Odometry | None = None
    _warning_names: set[str] = set()

    def _port_to_stream(self, name: str, input_topic: In[Any], stream: Stream[Any]) -> None:
        def on_msg(msg: Any) -> None:
            ts = time.time()
            pose = None
            if name == "fastlio_odometry":
                self._latest_fastlio_odom = msg
                world_to_mid360 = self._world_to_mid360_from_fastlio()
                if world_to_mid360 is not None:
                    pose = world_to_mid360.to_pose()
            elif name == "fastlio_lidar":
                world_to_mid360 = self._world_to_mid360_from_fastlio()
                if world_to_mid360 is not None:
                    pose = world_to_mid360.to_pose()
            elif name in ("color_image", "realsense_depth_image", "realsense_pointcloud"):
                world_to_mid360 = self._world_to_mid360_from_fastlio()
                if world_to_mid360 is not None:
                    pose = (world_to_mid360 + MID360_TO_CAMERA_OPTICAL).to_pose()
            elif name == "go2_odom" or name == "odom":
                pose = msg
            elif "odom" in name or "camera_info" in name or "imu" in name:
                pass
            else:
                if name not in self._warning_names:
                    self._warning_names.add(name)
                    logger.warning(f"cannot compute pose for {name}; recording without pose")

            stream.append(msg, ts=ts, pose=pose)

        self.register_disposable(Disposable(input_topic.subscribe(on_msg)))

    def _world_to_mid360_from_fastlio(self) -> Transform | None:
        odom = self._latest_fastlio_odom
        if odom is None:
            return None
        fastlio_pose = Transform(
            translation=odom.position,
            rotation=odom.orientation,
            frame_id="fastlio_world",
            child_frame_id="mid360_link",
            ts=odom.ts,
        )
        world_to_mid360 = WORLD_TO_MID360 + fastlio_pose
        world_to_mid360.frame_id = "world"
        world_to_mid360.child_frame_id = "mid360_link"
        world_to_mid360.ts = odom.ts
        return world_to_mid360


realsense_mid360_record = autoconnect(
    RealSenseCamera.blueprint().remappings(
        [
            (RealSenseCamera, "depth_image", "realsense_depth_image"),
            (RealSenseCamera, "pointcloud", "realsense_pointcloud"),
            (RealSenseCamera, "camera_info", "realsense_camera_info"),
            (RealSenseCamera, "depth_camera_info", "realsense_depth_camera_info"),
            (RealSenseCamera, "imu", "realsense_imu"),
        ]
    ),
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
    ).remappings(
        [
            (FastLio2, "lidar", "fastlio_lidar"),
            (FastLio2, "odometry", "fastlio_odometry"),
        ]
    ),
    TfHackRecorder.blueprint(lidar_ip=_LIDAR_IP, record_pcap=True),
).global_config(n_workers=6)


if __name__ == "__main__":
    recording_dir = _default_recording_dir().resolve()
    recording_dir.mkdir(parents=True, exist_ok=True)
    set_run_log_dir(recording_dir)
    global_config.obstacle_avoidance = False
    coordinator = ModuleCoordinator.build(
        realsense_mid360_record,
        {TfHackRecorder.name: {"recording_dir": recording_dir}},
    )
    coordinator.loop()
