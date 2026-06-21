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

from datetime import datetime
import os
from pathlib import Path
from typing import Any

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.global_config import global_config
from dimos.core.stream import In
from dimos.hardware.sensors.lidar.fastlio2.module import FastLio2
from dimos.hardware.sensors.lidar.fastlio2.recorder import FastLio2Recorder
from dimos.hardware.sensors.lidar.fastlio2.speed_warner import SpeedWarner
from dimos.hardware.sensors.lidar.livox.module import Mid360
from dimos.mapping.recording.go2_mid360.static_transforms import (
    BASE_TO_CAMERA_OPTICAL,
    MID360_TO_BASE,
)
from dimos.memory2.module import pose_setter_for
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.robot.unitree.go2.connection import GO2Connection
from dimos.robot.unitree.keyboard_teleop import KeyboardTeleop
from dimos.utils.logging_config import set_run_log_dir, setup_logger

logger = setup_logger()

_LIDAR_IP = os.getenv("LIDAR_IP", "192.168.1.171")
_LIDAR_HOST_IP = os.getenv("LIDAR_HOST_IP", "192.168.1.100")


def _default_recording_dir() -> Path:
    now = datetime.now()
    stamp = now.strftime("%Y-%m-%d") + "_" + now.strftime("%I-%M%p").lower() + "-PST"
    return Path("recordings") / stamp


class Go2TfHackRecorder(FastLio2Recorder):
    """Records with statically-applied transforms instead of querying tf.

    FastLio2 tracks the Mid-360 (``mid360_link``) and reports its pose in the
    ``world`` frame as ``fastlio_odometry``; its registered cloud is likewise
    already in that world frame. We anchor recorded observations to the robot
    body, building every pose from the latest fastlio odom and fixed mounts:

    - ``fastlio_lidar`` -> ``base_link`` pose in world (odom, then mid360_link -> base_link)
    - ``color_image``   -> ``camera_optical`` pose in world (odom, mid360_link -> base_link,
      then base_link -> camera_optical)
    - everything else (odom streams, imu) -> tf fallback / no pose
    """

    fastlio_lidar: In[PointCloud2]
    fastlio_odometry: In[Odometry]
    go2_lidar: In[PointCloud2]
    go2_odom: In[PoseStamped]
    color_image: In[Image]
    zed_color_image: In[Image]
    zed_imu: In[Imu]
    livox_lidar: In[PointCloud2]
    livox_imu: In[Imu]

    _latest_fastlio_odom: Odometry | None = None

    @pose_setter_for("fastlio_odometry")
    def _odom_pose(self, msg: Odometry) -> Pose | None:
        self._latest_fastlio_odom = msg
        world_to_base = self._world_to_base_from_fastlio()
        return world_to_base.to_pose() if world_to_base is not None else None

    @pose_setter_for("fastlio_lidar")
    def _lidar_pose(self, msg: PointCloud2) -> Pose | None:
        world_to_base = self._world_to_base_from_fastlio()
        return world_to_base.to_pose() if world_to_base is not None else None

    @pose_setter_for("color_image", "zed_color_image")
    def _image_pose(self, msg: Image) -> Pose | None:
        world_to_base = self._world_to_base_from_fastlio()
        if world_to_base is None:
            return None
        return (world_to_base + BASE_TO_CAMERA_OPTICAL).to_pose()

    @pose_setter_for("go2_odom")
    def _go2_odom_pose(self, msg: PoseStamped) -> Pose | None:
        return msg

    def _world_to_base_from_fastlio(self) -> Transform | None:
        odom = self._latest_fastlio_odom
        if odom is None:
            return None
        world_to_mid360 = Transform(
            translation=odom.position,
            rotation=odom.orientation,
            frame_id="world",
            child_frame_id="mid360_link",
            ts=odom.ts,
        )
        return world_to_mid360 + MID360_TO_BASE


def _zed_camera_blueprint() -> Any:
    """ZED color source, remapped to ``zed_color_image``.

    Prefer the SDK-backed ``ZEDCamera`` (depth/imu/pointcloud); fall back to the
    UVC-only ``ZedSimple`` (color only) when ``pyzed`` is not installed.
    """
    try:
        import pyzed.sl  # noqa: F401

        from dimos.hardware.sensors.camera.zed.camera import ZEDCamera

        return ZEDCamera.blueprint(enable_depth=False, enable_pointcloud=False).remappings(
            [
                (ZEDCamera, "color_image", "zed_color_image"),
                (ZEDCamera, "imu", "zed_imu"),
            ]
        )
    except ImportError:
        from dimos.hardware.sensors.camera.zed.simple import ZedSimple

        return ZedSimple.blueprint().remappings(
            [
                (ZedSimple, "color_image", "zed_color_image"),
                (ZedSimple, "imu", "zed_imu"),
            ]
        )


unitree_go2_record = autoconnect(
    _zed_camera_blueprint(),
    MovementManager.blueprint(),
    GO2Connection.blueprint().remappings(
        [
            (GO2Connection, "lidar", "go2_lidar"),
            (GO2Connection, "odom", "go2_odom"),
        ]
    ),
    Mid360.blueprint(
        lidar_ip=_LIDAR_IP,
        host_ip=_LIDAR_HOST_IP,
    ).remappings(
        [
            (Mid360, "lidar", "livox_lidar"),
            (Mid360, "imu", "livox_imu"),
        ]
    ),
    FastLio2.blueprint(
        frame_id="world",
        lidar_ip=_LIDAR_IP,
    ).remappings(
        [
            (FastLio2, "lidar", "fastlio_lidar"),
            (FastLio2, "odometry", "fastlio_odometry"),
        ]
    ),
    Go2TfHackRecorder.blueprint(),
    SpeedWarner.blueprint().remappings(
        [
            (SpeedWarner, "odometry", "fastlio_odometry"),
        ]
    ),
    # Pygame keyboard teleop (WASD + Q/E, Z=lie down, X=stand). Its cmd_vel
    # feeds MovementManager's tele_cmd_vel; sit/stand are handled internally
    # via the auto-wired GO2ConnectionSpec.
    KeyboardTeleop.blueprint(linear_speed=0.3, angular_speed=0.6).remappings(
        [
            (KeyboardTeleop, "cmd_vel", "tele_cmd_vel"),
        ]
    ),
).global_config(n_workers=10, robot_model="unitree_go2")


if __name__ == "__main__":
    recording_dir = _default_recording_dir().resolve()
    recording_dir.mkdir(parents=True, exist_ok=True)
    set_run_log_dir(recording_dir)
    global_config.obstacle_avoidance = False
    coordinator = ModuleCoordinator.build(
        unitree_go2_record,
        {Go2TfHackRecorder.name: {"db_path": str(recording_dir / "mem2.db")}},
    )
    coordinator.loop()
