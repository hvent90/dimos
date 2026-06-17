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

"""Python NativeModule wrapper for the FAST-LIO2 + Livox Mid-360 binary.

Binds Livox SDK2 into FAST-LIO-NON-ROS for real-time LiDAR SLAM; outputs
registered (world-frame) point clouds and odometry with covariance.
"""

from __future__ import annotations

from pathlib import Path
import time
from typing import TYPE_CHECKING, Annotated

from pydantic.experimental.pipeline import validate_as
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import Out
from dimos.hardware.sensors.lidar.livox.net import resolve_host_ip
from dimos.hardware.sensors.lidar.livox.ports import (
    SDK_CMD_DATA_PORT,
    SDK_HOST_CMD_DATA_PORT,
    SDK_HOST_IMU_DATA_PORT,
    SDK_HOST_LOG_DATA_PORT,
    SDK_HOST_POINT_DATA_PORT,
    SDK_HOST_PUSH_MSG_PORT,
    SDK_IMU_DATA_PORT,
    SDK_LOG_DATA_PORT,
    SDK_POINT_DATA_PORT,
    SDK_PUSH_MSG_PORT,
)
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.nav_stack.frames import FRAME_BODY, FRAME_ODOM
from dimos.spec import perception

_CONFIG_DIR = Path(__file__).parent / "config"


class FastLio2Config(NativeModuleConfig):
    cwd: str | None = "cpp"
    executable: str = "result/bin/fastlio2_native"
    build_command: str | None = "nix build .#fastlio2_native"
    # Livox SDK hardware config
    host_ip: str = "192.168.1.5"
    lidar_ip: str = "192.168.1.155"
    frequency: float = 10.0

    # "odom" frame: FastLio2 gives smooth continuous odometry; PGO publishes the
    # map→odom correction via TF.
    frame_id: str = FRAME_ODOM
    child_frame_id: str = FRAME_BODY

    # FAST-LIO internal processing rates
    msr_freq: float = 50.0
    main_freq: float = 5000.0

    # Output publish rates (Hz)
    pointcloud_freq: float = 10.0
    odom_freq: float = 30.0

    # FAST-LIO YAML config (relative to config/ dir, or absolute path)
    # C++ binary reads YAML directly via yaml-cpp
    config: Annotated[
        Path, validate_as(...).transform(lambda p: p if p.is_absolute() else _CONFIG_DIR / p)
    ] = Path("mid360.yaml")

    debug: bool = False

    # SDK port configuration (see livox/ports.py for defaults)
    cmd_data_port: int = SDK_CMD_DATA_PORT
    push_msg_port: int = SDK_PUSH_MSG_PORT
    point_data_port: int = SDK_POINT_DATA_PORT
    imu_data_port: int = SDK_IMU_DATA_PORT
    log_data_port: int = SDK_LOG_DATA_PORT
    host_cmd_data_port: int = SDK_HOST_CMD_DATA_PORT
    host_push_msg_port: int = SDK_HOST_PUSH_MSG_PORT
    host_point_data_port: int = SDK_HOST_POINT_DATA_PORT
    host_imu_data_port: int = SDK_HOST_IMU_DATA_PORT
    host_log_data_port: int = SDK_HOST_LOG_DATA_PORT

    # Resolved in __post_init__, passed as --config_path to the binary
    config_path: str | None = None

    cli_exclude: frozenset[str] = frozenset({"config"})

    def model_post_init(self, __context: object) -> None:
        """Resolve config_path."""
        super().model_post_init(__context)
        cfg = self.config
        if not cfg.is_absolute():
            cfg = _CONFIG_DIR / cfg
        self.config_path = str(cfg.resolve())


class FastLio2(NativeModule, perception.Lidar, perception.Odometry):
    config: FastLio2Config

    lidar: Out[PointCloud2]
    odometry: Out[Odometry]

    @rpc
    def start(self) -> None:
        self._validate_network()
        super().start()
        self.register_disposable(
            Disposable(self.odometry.transport.subscribe(self._on_odom_for_tf, self.odometry))
        )

    def _on_odom_for_tf(self, msg: Odometry) -> None:
        self.tf.publish(
            Transform(
                frame_id=self.frame_id,
                child_frame_id=self.config.child_frame_id,
                translation=Vector3(
                    msg.pose.position.x,
                    msg.pose.position.y,
                    msg.pose.position.z,
                ),
                rotation=Quaternion(
                    msg.pose.orientation.x,
                    msg.pose.orientation.y,
                    msg.pose.orientation.z,
                    msg.pose.orientation.w,
                ),
                ts=msg.ts or time.time(),
            )
        )

    @rpc
    def stop(self) -> None:
        super().stop()

    def _validate_network(self) -> None:
        # Auto-derive host_ip from a local NIC on the lidar's subnet (shared with
        # the Mid360 driver / Point-LIO) when the configured value isn't local.
        self.config.host_ip = resolve_host_ip(
            self.config.lidar_ip, self.config.host_ip, label="FastLio2"
        )


# Verify protocol port compliance (mypy will flag missing ports)
if TYPE_CHECKING:
    FastLio2()
