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

"""Python NativeModule wrapper for the C++ Livox Mid-360 driver.

Usage::
    from dimos.hardware.sensors.lidar.livox.module import Mid360
    from dimos.core.coordination.blueprints import autoconnect

    from dimos.core.coordination.module_coordinator import ModuleCoordinator
    ModuleCoordinator.build(autoconnect(
        Mid360.blueprint(host_ip="192.168.1.5"),
        SomeConsumer.blueprint(),
    )).loop()
"""

from __future__ import annotations

import ipaddress
from typing import TYPE_CHECKING

from dimos.core.core import rpc
from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import Out
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
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.spec import perception
from dimos.utils.generic import get_local_ips
from dimos.utils.logging_config import setup_logger

_logger = setup_logger()


class Mid360Config(NativeModuleConfig):
    """Config for the C++ Mid-360 native module."""

    cwd: str | None = "cpp"
    executable: str = "result/bin/mid360_native"
    build_command: str | None = "nix build .#mid360_native"
    host_ip: str = "192.168.1.5"
    lidar_ip: str = "192.168.1.155"
    frequency: float = 10.0
    enable_imu: bool = True
    frame_id: str = "lidar_link"
    imu_frame_id: str = "imu_link"

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


class Mid360(NativeModule, perception.Lidar, perception.IMU):
    """Livox Mid-360 LiDAR module backed by a native C++ binary.

    Ports:
        lidar (Out[PointCloud2]): Point cloud frames at configured frequency.
        imu (Out[Imu]): IMU data at ~200 Hz (if enabled).
    """

    config: Mid360Config

    lidar: Out[PointCloud2]
    imu: Out[Imu]

    @rpc
    def start(self) -> None:
        self._correct_host_ip()
        super().start()

    @rpc
    def stop(self) -> None:
        super().stop()

    def _correct_host_ip(self) -> None:
        """Auto-correct ``host_ip`` to a local interface on the lidar's subnet.

        The native driver binds to ``host_ip``; if it is not an address on this
        machine the bind fails and the process dies. Mirrors FastLio2 so both
        agree on the host address regardless of which machine runs the stack.
        """
        host_ip = self.config.host_ip
        lidar_ip = self.config.lidar_ip
        local_ips = [ip for ip, _iface in get_local_ips()]
        if host_ip in local_ips:
            return
        try:
            lidar_net = ipaddress.IPv4Network(f"{lidar_ip}/24", strict=False)
            same_subnet = [ip for ip in local_ips if ipaddress.IPv4Address(ip) in lidar_net]
        except (ValueError, TypeError):
            same_subnet = []
        if not same_subnet:
            _logger.warning(
                f"Mid360: host_ip={host_ip!r} not assigned locally and no interface shares "
                f"the lidar subnet ({lidar_ip}); bind will likely fail.",
                local_ips=local_ips,
            )
            return
        picked = same_subnet[0]
        _logger.warning(
            f"Mid360: host_ip={host_ip!r} not found locally. "
            f"Auto-correcting to {picked!r} (same subnet as lidar {lidar_ip}).",
            configured_ip=host_ip,
            corrected_ip=picked,
            lidar_ip=lidar_ip,
            local_ips=local_ips,
        )
        self.config.host_ip = picked


# Verify protocol port compliance (mypy will flag missing ports)
if TYPE_CHECKING:
    Mid360()
