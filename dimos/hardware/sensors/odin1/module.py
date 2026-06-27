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

"""Python NativeModule wrapper for the Manifold Tech Odin1 Rust binary.

The Odin1 runs SLAM onboard, so the binary is a thin source: it publishes lidar
point clouds, the RGB camera image, and onboard odometry. Config is sent to the
binary as a JSON object on stdin (Python owns every default).
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import Out
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.cmu_nav.frames import FRAME_BODY, FRAME_ODOM
from dimos.spec import perception


class Odin1Config(NativeModuleConfig):
    cwd: str | None = "."
    # Local dev: run the cargo release binary directly. (nix build .#default is the
    # reproducible path once the odin1/odin1-sys git hashes are pinned.)
    executable: str = "target/release/odin1_module"
    build_command: str | None = "cargo build --release"
    # The Rust binary reads its config as a JSON object on stdin (required).
    stdin_config: bool = True

    # "slam" exposes raw streams plus onboard odometry. "raw" omits odometry.
    mode: str = "slam"
    # Named to avoid the reserved base-config `frame_id` (CLI-only, stripped from
    # the stdin JSON the Rust module reads).
    odom_frame_id: str = FRAME_ODOM
    child_frame_id: str = FRAME_BODY
    lidar_frame_id: str = "odin1_dtof"
    camera_frame_id: str = "odin1_camera"
    # Drop dtof points below this confidence. SDK suggests ~30-35.
    confidence_min: int = 32
    publish_image: bool = True
    # Publish odometry at IMU rate instead of the ~10Hz SLAM rate.
    odometry_highfreq: bool = False
    discovery_timeout_s: float = 10.0
    # Directory the per-device calib.yaml is written to (must exist). Empty skips.
    calib_out_path: str = "/tmp"
    # DTOF depth output rate in Hz: "10", "14.5", or "29". Required for the depth
    # sensor to produce frames.
    depth_rate_hz: str = "10"
    # Bounded frame-channel capacity; newest frames drop if the consumer lags.
    channel_capacity: int = 256


class Odin1(NativeModule, perception.Lidar, perception.Odometry, perception.Image):
    config: Odin1Config

    # Live per-frame depth cloud.
    lidar: Out[PointCloud2]
    # Onboard SLAM map cloud (colored).
    slam_cloud: Out[PointCloud2]
    odometry: Out[Odometry]
    color_image: Out[Image]


# Verify the module constructs (mirrors the pointlio/virtual_mid360 wrappers).
if TYPE_CHECKING:
    Odin1()
