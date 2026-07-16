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

"""Native scene lidar backed by a cooked browser collision mesh."""

from __future__ import annotations

from typing import Any

from dimos.core.coordination.blueprints import Blueprint
from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.core.transport import LCMTransport
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.sim2.spec import (
    RaycastLidarSpec,
    SensorImplementation,
    SensorReady,
    WorldManifest,
    WorldStateFrame,
)
from dimos.simulation.scene_assets.spec import ScenePackage


class SceneLidarConfig(NativeModuleConfig):
    cwd: str | None = "rust/scene_lidar"
    executable: str = "target/release/scene_lidar"
    build_command: str | None = "cargo build --release"
    stdin_config: bool = True
    auto_build: bool = True

    scene_metadata_path: str
    # Explicit collision GLB; "" = resolve from scene metadata (native_config
    # forbids Option on the Rust side, so the sentinel is an empty string).
    collision_path: str = ""
    sensor_id: str = "lidar"
    scan_model: str = "uniform"
    frame_id: str = "lidar_link"
    publish_sensor_frame: bool = False
    hz: float = 10.0
    point_rate: int = 200_000
    horizontal_samples: int = 720
    vertical_samples: int = 16
    elevation_min_deg: float = -22.5
    elevation_max_deg: float = 22.5
    min_range: float = 0.0
    max_range: float = 10.0
    sensor_x: float = 0.0
    sensor_y: float = 0.0
    sensor_z: float = 1.0
    sensor_roll_deg: float = 0.0
    sensor_pitch_deg: float = 0.0
    sensor_yaw_deg: float = 0.0
    yaw_offset_deg: float = 0.0
    output_voxel_size: float = 0.03
    support_floor: bool = False
    support_floor_z: float = 0.0
    support_floor_size: float = 0.0

    def to_config_dict(self) -> dict[str, Any]:
        values = super().to_config_dict()
        values["frame_id"] = self.frame_id
        return values

    @classmethod
    def for_scene(
        cls,
        scene: ScenePackage,
        sensor: RaycastLidarSpec,
        **overrides: Any,
    ) -> SceneLidarConfig:
        if sensor.implementation != SensorImplementation.PORTABLE:
            raise ValueError("SceneLidarConfig requires a portable RaycastLidarSpec")
        if scene.metadata_path is None:
            raise ValueError("portable scene lidar requires cooked scene metadata")
        values: dict[str, Any] = {
            "scene_metadata_path": str(scene.metadata_path),
            "sensor_id": sensor.sensor_id,
            "frame_id": sensor.frame_id,
            "publish_sensor_frame": False,
            "hz": sensor.rate_hz,
            "horizontal_samples": sensor.width,
            "vertical_samples": sensor.height,
            "min_range": sensor.min_range,
            "max_range": sensor.max_range,
            "output_voxel_size": sensor.voxel_size,
        }
        values.update(overrides)
        return cls(**values)


class SceneLidarModule(NativeModule):
    """Raycast lidar from a cooked scene and authoritative sim2 state."""

    config: SceneLidarConfig

    odom: In[PoseStamped]
    world_manifest: In[WorldManifest]
    world_state: In[WorldStateFrame]
    pointcloud: Out[PointCloud2]
    sensor_ready: Out[SensorReady]


def scene_lidar_blueprint(config: SceneLidarConfig) -> Blueprint:
    """Build a portable lidar with its required native LCM boundary."""
    return SceneLidarModule.blueprint(**config.model_dump()).transports(
        {
            ("odom", PoseStamped): LCMTransport.spec("/odom", PoseStamped, preserve_backend=True),
            ("world_manifest", WorldManifest): LCMTransport.spec(
                "/world_manifest", WorldManifest, preserve_backend=True
            ),
            ("world_state", WorldStateFrame): LCMTransport.spec(
                "/world_state", WorldStateFrame, preserve_backend=True
            ),
            ("pointcloud", PointCloud2): LCMTransport.spec(
                "/pointcloud", PointCloud2, preserve_backend=True
            ),
            ("sensor_ready", SensorReady): LCMTransport.spec(
                "/sensor_ready", SensorReady, preserve_backend=True
            ),
        }
    )
