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

from pathlib import Path

import pytest

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.sim2.spec import (
    RaycastLidarSpec,
    SensorImplementation,
    SensorReady,
    WorldManifest,
    WorldStateFrame,
)
from dimos.simulation.scene_assets.spec import SceneMeshAlignment, ScenePackage
from dimos.simulation.sensors.scene_lidar import SceneLidarConfig, scene_lidar_blueprint


def _scene(tmp_path: Path) -> ScenePackage:
    metadata_path = tmp_path / "scene.meta.json"
    metadata_path.write_text("{}")
    return ScenePackage(
        package_dir=tmp_path,
        source_path=tmp_path / "source.glb",
        alignment=SceneMeshAlignment(),
        metadata_path=metadata_path,
    )


def test_portable_lidar_config_is_derived_from_typed_sensor_spec(tmp_path: Path) -> None:
    sensor = RaycastLidarSpec(
        sensor_id="front-lidar",
        frame_id="world",
        implementation=SensorImplementation.PORTABLE,
        width=120,
        height=8,
        rate_hz=5.0,
        min_range=0.2,
        max_range=8.0,
        voxel_size=0.04,
    )

    config = SceneLidarConfig.for_scene(_scene(tmp_path), sensor, sensor_z=0.5)

    assert config.sensor_id == "front-lidar"
    assert config.horizontal_samples == 120
    assert config.vertical_samples == 8
    assert config.hz == 5.0
    assert config.max_range == 8.0
    assert config.output_voxel_size == 0.04
    assert config.sensor_z == 0.5
    assert config.to_config_dict()["frame_id"] == "world"


def test_portable_lidar_config_rejects_native_sensor(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="portable"):
        SceneLidarConfig.for_scene(_scene(tmp_path), RaycastLidarSpec())


def test_portable_lidar_blueprint_pins_native_lcm_boundary(tmp_path: Path) -> None:
    sensor = RaycastLidarSpec(implementation=SensorImplementation.PORTABLE)
    blueprint = scene_lidar_blueprint(SceneLidarConfig.for_scene(_scene(tmp_path), sensor))

    expected = {
        ("odom", PoseStamped),
        ("world_manifest", WorldManifest),
        ("world_state", WorldStateFrame),
        ("pointcloud", PointCloud2),
        ("sensor_ready", SensorReady),
    }
    assert set(blueprint.transport_map) == expected
    assert all(spec.kwargs["preserve_backend"] for spec in blueprint.transport_map.values())
