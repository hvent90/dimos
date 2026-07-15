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

"""Unit tests for MapNavPlant surface snap and map-stream helpers."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from dimos.memory2.replay import resolve_db_path
from dimos.memory2.store.sqlite import SqliteStore
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.map_nav.map_nav_plant import (
    BuiltStaticMap,
    _load_pgo_cache,
    _save_pgo_cache,
    build_global_map_from_db,
    build_surface_columns,
    resolve_lidar_stream,
    snap_z_to_surface,
)


def test_snap_z_picks_nearest_floor() -> None:
    pts = []
    for y in (0.0, 5.0):
        for x in np.linspace(-1.0, 1.0, 5):
            pts.append([x, y, 0.0 if y < 2 else 2.5])
    cloud = PointCloud2.from_numpy(np.asarray(pts, dtype=np.float32), frame_id="world")
    cols = build_surface_columns(cloud, voxel_size=0.05)
    assert (
        snap_z_to_surface(cols, x=0.0, y=0.0, z_hint=0.2, voxel_size=0.05, max_step_m=0.30) == 0.0
    )
    assert (
        snap_z_to_surface(cols, x=0.0, y=5.0, z_hint=2.4, voxel_size=0.05, max_step_m=0.30) == 2.5
    )


def test_snap_z_prefers_local_tread_over_distant_same_floor() -> None:
    """Floor behind you must not pin Z while standing on the next tread."""
    pts = [
        [0.0, 0.0, 0.0],  # previous flat (within old 1m search)
        [0.0, 0.30, 0.15],  # current tread
    ]
    cloud = PointCloud2.from_numpy(np.asarray(pts, dtype=np.float32), frame_id="world")
    cols = build_surface_columns(cloud, voxel_size=0.05)
    assert snap_z_to_surface(
        cols,
        x=0.0,
        y=0.30,
        z_hint=0.0,
        voxel_size=0.05,
        search_radius_m=0.15,
        max_step_m=0.16,
    ) == pytest.approx(0.15)


def test_snap_z_steps_down_locally() -> None:
    pts = [
        [0.0, 0.0, 2.55],
        [0.0, -0.30, 2.40],
    ]
    cloud = PointCloud2.from_numpy(np.asarray(pts, dtype=np.float32), frame_id="world")
    cols = build_surface_columns(cloud, voxel_size=0.05)
    assert snap_z_to_surface(
        cols,
        x=0.0,
        y=-0.30,
        z_hint=2.55,
        voxel_size=0.05,
        search_radius_m=0.15,
        max_step_m=0.16,
    ) == pytest.approx(2.40)


def test_snap_z_holds_when_step_too_large() -> None:
    pts = [[0.0, 0.0, 0.0], [0.0, 0.0, 2.55]]
    cloud = PointCloud2.from_numpy(np.asarray(pts, dtype=np.float32), frame_id="world")
    cols = build_surface_columns(cloud, voxel_size=0.05)
    assert (
        snap_z_to_surface(cols, x=0.0, y=0.0, z_hint=0.0, voxel_size=0.05, max_step_m=0.16) == 0.0
    )
    assert snap_z_to_surface(
        cols, x=0.0, y=0.0, z_hint=2.55, voxel_size=0.05, max_step_m=0.16
    ) == pytest.approx(2.55)


def test_teleop_walk_climbs_synthetic_stair_columns() -> None:
    """Simulate teleop along +Y: Z must rise with treads, not stay on floor0."""
    pts = []
    for i in range(17):
        y = 4.0 + i * 0.30
        z = i * 0.15
        for x in np.linspace(-0.2, 0.2, 3):
            pts.append([x, y, z])
    # flat behind the flight (would trap the old 1m nearest-to-hint snap)
    for y in np.linspace(0.0, 4.0, 20):
        pts.append([0.0, y, 0.0])
    cloud = PointCloud2.from_numpy(np.asarray(pts, dtype=np.float32), frame_id="world")
    cols = build_surface_columns(cloud, voxel_size=0.05)
    z = 0.0
    for i in range(17):
        y = 4.0 + i * 0.30 + 0.05
        z = snap_z_to_surface(
            cols,
            x=0.0,
            y=y,
            z_hint=z,
            voxel_size=0.05,
            search_radius_m=0.15,
            max_step_m=0.16,
        )
        assert abs(z - i * 0.15) < 1e-6, (i, y, z)


def test_resolve_lidar_stream_falls_back_to_pointlio() -> None:
    path = resolve_db_path("mid360_athens_stairs")
    store = SqliteStore(path=str(path), must_exist=True)
    store.start()
    try:
        assert resolve_lidar_stream(store, "lidar") == "pointlio_lidar"
    finally:
        store.stop()


@pytest.mark.timeout(120)
def test_build_mid360_athens_stairs_map_has_elevation() -> None:
    """Mid360 bags store sensor-frame clouds on pointlio_lidar; map must span Z."""
    built = build_global_map_from_db(
        "mid360_athens_stairs",
        lidar_stream="lidar",
        voxel_size=0.10,
        dedup_tol_m=1.0,
    )
    assert built.lidar_stream == "pointlio_lidar"
    assert built.frames_used > 10
    assert len(built.cloud) > 1000
    zs = built.cloud.points_f32()[:, 2]
    assert float(zs.max() - zs.min()) > 2.0


def test_pgo_cache_roundtrip(tmp_path: Path) -> None:
    pts = np.asarray([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.1]], dtype=np.float32)
    cloud = PointCloud2.from_numpy(pts, frame_id="world")
    cloud.ts = 0.0
    built = BuiltStaticMap(
        cloud=cloud,
        start_x=-1.5,
        start_y=2.0,
        start_yaw=0.25,
        lidar_stream="lidar",
        world_frame="world",
        frames_used=42,
        pgo=True,
        from_cache=False,
    )
    db_path = tmp_path / "go2_fake.db"
    db_path.write_bytes(b"")  # path only; cache keys off stem/parent
    _save_pgo_cache(db_path, built)
    loaded = _load_pgo_cache(db_path, start_x=0.0, start_y=0.0, start_yaw=0.0)
    assert loaded is not None
    assert loaded.from_cache is True
    assert loaded.pgo is True
    assert loaded.frames_used == 42
    assert loaded.start_x == -1.5
    assert loaded.start_y == 2.0
    assert abs(loaded.start_yaw - 0.25) < 1e-9
    assert len(loaded.cloud) == 3
