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

"""Golden reference map: full-recording voxel map plus walked-corridor free space.

The golden occupancy is what returned paths are collision-checked against.
The walked corridor marks voxels the robot's body physically swept, which are
free space regardless of what any mapper claims.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import itertools
import json
from time import perf_counter
from typing import TYPE_CHECKING

import numpy as np

from dimos.navigation.nav_3d.evaluator.recording import iter_world_frames, load_trajectory
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from pathlib import Path

    from numpy.typing import NDArray

    from dimos.navigation.nav_3d.evaluator.cases import Suite
    from dimos.navigation.nav_3d.evaluator.config import EvalConfig
    from dimos.navigation.nav_3d.evaluator.recording import Trajectory

logger = setup_logger()

_KEY_OFFSET = 1 << 20


def voxel_keys(points: NDArray[np.float32], voxel_size: float) -> NDArray[np.int64]:
    """Pack voxel indices into sortable int64 keys, one per point."""
    idx = np.floor(points.astype(np.float64) / voxel_size).astype(np.int64) + _KEY_OFFSET
    return (idx[:, 0] << 42) | (idx[:, 1] << 21) | idx[:, 2]


def key_centers(keys: NDArray[np.int64], voxel_size: float) -> NDArray[np.float32]:
    """Voxel center positions for packed keys, the inverse of voxel_keys."""
    mask = (1 << 21) - 1
    idx = np.stack([keys >> 42, (keys >> 21) & mask, keys & mask], axis=1) - _KEY_OFFSET
    return ((idx + 0.5) * voxel_size).astype(np.float32)


def keys_contain(sorted_keys: NDArray[np.int64], query: NDArray[np.int64]) -> NDArray[np.bool_]:
    if len(sorted_keys) == 0:
        return np.zeros(len(query), dtype=bool)
    pos = np.clip(np.searchsorted(sorted_keys, query), 0, len(sorted_keys) - 1)
    return np.asarray(sorted_keys[pos] == query)


def cylinder_offsets(
    radius: float, z_lo: float, z_hi: float, voxel_size: float
) -> NDArray[np.int64]:
    """Integer voxel offsets forming a vertical cylinder."""
    r_vox = int(np.ceil(radius / voxel_size))
    span = np.arange(-r_vox, r_vox + 1)
    dx, dy = np.meshgrid(span, span, indexing="ij")
    in_disc = (dx * voxel_size) ** 2 + (dy * voxel_size) ** 2 <= radius**2
    dz = np.arange(int(np.floor(z_lo / voxel_size)), int(np.ceil(z_hi / voxel_size)) + 1)
    disc = np.stack([dx[in_disc], dy[in_disc]], axis=1)
    out = np.concatenate([np.hstack([disc, np.full((len(disc), 1), z)]) for z in dz])
    return np.asarray(out, dtype=np.int64)


def offset_keys(
    points: NDArray[np.float32], offsets: NDArray[np.int64], voxel_size: float
) -> NDArray[np.int64]:
    """Keys of every (point voxel + offset) pair, shape (P * O,)."""
    idx = np.floor(points.astype(np.float64) / voxel_size).astype(np.int64) + _KEY_OFFSET
    swept = idx[:, None, :] + offsets[None, :, :]
    return np.asarray((swept[..., 0] << 42) | (swept[..., 1] << 21) | swept[..., 2])


def densify(points: NDArray[np.float32], step: float) -> NDArray[np.float32]:
    """Resample a polyline so consecutive samples are at most step apart."""
    if len(points) < 2:
        return points.astype(np.float32)
    out = [points[:1]]
    for a, b in itertools.pairwise(points):
        seg = np.linalg.norm(b - a)
        n = max(int(np.ceil(seg / step)), 1)
        t = np.linspace(0.0, 1.0, n + 1)[1:, None]
        out.append(a[None, :] * (1 - t) + b[None, :] * t)
    return np.concatenate(out).astype(np.float32)


def walked_corridor_keys(
    trajectory: Trajectory,
    voxel_size: float,
    radius: float,
    z_lo: float,
    z_hi: float,
) -> NDArray[np.int64]:
    """Voxels swept by the robot body cylinder along the trajectory, sorted.

    z_lo and z_hi are relative to the odometry pose. The carved volume must
    cover the collision gate's checked volume, or the walked path itself
    fails the gate.
    """
    dense = densify(trajectory.positions, voxel_size / 2)
    offsets = cylinder_offsets(radius, z_lo, z_hi, voxel_size)
    return np.unique(offset_keys(dense, offsets, voxel_size))


@dataclass
class GoldenMap:
    voxel_size: float
    occupied: NDArray[np.float32]
    occupied_keys: NDArray[np.int64]
    walked_keys: NDArray[np.int64]
    frames: int
    add_frame_ms: dict[str, float]
    build_ms: float

    def obstacle_keys(self) -> NDArray[np.int64]:
        """Occupied minus walked-free, the set paths must not intersect."""
        return np.setdiff1d(self.occupied_keys, self.walked_keys, assume_unique=True)


CACHE_VERSION = 2


def _cache_path(db_path: Path, params: dict[str, float | int | str]) -> Path:
    digest = hashlib.sha1(json.dumps(params, sort_keys=True).encode()).hexdigest()[:10]
    return db_path.parent / ".golden" / f"{db_path.stem}.{digest}.npz"


def load_or_build_golden(
    db_path: Path,
    suite: Suite,
    cfg: EvalConfig,
    corridor_radius: float,
    corridor_z_lo: float,
    corridor_z_hi: float,
) -> GoldenMap:
    params: dict[str, float | int | str] = {
        **cfg.mapper_fingerprint(),
        "corridor_radius": corridor_radius,
        "corridor_z_lo": corridor_z_lo,
        "corridor_z_hi": corridor_z_hi,
        "align_tol": cfg.align_tol,
        "lidar_stream": suite.lidar_stream,
        "odom_stream": suite.odom_stream,
        "cache_version": CACHE_VERSION,
    }
    voxel_size = cfg.voxel_size
    cache = _cache_path(db_path, params)
    if cache.exists():
        data = np.load(cache)
        return GoldenMap(
            voxel_size=voxel_size,
            occupied=data["occupied"],
            occupied_keys=data["occupied_keys"],
            walked_keys=data["walked_keys"],
            frames=int(data["frames"]),
            add_frame_ms={
                "p50": float(data["add_p50"]),
                "p95": float(data["add_p95"]),
                "max": float(data["add_max"]),
            },
            build_ms=0.0,
        )

    logger.info("building golden map for %s (cache miss)", db_path.name)
    mapper = cfg.make_mapper()
    add_ms: list[float] = []
    t0 = perf_counter()
    for frame in iter_world_frames(db_path, suite.lidar_stream, suite.odom_stream, cfg.align_tol):
        t1 = perf_counter()
        mapper.add_frame(frame.points, frame.origin)
        add_ms.append((perf_counter() - t1) * 1000)
    build_ms = (perf_counter() - t0) * 1000
    add_arr = np.asarray(add_ms) if add_ms else np.zeros(1)
    occupied = mapper.global_map()
    occupied_keys = np.unique(voxel_keys(occupied, voxel_size))
    trajectory = load_trajectory(db_path, suite.odom_stream)
    walked = walked_corridor_keys(
        trajectory, voxel_size, corridor_radius, corridor_z_lo, corridor_z_hi
    )

    cache.parent.mkdir(exist_ok=True)
    np.savez_compressed(
        cache,
        occupied=occupied,
        occupied_keys=occupied_keys,
        walked_keys=walked,
        frames=len(add_ms),
        add_p50=np.percentile(add_arr, 50),
        add_p95=np.percentile(add_arr, 95),
        add_max=add_arr.max(),
    )
    logger.info("golden map cached: %s (%d voxels)", cache.name, len(occupied))
    return GoldenMap(
        voxel_size=voxel_size,
        occupied=occupied,
        occupied_keys=occupied_keys,
        walked_keys=walked,
        frames=len(add_ms),
        add_frame_ms={
            "p50": float(np.percentile(add_arr, 50)),
            "p95": float(np.percentile(add_arr, 95)),
            "max": float(add_arr.max()),
        },
        build_ms=build_ms,
    )
