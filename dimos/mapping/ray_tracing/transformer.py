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

from __future__ import annotations

from typing import TYPE_CHECKING, Any

import numpy as np
import open3d as o3d  # type: ignore[import-untyped]
import open3d.core as o3c  # type: ignore[import-untyped]

from dimos.mapping.ray_tracing.voxel_map import VoxelRayMapper
from dimos.memory2.transform import Transformer
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

if TYPE_CHECKING:
    from collections.abc import Iterator

    from dimos.memory2.type.observation import Observation


class RayTraceMap(Transformer[PointCloud2, PointCloud2]):
    """Accumulate world-frame lidar into a voxel map with raycast clearing."""

    def __init__(
        self,
        *,
        voxel_size: float = 0.1,
        max_range: float = 30.0,
        emit_every: int = 1,
        emit_local: bool = False,
        region_percentile: float = 95.0,
        **mapper_kwargs: Any,
    ) -> None:
        if emit_every < 0:
            raise ValueError(f"emit_every must be >= 0, got {emit_every}")
        self.voxel_size = voxel_size
        self.max_range = max_range
        self.emit_every = emit_every
        self.emit_local = emit_local
        self.region_percentile = region_percentile
        self._mapper_kwargs = mapper_kwargs

    def _robust_bounds(
        self, points: np.ndarray, origins: np.ndarray
    ) -> tuple[float, float, float, float, float]:
        """Robot-centered cylinder sized to a percentile of the observed points,
        so a sparse far tail of returns does not inflate the local region.

        The radius covers `region_percentile` of points by xy distance, and the
        z band is trimmed to the same percentile to drop ceiling/tall returns.
        """
        margin = self._mapper_kwargs.get("shadow_depth", 0.2) + self.voxel_size
        cx = float(origins[:, 0].mean())
        cy = float(origins[:, 1].mean())
        dist = np.hypot(points[:, 0] - cx, points[:, 1] - cy)
        radius = float(np.percentile(dist, self.region_percentile)) + margin

        lo_pct = 100.0 - self.region_percentile
        z_min = float(np.percentile(points[:, 2], lo_pct)) - margin
        z_max = float(np.percentile(points[:, 2], self.region_percentile)) + margin
        return cx, cy, radius, z_min, z_max

    def _make_obs(
        self,
        mapper: VoxelRayMapper,
        last_obs: Observation[PointCloud2],
        count: int,
        batch_points: list[np.ndarray],
        batch_origins: list[tuple[float, float, float]],
    ) -> Observation[PointCloud2]:
        tags = {**last_obs.tags, "frame_count": count}
        if self.emit_local and batch_points:
            points = np.concatenate(batch_points, axis=0)
            origins = np.asarray(batch_origins, dtype=np.float64)
            cx, cy, radius, z_min, z_max = self._robust_bounds(points, origins)
            positions = mapper.local_map((cx, cy, 0.0), radius, z_min, z_max)
            tags["region_bounds"] = (cx, cy, radius, z_min, z_max)
        else:
            positions = mapper.global_map()
        pcd = o3d.t.geometry.PointCloud()
        pcd.point["positions"] = o3c.Tensor.from_numpy(positions)
        cloud = PointCloud2(pointcloud=pcd, frame_id="world", ts=last_obs.ts)
        return last_obs.derive(data=cloud, tags=tags)

    def __call__(
        self,
        upstream: Iterator[Observation[PointCloud2]],
    ) -> Iterator[Observation[PointCloud2]]:
        mapper = VoxelRayMapper(
            voxel_size=self.voxel_size, max_range=self.max_range, **self._mapper_kwargs
        )
        last_obs: Observation[PointCloud2] | None = None
        count = 0
        batch_points: list[np.ndarray] = []
        batch_origins: list[tuple[float, float, float]] = []

        for obs in upstream:
            if obs.pose_tuple is None:
                continue
            x, y, z, *_ = obs.pose_tuple
            pts = obs.data.points_f32()
            mapper.add_frame(pts, (x, y, z))
            if self.emit_local and pts.size:
                batch_points.append(pts)
                batch_origins.append((x, y, z))
            last_obs = obs
            count += 1

            if self.emit_every > 0 and count % self.emit_every == 0:
                yield self._make_obs(mapper, last_obs, count, batch_points, batch_origins)
                batch_points = []
                batch_origins = []

        if last_obs is not None and (self.emit_every == 0 or count % self.emit_every != 0):
            yield self._make_obs(mapper, last_obs, count, batch_points, batch_origins)
