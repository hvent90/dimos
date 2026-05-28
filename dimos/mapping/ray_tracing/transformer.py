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

from typing import TYPE_CHECKING

import open3d as o3d  # type: ignore[import-untyped]
import open3d.core as o3c  # type: ignore[import-untyped]

from dimos.mapping.ray_tracing._voxel_ray_tracing import VoxelRayMap
from dimos.memory2.transform import Transformer
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from collections.abc import Iterator

    from dimos.memory2.type.observation import Observation

logger = setup_logger()


class RayTraceMap(Transformer[PointCloud2, PointCloud2]):
    """Accumulate lidar PointCloud2 observations into a global voxel map with raycast clearing.

    Wraps the pyo3-bound :class:`VoxelRayMap`. Input clouds must be in world
    frame; ``obs.pose`` must be populated with the lidar sensor's pose in world
    frame. Observations without a pose are skipped.

    Args:
        voxel_size: edge length of each voxel (m).
        max_range: maximum ray casting range (m); 0 means no limit.
        ray_subsample: keep every Nth ray for clearing (>= 1).
        shadow_depth: ray extension past endpoint to clear shadows (m).
        grace_depth: spare voxels within this distance of endpoints from clearing (m).
        min_health: voxel removal threshold.
        max_health: voxel saturation cap.
        emit_every: yield the current accumulated map every N frames.
            1 (default) = yield after every frame.
            0 = only emit when upstream exhausts (batch mode).
    """

    def __init__(
        self,
        *,
        voxel_size: float = 0.1,
        max_range: float = 30.0,
        ray_subsample: int = 1,
        shadow_depth: float = 0.2,
        grace_depth: float = 0.2,
        min_health: int = -2,
        max_health: int = 1,
        emit_every: int = 1,
    ) -> None:
        self.voxel_size = voxel_size
        self.max_range = max_range
        self.ray_subsample = ray_subsample
        self.shadow_depth = shadow_depth
        self.grace_depth = grace_depth
        self.min_health = min_health
        self.max_health = max_health
        self.emit_every = emit_every

    def _make_obs(
        self,
        m: VoxelRayMap,
        last_obs: Observation[PointCloud2],
        count: int,
    ) -> Observation[PointCloud2]:
        positions = m.global_map()
        pcd = o3d.t.geometry.PointCloud()
        pcd.point["positions"] = o3c.Tensor.from_numpy(positions)
        cloud = PointCloud2(pointcloud=pcd, frame_id="world", ts=last_obs.ts)
        return last_obs.derive(
            data=cloud,
            pose=None,
            tags={**last_obs.tags, "frame_count": count},
        )

    def __call__(
        self,
        upstream: Iterator[Observation[PointCloud2]],
    ) -> Iterator[Observation[PointCloud2]]:
        m = VoxelRayMap(
            voxel_size=self.voxel_size,
            max_range=self.max_range,
            ray_subsample=self.ray_subsample,
            shadow_depth=self.shadow_depth,
            grace_depth=self.grace_depth,
            min_health=self.min_health,
            max_health=self.max_health,
        )
        last_obs: Observation[PointCloud2] | None = None
        count = 0

        for obs in upstream:
            if obs.pose is None:
                logger.debug("RayTraceMap: obs %s has no .pose; skipping", obs.id)
                continue
            positions = obs.data.points_f32()
            m.add_frame(positions, (obs.pose[0], obs.pose[1], obs.pose[2]))
            last_obs = obs
            count += 1

            if self.emit_every > 0 and count % self.emit_every == 0:
                yield self._make_obs(m, last_obs, count)

        if last_obs is not None and (self.emit_every == 0 or count % self.emit_every != 0):
            yield self._make_obs(m, last_obs, count)
