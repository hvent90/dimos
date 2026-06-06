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

import time
from typing import TYPE_CHECKING

from dimos.memory2.transform import Transformer
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.Path import Path
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.nav_3d.mls_planner.mls_planner import MLSPlanner
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from collections.abc import Iterator

    import numpy as np
    from numpy.typing import NDArray

    from dimos.memory2.type.observation import Observation

logger = setup_logger()


class MLSPlan(Transformer[PointCloud2, Path]):
    """Plan paths from current pose to a fixed goal over an accumulating voxel map."""

    def __init__(
        self,
        *,
        goal: tuple[float, float, float],
        voxel_size: float = 0.1,
        robot_height: float = 1.5,
        surface_dilation_passes: int = 3,
        surface_erosion_passes: int = 3,
        node_spacing_m: float = 1.0,
        node_wall_buffer_m: float = 0.3,
        node_step_threshold_m: float = 0.25,
    ) -> None:
        self.goal = goal
        self.voxel_size = voxel_size
        self.robot_height = robot_height
        self.surface_dilation_passes = surface_dilation_passes
        self.surface_erosion_passes = surface_erosion_passes
        self.node_spacing_m = node_spacing_m
        self.node_wall_buffer_m = node_wall_buffer_m
        self.node_step_threshold_m = node_step_threshold_m

    def _path_from_waypoints(self, waypoints: NDArray[np.float32] | None, ts: float) -> Path:
        poses: list[PoseStamped] = []
        if waypoints is not None:
            for x, y, z in waypoints:
                poses.append(
                    PoseStamped(
                        ts=ts,
                        frame_id="world",
                        position=(float(x), float(y), float(z)),
                        orientation=(0.0, 0.0, 0.0, 1.0),
                    )
                )
        return Path(ts=ts, frame_id="world", poses=poses)

    def __call__(
        self,
        upstream: Iterator[Observation[PointCloud2]],
    ) -> Iterator[Observation[Path]]:
        planner = MLSPlanner(
            voxel_size=self.voxel_size,
            robot_height=self.robot_height,
            surface_dilation_passes=self.surface_dilation_passes,
            surface_erosion_passes=self.surface_erosion_passes,
            node_spacing_m=self.node_spacing_m,
            node_wall_buffer_m=self.node_wall_buffer_m,
            node_step_threshold_m=self.node_step_threshold_m,
        )
        for obs in upstream:
            if obs.pose_tuple is None:
                logger.debug("MLSPlan: obs %s has no pose; skipping", obs.id)
                continue
            x, y, z, *_ = obs.pose_tuple
            start = (float(x), float(y), float(z) - self.robot_height)

            bounds = obs.tags.get("region_bounds")
            if bounds is None:
                raise ValueError(
                    "MLSPlan consumes local map slices; construct RayTraceMap(emit_local=True)"
                )
            ox, oy, radius, z_min, z_max = bounds
            planner.update_region(obs.data.points_f32(), (ox, oy), radius, z_min, z_max)
            t_plan = time.perf_counter()
            waypoints = planner.plan(start, self.goal)
            plan_ms = (time.perf_counter() - t_plan) * 1000
            path = self._path_from_waypoints(waypoints, obs.ts)

            timings = {**planner.last_timings(), "plan_ms": plan_ms}
            timings["total_ms"] = sum(timings.values())

            yield obs.derive(
                data=path,
                tags={
                    **obs.tags,
                    "voxel_map": planner.voxel_map(),
                    "surface_map": planner.surface_map(),
                    "nodes": planner.nodes(),
                    "node_edges": planner.node_edges(),
                    "start": start,
                    "planned": waypoints is not None,
                    "timings": timings,
                    "voxels": planner.voxel_count(),
                },
            )
