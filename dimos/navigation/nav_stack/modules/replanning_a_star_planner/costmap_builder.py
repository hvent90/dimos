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

"""Costmap construction from PointCloud2 inputs for ReplanningAStarPlanner.

Tracks per-cell max obstacle height *and* an "observed" set so the emitted
``OccupancyGrid`` carries three states (FREE / OCCUPIED / UNKNOWN). This is
what lets the gradient-costmap A* (re-used from
``dimos.navigation.replanning_a_star``) respect unknowns the same way the
original planner does.
"""

from __future__ import annotations

import math
import threading

import numpy as np

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.nav_msgs.OccupancyGrid import CostValues, OccupancyGrid


class HeightMapCostmap:
    """Sparse per-cell height map + observed-cell set.

    Mirrors SimplePlanner's ``Costmap`` shape, but separately tracks which
    cells have ever received a point so we can distinguish observed-free
    from unobserved (the original replanning A* costmap is FREE/OCC/UNK).
    """

    def __init__(self, cell_size: float, obstacle_height_threshold: float) -> None:
        if cell_size <= 0.0:
            raise ValueError(f"cell_size must be positive, got {cell_size}")
        self.cell_size = float(cell_size)
        self.obstacle_height_threshold = float(obstacle_height_threshold)
        self._heights: dict[tuple[int, int], float] = {}
        self._observed: set[tuple[int, int]] = set()
        self._lock = threading.Lock()

    def world_to_cell(self, x: float, y: float) -> tuple[int, int]:
        return (math.floor(x / self.cell_size), math.floor(y / self.cell_size))

    def ingest(self, points: np.ndarray, ground_z: float) -> None:
        """Add an Nx3 point array. ``ground_z`` is the ground-plane height
        (typically robot_z - sensor_offset). Points above ground update the
        per-cell max-height; every input point marks its cell as observed.
        """
        if points is None or len(points) == 0:
            return
        cell_size = self.cell_size
        ixs = np.floor(points[:, 0] / cell_size).astype(np.int64)
        iys = np.floor(points[:, 1] / cell_size).astype(np.int64)
        heights = points[:, 2] - ground_z

        keys = np.column_stack((ixs, iys))
        unique_keys, inverse = np.unique(keys, axis=0, return_inverse=True)
        max_h = np.full(len(unique_keys), float("-inf"))
        np.maximum.at(max_h, inverse, heights)

        with self._lock:
            heights_dict = self._heights
            observed = self._observed
            for (ix, iy), h in zip(unique_keys.tolist(), max_h.tolist(), strict=True):
                key = (ix, iy)
                observed.add(key)
                if h > 0.0 and h > heights_dict.get(key, float("-inf")):
                    heights_dict[key] = h

    def reset(self) -> None:
        with self._lock:
            self._heights.clear()
            self._observed.clear()

    @property
    def observed_count(self) -> int:
        with self._lock:
            return len(self._observed)

    def snapshot(self) -> tuple[dict[tuple[int, int], float], set[tuple[int, int]]]:
        """Cheap copy of internal state for downstream processing."""
        with self._lock:
            return dict(self._heights), set(self._observed)

    def to_occupancy_grid(
        self,
        center_x: float,
        center_y: float,
        radius: float,
        *,
        extra_points: list[tuple[float, float]] | None = None,
        frame_id: str = "world",
    ) -> OccupancyGrid:
        """Slice a square window of the costmap into a numpy OccupancyGrid.

        The window is a square of side ``2*radius`` centered at
        ``(center_x, center_y)``. If ``extra_points`` is given, the window
        is expanded to contain them (with the same ``radius`` margin) —
        useful for making sure goal cells fall inside the grid.

        Cells: FREE (0) if observed-and-not-obstacle, OCCUPIED (100) if
        observed-and-tall, UNKNOWN (-1) otherwise.
        """
        cell = self.cell_size

        min_x = center_x - radius
        max_x = center_x + radius
        min_y = center_y - radius
        max_y = center_y + radius
        if extra_points:
            for ex, ey in extra_points:
                min_x = min(min_x, ex - radius)
                max_x = max(max_x, ex + radius)
                min_y = min(min_y, ey - radius)
                max_y = max(max_y, ey + radius)

        ix_min = math.floor(min_x / cell)
        iy_min = math.floor(min_y / cell)
        ix_max = math.ceil(max_x / cell)
        iy_max = math.ceil(max_y / cell)
        width = max(1, ix_max - ix_min)
        height = max(1, iy_max - iy_min)

        grid = np.full((height, width), CostValues.UNKNOWN, dtype=np.int8)

        heights_dict, observed = self.snapshot()
        threshold = self.obstacle_height_threshold

        for ix, iy in observed:
            gx = ix - ix_min
            gy = iy - iy_min
            if 0 <= gx < width and 0 <= gy < height:
                grid[gy, gx] = CostValues.FREE

        for (ix, iy), h in heights_dict.items():
            if h < threshold:
                continue
            gx = ix - ix_min
            gy = iy - iy_min
            if 0 <= gx < width and 0 <= gy < height:
                grid[gy, gx] = CostValues.OCCUPIED

        origin = Pose()  # type: ignore[call-arg]
        origin.position.x = ix_min * cell
        origin.position.y = iy_min * cell
        origin.position.z = 0.0
        origin.orientation.w = 1.0
        return OccupancyGrid(
            grid=grid,
            resolution=cell,
            origin=origin,
            frame_id=frame_id,
        )
