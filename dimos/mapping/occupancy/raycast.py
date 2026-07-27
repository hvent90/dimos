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

"""Line-of-sight queries over a 2D occupancy grid.

Distinct from :mod:`dimos.mapping.ray_tracing`, which traces 3D voxels to
carve free space out of sensor returns. This marches a costmap in the plane to
answer "what is in the way from here, along this heading" — and reports *what*
stopped the ray, since running out of map, running into unexplored space, and
hitting a wall mean different things to whoever asked.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Literal, TypeAlias

from dimos.mapping.occupancy.types import DEFAULT_OBSTACLE_THRESHOLD
from dimos.msgs.nav_msgs.OccupancyGrid import CostValues, OccupancyGrid

RayOutcome: TypeAlias = Literal["obstacle", "unknown", "map_edge", "max_range"]


@dataclass(frozen=True)
class RayResult:
    """Where a ray stopped, and why."""

    outcome: RayOutcome
    distance_m: float
    x: float
    y: float


def raycast(
    occupancy_grid: OccupancyGrid,
    x: float,
    y: float,
    angle_rad: float,
    max_range_m: float,
    obstacle_threshold: int = DEFAULT_OBSTACLE_THRESHOLD,
) -> RayResult | None:
    """March the grid from a world point along a heading until something stops it.

    Steps at half the cell resolution, so no cell along the ray is skipped.
    Returns None if the origin itself is off-map — a ray that starts nowhere
    has no answer, as distinct from one that immediately leaves the map.

    Args:
        occupancy_grid: The grid to march.
        x: Ray origin world x, in meters.
        y: Ray origin world y, in meters.
        angle_rad: Heading in radians (0 = +x, counterclockwise).
        max_range_m: Give up after this distance, in meters.
        obstacle_threshold: Cell values >= this are obstacles (default: 50).

    Raises:
        ValueError: If max_range_m is not positive.
    """
    if max_range_m <= 0:
        raise ValueError(f"max_range_m must be positive, got {max_range_m}")
    if occupancy_grid.cell_index((x, y, 0.0)) is None:
        return None

    step = occupancy_grid.resolution / 2.0
    dx, dy = math.cos(angle_rad), math.sin(angle_rad)
    outcome: RayOutcome = "max_range"
    distance = max_range_m

    for i in range(1, int(max_range_m / step) + 1):
        d = i * step
        cell = occupancy_grid.cell_index((x + dx * d, y + dy * d, 0.0))
        if cell is None:
            outcome, distance = "map_edge", d
            break
        column, row = cell
        value = int(occupancy_grid.grid[row, column])
        if value == CostValues.UNKNOWN:
            outcome, distance = "unknown", d
            break
        if value >= obstacle_threshold:
            outcome, distance = "obstacle", d
            break

    return RayResult(
        outcome=outcome,
        distance_m=distance,
        x=x + dx * distance,
        y=y + dy * distance,
    )
