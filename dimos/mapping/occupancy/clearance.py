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

"""Metric free-space reads over an occupancy grid.

Everything here answers a question *about* a costmap instead of producing a
new one: how far is the nearest obstacle, where is the closest spot the robot
could stand, which nearby cells are open enough to stand on.

``clearance_field`` is the shared primitive — meters from each cell to the
nearest obstacle, unbounded and unquantized. :func:`~dimos.mapping.occupancy.
gradient.gradient` encodes that same field into a clipped int8 planning cost
map; consumers that need the measurement rather than the encoding read it from
here instead of decoding it back out of a gradient.

Unknown space is never standable — an unexplored cell is not a free cell. It
is also not an obstacle, so it never shortens anyone's clearance.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import TYPE_CHECKING, Literal, TypeAlias, cast

import numpy as np
from scipy import ndimage

from dimos.mapping.occupancy.types import DEFAULT_OBSTACLE_THRESHOLD
from dimos.msgs.nav_msgs.OccupancyGrid import CostValues, OccupancyGrid

if TYPE_CHECKING:
    from numpy.typing import NDArray

CellState: TypeAlias = Literal["free", "occupied", "unknown"]


@dataclass(frozen=True)
class FreePoint:
    """A standable world point, with the measurements that qualified it."""

    x: float
    y: float
    clearance_m: float
    distance_m: float


def obstacle_mask(
    occupancy_grid: OccupancyGrid, obstacle_threshold: int = DEFAULT_OBSTACLE_THRESHOLD
) -> NDArray[np.bool_]:
    """Cells at or above the obstacle cost.

    Unknown (-1) is below any threshold, so it is never an obstacle here — it
    is excluded from free space by :func:`standable_mask` instead.
    """
    return occupancy_grid.grid >= obstacle_threshold


def clearance_field(
    occupancy_grid: OccupancyGrid, obstacle_threshold: int = DEFAULT_OBSTACLE_THRESHOLD
) -> NDArray[np.float64]:
    """Distance in meters from each cell to the nearest obstacle cell.

    Unbounded and unquantized: unlike the gradient cost map, a cell 8 m from
    anything reads as 8.0, not as a saturated maximum. Obstacle cells are 0.0.
    A map with no obstacles at all is ``inf`` everywhere.

    Args:
        occupancy_grid: The grid to measure.
        obstacle_threshold: Cell values >= this are obstacles (default: 50).
    """
    obstacles = obstacle_mask(occupancy_grid, obstacle_threshold)
    if not obstacles.any():
        return np.full(obstacles.shape, np.inf, dtype=np.float64)
    distance_cells = cast("NDArray[np.float64]", ndimage.distance_transform_edt(~obstacles))
    return distance_cells * occupancy_grid.resolution


def standable_mask(
    occupancy_grid: OccupancyGrid,
    min_clearance: float,
    obstacle_threshold: int = DEFAULT_OBSTACLE_THRESHOLD,
) -> NDArray[np.bool_]:
    """Cells the robot could stand in: known, not an obstacle, and clear enough.

    Args:
        occupancy_grid: The grid to search.
        min_clearance: Required distance to the nearest obstacle, in meters.
        obstacle_threshold: Cell values >= this are obstacles (default: 50).
    """
    field = clearance_field(occupancy_grid, obstacle_threshold)
    return _standable_mask(occupancy_grid, field, min_clearance, obstacle_threshold)


def cell_state(
    occupancy_grid: OccupancyGrid,
    x: float,
    y: float,
    obstacle_threshold: int = DEFAULT_OBSTACLE_THRESHOLD,
) -> CellState | None:
    """State of the cell containing a world point, or None if it is off-map.

    Distinguishes "outside the mapped area" (None) from "inside it but never
    observed" ("unknown"), which ``OccupancyGrid.cell_value`` conflates.
    """
    cell = occupancy_grid.cell_index((x, y, 0.0))
    if cell is None:
        return None
    column, row = cell
    value = int(occupancy_grid.grid[row, column])
    if value == CostValues.UNKNOWN:
        return "unknown"
    return "occupied" if value >= obstacle_threshold else "free"


def clearance_at(
    occupancy_grid: OccupancyGrid,
    x: float,
    y: float,
    obstacle_threshold: int = DEFAULT_OBSTACLE_THRESHOLD,
) -> float | None:
    """Meters from a world point to the nearest obstacle, or None if off-map."""
    cell = occupancy_grid.cell_index((x, y, 0.0))
    if cell is None:
        return None
    column, row = cell
    return float(clearance_field(occupancy_grid, obstacle_threshold)[row, column])


def nearest_free(
    occupancy_grid: OccupancyGrid,
    x: float,
    y: float,
    min_clearance: float,
    obstacle_threshold: int = DEFAULT_OBSTACLE_THRESHOLD,
) -> FreePoint | None:
    """Closest standable cell center to a world point, or None if there is none.

    Turns a rough target — an object's position, a spot beside its extent —
    into a point the robot could actually occupy. The target itself may be
    occupied, unknown, or off-map; only the answer has to be standable.

    Args:
        occupancy_grid: The grid to search.
        x: Target world x, in meters.
        y: Target world y, in meters.
        min_clearance: Required obstacle clearance, in meters.
        obstacle_threshold: Cell values >= this are obstacles (default: 50).

    Raises:
        ValueError: If min_clearance is not positive.
    """
    if min_clearance <= 0:
        raise ValueError(f"min_clearance must be positive, got {min_clearance}")
    field = clearance_field(occupancy_grid, obstacle_threshold)
    mask = _standable_mask(occupancy_grid, field, min_clearance, obstacle_threshold)
    if not mask.any():
        return None
    rows, columns, xs, ys = _masked_cell_centers(occupancy_grid, mask)
    i = int(np.argmin((xs - x) ** 2 + (ys - y) ** 2))
    px, py = float(xs[i]), float(ys[i])
    return FreePoint(
        x=px,
        y=py,
        clearance_m=float(field[rows[i], columns[i]]),
        distance_m=math.hypot(px - x, py - y),
    )


def free_space_near(
    occupancy_grid: OccupancyGrid,
    x: float,
    y: float,
    radius: float,
    min_clearance: float,
    max_results: int,
    spacing: float = 0.4,
    obstacle_threshold: int = DEFAULT_OBSTACLE_THRESHOLD,
) -> list[FreePoint]:
    """Standable points near a world point, most open first.

    Ranked by clearance so the caller sees the roomiest options first, and
    thinned so the results are spread over the free area rather than clustered
    in one corner of it. Ties break by distance, then by row-major cell order,
    so the same grid always yields the same answer.

    Args:
        occupancy_grid: The grid to search.
        x: Target world x, in meters.
        y: Target world y, in meters.
        radius: Search radius around the target, in meters.
        min_clearance: Required obstacle clearance, in meters.
        max_results: Stop after this many points.
        spacing: Discard a point within this distance of a better one, in meters.
        obstacle_threshold: Cell values >= this are obstacles (default: 50).

    Raises:
        ValueError: If radius, min_clearance, or max_results is not positive.
    """
    if radius <= 0:
        raise ValueError(f"radius must be positive, got {radius}")
    if min_clearance <= 0:
        raise ValueError(f"min_clearance must be positive, got {min_clearance}")
    if max_results <= 0:
        raise ValueError(f"max_results must be positive, got {max_results}")

    field = clearance_field(occupancy_grid, obstacle_threshold)
    mask = _standable_mask(occupancy_grid, field, min_clearance, obstacle_threshold)
    rows, columns, xs, ys = _masked_cell_centers(occupancy_grid, mask)
    distances = np.hypot(xs - x, ys - y)
    within = distances <= radius
    rows, columns, xs, ys = rows[within], columns[within], xs[within], ys[within]
    distances = distances[within]
    clearances = field[rows, columns]

    # Rank by openness; ties resolve by distance, then row-major cell order.
    order = np.lexsort((columns, rows, distances, -clearances))
    points: list[FreePoint] = []
    for i in order:
        px, py = float(xs[i]), float(ys[i])
        if any(math.hypot(px - kept.x, py - kept.y) < spacing for kept in points):
            continue
        points.append(
            FreePoint(
                x=px,
                y=py,
                clearance_m=float(clearances[i]),
                distance_m=float(distances[i]),
            )
        )
        if len(points) >= max_results:
            break
    return points


def _standable_mask(
    occupancy_grid: OccupancyGrid,
    field: NDArray[np.float64],
    min_clearance: float,
    obstacle_threshold: int,
) -> NDArray[np.bool_]:
    """standable_mask against an already-computed clearance field."""
    return cast(
        "NDArray[np.bool_]",
        (occupancy_grid.grid != CostValues.UNKNOWN)
        & (occupancy_grid.grid < obstacle_threshold)
        & (field >= min_clearance),
    )


def _masked_cell_centers(
    occupancy_grid: OccupancyGrid, mask: NDArray[np.bool_]
) -> tuple[NDArray[np.intp], NDArray[np.intp], NDArray[np.float64], NDArray[np.float64]]:
    """(rows, columns, world_x, world_y) of the masked cells' centers."""
    rows, columns = np.nonzero(mask)
    origin_x, origin_y = occupancy_grid.origin.position.x, occupancy_grid.origin.position.y
    xs = origin_x + (columns.astype(np.float64) + 0.5) * occupancy_grid.resolution
    ys = origin_y + (rows.astype(np.float64) + 0.5) * occupancy_grid.resolution
    return rows, columns, xs, ys
