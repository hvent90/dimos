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

"""The one way a region on a grid is measured.

Every region carries the same five properties — outline, area, centroid,
anchor, max clearance — and they only compare across regions if they were
measured the same way. Segmentation derives regions from the grid; an agent
can draw one by hand. Both land in the same store and get read back by the
same queries, so both measure here.

The measurement is over *free cells*, with the clearance field taken over
free space as a whole: unknown space acts as a wall (it is not somewhere the
robot has seen it can stand), and clearance is the distance to the nearest
such wall, not to the region's own boundary. That is what makes the anchor a
usable navigation target and the area an honest floor area.

Measuring an agent-drawn outline against itself would answer a different
question — how deep is the polygon you drew, over ground that may be solid
obstacle — and would quietly give ``area_m2`` and ``max_clearance_m`` two
meanings depending on who created the region. So an outline is only ever the
outline: :func:`polygon_region_geometry` keeps it verbatim and measures the
free cells underneath it.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, cast

import numpy as np
from scipy import ndimage

from dimos.mapping.occupancy.clearance import obstacle_mask
from dimos.mapping.occupancy.rooms.polygons import (
    cells_to_world,
    mask_to_polygon,
    points_in_polygon,
)
from dimos.mapping.occupancy.types import DEFAULT_OBSTACLE_THRESHOLD
from dimos.msgs.nav_msgs.OccupancyGrid import CostValues, OccupancyGrid

if TYPE_CHECKING:
    from numpy.typing import NDArray

# Douglas-Peucker tolerance for region outlines, in cells.
DEFAULT_POLYGON_EPSILON_CELLS = 1.5


@dataclass(frozen=True)
class RegionGeometry:
    """Derived geometric properties of one region outline."""

    polygon: NDArray[np.float64]  # (N, 2) world xy
    area_m2: float
    centroid_xy: tuple[float, float]
    # The region's most open point (max clearance) — a good nav target.
    anchor_xy: tuple[float, float]
    max_clearance_m: float


def free_space(
    grid: OccupancyGrid, free_cost_max: int = DEFAULT_OBSTACLE_THRESHOLD
) -> tuple[NDArray[np.bool_], NDArray[np.bool_], NDArray[np.bool_]]:
    """(free, occupied, unknown) cell masks. The three are mutually exclusive."""
    occupied = obstacle_mask(grid, free_cost_max)
    unknown = grid.grid == CostValues.UNKNOWN
    free = (grid.grid >= 0) & ~occupied
    # Drop free-space speckles that aren't part of a meaningful component.
    free = ndimage.binary_opening(free, iterations=1)
    return free, occupied, unknown


def free_clearance(free: NDArray[np.bool_], resolution: float) -> NDArray[np.float64]:
    """Meters from each free cell to the nearest non-free cell.

    Unknown space is non-free here, so it bounds clearance the same way an
    obstacle does. This is deliberately *not*
    :func:`dimos.mapping.occupancy.clearance.clearance_field`, whose
    background is obstacles alone — unexplored ground would otherwise read as
    wide-open floor.
    """
    edt = cast("NDArray[np.float64]", ndimage.distance_transform_edt(free))
    return edt * resolution


def polygon_cell_mask(grid: OccupancyGrid, polygon: NDArray[np.float64]) -> NDArray[np.bool_]:
    """Grid-shaped mask of the cells whose centers fall inside the polygon."""
    ox, oy = float(grid.origin.position.x), float(grid.origin.position.y)
    res = float(grid.resolution)
    mask = np.zeros((grid.height, grid.width), dtype=bool)
    # Only rasterize the polygon's bounding box; a room is a small part of a
    # building-scale map.
    c0 = int(np.clip(np.floor((polygon[:, 0].min() - ox) / res) - 1, 0, grid.width))
    c1 = int(np.clip(np.ceil((polygon[:, 0].max() - ox) / res) + 1, 0, grid.width))
    r0 = int(np.clip(np.floor((polygon[:, 1].min() - oy) / res) - 1, 0, grid.height))
    r1 = int(np.clip(np.ceil((polygon[:, 1].max() - oy) / res) + 1, 0, grid.height))
    if c0 >= c1 or r0 >= r1:
        return mask
    xs = ox + (np.arange(c0, c1, dtype=np.float64) + 0.5) * res
    ys = oy + (np.arange(r0, r1, dtype=np.float64) + 0.5) * res
    gx, gy = np.meshgrid(xs, ys)
    inside = points_in_polygon(np.column_stack([gx.ravel(), gy.ravel()]), polygon)
    mask[r0:r1, c0:c1] = inside.reshape(gy.shape)
    return mask


def region_geometry(
    mask: NDArray[np.bool_],
    clearance_m: NDArray[np.float64],
    resolution: float,
    origin_xy: tuple[float, float],
    *,
    polygon: NDArray[np.float64] | None = None,
    epsilon_cells: float = DEFAULT_POLYGON_EPSILON_CELLS,
) -> RegionGeometry:
    """Measure one region from its free cells and the free-space clearance.

    Args:
        mask: (rows, cols) cells belonging to the region. Free cells only —
            pass ``mask & free`` if the mask came from an outline.
        clearance_m: (rows, cols) field from :func:`free_clearance`.
        resolution: Cell size in meters.
        origin_xy: World coordinates of the map's (0, 0) cell corner.
        polygon: Outline override, kept verbatim. Defaults to the simplified
            contour of ``mask``.
        epsilon_cells: Douglas-Peucker tolerance for that contour.
    """
    if not mask.any():
        raise ValueError("region mask has no cells")
    area_m2 = float(mask.sum()) * resolution * resolution
    rows, cols = np.nonzero(mask)
    centroid_x, centroid_y = cells_to_world((cols.mean(), rows.mean()), resolution, origin_xy)
    anchor_row, anchor_col = np.unravel_index(
        int(np.argmax(np.where(mask, clearance_m, -1.0))), mask.shape
    )
    anchor_x, anchor_y = cells_to_world((anchor_col, anchor_row), resolution, origin_xy)
    return RegionGeometry(
        polygon=(
            polygon
            if polygon is not None
            else mask_to_polygon(mask, resolution, origin_xy, epsilon_cells)
        ),
        area_m2=round(area_m2, 1),
        centroid_xy=(float(centroid_x), float(centroid_y)),
        anchor_xy=(float(anchor_x), float(anchor_y)),
        max_clearance_m=round(float(clearance_m[anchor_row, anchor_col]), 2),
    )


def polygon_region_geometry(
    grid: OccupancyGrid,
    polygon: NDArray[np.float64],
    *,
    free_cost_max: int = DEFAULT_OBSTACLE_THRESHOLD,
    epsilon_cells: float = DEFAULT_POLYGON_EPSILON_CELLS,
) -> RegionGeometry | None:
    """Measure a hand-drawn outline against the grid it was drawn over.

    The outline is authoritative and comes back unchanged; only the
    measurements are read off the free cells inside it — so an agent-edited
    room reports the same kind of area and clearance a segmented one does.

    Returns None when the outline covers no free space: there is nothing to
    measure, and what that means (reject the edit, fall back to a
    grid-independent estimate) is the caller's call.
    """
    free, _occupied, _unknown = free_space(grid, free_cost_max)
    inside = polygon_cell_mask(grid, polygon) & free
    if not inside.any():
        return None
    origin_xy = (float(grid.origin.position.x), float(grid.origin.position.y))
    return region_geometry(
        inside,
        free_clearance(free, float(grid.resolution)),
        float(grid.resolution),
        origin_xy,
        polygon=polygon,
        epsilon_cells=epsilon_cells,
    )
