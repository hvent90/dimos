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

"""Hand-drawn region edits, measured against the map they are drawn over.

Segmentation derives regions from the grid; an agent redraws them by hand.
The grid work that makes a hand edit as trustworthy as a derived region
lives here — free-cell contiguity for a merge, the cut for a split, the
fallback when there is no map to measure against — so the skills that
expose these edits stay argument validation plus a graph write, and the
geometry is testable without a running module.

Every result is measured by :mod:`dimos.mapping.occupancy.rooms.geometry`,
so an edited room's area and anchor mean exactly what a derived room's do.
A ``None`` return is the geometry refusing the edit: the outlines do not
touch, or the line does not divide. What to tell the agent is the caller's
call.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
from typing import TYPE_CHECKING, cast

import numpy as np
from scipy import ndimage

from dimos.mapping.occupancy.rooms.geometry import (
    RegionGeometry,
    free_clearance,
    free_space,
    polygon_cell_mask,
    polygon_region_geometry,
    region_geometry,
)
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid

if TYPE_CHECKING:
    from numpy.typing import NDArray

# A split half this small is a sliver the line clipped off a corner, not a
# room: the cut missed, and saying so beats creating a region with no floor.
DEFAULT_MIN_SPLIT_AREA_M2 = 0.25


@dataclass(frozen=True)
class SplitRegions:
    """The two halves a cut produced, plus the doorway it implies."""

    a: RegionGeometry
    b: RegionGeometry
    # Where the cut crosses the region — the two halves are joined here.
    doorway_xy: tuple[float, float]
    seam_width_m: float


def _largest_component(mask: NDArray[np.bool_]) -> NDArray[np.bool_]:
    """The biggest connected blob in ``mask``; a cut can strand islands."""
    labels, count = ndimage.label(mask)
    if count <= 1:
        return mask
    sizes = ndimage.sum_labels(mask, labels, index=range(1, count + 1))
    keep: NDArray[np.bool_] = labels == (1 + int(np.argmax(sizes)))
    return keep


def analytic_region_geometry(polygon: NDArray[np.float64]) -> RegionGeometry:
    """Measure an outline by shoelace alone, with no grid to consult.

    The honest answer when no map has arrived: the polygon's own area and
    centroid. Clearance is reported as zero rather than guessed, since
    nothing here knows where the walls are.
    """
    x, y = polygon[:, 0], polygon[:, 1]
    x2, y2 = np.roll(x, -1), np.roll(y, -1)
    cross = x * y2 - x2 * y
    area2 = float(cross.sum())
    if abs(area2) < 1e-9:
        centroid = (float(x.mean()), float(y.mean()))
    else:
        centroid = (
            float(((x + x2) * cross).sum() / (3.0 * area2)),
            float(((y + y2) * cross).sum() / (3.0 * area2)),
        )
    return RegionGeometry(
        polygon=polygon,
        area_m2=round(abs(area2) / 2.0, 1),
        centroid_xy=(round(centroid[0], 3), round(centroid[1], 3)),
        anchor_xy=centroid,
        max_clearance_m=0.0,
    )


def boundary_region(grid: OccupancyGrid | None, polygon: NDArray[np.float64]) -> RegionGeometry:
    """Measure an agent-drawn outline, however much map there is to do it with.

    The outline is authoritative either way and comes back unchanged. With
    a grid the numbers are read off the free cells under it, matching a
    derived room; without one (or with an outline over no free space) they
    fall back to :func:`analytic_region_geometry`.
    """
    if grid is None:
        return analytic_region_geometry(polygon)
    measured = polygon_region_geometry(grid, polygon)
    return measured if measured is not None else analytic_region_geometry(polygon)


def merged_region(
    grid: OccupancyGrid, polygons: Sequence[NDArray[np.float64]]
) -> RegionGeometry | None:
    """Measure the union of adjacent regions, or None if they do not touch.

    Contiguity is judged over free cells: adjacent rooms join through their
    doorway, while a solid wall between them leaves two components. That is
    what stops a merge inventing one room out of two disconnected areas.
    """
    free, _occupied, _unknown = free_space(grid)
    resolution = float(grid.resolution)
    origin_xy = (float(grid.origin.position.x), float(grid.origin.position.y))
    masks = [polygon_cell_mask(grid, polygon) & free for polygon in polygons]
    if not masks or not all(mask.any() for mask in masks):
        return None
    union = np.logical_or.reduce(masks)
    # Close first: rooms that meet at a doorway are one component, but a
    # one-cell rasterization seam between them should not read as a wall.
    closed = cast(
        "NDArray[np.bool_]",
        ndimage.binary_closing(union, structure=np.ones((3, 3)), iterations=2),
    )
    labels, _ = ndimage.label(closed)
    label_sets = [set(np.unique(labels[mask])) - {0} for mask in masks]
    common = set.intersection(*label_sets)
    if not common:
        return None
    merged_mask = (labels == min(common)) & free
    return region_geometry(merged_mask, free_clearance(free, resolution), resolution, origin_xy)


def split_region(
    grid: OccupancyGrid,
    polygon: NDArray[np.float64],
    p0: NDArray[np.float64],
    p1: NDArray[np.float64],
    min_area_m2: float = DEFAULT_MIN_SPLIT_AREA_M2,
) -> SplitRegions | None:
    """Cut a region along the line through ``p0``/``p1``.

    Halves are ordered by the line's direction: ``a`` is left of
    ``p0 -> p1``, ``b`` is right. Each is reduced to its largest connected
    blob, so a line that clips a corner does not leave debris attached.

    Returns None when the cut does not divide — either side smaller than
    ``min_area_m2`` means the line missed the region rather than split it.
    """
    free, _occupied, _unknown = free_space(grid)
    resolution = float(grid.resolution)
    origin_xy = (float(grid.origin.position.x), float(grid.origin.position.y))
    mask = polygon_cell_mask(grid, polygon) & free
    rows, cols = np.nonzero(mask)
    if rows.size == 0:
        return None

    xs = origin_xy[0] + (cols.astype(np.float64) + 0.5) * resolution
    ys = origin_xy[1] + (rows.astype(np.float64) + 0.5) * resolution
    cross = (p1[0] - p0[0]) * (ys - p0[1]) - (p1[1] - p0[1]) * (xs - p0[0])
    side_a = np.zeros_like(mask)
    side_b = np.zeros_like(mask)
    side_a[rows[cross > 0], cols[cross > 0]] = True
    side_b[rows[cross <= 0], cols[cross <= 0]] = True
    side_a = _largest_component(side_a)
    side_b = _largest_component(side_b)

    min_cells = max(4, int(min_area_m2 / (resolution * resolution)))
    if side_a.sum() < min_cells or side_b.sum() < min_cells:
        return None

    clearance = free_clearance(free, resolution)
    # The doorway sits where the cut crosses the region's middle, and its
    # width is the length of the seam the two halves share.
    centroid = polygon.mean(axis=0)
    direction = p1 - p0
    t = float(np.dot(centroid - p0, direction) / np.dot(direction, direction))
    doorway = p0 + t * direction
    seam_pairs = int(
        (side_a[1:, :] & side_b[:-1, :]).sum()
        + (side_a[:-1, :] & side_b[1:, :]).sum()
        + (side_a[:, 1:] & side_b[:, :-1]).sum()
        + (side_a[:, :-1] & side_b[:, 1:]).sum()
    )
    return SplitRegions(
        a=region_geometry(side_a, clearance, resolution, origin_xy),
        b=region_geometry(side_b, clearance, resolution, origin_xy),
        doorway_xy=(float(doorway[0]), float(doorway[1])),
        seam_width_m=round(seam_pairs * resolution, 2),
    )
