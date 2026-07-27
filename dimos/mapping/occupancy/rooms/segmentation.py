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

"""Room segmentation over an occupancy grid.

Watershed on the free-space distance transform (HOV-SG's BEV-watershed family):
seeds are pockets of free space with clearance above a door's half-width, the
priority flood grows them downhill along the Euclidean distance transform,
so region boundaries land in doorways — the clearance minima. Small regions
merge into their dominant neighbor; elongated low-clearance regions classify
as corridors. Doorways are the interface midpoints between adjacent regions.
"""

from __future__ import annotations

from dataclasses import dataclass
import heapq
from typing import Literal

import cv2
import numpy as np
from numpy.typing import NDArray
from scipy import ndimage

from dimos.mapping.occupancy.rooms.geometry import (
    DEFAULT_POLYGON_EPSILON_CELLS,
    free_clearance,
    free_space,
    region_geometry,
)
from dimos.mapping.occupancy.rooms.polygons import cells_to_world
from dimos.mapping.occupancy.types import DEFAULT_OBSTACLE_THRESHOLD
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid


@dataclass(frozen=True)
class RoomSegmentationConfig:
    """Tuning knobs, defaults verified on the china-office recording."""

    # Cells with 0 <= cost < this count as traversable free space; at or above
    # it they are obstacles, on the same threshold the rest of the stack uses.
    free_cost_max: int = DEFAULT_OBSTACLE_THRESHOLD
    # Watershed seeds need clearance above a door's half-width (m).
    door_half_width_m: float = 0.45
    # Regions smaller than this merge into their dominant neighbor (m^2).
    min_room_area_m2: float = 1.5
    # Corridor test: elongation above this AND mean clearance below the cap.
    corridor_min_elongation: float = 2.5
    corridor_max_mean_clearance_m: float = 0.55
    # Douglas-Peucker tolerance for region outline polygons (in cells).
    polygon_epsilon_cells: float = DEFAULT_POLYGON_EPSILON_CELLS


@dataclass(frozen=True)
class Region:
    """One segmented region with a world-frame outline polygon."""

    id: int
    kind: Literal["room", "corridor"]
    area_m2: float
    centroid_xy: tuple[float, float]
    # The region's most open point (max clearance) — a good nav target.
    anchor_xy: tuple[float, float]
    max_clearance_m: float
    polygon: NDArray[np.float64]  # (N, 2) world xy outline


@dataclass(frozen=True)
class Doorway:
    """Interface midpoint between two adjacent regions."""

    between: tuple[int, int]  # region ids, low first
    position_xy: tuple[float, float]
    approx_width_m: float


@dataclass(frozen=True)
class RoomSegmentation:
    """Full segmentation result over one occupancy grid."""

    regions: tuple[Region, ...]
    doorways: tuple[Doorway, ...]
    labels: NDArray[np.int32]  # per-cell region id, 0 = not assigned
    explored_fraction: float
    resolution: float
    origin_xy: tuple[float, float]
    derived_ts: float

    def rooms(self) -> tuple[Region, ...]:
        return tuple(r for r in self.regions if r.kind == "room")

    def corridors(self) -> tuple[Region, ...]:
        return tuple(r for r in self.regions if r.kind == "corridor")


def _watershed(
    free: NDArray[np.bool_], edt: NDArray[np.float64], seed_clearance_m: float
) -> NDArray[np.int32]:
    """Priority-flood watershed on -EDT, seeded at clearance > seed_clearance_m."""
    seeds = edt > seed_clearance_m
    seed_labels, _ = ndimage.label(seeds)
    out = np.where(seeds, seed_labels, 0).astype(np.int32)
    h, w = free.shape
    # The flood itself is inherently sequential (a priority queue over cells),
    # so this loops in Python; grids are small (hundreds x hundreds of cells).
    heap: list[tuple[float, int, int]] = []
    ys, xs = np.nonzero(seeds)
    for y, x in zip(ys.tolist(), xs.tolist(), strict=True):
        heapq.heappush(heap, (-edt[y, x], y, x))
    while heap:
        _, y, x = heapq.heappop(heap)
        label = out[y, x]
        for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            ny, nx = y + dy, x + dx
            if 0 <= ny < h and 0 <= nx < w and free[ny, nx] and out[ny, nx] == 0:
                out[ny, nx] = label
                heapq.heappush(heap, (-edt[ny, nx], ny, nx))
    return out


def _merge_small(labels: NDArray[np.int32], min_cells: float) -> tuple[NDArray[np.int32], int]:
    """Merge regions under min_cells into the neighbor sharing the most boundary."""
    changed = True
    while changed:
        changed = False
        ids, counts = np.unique(labels[labels > 0], return_counts=True)
        for region_id, count in sorted(
            zip(ids.tolist(), counts.tolist(), strict=True), key=lambda t: t[1]
        ):
            if count >= min_cells:
                continue
            mask = labels == region_id
            ring = ndimage.binary_dilation(mask) & ~mask
            neighbors = labels[ring]
            neighbors = neighbors[neighbors > 0]
            if neighbors.size:
                labels[mask] = np.bincount(neighbors).argmax()
                changed = True
                break
    ids = np.unique(labels[labels > 0])
    remap = np.zeros(int(labels.max()) + 1, dtype=np.int32)
    for new_id, old_id in enumerate(ids.tolist(), start=1):
        remap[old_id] = new_id
    return remap[labels], len(ids)


def _doorways(
    labels: NDArray[np.int32], resolution: float, origin_xy: tuple[float, float]
) -> list[Doorway]:
    """Cluster label-interface cells per region pair; midpoint = doorway."""
    by_pair: dict[tuple[int, int], list[tuple[int, int]]] = {}
    for dy, dx in ((1, 0), (0, 1)):
        a = labels[: -dy or None, : -dx or None]
        b = labels[dy:, dx:]
        both = (a > 0) & (b > 0) & (a != b)
        ys, xs = np.nonzero(both)
        for y, x in zip(ys.tolist(), xs.tolist(), strict=True):
            pair = (min(a[y, x], b[y, x]), max(a[y, x], b[y, x]))
            by_pair.setdefault((int(pair[0]), int(pair[1])), []).append((y, x))
    doorways = []
    for pair, cells in sorted(by_pair.items()):
        cy, cx = np.asarray(cells, dtype=np.float64).mean(axis=0)
        # Interface cells are counted along both axes — halve for width.
        width_m = len(cells) * resolution / 2
        x, y = cells_to_world((cx, cy), resolution, origin_xy)
        doorways.append(
            Doorway(
                between=pair,
                position_xy=(float(x), float(y)),
                approx_width_m=round(width_m, 2),
            )
        )
    return doorways


def segment_rooms(
    grid: OccupancyGrid, config: RoomSegmentationConfig = RoomSegmentationConfig()
) -> RoomSegmentation:
    """Segment an occupancy grid into rooms/corridors with doorways."""
    origin_xy = (float(grid.origin.position.x), float(grid.origin.position.y))
    resolution = float(grid.resolution)
    free, _occupied, unknown = free_space(grid, config.free_cost_max)
    edt = free_clearance(free, resolution)
    labels = _watershed(free, edt, config.door_half_width_m)
    min_cells = config.min_room_area_m2 / (resolution * resolution)
    labels, n_regions = _merge_small(labels, min_cells)

    regions = []
    for region_id in range(1, n_regions + 1):
        mask = labels == region_id
        geometry = region_geometry(
            mask, edt, resolution, origin_xy, epsilon_cells=config.polygon_epsilon_cells
        )
        ys, xs = np.nonzero(mask)
        covariance = np.cov(np.vstack([xs, ys]).astype(np.float64))
        eigenvalues = np.sort(np.linalg.eigvalsh(covariance))[::-1]
        elongation = float(np.sqrt(eigenvalues[0] / max(eigenvalues[1], 1e-6)))
        mean_clearance = float(edt[mask].mean())
        kind: Literal["room", "corridor"] = (
            "corridor"
            if (
                elongation > config.corridor_min_elongation
                and mean_clearance < config.corridor_max_mean_clearance_m
            )
            else "room"
        )
        regions.append(
            Region(
                id=region_id,
                kind=kind,
                area_m2=geometry.area_m2,
                centroid_xy=geometry.centroid_xy,
                anchor_xy=geometry.anchor_xy,
                max_clearance_m=geometry.max_clearance_m,
                polygon=geometry.polygon,
            )
        )

    return RoomSegmentation(
        regions=tuple(regions),
        doorways=tuple(_doorways(labels, resolution, origin_xy)),
        labels=labels,
        explored_fraction=round(1.0 - float(unknown.sum()) / grid.grid.size, 3),
        resolution=resolution,
        origin_xy=origin_xy,
        derived_ts=float(grid.ts),
    )


REGION_PALETTE = [
    (31, 119, 180), (255, 127, 14), (44, 160, 44), (214, 39, 40), (148, 103, 189),
    (140, 86, 75), (227, 119, 194), (23, 190, 207), (188, 189, 34), (127, 127, 127),
]  # fmt: skip


def render_regions(
    grid: OccupancyGrid,
    segmentation: RoomSegmentation,
    config: RoomSegmentationConfig = RoomSegmentationConfig(),
    upscale: int = 4,
) -> NDArray[np.uint8]:
    """Debug render: region tints + ids + doorway rings over the grid (RGB)."""
    free, occupied, unknown = free_space(grid, config.free_cost_max)
    img = np.zeros((*grid.grid.shape, 3), dtype=np.uint8)
    img[unknown] = (150, 150, 150)
    img[free] = (255, 255, 255)
    img[occupied] = (20, 20, 20)
    tint = img.copy()
    for region in segmentation.regions:
        color = np.array(REGION_PALETTE[(region.id - 1) % len(REGION_PALETTE)], np.uint8)
        tint[segmentation.labels == region.id] = color
    img = cv2.addWeighted(tint, 0.45, img, 0.55, 0)
    px = cv2.resize(img[::-1], None, fx=upscale, fy=upscale, interpolation=cv2.INTER_NEAREST)

    height = grid.grid.shape[0]
    ox, oy = segmentation.origin_xy

    def world_to_px(x: float, y: float) -> tuple[int, int]:
        cx = (x - ox) / segmentation.resolution
        cy = (y - oy) / segmentation.resolution
        return int(cx * upscale), int((height - cy) * upscale)

    for doorway in segmentation.doorways:
        cv2.circle(px, world_to_px(*doorway.position_xy), 2 * upscale, (255, 255, 0), 2)
    for region in segmentation.regions:
        pt = world_to_px(*region.anchor_xy)
        label = f"{region.id}"
        cv2.putText(px, label, pt, cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255), 5, cv2.LINE_AA)
        cv2.putText(px, label, pt, cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 0), 2, cv2.LINE_AA)
    return px.astype(np.uint8)
