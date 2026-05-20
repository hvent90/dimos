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

"""Pure functions for the multi-level surface map (MLS) planner.

No LCM, no Module — just numpy in, numpy/dicts out, so this is unit-testable
without the framework.
"""

from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass
import heapq
import math

import numpy as np

# Surface-graph node: (x_cell, y_cell, z_top_voxel).
Node = tuple[int, int, int]


@dataclass(frozen=True)
class SurfacePatch:
    """A standable surface in a single (x, y) voxel column.

    ``z_top`` is the voxel index of the top voxel of the solid the robot would
    stand on. World height of the surface center is ``(z_top + 0.5) * voxel_size``.
    """

    z_top: int


# Voxel column → list of surfaces (sorted by z_top ascending).
MLS = dict[tuple[int, int], list[SurfacePatch]]


def points_to_mls(
    points: np.ndarray,
    voxel_size: float,
    robot_height_voxels: int,
) -> MLS:
    """Extract a multi-level surface map from a voxel-center point cloud.

    Buckets points by (x_cell, y_cell), then walks each column bottom-to-top
    and emits a surface patch every time we leave a contiguous run of solid
    voxels with enough clear air above for the robot to stand. Surfaces with
    no upper bound (top of the column) are always emitted.
    """
    if points.size == 0:
        return {}
    if voxel_size <= 0.0:
        raise ValueError(f"voxel_size must be > 0, got {voxel_size}")
    if robot_height_voxels <= 0:
        raise ValueError(f"robot_height_voxels must be > 0, got {robot_height_voxels}")

    indices = np.floor(points / voxel_size).astype(np.int64)
    columns: dict[tuple[int, int], list[int]] = defaultdict(list)
    for kx, ky, kz in indices:
        columns[(int(kx), int(ky))].append(int(kz))

    mls: MLS = {}
    for col, zs in columns.items():
        surfaces = _extract_surfaces(zs, robot_height_voxels)
        if surfaces:
            mls[col] = surfaces
    return mls


def _extract_surfaces(z_indices: list[int], robot_height_voxels: int) -> list[SurfacePatch]:
    """Walk one column's z-indices and emit surface candidates.

    Algorithm: for each gap between consecutive populated voxels, if the gap is
    at least ``robot_height_voxels`` cells of clear air, the lower voxel is the
    top of a standable surface. The topmost populated voxel is always emitted
    (infinite air above).
    """
    z_sorted = sorted(set(z_indices))
    if not z_sorted:
        return []

    surfaces: list[SurfacePatch] = []
    prev_z = z_sorted[0]
    for z in z_sorted[1:]:
        gap = z - prev_z - 1
        if gap >= robot_height_voxels:
            surfaces.append(SurfacePatch(z_top=prev_z))
        prev_z = z
    surfaces.append(SurfacePatch(z_top=prev_z))
    return surfaces


def robot_height_in_voxels(robot_height: float, voxel_size: float) -> int:
    """Conservative clearance: round up so we never accept a too-cramped surface."""
    return max(1, math.ceil(robot_height / voxel_size))


def max_step_in_voxels(max_step_height: float, voxel_size: float) -> int:
    """Conservative step limit: round down so we never accept too-tall a step."""
    return max(0, math.floor(max_step_height / voxel_size))


def snap_to_surface(
    mls: MLS,
    x_cell: int,
    y_cell: int,
    z_world: float,
    voxel_size: float,
) -> Node | None:
    """Pick the surface patch in column (x_cell, y_cell) closest to ``z_world``.

    Used to resolve a world-frame pose into a surface-graph node. Returns
    None if the column has no surfaces (planner should refuse to plan).
    """
    patches = mls.get((x_cell, y_cell))
    if not patches:
        return None
    target = z_world / voxel_size
    closest = min(patches, key=lambda p: abs(p.z_top - target))
    return (x_cell, y_cell, closest.z_top)


# 8-connected horizontal neighbors in (dx, dy).
_NEIGHBORS_8: tuple[tuple[int, int], ...] = (
    (-1, -1),
    (-1, 0),
    (-1, 1),
    (0, -1),
    (0, 1),
    (1, -1),
    (1, 0),
    (1, 1),
)


def _edge_cost(dx: int, dy: int, dz: int) -> float:
    """Euclidean distance in voxel units between two surface nodes."""
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def _heuristic(node: Node, goal: Node) -> float:
    return _edge_cost(node[0] - goal[0], node[1] - goal[1], node[2] - goal[2])


def astar(
    mls: MLS,
    start: Node,
    goal: Node,
    max_step_voxels: int,
) -> list[Node] | None:
    """Surface-graph A* on an MLS. Returns the path or None if no path exists.

    Expansion: 8-connected horizontal neighbors. For each, candidate surfaces are
    pulled from the MLS and filtered by ``|dz| <= max_step_voxels``. Edge cost
    is 3D Euclidean in voxel units; the heuristic is straight-line distance to
    the goal, which is admissible since the true path is a sequence of
    grid steps that can't be shorter than the straight line.
    """
    if start == goal:
        return [start]
    if not _node_in_mls(mls, start) or not _node_in_mls(mls, goal):
        return None

    open_heap: list[tuple[float, int, Node]] = []
    g_score: dict[Node, float] = {start: 0.0}
    came_from: dict[Node, Node] = {}
    closed: set[Node] = set()
    counter = 0

    heapq.heappush(open_heap, (_heuristic(start, goal), counter, start))

    while open_heap:
        _, _, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        if current == goal:
            return _reconstruct(came_from, current)
        closed.add(current)

        cx, cy, cz = current
        for dx, dy in _NEIGHBORS_8:
            nx, ny = cx + dx, cy + dy
            for patch in mls.get((nx, ny), ()):
                nz = patch.z_top
                if abs(nz - cz) > max_step_voxels:
                    continue
                neighbor: Node = (nx, ny, nz)
                if neighbor in closed:
                    continue
                tentative_g = g_score[current] + _edge_cost(dx, dy, nz - cz)
                if tentative_g < g_score.get(neighbor, math.inf):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + _heuristic(neighbor, goal)
                    counter += 1
                    heapq.heappush(open_heap, (f_score, counter, neighbor))

    return None


def _node_in_mls(mls: MLS, node: Node) -> bool:
    return any(p.z_top == node[2] for p in mls.get((node[0], node[1]), ()))


def _reconstruct(came_from: dict[Node, Node], end: Node) -> list[Node]:
    path = [end]
    while end in came_from:
        end = came_from[end]
        path.append(end)
    path.reverse()
    return path


def surface_centers(mls: MLS, voxel_size: float) -> np.ndarray:
    """Flatten an MLS to an (N, 3) array of surface-patch world-frame centers.

    Useful for publishing the MLS as a debug PointCloud2.
    """
    if not mls:
        return np.zeros((0, 3), dtype=np.float32)
    half = 0.5 * voxel_size
    out = np.empty((sum(len(v) for v in mls.values()), 3), dtype=np.float32)
    i = 0
    for (kx, ky), patches in mls.items():
        for p in patches:
            out[i, 0] = kx * voxel_size + half
            out[i, 1] = ky * voxel_size + half
            out[i, 2] = p.z_top * voxel_size + half
            i += 1
    return out
