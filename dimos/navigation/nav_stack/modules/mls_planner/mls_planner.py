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

"""Multi-Level Surface (MLS) path planner.

Extracts walkable surfaces from a voxelized global map, builds a sparse
node graph over those surfaces, and plans paths via local A* plus
shortest-path search on the graph. Skeleton — algorithm is filled in
piecewise.
"""

from __future__ import annotations

import heapq
import math
import time
from typing import Any

from dimos_lcm.geometry_msgs import (
    Point as LCMPoint,
    Pose as LCMPose,
    PoseStamped as LCMPoseStamped,
    Quaternion as LCMQuaternion,
)
from dimos_lcm.nav_msgs import Path as LCMPath
from dimos_lcm.std_msgs import Header as LCMHeader, Time as LCMTime
import networkx as nx
import numpy as np
from scipy import ndimage

from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.nav_msgs.LineSegments3D import LineSegments3D
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.nav_msgs.Path import Path, sec_nsec
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

SURFACE_DILATION_PASSES = 3
SURFACE_EROSION_PASSES = 3

NODE_SPACING_M = 2.0
NODE_Z_TOLERANCE_M = 1.0
NODE_STEP_THRESHOLD_M = 0.25
NODE_MAX_EDGE_COST_M = 3.0
NODE_SUB_SAMPLE_STRIDE = 20


class MLSPlannerConfig(ModuleConfig):
    world_frame: str = "map"
    voxel_size: float = 0.1
    robot_height: float = 1.5


def _extract_surfaces(points: np.ndarray, voxel_size: float, robot_height: float) -> np.ndarray:
    """Find walkable surface tops in a voxelized point cloud.

    Iterate through all the columns, find continuous areas of
    free space. If the free space column is at least robot height,
    add the bottom of this range as a surface.
    """
    if len(points) == 0:
        return np.zeros((0, 3), dtype=np.float32)

    indices = np.floor(points / voxel_size).astype(np.int64)
    ix, iy, iz = indices[:, 0], indices[:, 1], indices[:, 2]

    order = np.lexsort((iz, iy, ix))
    sx, sy, sz = ix[order], iy[order], iz[order]

    height_cells = int(np.ceil(robot_height / voxel_size))

    next_same_col = np.zeros(len(sx), dtype=bool)
    next_same_col[:-1] = (sx[:-1] == sx[1:]) & (sy[:-1] == sy[1:])

    gap = np.empty(len(sx), dtype=np.int64)
    gap[:-1] = sz[1:] - sz[:-1]
    gap[-1] = 0

    is_surface = (~next_same_col) | (gap > height_cells)

    surf_ix = sx[is_surface]
    surf_iy = sy[is_surface]
    surf_iz = sz[is_surface]

    surf_ix, surf_iy, surf_iz = _close_surface_holes(
        surf_ix, surf_iy, surf_iz, SURFACE_DILATION_PASSES, SURFACE_EROSION_PASSES
    )

    x = (surf_ix.astype(np.float32) + 0.5) * voxel_size
    y = (surf_iy.astype(np.float32) + 0.5) * voxel_size
    z = (surf_iz.astype(np.float32) + 1.0) * voxel_size
    return np.column_stack([x, y, z]).astype(np.float32)


def _close_surface_holes(
    surf_ix: np.ndarray,
    surf_iy: np.ndarray,
    surf_iz: np.ndarray,
    dilation_passes: int,
    erosion_passes: int,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Do dilation then erosion on the surface map at each z level.

    Closes a lot of small holes that are artifacts missing lidar points.
    """
    if len(surf_ix) == 0 or (dilation_passes <= 0 and erosion_passes <= 0):
        return surf_ix, surf_iy, surf_iz

    pad = max(dilation_passes, 0)
    new_ix: list[np.ndarray] = []
    new_iy: list[np.ndarray] = []
    new_iz: list[np.ndarray] = []
    for level_iz in np.unique(surf_iz):
        sel = surf_iz == level_iz
        lx = surf_ix[sel]
        ly = surf_iy[sel]
        x0, x1 = int(lx.min()), int(lx.max())
        y0, y1 = int(ly.min()), int(ly.max())
        w = x1 - x0 + 1 + 2 * pad
        h = y1 - y0 + 1 + 2 * pad
        mask = np.zeros((h, w), dtype=bool)
        mask[ly - y0 + pad, lx - x0 + pad] = True
        if dilation_passes > 0:
            mask = ndimage.binary_dilation(mask, iterations=dilation_passes)
        if erosion_passes > 0:
            mask = ndimage.binary_erosion(mask, iterations=erosion_passes)
        ys, xs = np.where(mask)
        new_ix.append(xs.astype(np.int64) + x0 - pad)
        new_iy.append(ys.astype(np.int64) + y0 - pad)
        new_iz.append(np.full(len(xs), level_iz, dtype=np.int64))

    return (
        np.concatenate(new_ix),
        np.concatenate(new_iy),
        np.concatenate(new_iz),
    )


class _GridHash:
    """Sparse 2D bucket index over integer cell coordinates."""

    def __init__(self, bucket_size_cells: int) -> None:
        self._bucket_size = max(1, bucket_size_cells)
        self._buckets: dict[tuple[int, int], list[int]] = {}

    def _key(self, ix: int, iy: int) -> tuple[int, int]:
        return (ix // self._bucket_size, iy // self._bucket_size)

    def add(self, node_id: int, ix: int, iy: int) -> None:
        self._buckets.setdefault(self._key(ix, iy), []).append(node_id)

    def nearby(self, ix: int, iy: int, radius_cells: int) -> list[int]:
        bucket_radius = radius_cells // self._bucket_size + 1
        bx, by = self._key(ix, iy)
        result: list[int] = []
        for dbx in range(-bucket_radius, bucket_radius + 1):
            for dby in range(-bucket_radius, bucket_radius + 1):
                ids = self._buckets.get((bx + dbx, by + dby))
                if ids:
                    result.extend(ids)
        return result


def _build_surface_lookup(
    sx: np.ndarray, sy: np.ndarray, sz: np.ndarray
) -> dict[tuple[int, int], np.ndarray]:
    """Group surface cells by XY column for fast neighbor lookup in inner A*."""
    by_column: dict[tuple[int, int], list[int]] = {}
    for ix_, iy_, iz_ in zip(sx.tolist(), sy.tolist(), sz.tolist(), strict=True):
        by_column.setdefault((ix_, iy_), []).append(iz_)
    return {key: np.array(sorted(vs), dtype=np.int64) for key, vs in by_column.items()}


def _surface_dijkstra(
    surface_lookup: dict[tuple[int, int], np.ndarray],
    start: tuple[int, int, int],
    voxel_size: float,
    step_threshold_cells: int,
    max_cost: float,
    targets: set[tuple[int, int, int]],
) -> tuple[dict[tuple[int, int, int], float], dict[tuple[int, int, int], tuple[int, int, int]]]:
    """Single-source Dijkstra over surface cells with per-step delta-z cap.

    Explores until every cell in ``targets`` has been reached, the heap
    drains, or every popped cell has g > ``max_cost``. Returns ``(g_score,
    came_from)`` so callers can recover both costs and paths for any target
    that's in ``g_score``.
    """
    heap: list[tuple[float, int, tuple[int, int, int]]] = [(0.0, 0, start)]
    g_score: dict[tuple[int, int, int], float] = {start: 0.0}
    came_from: dict[tuple[int, int, int], tuple[int, int, int]] = {}
    remaining_targets = set(targets)
    remaining_targets.discard(start)
    counter = 0

    while heap:
        cur_g, _, current = heapq.heappop(heap)
        if cur_g > g_score[current]:
            continue
        remaining_targets.discard(current)
        if not remaining_targets:
            break
        cx, cy, cz = current
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                nx_, ny_ = cx + dx, cy + dy
                surfaces_here = surface_lookup.get((nx_, ny_))
                if surfaces_here is None:
                    continue
                for nz_arr in surfaces_here:
                    nz = int(nz_arr)
                    dz = nz - cz
                    if abs(dz) > step_threshold_cells:
                        continue
                    step_cost = math.sqrt(dx * dx + dy * dy + dz * dz) * voxel_size
                    new_g = cur_g + step_cost
                    if new_g > max_cost:
                        continue
                    neighbor = (nx_, ny_, nz)
                    if new_g < g_score.get(neighbor, float("inf")):
                        came_from[neighbor] = current
                        g_score[neighbor] = new_g
                        counter += 1
                        heapq.heappush(heap, (new_g, counter, neighbor))

    return g_score, came_from


def _reconstruct_path(
    came_from: dict[tuple[int, int, int], tuple[int, int, int]],
    end: tuple[int, int, int],
) -> np.ndarray:
    path = [end]
    while end in came_from:
        end = came_from[end]
        path.append(end)
    path.reverse()
    return np.array(path, dtype=np.int64)


def place_nodes(
    surface_points: np.ndarray,
    voxel_size: float,
    *,
    node_spacing: float,
    node_z_tolerance: float,
    sub_sample_stride: int,
) -> tuple[nx.Graph, dict[tuple[int, int], np.ndarray]]:
    """Place sparse nodes over the surface map; returns ``(graph, surface_lookup)``.

    Strides through surface cells in lex order and adds a node whenever no
    existing node sits within a cylinder of XY radius ``node_spacing`` and
    half-height ``node_z_tolerance``. Edges are added separately by
    ``add_node_edges`` so callers can publish the node cloud as soon as
    placement returns.
    """
    graph = nx.Graph()
    if len(surface_points) == 0:
        return graph, {}

    sx = np.floor(surface_points[:, 0] / voxel_size).astype(np.int64)
    sy = np.floor(surface_points[:, 1] / voxel_size).astype(np.int64)
    sz = np.floor(surface_points[:, 2] / voxel_size).astype(np.int64)

    surface_lookup = _build_surface_lookup(sx, sy, sz)

    spacing_cells = max(1, int(node_spacing / voxel_size))
    z_tol_cells = max(0, int(node_z_tolerance / voxel_size))

    grid = _GridHash(spacing_cells)
    node_ix: list[int] = []
    node_iy: list[int] = []
    node_iz: list[int] = []

    order = np.lexsort((sz, sy, sx))
    spacing_sq = spacing_cells * spacing_cells
    stride = max(1, sub_sample_stride)

    for idx in order[::stride]:
        cix, ciy, ciz = int(sx[idx]), int(sy[idx]), int(sz[idx])

        in_cylinder = False
        for nid in grid.nearby(cix, ciy, spacing_cells):
            dx = node_ix[nid] - cix
            dy = node_iy[nid] - ciy
            dz = node_iz[nid] - ciz
            if dx * dx + dy * dy < spacing_sq and abs(dz) < z_tol_cells:
                in_cylinder = True
                break
        if in_cylinder:
            continue

        new_id = len(node_ix)
        node_ix.append(cix)
        node_iy.append(ciy)
        node_iz.append(ciz)
        grid.add(new_id, cix, ciy)
        graph.add_node(
            new_id,
            pos=(
                (cix + 0.5) * voxel_size,
                (ciy + 0.5) * voxel_size,
                ciz * voxel_size,
            ),
            cell=(cix, ciy, ciz),
        )

    return graph, surface_lookup


def add_node_edges(
    graph: nx.Graph,
    surface_lookup: dict[tuple[int, int], np.ndarray],
    voxel_size: float,
    *,
    step_threshold: float,
    max_edge_cost: float,
) -> None:
    """Connect each node to nearby nodes the surface Dijkstra can reach.

    For every node, runs a multi-target Dijkstra over surface cells against
    any higher-id node whose cell sits within ``max_edge_cost`` (euclidean)
    and adds an edge with the cached A* path whenever the constrained
    shortest-path cost is also within ``max_edge_cost``.
    """
    if graph.number_of_nodes() == 0:
        return

    step_cells = max(0, int(step_threshold / voxel_size))
    edge_radius_cells = max(1, int(max_edge_cost / voxel_size))

    grid = _GridHash(edge_radius_cells)
    cells: dict[int, tuple[int, int, int]] = {}
    for node_id, data in graph.nodes(data=True):
        cix, ciy, ciz = data["cell"]
        cells[node_id] = (cix, ciy, ciz)
        grid.add(node_id, cix, ciy)

    for node_id in sorted(graph.nodes()):
        cix, ciy, ciz = cells[node_id]
        candidate_cells: dict[tuple[int, int, int], int] = {}
        for other_id in grid.nearby(cix, ciy, edge_radius_cells):
            if other_id <= node_id:
                continue
            ox, oy, oz = cells[other_id]
            dx, dy, dz = ox - cix, oy - ciy, oz - ciz
            if math.sqrt(dx * dx + dy * dy + dz * dz) * voxel_size > max_edge_cost:
                continue
            candidate_cells[(ox, oy, oz)] = other_id
        if not candidate_cells:
            continue

        g_score, came_from = _surface_dijkstra(
            surface_lookup,
            start=(cix, ciy, ciz),
            voxel_size=voxel_size,
            step_threshold_cells=step_cells,
            max_cost=max_edge_cost,
            targets=set(candidate_cells),
        )
        for cell, other_id in candidate_cells.items():
            if cell in g_score:
                graph.add_edge(
                    node_id,
                    other_id,
                    weight=float(g_score[cell]),
                    path=_reconstruct_path(came_from, cell),
                )


class _PublishableLineSegments3D(LineSegments3D):
    """LineSegments3D with a Python lcm_encode that matches the C++ wire format.

    Upstream only implements decode (encode raises NotImplementedError); this
    subclass produces the same nav_msgs/Path wire layout, where consecutive
    pose pairs are interpreted as segments and pose.orientation.w carries
    traversability.
    """

    def lcm_encode(self) -> bytes:
        lcm_msg = LCMPath()
        sec, nsec = sec_nsec(self.ts)
        lcm_poses = []
        for (p1, p2), trav in zip(self._segments, self._traversability, strict=False):
            for pt in (p1, p2):
                lp = LCMPoseStamped()
                lp.pose = LCMPose()
                lp.pose.position = LCMPoint()
                lp.pose.orientation = LCMQuaternion()
                lp.pose.position.x = pt[0]
                lp.pose.position.y = pt[1]
                lp.pose.position.z = pt[2]
                lp.pose.orientation.w = trav
                lp.header = LCMHeader()
                lp.header.stamp = LCMTime()
                lp.header.stamp.sec = sec
                lp.header.stamp.nsec = nsec
                lp.header.frame_id = self.frame_id
                lcm_poses.append(lp)
        lcm_msg.poses_length = len(lcm_poses)
        lcm_msg.poses = lcm_poses
        lcm_msg.header.stamp.sec = sec
        lcm_msg.header.stamp.nsec = nsec
        lcm_msg.header.frame_id = self.frame_id
        return lcm_msg.lcm_encode()  # type: ignore[no-any-return]


def _nodes_to_cloud(graph: nx.Graph) -> np.ndarray:
    if graph.number_of_nodes() == 0:
        return np.zeros((0, 3), dtype=np.float32)
    return np.array([graph.nodes[n]["pos"] for n in graph.nodes()], dtype=np.float32)


def _edges_to_segments(
    graph: nx.Graph, voxel_size: float
) -> list[tuple[tuple[float, float, float], tuple[float, float, float]]]:
    """Walk each edge's cached A* path and emit consecutive cell pairs as segments."""
    segments: list[tuple[tuple[float, float, float], tuple[float, float, float]]] = []
    for _, _, data in graph.edges(data=True):
        path_cells: np.ndarray = data["path"]
        for i in range(len(path_cells) - 1):
            a = path_cells[i]
            b = path_cells[i + 1]
            ax = (float(a[0]) + 0.5) * voxel_size
            ay = (float(a[1]) + 0.5) * voxel_size
            az = float(a[2]) * voxel_size
            bx = (float(b[0]) + 0.5) * voxel_size
            by = (float(b[1]) + 0.5) * voxel_size
            bz = float(b[2]) * voxel_size
            segments.append(((ax, ay, az), (bx, by, bz)))
    return segments


class MLSPlanner(Module):
    config: MLSPlannerConfig

    global_map: In[PointCloud2]
    start_pose: In[Odometry]
    goal_pose: In[Odometry]
    path: Out[Path]
    surface_map: Out[PointCloud2]
    nodes: Out[PointCloud2]
    node_edges: Out[LineSegments3D]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._latest_start: Odometry | None = None
        self._graph: nx.Graph | None = None

    async def handle_global_map(self, msg: PointCloud2) -> None:
        points, _ = msg.as_numpy()
        if points is None or len(points) == 0:
            return

        t0 = time.perf_counter()
        surface_points = _extract_surfaces(points, self.config.voxel_size, self.config.robot_height)
        surfaces_ms = (time.perf_counter() - t0) * 1000
        self.surface_map.publish(
            PointCloud2.from_numpy(
                surface_points, frame_id=self.config.world_frame, timestamp=time.time()
            )
        )
        logger.info(
            "Surfaces ready",
            surfaces=len(surface_points),
            surface_ms=round(surfaces_ms, 1),
        )

        logger.info(
            "Placing nodes",
            spacing_m=NODE_SPACING_M,
            stride=NODE_SUB_SAMPLE_STRIDE,
        )
        t1 = time.perf_counter()
        graph, surface_lookup = place_nodes(
            surface_points,
            self.config.voxel_size,
            node_spacing=NODE_SPACING_M,
            node_z_tolerance=NODE_Z_TOLERANCE_M,
            sub_sample_stride=NODE_SUB_SAMPLE_STRIDE,
        )
        place_ms = (time.perf_counter() - t1) * 1000
        self.nodes.publish(
            PointCloud2.from_numpy(
                _nodes_to_cloud(graph),
                frame_id=self.config.world_frame,
                timestamp=time.time(),
            )
        )
        logger.info(
            "Nodes placed",
            nodes=graph.number_of_nodes(),
            place_ms=round(place_ms, 1),
        )

        logger.info("Building edges", max_edge_cost_m=NODE_MAX_EDGE_COST_M)
        t2 = time.perf_counter()
        add_node_edges(
            graph,
            surface_lookup,
            self.config.voxel_size,
            step_threshold=NODE_STEP_THRESHOLD_M,
            max_edge_cost=NODE_MAX_EDGE_COST_M,
        )
        edges_ms = (time.perf_counter() - t2) * 1000
        self._graph = graph
        logger.info(
            "Edges built",
            edges=graph.number_of_edges(),
            edges_ms=round(edges_ms, 1),
        )
        self.node_edges.publish(
            _PublishableLineSegments3D(
                ts=time.time(),
                frame_id=self.config.world_frame,
                segments=_edges_to_segments(graph, self.config.voxel_size),
            )
        )

    async def handle_start_pose(self, msg: Odometry) -> None:
        self._latest_start = msg

    async def handle_goal_pose(self, msg: Odometry) -> None:
        if self._latest_start is None:
            logger.warning("MLSPlanner received goal before start; skipping")
            return
        logger.info(
            "MLSPlanner goal received (not yet implemented)",
            start=(self._latest_start.x, self._latest_start.y, self._latest_start.z),
            goal=(msg.x, msg.y, msg.z),
        )
