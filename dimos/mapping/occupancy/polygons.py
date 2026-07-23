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

"""2D polygon containment tests for map regions (rooms, hand-labeled zones)."""

from collections.abc import Sequence

import numpy as np
from numpy.typing import NDArray


def points_in_polygon(
    points: NDArray[np.float64], polygon: NDArray[np.float64]
) -> NDArray[np.bool_]:
    """Test which 2D points fall inside a simple polygon (ray casting).

    Args:
        points: (N, 2) array of x, y coordinates.
        polygon: (M, 2) array of vertices in order (open or closed ring),
            M >= 3.
    Returns:
        (N,) boolean mask; points exactly on an edge may land on either side.
    """
    points = np.asarray(points, dtype=np.float64)
    polygon = np.asarray(polygon, dtype=np.float64)
    if polygon.ndim != 2 or polygon.shape[1] != 2 or polygon.shape[0] < 3:
        raise ValueError(f"polygon must be (M>=3, 2), got shape {polygon.shape}")
    if points.ndim != 2 or points.shape[1] != 2:
        raise ValueError(f"points must be (N, 2), got shape {points.shape}")

    px = points[:, 0][:, None]  # (N, 1)
    py = points[:, 1][:, None]
    x1, y1 = polygon[:, 0][None, :], polygon[:, 1][None, :]  # (1, M) edge starts
    x2, y2 = np.roll(polygon[:, 0], -1)[None, :], np.roll(polygon[:, 1], -1)[None, :]

    # Horizontal ray to +x: an edge crosses it when the point's y is between
    # the edge's y span (half-open, so shared vertices count once) and the
    # edge's x at that y lies to the right of the point.
    spans = (y1 <= py) != (y2 <= py)
    with np.errstate(divide="ignore", invalid="ignore"):
        x_at_y = x1 + (py - y1) * (x2 - x1) / (y2 - y1)
    crossings = spans & (px < x_at_y)
    result: NDArray[np.bool_] = (crossings.sum(axis=1) % 2).astype(bool)
    return result


def distance_to_polygon(
    points: NDArray[np.float64], polygon: NDArray[np.float64]
) -> NDArray[np.float64]:
    """Euclidean distance from each 2D point to the polygon outline.

    The distance is to the boundary, not the filled shape — points inside
    the polygon get their distance to the nearest edge, not 0.

    Args:
        points: (N, 2) array of x, y coordinates.
        polygon: (M, 2) array of vertices in order (open or closed ring),
            M >= 3.
    Returns:
        (N,) distances in the polygon's units.
    """
    points = np.asarray(points, dtype=np.float64)
    polygon = np.asarray(polygon, dtype=np.float64)
    if polygon.ndim != 2 or polygon.shape[1] != 2 or polygon.shape[0] < 3:
        raise ValueError(f"polygon must be (M>=3, 2), got shape {polygon.shape}")
    if points.ndim != 2 or points.shape[1] != 2:
        raise ValueError(f"points must be (N, 2), got shape {points.shape}")

    edge_start = polygon  # (M, 2)
    edge_vec = np.roll(polygon, -1, axis=0) - polygon  # (M, 2)
    to_point = points[:, None, :] - edge_start[None, :, :]  # (N, M, 2)
    length_sq = (edge_vec * edge_vec).sum(axis=1)  # (M,)
    t = np.clip(
        (to_point * edge_vec[None, :, :]).sum(axis=2) / np.maximum(length_sq, 1e-12), 0.0, 1.0
    )  # (N, M) position of the closest point along each edge
    closest = edge_start[None, :, :] + t[..., None] * edge_vec[None, :, :]
    distances: NDArray[np.float64] = np.linalg.norm(points[:, None, :] - closest, axis=2).min(
        axis=1
    )
    return distances


def assign_to_polygons(
    points: NDArray[np.float64],
    polygons: Sequence[NDArray[np.float64]],
    snap: float,
) -> NDArray[np.int64]:
    """Exclusively assign each point to one polygon index (-1 = none).

    A point inside a polygon belongs to it (distance 0); otherwise it snaps
    to the polygon with the nearest outline if that is within ``snap``.
    Ties break to the lowest polygon index.
    """
    if len(points) == 0 or not polygons:
        return np.full(len(points), -1, dtype=np.int64)
    effective = np.empty((len(points), len(polygons)))
    for j, polygon in enumerate(polygons):
        inside = points_in_polygon(points, polygon)
        effective[:, j] = np.where(inside, 0.0, distance_to_polygon(points, polygon))
    best = effective.argmin(axis=1)
    assigned: NDArray[np.int64] = np.where(
        effective[np.arange(len(points)), best] <= snap, best, -1
    ).astype(np.int64)
    return assigned


def polygon_from_flat(flat: list[float]) -> NDArray[np.float64]:
    """Convert a flat [x1, y1, x2, y2, ...] list to an (M, 2) polygon array."""
    if len(flat) < 6 or len(flat) % 2 != 0:
        raise ValueError(f"Flat polygon needs an even number of >= 6 coordinates, got {len(flat)}")
    return np.asarray(flat, dtype=np.float64).reshape(-1, 2)
