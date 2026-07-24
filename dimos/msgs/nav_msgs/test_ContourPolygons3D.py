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

"""Filled-mesh rendering of contour polygons (ear clipping + palette)."""

import numpy as np
from numpy.typing import NDArray

from dimos.msgs.nav_msgs.ContourPolygons3D import (
    DEFAULT_MESH_PALETTE,
    ContourPolygons3D,
    _ear_clip,
    build_mesh_arrays,
)
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

SQUARE = np.array([[0.0, 0.0], [2.0, 0.0], [2.0, 2.0], [0.0, 2.0]])
# Concave L: 4x4 with the top-right 2x2 bite removed (area 12).
L_SHAPE = np.array([[0.0, 0.0], [4.0, 0.0], [4.0, 2.0], [2.0, 2.0], [2.0, 4.0], [0.0, 4.0]])


def _shoelace(ring: NDArray[np.float64]) -> float:
    x, y = ring[:, 0], ring[:, 1]
    return abs(float(np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y))) / 2.0


def _tri_area(verts: list[list[float]], indices: list[list[int]]) -> float:
    v = np.asarray(verts)[:, :2]
    total = 0.0
    for a, b, c in indices:
        pa, pb, pc = v[a], v[b], v[c]
        total += abs((pb[0] - pa[0]) * (pc[1] - pa[1]) - (pb[1] - pa[1]) * (pc[0] - pa[0])) / 2.0
    return total


def _build(
    polys: dict[int, NDArray[np.float64]], z: float = 0.03, alpha: int = 130, simplify: float = 0.0
) -> tuple[list[list[float]], list[list[int]], list[tuple[int, int, int, int]]]:
    return build_mesh_arrays(polys, z, DEFAULT_MESH_PALETTE, alpha, simplify)


def test_square_triangulates_to_two_triangles() -> None:
    verts, indices, _colors = _build({1: SQUARE}, z=0.5)
    assert len(indices) == 2
    assert all(v[2] == 0.5 for v in verts)
    assert _tri_area(verts, indices) == _shoelace(SQUARE)


def test_concave_polygon_area_is_preserved() -> None:
    verts, indices, _colors = _build({2: L_SHAPE})
    # Ear clipping a simple polygon yields exactly n - 2 triangles, and the
    # fills cover the polygon without overlap iff the areas match.
    assert len(indices) == len(L_SHAPE) - 2
    assert abs(_tri_area(verts, indices) - _shoelace(L_SHAPE)) < 1e-9


def test_palette_keyed_by_polygon_id() -> None:
    verts, _indices, colors = _build({1: SQUARE, 3: L_SHAPE + 10.0}, alpha=99)
    assert colors[0] == (*DEFAULT_MESH_PALETTE[0], 99)  # id 1
    assert colors[len(SQUARE)] == (*DEFAULT_MESH_PALETTE[2], 99)  # id 3
    assert len(colors) == len(verts) == len(SQUARE) + len(L_SHAPE)


def test_stair_steps_simplify_away() -> None:
    # A 2x2 square whose bottom edge staircases in 0.05 m grid steps.
    steps = [[x, 0.02 * (i % 2)] for i, x in enumerate(np.arange(0.0, 2.0, 0.05))]
    jagged = np.array([*steps, [2.0, 0.0], [2.0, 2.0], [0.0, 2.0]])
    verts, indices, _colors = _build({1: jagged}, simplify=0.08)
    assert len(verts) <= 8  # staircase collapsed to (near-)corner vertices
    assert abs(_tri_area(verts, indices) - 4.0) < 0.2


def test_cw_input_is_reoriented() -> None:
    verts, indices, _colors = _build({1: SQUARE[::-1].copy()})
    assert _tri_area(verts, indices) == _shoelace(SQUARE)


def test_ear_clip_covers_concave_ring_directly() -> None:
    tris = _ear_clip(L_SHAPE)
    assert len(tris) == len(L_SHAPE) - 2
    total = sum(
        abs(
            (L_SHAPE[b][0] - L_SHAPE[a][0]) * (L_SHAPE[c][1] - L_SHAPE[a][1])
            - (L_SHAPE[b][1] - L_SHAPE[a][1]) * (L_SHAPE[c][0] - L_SHAPE[a][0])
        )
        / 2.0
        for a, b, c in tris
    )
    assert abs(total - _shoelace(L_SHAPE)) < 1e-9


def test_message_roundtrip_renders_mesh() -> None:
    points = np.column_stack([SQUARE, np.zeros(len(SQUARE))])
    cloud = PointCloud2.from_numpy(
        points, frame_id="world", timestamp=1.0, intensities=np.full(len(SQUARE), 1.0)
    )
    msg = ContourPolygons3D(ts=1.0, frame_id="world", raw_bytes=cloud.lcm_encode())
    mesh = msg.to_rerun_mesh(simplify_m=0.0)
    assert type(mesh).__name__ == "Mesh3D"
