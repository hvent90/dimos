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

"""ContourPolygons3D: filled 2D contour polygons in 3D space.

On the wire this uses ``sensor_msgs/PointCloud2``.  Each point's
``intensity`` field encodes its polygon id.  The Python side groups
points by id, ear-clips each polygon into triangles, and renders via
``rr.Mesh3D``.
"""

from __future__ import annotations

from collections import defaultdict
import struct
from typing import TYPE_CHECKING, BinaryIO

from dimos_lcm.sensor_msgs import PointCloud2 as LCMPointCloud2
import numpy as np
from numpy.typing import NDArray

from dimos.types.timestamped import Timestamped

if TYPE_CHECKING:
    from rerun._baseclasses import Archetype

# Matplotlib tab10 — same palette as room_segmentation.REGION_PALETTE, keyed
# by (polygon id - 1), so viewer fills match the 2D debug renders.
DEFAULT_MESH_PALETTE = [
    (31, 119, 180), (255, 127, 14), (44, 160, 44), (214, 39, 40), (148, 103, 189),
    (140, 86, 75), (227, 119, 194), (23, 190, 207), (188, 189, 34), (127, 127, 127),
]  # fmt: skip


def _ear_clip(ring: NDArray[np.float64]) -> list[tuple[int, int, int]]:
    """Triangulate a simple CCW polygon (N, 2) by ear clipping.

    Falls back to a centroid-free fan over the remaining vertices if no ear
    is found (degenerate input) — wrong fills beat no fills for debug viz.
    """

    def cross(o: NDArray[np.float64], a: NDArray[np.float64], b: NDArray[np.float64]) -> float:
        return float((a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]))

    idx = list(range(len(ring)))
    tris: list[tuple[int, int, int]] = []
    while len(idx) > 3:
        for k in range(len(idx)):
            i0, i1, i2 = idx[k - 1], idx[k], idx[(k + 1) % len(idx)]
            a, b, c = ring[i0], ring[i1], ring[i2]
            if cross(a, b, c) <= 1e-12:  # reflex or collinear corner
                continue
            others = [j for j in idx if j not in (i0, i1, i2)]
            pts = ring[others]
            d1 = (b[0] - a[0]) * (pts[:, 1] - a[1]) - (b[1] - a[1]) * (pts[:, 0] - a[0])
            d2 = (c[0] - b[0]) * (pts[:, 1] - b[1]) - (c[1] - b[1]) * (pts[:, 0] - b[0])
            d3 = (a[0] - c[0]) * (pts[:, 1] - c[1]) - (a[1] - c[1]) * (pts[:, 0] - c[0])
            # On-edge counts as blocking: a reflex vertex exactly on the
            # candidate diagonal would let the ear poke outside the polygon.
            if bool(np.any((d1 >= 0) & (d2 >= 0) & (d3 >= 0))):
                continue
            tris.append((i0, i1, i2))
            idx.pop(k)
            break
        else:
            break  # no ear found — degenerate remainder
    if len(idx) == 3:
        tris.append((idx[0], idx[1], idx[2]))
    else:
        for k in range(1, len(idx) - 1):
            tris.append((idx[0], idx[k], idx[k + 1]))
    return tris


def _simplify_ring(ring: NDArray[np.float64], tolerance_m: float) -> NDArray[np.float64]:
    """Drop grid stair-step vertices via Douglas-Peucker (closed contour)."""
    import cv2

    if len(ring) < 4 or tolerance_m <= 0:
        return ring
    approx = cv2.approxPolyDP(ring.astype(np.float32).reshape(-1, 1, 2), tolerance_m, True)
    simplified = approx.reshape(-1, 2).astype(np.float64)
    return simplified if len(simplified) >= 3 else ring


def build_mesh_arrays(
    polys: dict[int, NDArray[np.float64]],
    z_offset: float,
    palette: list[tuple[int, int, int]],
    alpha: int,
    simplify_m: float,
) -> tuple[list[list[float]], list[list[int]], list[tuple[int, int, int, int]]]:
    """Vertices, triangle indices, and per-vertex colors for the filled mesh."""
    vertices: list[list[float]] = []
    indices: list[list[int]] = []
    vertex_colors: list[tuple[int, int, int, int]] = []
    for poly_id, raw_ring in sorted(polys.items()):
        ring = _simplify_ring(np.asarray(raw_ring, dtype=np.float64), simplify_m)
        keep = np.linalg.norm(np.diff(ring, axis=0, append=ring[:1]), axis=1) > 1e-9
        ring = ring[keep]
        if len(ring) < 3:
            continue
        area2 = float(
            np.sum(ring[:, 0] * np.roll(ring[:, 1], -1) - np.roll(ring[:, 0], -1) * ring[:, 1])
        )
        if area2 < 0:  # ear clipping expects CCW
            ring = ring[::-1]
        base = len(vertices)
        vertices.extend([float(x), float(y), z_offset] for x, y in ring)
        indices.extend([base + a, base + b, base + c] for a, b, c in _ear_clip(ring))
        color = palette[(poly_id - 1) % len(palette)]
        vertex_colors.extend([(*color, alpha)] * len(ring))
    return vertices, indices, vertex_colors


class ContourPolygons3D(Timestamped):
    """Filled contour polygons for debug visualization."""

    msg_name = "nav_msgs.ContourPolygons3D"
    ts: float
    frame_id: str
    _raw_bytes: bytes | None  # store raw LCM bytes to preserve intensity

    def __init__(
        self,
        ts: float = 0.0,
        frame_id: str = "map",
        raw_bytes: bytes | None = None,
    ) -> None:
        self.frame_id = frame_id
        self.ts = ts
        self._raw_bytes = raw_bytes

    def lcm_encode(self) -> bytes:
        if self._raw_bytes is None:
            raise ValueError("No data to encode")
        return self._raw_bytes

    @classmethod
    def lcm_decode(cls, data: bytes | BinaryIO) -> ContourPolygons3D:
        raw = data if isinstance(data, bytes) else data.read()
        lcm_msg = LCMPointCloud2.lcm_decode(raw)
        header_ts = lcm_msg.header.stamp.sec + lcm_msg.header.stamp.nsec / 1e9
        frame_id = lcm_msg.header.frame_id
        return cls(ts=header_ts, frame_id=frame_id, raw_bytes=raw)

    def _parse_xyzi(self) -> list[tuple[float, float, float, float]]:
        """Extract (x, y, z, intensity) from raw PointCloud2 bytes."""
        if self._raw_bytes is None:
            return []

        lcm_msg = LCMPointCloud2.lcm_decode(self._raw_bytes)

        offsets: dict[str, int] = {}
        for f in lcm_msg.fields:
            offsets[f.name] = f.offset
        if "x" not in offsets or "y" not in offsets or "z" not in offsets:
            return []

        data = bytes(lcm_msg.data)
        step = lcm_msg.point_step
        n = lcm_msg.width * lcm_msg.height
        result: list[tuple[float, float, float, float]] = []
        for i in range(n):
            base = i * step
            if base + step > len(data):
                break
            x = struct.unpack_from("<f", data, base + offsets["x"])[0]
            y = struct.unpack_from("<f", data, base + offsets["y"])[0]
            z = struct.unpack_from("<f", data, base + offsets["z"])[0]
            intensity = 0.0
            if "intensity" in offsets:
                intensity = struct.unpack_from("<f", data, base + offsets["intensity"])[0]
            result.append((x, y, z, intensity))
        return result

    def to_rerun(
        self,
        z_offset: float = 0.0,
        color: tuple[int, int, int, int] = (220, 30, 30, 255),
        radii: float = 0.08,
    ) -> Archetype:
        """Render polygon outlines as ``rr.LineStrips3D`` closed loops.

        ``z_offset`` is the *absolute* render height — the source point's z
        is discarded.  The C++ FAR planner emits contours at the lidar mount
        height (~1.2 m), which is too high for a flat 2D obstacle outline,
        so the visualization pins them to a fixed display height instead.
        """
        import rerun as rr

        pts = self._parse_xyzi()
        if not pts:
            return rr.LineStrips3D([])

        # Group points by polygon_id (intensity)
        polys: dict[int, list[tuple[float, float, float]]] = defaultdict(list)
        for x, y, z, intensity in pts:
            polys[int(intensity)].append((x, y, z))

        strips: list[list[list[float]]] = []
        for _poly_id, verts in polys.items():
            if len(verts) < 3:
                continue
            # Close the polygon by appending first vertex at the end
            ring = [[v[0], v[1], z_offset] for v in verts]
            ring.append(ring[0])
            strips.append(ring)

        if not strips:
            return rr.LineStrips3D([])

        return rr.LineStrips3D(
            strips,
            colors=[color] * len(strips),
            radii=[radii] * len(strips),
        )

    def to_rerun_mesh(
        self,
        z_offset: float = 0.03,
        palette: list[tuple[int, int, int]] | None = None,
        alpha: int = 130,
        simplify_m: float = 0.08,
    ) -> Archetype:
        """Render the polygons as filled, per-id tinted ``rr.Mesh3D``.

        Contours are Douglas-Peucker simplified by ``simplify_m`` (grid
        stair-steps vanish), ear-clipped into triangles, and colored
        ``palette[(id - 1) % len]`` — the same keying as the room
        segmentation's 2D debug render, so figures and viewer agree.
        ``z_offset`` is the absolute render height of the fill plane.
        """
        import rerun as rr

        grouped: dict[int, list[tuple[float, float]]] = defaultdict(list)
        for x, y, _z, intensity in self._parse_xyzi():
            grouped[int(intensity)].append((x, y))
        polys = {i: np.asarray(v, dtype=np.float64) for i, v in grouped.items()}
        vertices, indices, vertex_colors = build_mesh_arrays(
            polys, z_offset, palette or DEFAULT_MESH_PALETTE, alpha, simplify_m
        )
        return rr.Mesh3D(
            vertex_positions=vertices,
            triangle_indices=indices,
            vertex_colors=vertex_colors,
        )

    def __str__(self) -> str:
        n = len(self._parse_xyzi())
        return f"ContourPolygons3D(frame_id='{self.frame_id}', points={n})"
