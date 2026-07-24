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

"""Top-down map render an LLM agent can read coordinates off.

Built like a plot, not a screenshot: metric gridlines with world-coordinate
axis labels, room polygons with per-vertex coordinate tags, object markers,
and the robot pose. An agent that spots wrong room geometry in the picture
can read the correct world coordinates from the gridlines and answer with
numbers (e.g. a corrected polygon for a boundary edit). The ``view_map``
skill (dimos/agents/skills/scene_memory.py) pairs the image with the same
geometry as JSON.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
import re

import cv2
import numpy as np
from numpy.typing import NDArray

from dimos.mapping.occupancy.room_segmentation import REGION_PALETTE
from dimos.msgs.nav_msgs.OccupancyGrid import CostValues, OccupancyGrid

# Margins hold the axis tick labels; the map content renders inside them.
_MARGIN_LEFT = 74
_MARGIN_RIGHT = 26
_MARGIN_TOP = 26
_MARGIN_BOTTOM = 48

# The longest world edge maps to about this many pixels (bounded so tiny
# crops don't explode and building-scale maps stay readable).
_TARGET_CONTENT_PX = 880
_MIN_PX_PER_M = 12.0
_MAX_PX_PER_M = 160.0

_COLOR_BG = (245, 245, 245)
_COLOR_FREE = (255, 255, 255)
_COLOR_OCCUPIED = (30, 30, 30)
_COLOR_UNKNOWN = (150, 150, 150)
_COLOR_GRIDLINE = (205, 205, 205)
_COLOR_AXIS = (60, 60, 60)
_COLOR_AGENT = (40, 40, 200)  # BGR red — distinct from any region tint
_COLOR_MARKER = (30, 30, 30)

_ROOM_FILL_ALPHA = 0.35

# Auto vertex labeling caps the tags in view — a full floor's worth of jagged
# real-map polygons is unreadable soup; a zoomed room is where coordinates
# get read off anyway (the paired JSON always carries every vertex).
_MAX_VERTEX_TAGS = 48


@dataclass(frozen=True)
class MapRegion:
    """One room/corridor to draw: outline polygon plus id and display name."""

    id: str
    name: str
    kind: str  # "room" | "corridor"
    polygon: NDArray[np.float64]  # (N, 2) world xy


@dataclass(frozen=True)
class MapMarker:
    """One labeled point marker (an object node, a doorway, ...)."""

    xy: tuple[float, float]
    label: str


def grid_step_m(extent_m: float) -> float:
    """Gridline spacing that keeps roughly 6-16 lines across the view."""
    if extent_m <= 6.0:
        return 0.5
    if extent_m <= 16.0:
        return 1.0
    if extent_m <= 40.0:
        return 2.0
    return 5.0


def _palette_color(region_id: str, fallback_index: int) -> tuple[int, int, int]:
    """Stable BGR tint per region: keyed on the id's trailing number."""
    match = re.search(r"(\d+)$", region_id)
    index = int(match.group(1)) - 1 if match else fallback_index
    r, g, b = REGION_PALETTE[index % len(REGION_PALETTE)]
    return b, g, r


def _text(
    img: NDArray[np.uint8],
    text: str,
    org: tuple[int, int],
    scale: float,
    color: tuple[int, int, int],
    thickness: int = 1,
) -> None:
    """putText with a white halo so labels stay readable over any layer."""
    cv2.putText(
        img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (255, 255, 255), thickness + 2, cv2.LINE_AA
    )
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color, thickness, cv2.LINE_AA)


def _view_bounds(
    grid: OccupancyGrid, bounds: tuple[float, float, float, float] | None
) -> tuple[float, float, float, float]:
    ox, oy = float(grid.origin.position.x), float(grid.origin.position.y)
    map_bounds = (ox, oy, ox + grid.width * grid.resolution, oy + grid.height * grid.resolution)
    if bounds is None:
        return map_bounds
    x0 = max(bounds[0], map_bounds[0])
    y0 = max(bounds[1], map_bounds[1])
    x1 = min(bounds[2], map_bounds[2])
    y1 = min(bounds[3], map_bounds[3])
    if x0 >= x1 or y0 >= y1:
        raise ValueError(
            f"bounds {bounds} do not overlap the mapped area "
            f"x [{map_bounds[0]:.2f}, {map_bounds[2]:.2f}], "
            f"y [{map_bounds[1]:.2f}, {map_bounds[3]:.2f}]"
        )
    return x0, y0, x1, y1


def _base_layer(
    content: NDArray[np.uint8],
    grid: OccupancyGrid,
    view: tuple[float, float, float, float],
    scale: float,
    free_cost_max: int,
) -> None:
    """Paste the occupancy colors (free/occupied/unknown) onto the content canvas."""
    x0, y0, x1, y1 = view
    ox, oy = float(grid.origin.position.x), float(grid.origin.position.y)
    res = grid.resolution
    c0 = max(0, int(np.floor((x0 - ox) / res)))
    r0 = max(0, int(np.floor((y0 - oy) / res)))
    c1 = min(grid.width, int(np.ceil((x1 - ox) / res)))
    r1 = min(grid.height, int(np.ceil((y1 - oy) / res)))

    cells = grid.grid[r0:r1, c0:c1]
    colors = np.empty((*cells.shape, 3), dtype=np.uint8)
    colors[:] = _COLOR_FREE
    colors[cells == CostValues.UNKNOWN] = _COLOR_UNKNOWN
    colors[cells >= free_cost_max] = _COLOR_OCCUPIED

    # Row 0 of the grid is the y-min edge; image row 0 is the top (y-max).
    block = cv2.resize(
        np.flipud(colors),
        (max(1, round((c1 - c0) * res * scale)), max(1, round((r1 - r0) * res * scale))),
        interpolation=cv2.INTER_NEAREST,
    )
    dx = round((x0 - (ox + c0 * res)) * scale)
    dy = round(((oy + r1 * res) - y1) * scale)
    crop = block[dy : dy + content.shape[0], dx : dx + content.shape[1]]
    content[: crop.shape[0], : crop.shape[1]] = crop


def render_map(
    grid: OccupancyGrid,
    regions: Sequence[MapRegion] = (),
    markers: Sequence[MapMarker] = (),
    agent_xy: tuple[float, float] | None = None,
    agent_heading: float | None = None,
    bounds: tuple[float, float, float, float] | None = None,
    label_vertices: bool | None = None,
    free_cost_max: int = 50,
) -> NDArray[np.uint8]:
    """Render the map view as a BGR uint8 image.

    Args:
        grid: Occupancy grid; cost >= ``free_cost_max`` draws as obstacle.
        regions: Room/corridor outlines to overlay.
        markers: Labeled points (object nodes).
        agent_xy: Robot position; drawn as a red circle.
        agent_heading: Robot heading in radians (world frame); adds an arrow.
        bounds: Optional (x_min, y_min, x_max, y_max) world crop; clamped to
            the mapped area. None renders the full map.
        label_vertices: Tag region vertices with their world coordinates.
            None (auto) labels them when few enough are in view to read.
    Raises:
        ValueError: ``bounds`` does not overlap the mapped area.
    """
    x0, y0, x1, y1 = view = _view_bounds(grid, bounds)
    extent = max(x1 - x0, y1 - y0)
    scale = float(np.clip(_TARGET_CONTENT_PX / extent, _MIN_PX_PER_M, _MAX_PX_PER_M))
    content_w, content_h = round((x1 - x0) * scale), round((y1 - y0) * scale)

    # All map-space layers draw on a content-sized canvas so partially
    # visible geometry clips at the view edge instead of spilling into the
    # axis margins.
    content = np.full((content_h, content_w, 3), _COLOR_BG, dtype=np.uint8)

    def to_px(x: float, y: float) -> tuple[int, int]:
        return round((x - x0) * scale), round((y1 - y) * scale)

    _base_layer(content, grid, view, scale, free_cost_max)

    # Translucent room fills, all composited in one pass.
    if regions:
        overlay = content.copy()
        for i, region in enumerate(regions):
            pts = np.array([to_px(x, y) for x, y in region.polygon], dtype=np.int32)
            cv2.fillPoly(overlay, [pts], _palette_color(region.id, i))
        cv2.addWeighted(
            overlay, _ROOM_FILL_ALPHA, content, 1.0 - _ROOM_FILL_ALPHA, 0.0, dst=content
        )

    # Metric gridlines — the coordinate frame the agent reads positions from.
    step = grid_step_m(extent)
    xticks = [float(gx) for gx in np.arange(np.ceil(x0 / step) * step, x1 + 1e-9, step)]
    yticks = [float(gy) for gy in np.arange(np.ceil(y0 / step) * step, y1 + 1e-9, step)]
    for gx in xticks:
        px = to_px(gx, y0)[0]
        cv2.line(content, (px, 0), (px, content_h), _COLOR_GRIDLINE, 1)
    for gy in yticks:
        py = to_px(x0, gy)[1]
        cv2.line(content, (0, py), (content_w, py), _COLOR_GRIDLINE, 1)

    # Region outlines, vertex coordinate tags, and id/name labels.
    def in_view(x: float, y: float) -> bool:
        return x0 <= x <= x1 and y0 <= y <= y1

    if label_vertices is None:
        visible = sum(1 for r in regions for x, y in r.polygon if in_view(x, y))
        label_vertices = visible <= _MAX_VERTEX_TAGS
    for i, region in enumerate(regions):
        color = _palette_color(region.id, i)
        pts = np.array([to_px(x, y) for x, y in region.polygon], dtype=np.int32)
        cv2.polylines(content, [pts], isClosed=True, color=color, thickness=2, lineType=cv2.LINE_AA)
        centroid = region.polygon.mean(axis=0)
        if label_vertices:
            for x, y in region.polygon:
                if not in_view(float(x), float(y)):
                    continue
                px, py = to_px(float(x), float(y))
                cv2.circle(content, (px, py), 3, color, -1, lineType=cv2.LINE_AA)
                # Nudge the tag outward from the centroid to clear the outline.
                direction = np.array([x, y]) - centroid
                norm = float(np.linalg.norm(direction))
                offset = direction / norm * 10.0 if norm > 1e-9 else np.array([10.0, 0.0])
                tag = f"({x:.1f},{y:.1f})"
                (tw, th), _ = cv2.getTextSize(tag, cv2.FONT_HERSHEY_SIMPLEX, 0.34, 1)
                tx = px + int(offset[0]) - (tw if offset[0] < 0 else 0)
                ty = py - int(offset[1]) + (th if offset[1] < 0 else 0)
                _text(content, tag, (tx, ty), 0.34, (20, 20, 20))
        label = region.id if region.name in ("", region.id) else f"{region.id}: {region.name}"
        (tw, _), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2)
        cx, cy = to_px(float(centroid[0]), float(centroid[1]))
        _text(content, label, (cx - tw // 2, cy), 0.55, color, thickness=2)

    for marker in markers:
        px, py = to_px(*marker.xy)
        cv2.drawMarker(content, (px, py), _COLOR_MARKER, cv2.MARKER_DIAMOND, 10, 2, cv2.LINE_AA)
        _text(content, marker.label, (px + 7, py - 7), 0.42, _COLOR_MARKER)

    if agent_xy is not None:
        px, py = to_px(*agent_xy)
        cv2.circle(content, (px, py), 6, _COLOR_AGENT, 2, lineType=cv2.LINE_AA)
        if agent_heading is not None:
            tip = (
                agent_xy[0] + 0.45 * float(np.cos(agent_heading)),
                agent_xy[1] + 0.45 * float(np.sin(agent_heading)),
            )
            cv2.arrowedLine(
                content, (px, py), to_px(*tip), _COLOR_AGENT, 2, cv2.LINE_AA, tipLength=0.4
            )
        _text(content, "robot", (px + 9, py + 14), 0.42, _COLOR_AGENT)

    # Frame the canvas with margins, border, and world-coordinate ticks.
    img = np.full(
        (content_h + _MARGIN_TOP + _MARGIN_BOTTOM, content_w + _MARGIN_LEFT + _MARGIN_RIGHT, 3),
        _COLOR_BG,
        dtype=np.uint8,
    )
    img[_MARGIN_TOP : _MARGIN_TOP + content_h, _MARGIN_LEFT : _MARGIN_LEFT + content_w] = content
    cv2.rectangle(
        img,
        (_MARGIN_LEFT, _MARGIN_TOP),
        (_MARGIN_LEFT + content_w, _MARGIN_TOP + content_h),
        (120, 120, 120),
        1,
    )
    for gx in xticks:
        px = _MARGIN_LEFT + to_px(gx, y0)[0]
        label = f"{gx:g}"
        (tw, _), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.4, 1)
        _text(img, label, (px - tw // 2, _MARGIN_TOP + content_h + 20), 0.4, _COLOR_AXIS)
    for gy in yticks:
        py = _MARGIN_TOP + to_px(x0, gy)[1]
        label = f"{gy:g}"
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.4, 1)
        _text(img, label, (_MARGIN_LEFT - tw - 8, py + th // 2), 0.4, _COLOR_AXIS)
    _text(img, "x (m)", (_MARGIN_LEFT + content_w // 2 - 20, img.shape[0] - 8), 0.42, _COLOR_AXIS)
    _text(img, "y (m)", (8, _MARGIN_TOP + 14), 0.42, _COLOR_AXIS)

    return img
