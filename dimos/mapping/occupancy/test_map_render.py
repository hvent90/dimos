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

"""Agent-readable map renderer: layout, world->pixel mapping, crops."""

import numpy as np
import pytest

from dimos.mapping.occupancy.map_render import (
    _MARGIN_BOTTOM,
    _MARGIN_LEFT,
    _MARGIN_RIGHT,
    _MARGIN_TOP,
    MapMarker,
    MapRegion,
    grid_step_m,
    render_map,
)
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid


def _grid() -> OccupancyGrid:
    """4x4 m map: 2-cell border walls, free inside, unknown block top-right."""
    cells = np.zeros((80, 80), dtype=np.int16)
    cells[:2, :] = 100
    cells[-2:, :] = 100
    cells[:, :2] = 100
    cells[:, -2:] = 100
    cells[60:78, 60:78] = -1
    return OccupancyGrid(grid=cells.astype(np.int8), resolution=0.05, ts=1.0)


def _square(x0: float, y0: float, x1: float, y1: float) -> np.ndarray:
    return np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]], dtype=np.float64)


def test_layout_and_world_to_pixel_mapping() -> None:
    img = render_map(_grid())
    # 4 m extent: 880/4 = 220 px/m caps at 160 -> 640x640 content plus margins.
    assert img.shape == (640 + _MARGIN_TOP + _MARGIN_BOTTOM, 640 + _MARGIN_LEFT + _MARGIN_RIGHT, 3)

    def px(x: float, y: float) -> tuple[int, int]:
        return _MARGIN_TOP + round((4.0 - y) * 160), _MARGIN_LEFT + round(x * 160)

    # Sample off the 0.5 m gridlines, which overdraw the base colors.
    wall = img[px(2.2, 0.05)]  # bottom border wall
    assert (wall < 80).all()
    free = img[px(2.2, 2.2)]
    assert (free > 200).all()
    unknown = img[px(3.7, 3.7)]
    assert (np.abs(unknown.astype(int) - 150) < 30).all()


def test_region_fill_tints_free_space() -> None:
    region = MapRegion(id="room_1", name="kitchen", kind="room", polygon=_square(0.5, 0.5, 2, 2))
    img = render_map(_grid(), regions=[region])
    row = _MARGIN_TOP + round((4.0 - 1.2) * 160)
    col = _MARGIN_LEFT + round(1.2 * 160)
    inside = img[row, col].astype(int)
    outside = img[row, _MARGIN_LEFT + round(3.2 * 160)].astype(int)
    assert np.abs(inside - outside).max() > 20  # tinted vs plain free space
    assert not (inside < 80).all()  # and not mistaken for a wall


def test_markers_and_agent_draw_without_error() -> None:
    img = render_map(
        _grid(),
        regions=[MapRegion(id="room_2", name="", kind="room", polygon=_square(1, 1, 3, 3))],
        markers=[MapMarker(xy=(2.0, 2.0), label="couch")],
        agent_xy=(1.0, 1.0),
        agent_heading=0.5,
    )
    assert img.dtype == np.uint8


def test_bounds_crop_and_rejection() -> None:
    cropped = render_map(_grid(), bounds=(1.0, 1.0, 3.0, 3.0))
    full = render_map(_grid())
    # 2 m crop: 440 px/m caps at 160 -> 320 px content, smaller than full view.
    assert cropped.shape[0] < full.shape[0]
    with pytest.raises(ValueError, match="do not overlap"):
        render_map(_grid(), bounds=(10.0, 10.0, 12.0, 12.0))


def test_vertex_tags_auto_threshold() -> None:
    # A simple square labels its vertices; the same square with a jagged
    # 60-vertex outline suppresses them (auto mode).
    simple = MapRegion(id="room_1", name="", kind="room", polygon=_square(1, 1, 3, 3))
    angles = np.linspace(0.0, 2.0 * np.pi, 60, endpoint=False)
    jagged_poly = np.column_stack(
        [2.0 + np.cos(angles) * (1.0 + 0.1 * np.sin(9 * angles)), 2.0 + np.sin(angles)]
    )
    jagged = MapRegion(id="room_1", name="", kind="room", polygon=jagged_poly)
    with_tags = render_map(_grid(), regions=[simple])
    without_tags = render_map(_grid(), regions=[jagged], label_vertices=None)
    forced = render_map(_grid(), regions=[jagged], label_vertices=True)
    # Tag text adds dark pixels near the simple square's corner.
    corner = with_tags[
        _MARGIN_TOP + round(160 * 1.0) - 30 : _MARGIN_TOP + round(160 * 1.0) + 30,
        _MARGIN_LEFT + round(160 * 3.0) - 30 : _MARGIN_LEFT + round(160 * 3.0) + 30,
    ]
    assert (corner < 60).any()
    assert without_tags.shape == forced.shape


def test_grid_step_scales_with_extent() -> None:
    assert grid_step_m(4.0) == 0.5
    assert grid_step_m(12.0) == 1.0
    assert grid_step_m(30.0) == 2.0
    assert grid_step_m(60.0) == 5.0
