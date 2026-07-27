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

import numpy as np
import pytest

from dimos.mapping.occupancy.rooms.geometry import (
    free_clearance,
    free_space,
    polygon_cell_mask,
    polygon_region_geometry,
    region_geometry,
)
from dimos.mapping.occupancy.rooms.segmentation import segment_rooms
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid

RES = 0.05


def _room(height: int = 84, width: int = 84) -> OccupancyGrid:
    """A walled 4x4 m room (2-cell walls) on a 5 cm grid."""
    cells = np.full((height, width), 100, dtype=np.int16)
    cells[2:-2, 2:-2] = 0
    return OccupancyGrid(grid=cells.astype(np.int8), resolution=RES, ts=123.0)


def _whole_map(grid: OccupancyGrid) -> np.ndarray:
    """An outline covering the entire grid, walls included."""
    w, h = grid.width * RES, grid.height * RES
    return np.asarray([[0.0, 0.0], [w, 0.0], [w, h], [0.0, h]], dtype=np.float64)


def test_measures_area_clearance_and_anchor() -> None:
    grid = _room()
    geometry = polygon_region_geometry(grid, _whole_map(grid))
    assert geometry is not None
    # 80x80 free cells at 5 cm = 4x4 m, minus the opening's speckle trim.
    assert geometry.area_m2 == pytest.approx(16.0, rel=0.05)
    # The most open point of a 4 m room is its middle, 2 m from the walls.
    assert geometry.anchor_xy == pytest.approx((2.1, 2.1), abs=0.1)
    assert geometry.max_clearance_m == pytest.approx(2.0, abs=0.1)
    assert geometry.centroid_xy == pytest.approx((2.1, 2.1), abs=0.1)


def test_outline_is_kept_verbatim() -> None:
    grid = _room()
    outline = _whole_map(grid)
    geometry = polygon_region_geometry(grid, outline)
    assert geometry is not None
    # The agent's boundary is authoritative — measuring must not redraw it.
    np.testing.assert_array_equal(geometry.polygon, outline)


def test_obstacles_under_the_outline_are_not_floor() -> None:
    """The measurement is of free space, not of the polygon that was drawn.

    A pillar in the middle of the room takes its cells out of the area and
    pushes the anchor off-center; measuring the outline against itself would
    miss both and still report a 2 m clearance standing inside the pillar.
    """
    grid = _room()
    grid.grid[32:52, 32:52] = 100  # 1x1 m pillar at the room's center
    geometry = polygon_region_geometry(grid, _whole_map(grid))
    assert geometry is not None

    assert geometry.area_m2 == pytest.approx(15.0, rel=0.05)  # 16 - 1
    assert geometry.max_clearance_m < 2.0
    # The anchor is somewhere a robot could actually stand.
    free, _occupied, _unknown = free_space(grid)
    anchor_col = int((geometry.anchor_xy[0]) / RES)
    anchor_row = int((geometry.anchor_xy[1]) / RES)
    assert free[anchor_row, anchor_col]


def test_unknown_space_bounds_clearance_like_a_wall() -> None:
    grid = _room()
    grid.grid[:, 42:] = -1  # right half never observed
    geometry = polygon_region_geometry(grid, _whole_map(grid))
    assert geometry is not None
    # Only the explored half is floor, and its clearance stops at the frontier.
    assert geometry.area_m2 == pytest.approx(8.0, rel=0.1)
    assert geometry.max_clearance_m < 2.0


def test_outline_over_no_free_space_measures_nothing() -> None:
    grid = _room()
    wall = np.asarray([[0.0, 0.0], [0.1, 0.0], [0.1, 4.2], [0.0, 4.2]], dtype=np.float64)
    assert polygon_region_geometry(grid, wall) is None


def test_matches_what_segmentation_measures() -> None:
    """A segmented region re-measured through the outline path agrees.

    This is the property the shared measurement exists for: a region that an
    agent redraws over the same ground reports the same kind of numbers as
    the derived one it replaces.
    """
    grid = _room()
    region = segment_rooms(grid).regions[0]
    remeasured = polygon_region_geometry(grid, region.polygon)
    assert remeasured is not None
    # The outline's vertices sit at cell centers, so re-measuring through it
    # gives up the boundary ring — at most one cell all the way around (plus
    # the rounding on both areas), and nothing else.
    perimeter_m = float(
        np.linalg.norm(np.diff(region.polygon, axis=0, append=region.polygon[:1]), axis=1).sum()
    )
    lost_m2 = region.area_m2 - remeasured.area_m2
    assert 0.0 < lost_m2 <= perimeter_m * RES + 0.1
    assert remeasured.max_clearance_m == pytest.approx(region.max_clearance_m, abs=0.05)
    assert remeasured.anchor_xy == pytest.approx(region.anchor_xy, abs=0.05)
    assert remeasured.centroid_xy == pytest.approx(region.centroid_xy, abs=0.05)


def test_region_geometry_rejects_an_empty_mask() -> None:
    grid = _room()
    free, _occupied, _unknown = free_space(grid)
    clearance = free_clearance(free, RES)
    with pytest.raises(ValueError, match="no cells"):
        region_geometry(np.zeros_like(free), clearance, RES, (0.0, 0.0))


def test_polygon_cell_mask_covers_cell_centers_only() -> None:
    grid = _room()
    square = np.asarray([[1.0, 1.0], [2.0, 1.0], [2.0, 2.0], [1.0, 2.0]], dtype=np.float64)
    mask = polygon_cell_mask(grid, square)
    assert mask.sum() == pytest.approx((1.0 / RES) ** 2, rel=0.1)
    assert mask[int(1.5 / RES), int(1.5 / RES)]
    assert not mask[int(3.0 / RES), int(3.0 / RES)]
