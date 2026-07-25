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

from dimos.mapping.occupancy.rooms.edits import (
    analytic_region_geometry,
    boundary_region,
    merged_region,
    split_region,
)
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid

RES = 0.05


def _two_rooms() -> OccupancyGrid:
    """8.3 x 4.2 m: two rooms split by a wall at x ~4.1, joined by a doorway."""
    cells = np.full((84, 166), 100, dtype=np.int16)
    cells[2:-2, 2:-2] = 0
    cells[:, 82:84] = 100
    cells[34:50, 82:84] = 0
    return OccupancyGrid(grid=cells.astype(np.int8), resolution=RES, ts=7.0)


def _walled_off() -> OccupancyGrid:
    """The same two areas, separated by a solid 10-cell wall and no doorway.

    The wall has to be thicker than the closing's reach: joining rooms
    tolerates a few cells of rasterization seam between simplified
    outlines, so a hairline wall is bridged by design.
    """
    cells = np.full((84, 166), 100, dtype=np.int16)
    cells[2:-2, 2:76] = 0
    cells[2:-2, 86:-2] = 0
    return OccupancyGrid(grid=cells.astype(np.int8), resolution=RES, ts=7.0)


def _rect(x0: float, y0: float, x1: float, y1: float) -> np.ndarray:
    return np.asarray([[x0, y0], [x1, y0], [x1, y1], [x0, y1]], dtype=np.float64)


def _west() -> np.ndarray:
    return _rect(0.1, 0.1, 4.1, 4.1)


def _east() -> np.ndarray:
    return _rect(4.2, 0.1, 8.2, 4.1)


def test_merge_joins_rooms_through_a_doorway() -> None:
    geometry = merged_region(_two_rooms(), [_west(), _east()])
    assert geometry is not None
    # Both rooms' floor, not the wall between them: ~2 x (4 x 4) m.
    assert geometry.area_m2 == pytest.approx(31.0, rel=0.1)
    assert geometry.max_clearance_m > 1.5


def test_merge_refuses_rooms_a_wall_separates() -> None:
    """Without a doorway the two areas are disconnected free space.

    Polygon footprints that merely abut the same wall are not adjacency —
    a merge that accepted them would invent a room spanning solid brick.
    """
    assert merged_region(_walled_off(), [_rect(0.1, 0.1, 3.8, 4.1), _east()]) is None


def test_merge_refuses_an_outline_over_no_floor() -> None:
    outside = _rect(20.0, 20.0, 24.0, 24.0)
    assert merged_region(_two_rooms(), [_west(), outside]) is None


def test_split_divides_a_room_and_reports_the_seam() -> None:
    grid = _two_rooms()
    halves = split_region(grid, _west(), np.asarray([2.1, 0.0]), np.asarray([2.1, 4.2]))
    assert halves is not None
    # A 4x4 m room cut down the middle: two ~2x4 m halves.
    assert halves.a.area_m2 == pytest.approx(8.0, rel=0.15)
    assert halves.b.area_m2 == pytest.approx(8.0, rel=0.15)
    # 'a' is left of the p0 -> p1 direction (+y here), so it lies west.
    assert halves.a.centroid_xy[0] < 2.1 < halves.b.centroid_xy[0]
    # The doorway sits on the cut, and its width is the shared seam.
    assert halves.doorway_xy[0] == pytest.approx(2.1, abs=0.1)
    assert halves.seam_width_m == pytest.approx(4.0, rel=0.2)


def test_split_refuses_a_line_that_misses_the_room() -> None:
    grid = _two_rooms()
    assert split_region(grid, _west(), np.asarray([20.0, 0.0]), np.asarray([20.0, 1.0])) is None


def test_split_refuses_a_line_that_only_clips_a_corner() -> None:
    """A sliver is not a room; the cut has to leave floor on both sides."""
    grid = _two_rooms()
    halves = split_region(grid, _west(), np.asarray([0.0, 0.12]), np.asarray([4.0, 0.12]))
    assert halves is None


def test_boundary_measures_against_the_grid_when_there_is_one() -> None:
    grid = _two_rooms()
    outline = _west()
    geometry = boundary_region(grid, outline)
    # The outline is authoritative; only the numbers come from the map.
    np.testing.assert_array_equal(geometry.polygon, outline)
    assert geometry.area_m2 == pytest.approx(16.0, rel=0.1)
    assert geometry.max_clearance_m > 1.5


def test_boundary_falls_back_to_shoelace_without_a_map() -> None:
    outline = _west()
    geometry = boundary_region(None, outline)
    np.testing.assert_array_equal(geometry.polygon, outline)
    assert geometry.area_m2 == pytest.approx(16.0, rel=0.01)
    # Nothing here knows where the walls are, so clearance is not guessed.
    assert geometry.max_clearance_m == 0.0
    assert geometry.centroid_xy == analytic_region_geometry(outline).centroid_xy


def test_boundary_falls_back_when_the_outline_covers_no_floor() -> None:
    geometry = boundary_region(_two_rooms(), _rect(20.0, 20.0, 24.0, 24.0))
    assert geometry.area_m2 == pytest.approx(16.0, rel=0.01)
    assert geometry.max_clearance_m == 0.0
