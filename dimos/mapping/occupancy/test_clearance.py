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

"""Clearance field and the free-space reads built on it."""

import numpy as np
import pytest

from dimos.mapping.occupancy.clearance import (
    cell_state,
    clearance_at,
    clearance_field,
    free_space_near,
    nearest_free,
    obstacle_mask,
    standable_mask,
)
from dimos.mapping.occupancy.gradient import gradient
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid


def _walled_grid() -> OccupancyGrid:
    """4x4 m map: 2-cell walls, a 5x5-cell block at ~(2, 2), unknown strip at x~3."""
    cells = np.zeros((80, 80), dtype=np.int16)
    cells[:2, :] = 100
    cells[-2:, :] = 100
    cells[:, :2] = 100
    cells[:, -2:] = 100
    cells[38:43, 38:43] = 100  # block: world x,y in [1.9, 2.15]
    cells[20:61, 60:65] = -1  # unknown: world x in [3.0, 3.25], y in [1.0, 3.05]
    return OccupancyGrid(grid=cells.astype(np.int8), resolution=0.05, ts=900.0)


def _empty_grid() -> OccupancyGrid:
    return OccupancyGrid(grid=np.zeros((20, 20), dtype=np.int8), resolution=0.05, ts=900.0)


def test_clearance_field_measures_metric_distance() -> None:
    grid = _walled_grid()
    field = clearance_field(grid)

    assert field.shape == (grid.height, grid.width)
    # Obstacles are zero; the block's interior and the walls alike.
    assert field[40, 40] == 0.0
    assert field[0, 0] == 0.0
    # (1.0, 1.0) sits 0.9 m clear of the nearest wall face at 0.1 m.
    assert field[20, 20] == pytest.approx(0.9, abs=0.08)


def test_clearance_field_is_unbounded_unlike_the_gradient() -> None:
    """The measurement the gradient cost map cannot represent."""
    cells = np.zeros((400, 400), dtype=np.int8)
    cells[0, :] = 100  # one wall, far from most of the map
    grid = OccupancyGrid(grid=cells, resolution=0.05, ts=900.0)

    field = clearance_field(grid)
    # 300 cells from the wall at 0.05 m/cell.
    assert field[300, 200] == pytest.approx(15.0, abs=0.05)

    # The same cell in the default gradient saturates: cost 0 decodes to 2.0 m,
    # so anything past max_distance is indistinguishable once encoded.
    encoded = gradient(grid, max_distance=2.0)
    assert int(encoded.grid[300, 200]) == 0


def test_clearance_field_with_no_obstacles_is_infinite() -> None:
    field = clearance_field(_empty_grid())
    assert np.isinf(field).all()

    # And the gradient built on it costs nothing anywhere, matching the
    # obstacle-free guard voronoi_gradient already had.
    assert np.array_equal(gradient(_empty_grid()).grid, np.zeros((20, 20), dtype=np.int8))


def test_unknown_is_neither_obstacle_nor_standable() -> None:
    grid = _walled_grid()
    unknown_cell = (30, 62)  # inside the unknown strip

    assert not obstacle_mask(grid)[unknown_cell]
    assert not standable_mask(grid, min_clearance=0.3)[unknown_cell]
    # Not being an obstacle, it does not shorten a neighbour's clearance either.
    assert clearance_field(grid)[unknown_cell] > 0.0


def test_cell_state_separates_off_map_from_unknown() -> None:
    grid = _walled_grid()
    assert cell_state(grid, 1.0, 1.0) == "free"
    assert cell_state(grid, 2.0, 2.0) == "occupied"
    assert cell_state(grid, 3.1, 2.0) == "unknown"
    assert cell_state(grid, 9.0, 9.0) is None
    assert cell_state(grid, -1.0, 1.0) is None


def test_clearance_at_point() -> None:
    grid = _walled_grid()
    assert clearance_at(grid, 1.0, 1.0) == pytest.approx(0.9, abs=0.08)
    assert clearance_at(grid, 2.0, 2.0) == 0.0
    assert clearance_at(grid, 9.0, 9.0) is None


def test_nearest_free_escapes_an_obstacle() -> None:
    grid = _walled_grid()
    point = nearest_free(grid, 2.0, 2.0, min_clearance=0.3)

    assert point is not None
    assert point.clearance_m >= 0.3
    assert cell_state(grid, point.x, point.y) == "free"
    # Just outside the 0.25 m-wide block plus the clearance band.
    assert point.distance_m < 0.7
    assert point.distance_m == pytest.approx(np.hypot(point.x - 2.0, point.y - 2.0))


def test_nearest_free_returns_none_when_nothing_qualifies() -> None:
    assert nearest_free(_walled_grid(), 2.0, 2.0, min_clearance=5.0) is None


def test_nearest_free_rejects_nonpositive_clearance() -> None:
    with pytest.raises(ValueError, match="min_clearance"):
        nearest_free(_walled_grid(), 1.0, 1.0, min_clearance=0.0)


def test_free_space_near_ranks_by_openness_and_spaces_results() -> None:
    grid = _walled_grid()
    points = free_space_near(grid, 2.0, 2.0, radius=1.0, min_clearance=0.3, max_results=8)

    assert points
    assert len(points) <= 8
    assert all(p.clearance_m >= 0.3 for p in points)
    assert all(p.distance_m <= 1.0 for p in points)

    clearances = [p.clearance_m for p in points]
    assert clearances == sorted(clearances, reverse=True)

    for i, a in enumerate(points):
        for b in points[i + 1 :]:
            assert np.hypot(a.x - b.x, a.y - b.y) >= 0.4


def test_free_space_near_spacing_is_configurable() -> None:
    grid = _walled_grid()
    tight = free_space_near(
        grid, 2.0, 2.0, radius=1.0, min_clearance=0.3, max_results=8, spacing=0.05
    )
    spread = free_space_near(
        grid, 2.0, 2.0, radius=1.0, min_clearance=0.3, max_results=8, spacing=1.0
    )
    assert len(tight) > len(spread)


def test_free_space_near_is_deterministic() -> None:
    grid = _walled_grid()
    kwargs = {"radius": 1.0, "min_clearance": 0.3, "max_results": 8}
    assert free_space_near(grid, 2.0, 2.0, **kwargs) == free_space_near(grid, 2.0, 2.0, **kwargs)


def test_free_space_near_empty_when_area_is_blocked() -> None:
    assert (
        free_space_near(_walled_grid(), 2.0, 2.0, radius=0.1, min_clearance=0.3, max_results=8)
        == []
    )
