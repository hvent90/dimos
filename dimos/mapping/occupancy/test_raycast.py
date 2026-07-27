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

"""Line-of-sight marching over an occupancy grid."""

import math

import numpy as np
import pytest

from dimos.mapping.occupancy.raycast import raycast
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


def test_ray_stops_at_an_obstacle() -> None:
    hit = raycast(_walled_grid(), 1.0, 2.0, angle_rad=0.0, max_range_m=10.0)

    assert hit is not None
    assert hit.outcome == "obstacle"
    assert hit.distance_m == pytest.approx(0.9, abs=0.06)
    assert hit.x == pytest.approx(1.9, abs=0.06)
    assert hit.y == pytest.approx(2.0)


def test_ray_stops_at_a_wall_behind() -> None:
    wall = raycast(_walled_grid(), 1.0, 2.0, angle_rad=math.pi, max_range_m=10.0)

    assert wall is not None
    assert wall.outcome == "obstacle"
    assert wall.distance_m == pytest.approx(0.9, abs=0.06)


def test_unknown_space_stops_the_ray_and_is_reported_as_such() -> None:
    """Unexplored is not clear — the caller must be able to tell the difference."""
    ray = raycast(_walled_grid(), 1.0, 1.5, angle_rad=0.0, max_range_m=10.0)

    assert ray is not None
    assert ray.outcome == "unknown"
    assert ray.distance_m == pytest.approx(2.0, abs=0.06)


def test_ray_reaching_max_range_is_clear() -> None:
    clear = raycast(_walled_grid(), 1.0, 1.0, angle_rad=math.pi / 2, max_range_m=1.0)

    assert clear is not None
    assert clear.outcome == "max_range"
    assert clear.distance_m == 1.0
    assert clear.y == pytest.approx(2.0)


def test_ray_leaving_the_map_reports_the_edge() -> None:
    """An unwalled map ends without an obstacle to blame."""
    grid = OccupancyGrid(grid=np.zeros((40, 40), dtype=np.int8), resolution=0.05, ts=900.0)
    ray = raycast(grid, 1.0, 1.0, angle_rad=0.0, max_range_m=5.0)

    assert ray is not None
    assert ray.outcome == "map_edge"
    assert ray.distance_m == pytest.approx(1.0, abs=0.06)


def test_ray_from_off_map_has_no_answer() -> None:
    assert raycast(_walled_grid(), 9.0, 9.0, angle_rad=0.0, max_range_m=1.0) is None


def test_ray_rejects_nonpositive_range() -> None:
    with pytest.raises(ValueError, match="max_range_m"):
        raycast(_walled_grid(), 1.0, 1.0, angle_rad=0.0, max_range_m=0.0)
