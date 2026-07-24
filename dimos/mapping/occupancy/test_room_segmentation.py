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

from dimos.mapping.occupancy.polygons import points_in_polygon
from dimos.mapping.occupancy.room_segmentation import (
    RoomSegmentation,
    render_regions,
    segment_rooms,
)
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid

RES = 0.05


def _grid(cells: np.ndarray, ts: float = 123.0) -> OccupancyGrid:
    return OccupancyGrid(grid=cells.astype(np.int8), resolution=RES, ts=ts)


def _walled(height: int, width: int) -> np.ndarray:
    cells = np.full((height, width), 100, dtype=np.int16)
    cells[2:-2, 2:-2] = 0
    return cells


def _two_rooms() -> OccupancyGrid:
    # Two 4x4 m rooms separated by a wall with a 0.8 m door (16 cells).
    cells = _walled(84, 166)
    cells[:, 82:84] = 100  # dividing wall
    cells[34:50, 82:84] = 0  # door gap
    return _grid(cells)


def test_single_room() -> None:
    seg = segment_rooms(_grid(_walled(84, 84)))
    assert len(seg.regions) == 1
    assert seg.regions[0].kind == "room"
    assert seg.regions[0].area_m2 == pytest.approx(16.0, rel=0.05)
    assert seg.doorways == ()
    assert seg.derived_ts == 123.0


def test_two_rooms_one_doorway() -> None:
    seg = segment_rooms(_two_rooms())
    assert len(seg.regions) == 2
    assert {r.kind for r in seg.regions} == {"room"}
    assert len(seg.doorways) == 1
    doorway = seg.doorways[0]
    assert doorway.between == (1, 2)
    # The door sits on the dividing wall at x ~ 82.5 cells * 0.05, centered
    # vertically in the gap (rows 34..49 -> ~42 cells * 0.05).
    assert doorway.position_xy[0] == pytest.approx(4.15, abs=0.1)
    assert doorway.position_xy[1] == pytest.approx(2.1, abs=0.15)


def test_room_polygons_contain_own_centroid_only() -> None:
    seg = segment_rooms(_two_rooms())
    room1, room2 = seg.regions
    for own, other in ((room1, room2), (room2, room1)):
        inside_own = points_in_polygon(np.array([own.centroid_xy]), own.polygon)
        inside_other = points_in_polygon(np.array([own.centroid_xy]), other.polygon)
        assert inside_own.tolist() == [True]
        assert inside_other.tolist() == [False]


def test_corridor_classification() -> None:
    # Two 4x4 m rooms joined by a 1.0 m wide, 6 m long corridor, entered
    # through 0.8 m doors at both ends (the door pinch is what separates the
    # corridor's watershed seeds from the rooms').
    cells = np.full((84, 288), 100, dtype=np.int16)
    cells[2:82, 2:82] = 0  # left room
    cells[2:82, 206:286] = 0  # right room
    cells[32:52, 82:206] = 0  # corridor (20 cells = 1.0 m wide)
    cells[:, 82:86] = 100  # left door wall
    cells[34:50, 82:86] = 0  # 0.8 m door
    cells[:, 202:206] = 100  # right door wall
    cells[34:50, 202:206] = 0
    seg = segment_rooms(_grid(cells))
    kinds = sorted(r.kind for r in seg.regions)
    assert kinds == ["corridor", "room", "room"]
    corridor = next(r for r in seg.regions if r.kind == "corridor")
    assert corridor.area_m2 == pytest.approx(6.0, rel=0.25)
    # Each room adjoins the corridor.
    pairs = {d.between for d in seg.doorways}
    room_ids = {r.id for r in seg.regions if r.kind == "room"}
    assert all(corridor.id in pair for pair in pairs)
    assert {i for pair in pairs for i in pair if i != corridor.id} == room_ids


def test_small_region_merges() -> None:
    # A 1 m^2 alcove (< min_room_area_m2) merges into the main room.
    cells = _walled(84, 84)
    alcove = np.full((24, 26), 100, dtype=np.int16)
    alcove[2:22, 2:22] = 0  # 1x1 m free pocket
    cells_ext = np.hstack([cells, np.full((84, 26), 100, dtype=np.int16)])
    cells_ext[30:54, 82:108] = alcove
    cells_ext[36:48, 80:84] = 0  # opening between room and alcove
    seg = segment_rooms(_grid(cells_ext))
    assert len(seg.regions) == 1
    assert seg.regions[0].kind == "room"


def test_explored_fraction() -> None:
    cells = _walled(84, 84)
    cells[:20, :] = -1  # unknown band
    seg = segment_rooms(_grid(cells))
    expected = 1.0 - (20 * 84) / (84 * 84)
    assert seg.explored_fraction == pytest.approx(expected, abs=0.01)


def test_labels_cover_free_space() -> None:
    seg = segment_rooms(_two_rooms())
    # Every free cell belongs to a region; walls carry label 0.
    grid = _two_rooms().grid
    free = grid == 0
    labeled = seg.labels > 0
    assert (labeled & ~free).sum() == 0
    # binary_opening trims a few edge cells; the bulk of free space is labeled.
    assert labeled.sum() > 0.95 * free.sum()


def test_render_regions_shape() -> None:
    grid = _two_rooms()
    seg = segment_rooms(grid)
    img = render_regions(grid, seg, upscale=2)
    assert img.shape == (84 * 2, 166 * 2, 3)
    assert img.dtype == np.uint8


def test_rooms_and_corridors_accessors() -> None:
    seg = segment_rooms(_two_rooms())
    assert isinstance(seg, RoomSegmentation)
    assert len(seg.rooms()) == 2
    assert seg.corridors() == ()
