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

from pathlib import Path

import numpy as np

from dimos.mapping.occupancy.polygons import points_in_polygon
from dimos.mapping.occupancy.room_segmentation import segment_rooms
from dimos.mapping.occupancy.room_store import RoomStore
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid


def _two_room_segmentation(ts: float):  # type: ignore[no-untyped-def]
    cells = np.full((84, 166), 100, dtype=np.int16)
    cells[2:-2, 2:-2] = 0
    cells[:, 82:84] = 100
    cells[34:50, 82:84] = 0
    return segment_rooms(OccupancyGrid(grid=cells.astype(np.int8), resolution=0.05, ts=ts))


def test_save_and_latest_survive_reopen(tmp_path: Path) -> None:
    db = tmp_path / "rooms.db"
    segmentation = _two_room_segmentation(ts=500.0)
    with RoomStore(db) as store:
        store.save(segmentation, source="test")
    with RoomStore(db) as store:
        room_set = store.latest()
    assert room_set is not None
    assert room_set.derived_ts == 500.0
    assert room_set.source == "test"
    assert len(room_set.rooms) == 2
    assert [r.id for r in room_set.rooms] == [1, 2]
    assert len(room_set.doorways) == 1
    assert room_set.doorways[0]["between"] == [1, 2]
    # Polygons round-trip usably: each room's centroid resolves to it.
    for room in room_set.rooms:
        assert points_in_polygon(np.array([room.centroid_xy]), room.polygon).tolist() == [True]
    # Stored values match the segmentation they came from.
    for stored, region in zip(room_set.rooms, segmentation.regions, strict=True):
        assert stored.kind == region.kind
        assert stored.area_m2 == region.area_m2
        assert stored.max_clearance_m == region.max_clearance_m


def test_latest_returns_newest_derivation(tmp_path: Path) -> None:
    db = tmp_path / "rooms.db"
    with RoomStore(db) as store:
        store.save(_two_room_segmentation(ts=100.0), source="first")
        store.save(_two_room_segmentation(ts=200.0), source="second")
        room_set = store.latest()
        assert room_set is not None
        assert room_set.derived_ts == 200.0
        assert room_set.source == "second"
        # History is kept: rooms from both derivations exist in the stream,
        # but latest() only returns the newest set.
        assert len(room_set.rooms) == 2


def test_latest_on_empty_store(tmp_path: Path) -> None:
    with RoomStore(tmp_path / "empty.db") as store:
        assert store.latest() is None
