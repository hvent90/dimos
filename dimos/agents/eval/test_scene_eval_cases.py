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

"""Answer-key construction rules on synthetic scenes."""

import numpy as np

from dimos.agents.eval.scene_eval_cases import (
    build_answer_key,
    find_trap_instances,
    object_entries,
)
from dimos.agents.skills.scene_memory import PoseTrail
from dimos.mapping.occupancy.room_store import StoredRoom, StoredRoomSet
from dimos.perception.sightings import Sighting

T0 = 1_000_000.0


def _room(room_id: int, x0: float, y0: float, x1: float, y1: float) -> StoredRoom:
    return StoredRoom(
        id=room_id,
        kind="room",
        area_m2=(x1 - x0) * (y1 - y0),
        centroid_xy=((x0 + x1) / 2, (y0 + y1) / 2),
        anchor_xy=((x0 + x1) / 2, (y0 + y1) / 2),
        max_clearance_m=1.0,
        polygon=np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]], dtype=np.float64),
        derived_ts=900.0,
    )


# Two 4x4 m rooms: room 1 around (2, 2), room 2 around (7, 2).
ROOM_SET = StoredRoomSet(
    derived_ts=900.0,
    source="test",
    explored_fraction=0.5,
    rooms=(_room(1, 0.0, 0.0, 4.0, 4.0), _room(2, 5.0, 0.0, 9.0, 4.0)),
    doorways=(),
)

# The robot starts in room 1, crosses to room 2, revisits room 1, then leaves.
TRAIL = PoseTrail(
    ts=T0 + np.arange(15.0),
    xy=np.array([[2, 2]] * 5 + [[6, 2]] * 5 + [[2, 2]] * 2 + [[20, 20]] * 3, dtype=np.float64),
)

# "box" is the trap object: last seen in room 1 at T0+4, later in room 2 at
# T0+8. "couch" exists so query 2 prefers it.
SIGHTINGS = [
    Sighting(name="box", ts=T0 + 2.0, position=(2.0, 2.0, 0.1)),
    Sighting(name="box", ts=T0 + 3.0, position=(2.0, 2.0, 0.1)),
    Sighting(name="box", ts=T0 + 4.0, position=(2.0, 2.0, 0.1)),
    Sighting(name="box", ts=T0 + 8.0, position=(7.0, 2.0, 0.1)),
    Sighting(name="couch", ts=T0 + 3.0, position=(7.0, 1.0, 0.2)),
    Sighting(name="couch", ts=T0 + 6.0, position=(7.0, 1.0, 0.2)),
]
VOCABULARY = ["box", "couch", "mug"]


def _build_key() -> object:
    return build_answer_key("test_rec", TRAIL, SIGHTINGS, VOCABULARY, ROOM_SET)


def test_object_entries_room_stays() -> None:
    entries = object_entries(SIGHTINGS, ROOM_SET)
    box = next(e for e in entries if e.name == "box")
    assert box.sightings == 4
    assert box.last_ts == T0 + 8.0
    by_room = {stay.room_id: stay for stay in box.rooms}
    assert by_room[1].intervals == [[T0 + 2.0, T0 + 4.0]]
    assert by_room[1].last_ts == T0 + 4.0
    assert by_room[2].intervals == [[T0 + 8.0, T0 + 8.0]]


def test_find_trap_instances() -> None:
    entries = object_entries(SIGHTINGS, ROOM_SET)
    traps = find_trap_instances(entries, SIGHTINGS, ROOM_SET)
    assert len(traps) == 1
    trap = traps[0]
    assert (trap.name, trap.room_id) == ("box", 1)
    assert trap.last_in_room_ts == T0 + 4.0
    assert trap.global_last_ts == T0 + 8.0
    # (2, 2) is inside room 1 and 3 m from room 2 — an unambiguous assignment.
    assert trap.margin_m == 3.0


def test_q1_visits_and_last_exit() -> None:
    key = build_answer_key("test_rec", TRAIL, SIGHTINGS, VOCABULARY, ROOM_SET)
    case = key.case("q1_region_visits")
    # The region is a 4 m box on the start pose = room 1: two visits.
    assert case.expected["visits"] == [[T0, T0 + 4.0], [T0 + 10.0, T0 + 11.0]]
    assert case.expected["last_exit_ts"] == T0 + 11.0
    assert case.skill == "robot_visits_to_region"
    assert case.skill_args["region"] == [0.0, 0.0, 4.0, 0.0, 4.0, 4.0, 0.0, 4.0]


def test_q2_prefers_couch() -> None:
    key = build_answer_key("test_rec", TRAIL, SIGHTINGS, VOCABULARY, ROOM_SET)
    case = key.case("q2_last_seen")
    assert case.skill_args == {"name": "couch"}
    assert case.expected["last_ts"] == T0 + 6.0


def test_q3_counts_and_qualifier() -> None:
    key = build_answer_key("test_rec", TRAIL, SIGHTINGS, VOCABULARY, ROOM_SET)
    case = key.case("q3_room_count")
    assert case.expected == {"n_rooms": 2, "n_corridors": 0, "explored_fraction": 0.5}
    assert "lower bound" in case.grading_notes


def test_q4_is_the_trap_case() -> None:
    key = build_answer_key("test_rec", TRAIL, SIGHTINGS, VOCABULARY, ROOM_SET)
    case = key.case("q4_last_seen_in_room")
    assert case.skill_args == {"name": "box", "room_id": 1}
    assert case.expected["last_in_room_ts"] == T0 + 4.0
    assert case.expected["last_interval"] == [T0 + 2.0, T0 + 4.0]
    assert case.expected["global_last_ts"] == T0 + 8.0
    assert "TRAP" in case.grading_notes


def test_q5_absent_object_never_in_vocabulary() -> None:
    key = build_answer_key("test_rec", TRAIL, SIGHTINGS, VOCABULARY, ROOM_SET)
    case = key.case("q5_never_in_room")
    assert case.skill_args == {"name": "crocodile", "room_id": 1}
    assert case.expected["ever_seen_in_region"] is False
    assert case.expected["ever_in_vocabulary"] is False


def test_everything_starts_unconfirmed() -> None:
    key = build_answer_key("test_rec", TRAIL, SIGHTINGS, VOCABULARY, ROOM_SET)
    assert all(not c.confirmed for c in key.cases)
    assert all(not o.confirmed for o in key.objects)
    assert not key.rooms.confirmed
    assert len(key.unconfirmed()) == len(key.cases) + len(key.objects) + 1


def test_queries_subset_builds_only_those_cases() -> None:
    key = build_answer_key("test_rec", TRAIL, SIGHTINGS, VOCABULARY, ROOM_SET, queries=(1, 3))
    assert [c.id for c in key.cases] == ["q1_region_visits", "q3_room_count"]


def test_q4_dropped_without_a_trap_instance() -> None:
    no_trap = [s for s in SIGHTINGS if s.name != "box"]
    key = build_answer_key("test_rec", TRAIL, no_trap, VOCABULARY, ROOM_SET)
    assert "q4_last_seen_in_room" not in [c.id for c in key.cases]


def test_q2_dropped_without_sightings() -> None:
    key = build_answer_key("test_rec", TRAIL, [], VOCABULARY, ROOM_SET, queries=(2, 5))
    assert [c.id for c in key.cases] == ["q5_never_in_room"]
