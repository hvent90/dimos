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

from dimos.agents.eval.answer_key import AnswerKey
from dimos.agents.eval.scene_eval_cases import (
    RegionShape,
    build_answer_key,
    find_trap_instances,
    object_entries,
)
from dimos.agents.skills.scene_memory import PoseTrail
from dimos.perception.scene_graph import Sighting

T0 = 1_000_000.0


def _region(region_id: str, kind: str, x0: float, y0: float, x1: float, y1: float) -> RegionShape:
    return RegionShape(
        id=region_id,
        kind=kind,
        polygon=np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]], dtype=np.float64),
    )


# Two 4x4 m rooms around (2, 2) and (7, 2), plus a corridor above room_1
# that shares its doorway with both rooms.
REGIONS = [
    _region("room_1", "room", 0.0, 0.0, 4.0, 4.0),
    _region("room_2", "room", 5.0, 0.0, 9.0, 4.0),
    _region("corridor_3", "corridor", 0.0, 5.0, 9.0, 7.0),
]
ADJACENCY = {
    "room_1": ["corridor_3"],
    "room_2": ["corridor_3"],
    "corridor_3": ["room_1", "room_2"],
}
COVERED_ROOMS = ["room_1", "room_2"]
EXPLORED = 0.5

# The robot starts in room 1, crosses to room 2, revisits room 1, then leaves.
TRAIL = PoseTrail(
    ts=T0 + np.arange(15.0),
    xy=np.array([[2, 2]] * 5 + [[6, 2]] * 5 + [[2, 2]] * 2 + [[20, 20]] * 3, dtype=np.float64),
)

# "box" is the trap object: last seen in room 1 at T0+4, later in room 2 at
# T0+8 (far apart -> two nodes, as the fold would assign). "couch" exists so
# queries 2/6 prefer it; "person" for query 7.
SIGHTINGS = [
    Sighting(name="box", ts=T0 + 2.0, position=(2.0, 2.0, 0.1), node_id="object_1"),
    Sighting(name="box", ts=T0 + 3.0, position=(2.0, 2.0, 0.1), node_id="object_1"),
    Sighting(name="box", ts=T0 + 4.0, position=(2.0, 2.0, 0.1), node_id="object_1"),
    Sighting(name="box", ts=T0 + 8.0, position=(7.0, 2.0, 0.1), node_id="object_2"),
    Sighting(name="couch", ts=T0 + 3.0, position=(7.0, 1.0, 0.2), node_id="object_3"),
    Sighting(name="couch", ts=T0 + 6.0, position=(7.0, 1.0, 0.2), node_id="object_3"),
    Sighting(name="person", ts=T0 + 9.0, position=(2.5, 2.5, 0.4), node_id="object_4"),
]
VOCABULARY = ["box", "couch", "mug", "person"]


def _build_key(queries: tuple[int, ...] | None = None, sightings: list | None = None) -> AnswerKey:
    kwargs = {} if queries is None else {"queries": queries}
    return build_answer_key(
        "test_rec",
        TRAIL,
        SIGHTINGS if sightings is None else sightings,
        VOCABULARY,
        REGIONS,
        EXPLORED,
        "test",
        COVERED_ROOMS,
        ADJACENCY,
        **kwargs,
    )


def test_object_entries_room_stays() -> None:
    entries = object_entries(SIGHTINGS, REGIONS)
    box = next(e for e in entries if e.name == "box")
    assert box.sightings == 4
    assert box.last_ts == T0 + 8.0
    assert box.last_room_id == "room_2"
    by_room = {stay.room_id: stay for stay in box.rooms}
    assert by_room["room_1"].intervals == [[T0 + 2.0, T0 + 4.0]]
    assert by_room["room_1"].last_ts == T0 + 4.0
    assert by_room["room_2"].intervals == [[T0 + 8.0, T0 + 8.0]]


def test_find_trap_instances() -> None:
    entries = object_entries(SIGHTINGS, REGIONS)
    traps = find_trap_instances(entries, SIGHTINGS, REGIONS)
    assert len(traps) == 1
    trap = traps[0]
    assert (trap.name, trap.room_id) == ("box", "room_1")
    assert trap.last_in_room_ts == T0 + 4.0
    assert trap.global_last_ts == T0 + 8.0
    # (2, 2) is inside room 1 and 3 m from room 2 — an unambiguous assignment.
    assert trap.margin_m == 3.0


def test_q1_agent_visits_to_start_room() -> None:
    case = _build_key().case("q1_agent_last_in_room")
    # The robot starts in room_1: two visits, leaving for good at T0+11.
    assert case.skill == "last_seen"
    assert case.skill_args == {"name": "agent", "in_node": "room_1"}
    assert case.expected["visits"] == [[T0, T0 + 4.0], [T0 + 10.0, T0 + 11.0]]
    assert case.expected["last_interval"] == [T0 + 10.0, T0 + 11.0]
    assert case.expected["last_exit_ts"] == T0 + 11.0


def test_q2_prefers_couch() -> None:
    case = _build_key().case("q2_last_seen")
    assert case.skill == "last_seen"
    assert case.skill_args == {"name": "couch"}
    assert case.expected["last_ts"] == T0 + 6.0
    assert case.expected["last_room_id"] == "room_2"


def test_q3_counts_and_qualifier() -> None:
    case = _build_key().case("q3_room_count")
    assert case.skill == "get_scene"
    assert case.expected == {"n_rooms": 2, "n_corridors": 1, "explored_fraction": 0.5}
    assert "lower bound" in case.grading_notes


def test_q4_is_the_trap_case() -> None:
    case = _build_key().case("q4_last_seen_in_room")
    assert case.skill == "last_seen"
    assert case.skill_args == {"name": "box", "in_node": "room_1"}
    assert case.expected["last_in_room_ts"] == T0 + 4.0
    assert case.expected["last_interval"] == [T0 + 2.0, T0 + 4.0]
    assert case.expected["later_elsewhere_ts"] == T0 + 8.0
    assert "TRAP" in case.grading_notes


def test_q5_absent_object_never_in_vocabulary() -> None:
    case = _build_key().case("q5_never_in_room")
    assert case.skill == "last_seen"
    assert case.skill_args == {"name": "fire extinguisher", "in_node": "room_1"}
    assert case.expected["sightings_matched"] == 0
    assert case.expected["ever_in_vocabulary"] is False


def test_q6_where_is_requires_staleness_qualifier() -> None:
    case = _build_key().case("q6_where_is")
    assert case.skill == "find"
    assert case.skill_args == {"text": "couch"}
    assert case.expected["room_id"] == "room_2"
    assert case.expected["staleness_qualifier_required"] is True
    assert "staleness" in case.grading_notes or "as of" in case.grading_notes


def test_q7_which_room_prefers_person() -> None:
    case = _build_key().case("q7_which_room_last_seen")
    assert case.skill == "last_seen"
    assert case.skill_args == {"name": "person"}
    assert case.expected["room_id"] == "room_1"


def test_q8_node_level_containment() -> None:
    case = _build_key().case("q8_whats_in_room")
    # Node-level: room_1 holds the first box node + the person; room_2 the
    # second box node + the couch. The tie breaks to the first room by id.
    assert case.skill == "nodes_in"
    assert case.skill_args == {"node_id": "room_1"}
    assert case.expected["object_names"] == ["box", "person"]


def test_q9_current_room_is_none_when_outside() -> None:
    # The trail ends at (20, 20), outside every region — q9 must be dropped.
    key = _build_key(queries=(9,))
    assert [c.id for c in key.cases] == []


def test_q10_corridor_adjacency() -> None:
    case = _build_key().case("q10_rooms_on_corridor")
    assert case.skill == "adjacent"
    assert case.skill_args == {"node_id": "corridor_3"}
    assert case.expected["neighbor_ids"] == ["room_1", "room_2"]


def test_everything_starts_unconfirmed() -> None:
    key = _build_key()
    assert all(not c.confirmed for c in key.cases)
    assert all(not o.confirmed for o in key.objects)
    assert not key.rooms.confirmed
    assert len(key.unconfirmed()) == len(key.cases) + len(key.objects) + 1


def test_queries_subset_builds_only_those_cases() -> None:
    key = _build_key(queries=(1, 3))
    assert [c.id for c in key.cases] == ["q1_agent_last_in_room", "q3_room_count"]


def test_q4_dropped_without_a_trap_instance() -> None:
    no_trap = [s for s in SIGHTINGS if s.name != "box"]
    key = _build_key(sightings=no_trap)
    assert "q4_last_seen_in_room" not in [c.id for c in key.cases]


def test_q2_dropped_without_sightings() -> None:
    key = _build_key(queries=(2, 5), sightings=[])
    assert [c.id for c in key.cases] == ["q5_never_in_room"]
