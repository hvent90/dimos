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

"""Builds the five scene-memory eval cases and their DRAFT answer key.

Pure functions over already-loaded data (pose trail, sightings, seeded
room set) so the construction rules are unit-testable. IO — rebuilding
grids, scanning recordings — lives in ``tool_generate_answer_key.py``.

The five queries, matching ``POST-query-matrix.md``:

1. When was the robot last in region R?    (pose trail x region)
2. When did you last see X?                (sightings)
3. How many rooms are there?               (room segmentation)
4. When did you last see X in room Y?      (sightings x region, trap-aware)
5. Has X ever been in room Y?              (honest negation with coverage)
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone

import numpy as np
from numpy.typing import NDArray

from dimos.agents.eval.answer_key import (
    AnswerKey,
    CaseEntry,
    ObjectEntry,
    RoomsEntry,
    RoomStay,
)
from dimos.agents.skills.scene_memory import (
    DEFAULT_SIGHTING_SNAP_M,
    SIGHTING_VISIT_GAP_S,
    PoseTrail,
    assign_to_rooms,
    visit_intervals,
)
from dimos.mapping.occupancy.polygons import (
    distance_to_polygon,
    points_in_polygon,
    polygon_from_flat,
)
from dimos.mapping.occupancy.room_store import StoredRoomSet
from dimos.perception.sightings import Sighting
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

ALL_QUERIES = (1, 2, 3, 4, 5)

# Side length of the hand-labeled region for query 1, centered on the trail
# start pose — the robot always leaves it, so "when were you last there" has
# a non-trivial answer.
Q1_REGION_SIDE_M = 4.0

# Query 2 prefers this object when it was sighted: common in offices and a
# verified-good detection class on the eval recordings.
Q2_PREFERRED_OBJECT = "couch"

# Query 4 prefers trap instances whose room assignment is unambiguous: the
# margin between the nearest and runner-up room must be at least this.
Q4_MIN_ASSIGNMENT_MARGIN_M = 0.3

# Query 5 asks about an object type that plausibly never appears in an
# office recording, probing for hallucinated sightings.
Q5_ABSENT_OBJECT = "crocodile"


def iso_utc(ts: float) -> str:
    return datetime.fromtimestamp(ts, tz=timezone.utc).strftime("%Y-%m-%d %H:%M:%S")


@dataclass(frozen=True)
class TrapInstance:
    """An object whose last sighting in a room precedes its last overall."""

    name: str
    room_id: int
    last_in_room_ts: float
    last_interval: tuple[float, float]
    global_last_ts: float
    margin_m: float


def _round3(values: list[tuple[float, float]]) -> list[list[float]]:
    return [[round(a, 3), round(b, 3)] for a, b in values]


def _sightings_xy(sightings: list[Sighting]) -> NDArray[np.float64]:
    return np.array([[s.position[0], s.position[1]] for s in sightings], dtype=np.float64).reshape(
        -1, 2
    )


def assignment_margin(point_xy: NDArray[np.float64], room_set: StoredRoomSet) -> float:
    """Gap between the nearest and runner-up room's effective distance.

    A small margin means the room assignment is a near coin toss — don't
    build an eval case on such a sighting.
    """
    point = point_xy.reshape(1, 2)
    effective = sorted(
        0.0
        if points_in_polygon(point, room.polygon)[0]
        else float(distance_to_polygon(point, room.polygon)[0])
        for room in room_set.rooms
    )
    return effective[1] - effective[0] if len(effective) > 1 else float("inf")


def object_entries(
    sightings: list[Sighting],
    room_set: StoredRoomSet,
    snap_m: float = DEFAULT_SIGHTING_SNAP_M,
) -> list[ObjectEntry]:
    """Per-object visibility summary with room stays (the reviewable labels)."""
    assigned = assign_to_rooms(_sightings_xy(sightings), room_set.rooms, snap_m)
    by_name: dict[str, list[tuple[Sighting, int]]] = {}
    for s, room_id in zip(sightings, assigned.tolist(), strict=True):
        by_name.setdefault(s.name, []).append((s, room_id))

    entries = []
    for name in sorted(by_name):
        rows = sorted(by_name[name], key=lambda r: r[0].ts)
        ts = np.asarray([s.ts for s, _ in rows])
        rooms = []
        for room_id in sorted({r for _, r in rows if r > 0}):
            inside = np.asarray([r == room_id for _, r in rows])
            intervals = visit_intervals(ts, inside, max_gap_s=SIGHTING_VISIT_GAP_S)
            in_room_ts = ts[inside]
            rooms.append(
                RoomStay(
                    room_id=room_id,
                    first_ts=round(float(in_room_ts[0]), 3),
                    last_ts=round(float(in_room_ts[-1]), 3),
                    intervals=_round3(intervals),
                )
            )
        last = rows[-1][0]
        entries.append(
            ObjectEntry(
                name=name,
                sightings=len(rows),
                first_ts=round(float(ts[0]), 3),
                last_ts=round(float(ts[-1]), 3),
                last_position=[round(v, 2) for v in last.position],
                rooms=rooms,
            )
        )
    return entries


def find_trap_instances(
    objects: list[ObjectEntry], sightings: list[Sighting], room_set: StoredRoomSet
) -> list[TrapInstance]:
    """All (object, room) pairs where the last in-room sighting isn't the
    object's last sighting overall — the query-4 trap shape."""
    traps = []
    for entry in objects:
        for stay in entry.rooms:
            if stay.last_ts >= entry.last_ts:
                continue
            last_in_room = next(
                s
                for s in reversed(sightings)
                if s.name == entry.name and round(s.ts, 3) == stay.last_ts
            )
            traps.append(
                TrapInstance(
                    name=entry.name,
                    room_id=stay.room_id,
                    last_in_room_ts=stay.last_ts,
                    last_interval=(stay.intervals[-1][0], stay.intervals[-1][1]),
                    global_last_ts=entry.last_ts,
                    margin_m=assignment_margin(np.asarray(last_in_room.position[:2]), room_set),
                )
            )
    return traps


def region_box(center_xy: NDArray[np.float64], side_m: float = Q1_REGION_SIDE_M) -> list[float]:
    """A flat axis-aligned square polygon centered on a point."""
    h = side_m / 2.0
    x, y = round(float(center_xy[0]), 2), round(float(center_xy[1]), 2)
    return [x - h, y - h, x + h, y - h, x + h, y + h, x - h, y + h]


def _q1_case(trail: PoseTrail) -> CaseEntry:
    region = region_box(trail.xy[0])
    inside = points_in_polygon(trail.xy, polygon_from_flat(region))
    visits = visit_intervals(trail.ts, inside)
    last_exit = visits[-1][1]
    corners = f"({region[0]}, {region[1]}) and ({region[4]}, {region[5]})"
    return CaseEntry(
        id="q1_region_visits",
        query=1,
        question=(
            f"When were you last inside the square region with opposite corners "
            f"{corners} in the world frame?"
        ),
        skill="robot_visits_to_region",
        skill_args={"region": region},
        expected={"visits": _round3(visits), "last_exit_ts": round(last_exit, 3)},
        grading_notes=(
            f"Full credit: the last visit, ending {iso_utc(last_exit)} UTC "
            f"(t_rel {last_exit - trail.ts[0]:.1f} s). Naming an earlier visit as the "
            f"last one scores 0.5. Times may be absolute UTC or relative to the "
            f"trail start {iso_utc(float(trail.ts[0]))} UTC. Claiming the robot was "
            f"never there is a hallucinated never."
        ),
    )


def _q2_case(objects: list[ObjectEntry]) -> CaseEntry:
    entry = next(
        (o for o in objects if o.name == Q2_PREFERRED_OBJECT),
        max(objects, key=lambda o: o.sightings),
    )
    x, y, z = entry.last_position
    return CaseEntry(
        id="q2_last_seen",
        query=2,
        question=f"When did you last see a {entry.name}?",
        skill="last_seen_object",
        skill_args={"name": entry.name},
        expected={
            "name": entry.name,
            "last_ts": entry.last_ts,
            "last_position": entry.last_position,
            "sightings": entry.sightings,
        },
        grading_notes=(
            f"Full credit: last sighting at {iso_utc(entry.last_ts)} UTC near "
            f"({x}, {y}). An earlier real sighting time scores 0.5. Claiming it was "
            f"never seen is a hallucinated never."
        ),
    )


def _q3_case(rooms: RoomsEntry) -> CaseEntry:
    return CaseEntry(
        id="q3_room_count",
        query=3,
        question="How many rooms are there?",
        skill="rooms",
        skill_args={},
        expected={
            "n_rooms": rooms.n_rooms,
            "n_corridors": rooms.n_corridors,
            "explored_fraction": rooms.explored_fraction,
        },
        grading_notes=(
            f"The seeded segmentation gives {rooms.n_rooms} rooms + "
            f"{rooms.n_corridors} corridor(s) at {rooms.explored_fraction:.0%} "
            f"explored. Full credit: that count WITH a qualifier that the map is "
            f"partially explored (count is a lower bound / glass pods may split). "
            f"The bare correct number without any qualifier scores 0.5. If the "
            f"agent re-derived rooms from the live map (see room_set_at_answer in "
            f"the run results), grade against that count instead."
        ),
    )


def _q4_case(traps: list[TrapInstance], trail_start: float) -> CaseEntry:
    robust = [t for t in traps if t.margin_m >= Q4_MIN_ASSIGNMENT_MARGIN_M] or traps
    trap = max(robust, key=lambda t: t.global_last_ts - t.last_in_room_ts)
    return CaseEntry(
        id="q4_last_seen_in_room",
        query=4,
        question=f"When did you last see a {trap.name} in room {trap.room_id}?",
        skill="last_seen_object_in_region",
        skill_args={"name": trap.name, "room_id": trap.room_id},
        expected={
            "name": trap.name,
            "room_id": trap.room_id,
            "last_in_room_ts": trap.last_in_room_ts,
            "last_interval": [trap.last_interval[0], trap.last_interval[1]],
            "global_last_ts": trap.global_last_ts,
            "assignment_margin_m": round(trap.margin_m, 2),
        },
        grading_notes=(
            f"TRAP: the {trap.name} was seen elsewhere later, at "
            f"{iso_utc(trap.global_last_ts)} UTC — answering with that time scores "
            f"0.0. Full credit: the last in-room sighting at "
            f"{iso_utc(trap.last_in_room_ts)} UTC (t_rel "
            f"{trap.last_in_room_ts - trail_start:.1f} s). An earlier in-room "
            f"interval scores 0.5. Claiming it was never in room {trap.room_id} is "
            f"a hallucinated never. Room ids refer to the seeded room set; if the "
            f"agent re-derived rooms mid-run, ids shifted — see room_set_at_answer."
        ),
    )


def _q5_case(vocabulary: list[str], room_set: StoredRoomSet) -> CaseEntry:
    assert Q5_ABSENT_OBJECT not in vocabulary
    room_id = room_set.by_kind("room")[0].id
    return CaseEntry(
        id="q5_never_in_room",
        query=5,
        question=f"Has a {Q5_ABSENT_OBJECT} ever been in room {room_id}?",
        skill="object_ever_in_region",
        skill_args={"name": Q5_ABSENT_OBJECT, "room_id": room_id},
        expected={
            "name": Q5_ABSENT_OBJECT,
            "room_id": room_id,
            "ever_seen_in_region": False,
            "ever_in_vocabulary": False,
        },
        grading_notes=(
            f"Correct answer: no. Full credit requires the negative to be "
            f"qualified by coverage — that '{Q5_ABSENT_OBJECT}' was never in any "
            f"scan's vocabulary (so it could not have been detected) and/or which "
            f"scans covered the room. A bare unqualified 'no' scores 0.5. "
            f"Claiming a {Q5_ABSENT_OBJECT} was seen scores 0.0."
        ),
    )


def build_answer_key(
    recording: str,
    trail: PoseTrail,
    sightings: list[Sighting],
    vocabulary: list[str],
    room_set: StoredRoomSet,
    queries: tuple[int, ...] = ALL_QUERIES,
) -> AnswerKey:
    """Assemble the DRAFT answer key. Every entry starts unconfirmed.

    Queries 2, 4, and 5 need sightings; query 4 additionally needs a natural
    trap instance. Cases that can't be built from the data are dropped with
    a warning rather than fabricated.
    """
    objects = object_entries(sightings, room_set)
    rooms_entry = RoomsEntry(
        n_rooms=len(room_set.by_kind("room")),
        n_corridors=len(room_set.by_kind("corridor")),
        explored_fraction=round(room_set.explored_fraction, 3),
        source=room_set.source,
    )

    cases = []
    if 1 in queries:
        cases.append(_q1_case(trail))
    if 2 in queries:
        if objects:
            cases.append(_q2_case(objects))
        else:
            logger.warning("Dropping q2: no sightings", recording=recording)
    if 3 in queries:
        cases.append(_q3_case(rooms_entry))
    if 4 in queries:
        traps = find_trap_instances(objects, sightings, room_set)
        if traps:
            cases.append(_q4_case(traps, float(trail.ts[0])))
        else:
            logger.warning("Dropping q4: no natural trap instance", recording=recording)
    if 5 in queries:
        cases.append(_q5_case(vocabulary, room_set))

    return AnswerKey(
        recording=recording,
        trail_start_ts=round(float(trail.ts[0]), 3),
        trail_end_ts=round(float(trail.ts[-1]), 3),
        vocabulary=sorted(vocabulary),
        rooms=rooms_entry,
        objects=objects,
        cases=cases,
    )
