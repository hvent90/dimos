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

"""Builds the scene-memory eval cases and their DRAFT answer key.

Pure functions over already-loaded data (pose trail, sightings, region
shapes from the scene graph) so the construction rules are unit-testable.
IO — rebuilding grids, scanning recordings, reading the graph — lives in
``tool_generate_answer_key.py``.

The six primary queries (the task spec's "Query coverage"), answered by
the scene-graph skill surface:

1. When were you last in room R?      last_seen("agent", in_node=R)
2. When did you last see X?           last_seen(X)
3. How many rooms are there?          get_scene()
4. When did you last see X in Y?      last_seen(X, in_node=Y)  (trap-aware)
5. Has X ever been in room Y?         last_seen(X, in_node=Y)  (qualified never)
6. Where is my X?                     find(X)                  (staleness-qualified)

Plus four secondary adjacent-matrix cells, each a single call where the
first pass needed a probe loop or could not answer at all:

7. Which room did you last see X in?  last_seen(X)  (lineage IS the answer)
8. What's in room R?                  nodes_in(R)
9. What room are you in?              where_am_i()
10. Which rooms open onto corridor C? adjacent(C)

Region identity everywhere is the scene-graph node id ("room_3"), never a
raw polygon or bare index.
"""

from __future__ import annotations

from collections.abc import Callable
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
    visit_intervals,
)
from dimos.mapping.occupancy.polygons import (
    assign_to_polygons,
    distance_to_polygon,
    points_in_polygon,
)
from dimos.perception.scene_graph import Sighting
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

ALL_QUERIES = (1, 2, 3, 4, 5, 6, 7, 8, 9, 10)
PRIMARY_QUERIES = (1, 2, 3, 4, 5, 6)

# Queries 2/6/7 prefer these objects when sighted: common in offices and
# verified-good detection classes on the eval recordings.
Q2_PREFERRED_OBJECT = "couch"
Q7_PREFERRED_OBJECT = "person"

# Query 4 prefers trap instances whose room assignment is unambiguous: the
# margin between the nearest and runner-up room must be at least this.
Q4_MIN_ASSIGNMENT_MARGIN_M = 0.3

# Query 5 asks about an object type that plausibly never appears in an
# office recording, probing for hallucinated sightings. Must never be in
# any scan's vocabulary.
Q5_ABSENT_OBJECT = "fire extinguisher"


def iso_utc(ts: float) -> str:
    return datetime.fromtimestamp(ts, tz=timezone.utc).strftime("%Y-%m-%d %H:%M:%S")


@dataclass(frozen=True)
class RegionShape:
    """One derived region as the eval sees it: graph node id + polygon."""

    id: str  # scene-graph node id, e.g. "room_3" / "corridor_6"
    kind: str  # "room" | "corridor"
    polygon: NDArray[np.float64]  # (N, 2) world xy outline


@dataclass(frozen=True)
class TrapInstance:
    """An object whose last sighting in a room precedes its last overall."""

    name: str
    room_id: str
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


def assign_sightings(
    sightings: list[Sighting],
    regions: list[RegionShape],
    snap_m: float = DEFAULT_SIGHTING_SNAP_M,
) -> list[str]:
    """Region id per sighting ("" = none) — the fold's rule, recomputed.

    Independent recomputation of the fold-time ``room_id`` (same exclusive
    nearest-with-snap rule over the same polygons), so a layer-(a) check
    against these catches fold drift.
    """
    indices = assign_to_polygons(_sightings_xy(sightings), [r.polygon for r in regions], snap_m)
    return [regions[i].id if i >= 0 else "" for i in indices.tolist()]


def assignment_margin(point_xy: NDArray[np.float64], regions: list[RegionShape]) -> float:
    """Gap between the nearest and runner-up region's effective distance.

    A small margin means the room assignment is a near coin toss — don't
    build an eval case on such a sighting.
    """
    point = point_xy.reshape(1, 2)
    effective = sorted(
        0.0
        if points_in_polygon(point, region.polygon)[0]
        else float(distance_to_polygon(point, region.polygon)[0])
        for region in regions
    )
    return effective[1] - effective[0] if len(effective) > 1 else float("inf")


def object_entries(
    sightings: list[Sighting],
    regions: list[RegionShape],
    snap_m: float = DEFAULT_SIGHTING_SNAP_M,
) -> list[ObjectEntry]:
    """Per-object visibility summary with room stays (the reviewable labels)."""
    assigned = assign_sightings(sightings, regions, snap_m)
    by_name: dict[str, list[tuple[Sighting, str]]] = {}
    for s, room_id in zip(sightings, assigned, strict=True):
        by_name.setdefault(s.name, []).append((s, room_id))

    entries = []
    for name in sorted(by_name):
        rows = sorted(by_name[name], key=lambda r: r[0].ts)
        ts = np.asarray([s.ts for s, _ in rows])
        rooms = []
        for room_id in sorted({r for _, r in rows if r}):
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
                last_room_id=rows[-1][1],
                rooms=rooms,
            )
        )
    return entries


def node_room_assignments(
    sightings: list[Sighting],
    regions: list[RegionShape],
    snap_m: float = DEFAULT_SIGHTING_SNAP_M,
) -> dict[str, tuple[str, str]]:
    """Per object node: (name, room of its latest sighting).

    The fold parents each node by its latest sighting's position — this
    recomputes that containment re-check from the sightings log, giving an
    independent reference for "what's in room R" (node-level: two far-apart
    couches are two entries).
    """
    last_by_node: dict[str, Sighting] = {}
    for s in sightings:
        if s.node_id and (s.node_id not in last_by_node or s.ts >= last_by_node[s.node_id].ts):
            last_by_node[s.node_id] = s
    nodes = list(last_by_node.items())
    assigned = assign_sightings([s for _, s in nodes], regions, snap_m)
    return {
        node_id: (s.name, room_id) for (node_id, s), room_id in zip(nodes, assigned, strict=True)
    }


def find_trap_instances(
    objects: list[ObjectEntry], sightings: list[Sighting], regions: list[RegionShape]
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
                    margin_m=assignment_margin(np.asarray(last_in_room.position[:2]), regions),
                )
            )
    return traps


def agent_region_visits(trail: PoseTrail, region: RegionShape) -> tuple[list[list[float]], float]:
    """The robot's visit intervals to one region (strict containment).

    The agent moves through free space, so membership is strict
    point-in-polygon — the same rule last_seen("agent", ...) applies —
    unlike object sightings, which snap to the nearest room.
    """
    inside = points_in_polygon(trail.xy, region.polygon)
    visits = visit_intervals(trail.ts, inside)
    return _round3(visits), round(visits[-1][1], 3) if visits else 0.0


def region_at(trail_xy: NDArray[np.float64], regions: list[RegionShape]) -> RegionShape | None:
    """The region strictly containing a point, if any."""
    point = trail_xy.reshape(1, 2)
    for region in regions:
        if points_in_polygon(point, region.polygon)[0]:
            return region
    return None


def _q1_case(trail: PoseTrail, regions: list[RegionShape]) -> CaseEntry | None:
    region = region_at(trail.xy[0], regions)
    if region is None:
        return None
    visits, last_exit = agent_region_visits(trail, region)
    return CaseEntry(
        id="q1_agent_last_in_room",
        query=1,
        question=f"When were you last in {region.id}?",
        skill="last_seen",
        skill_args={"name": "agent", "in_node": region.id},
        expected={
            "in_node": region.id,
            "visits": visits,
            "last_interval": visits[-1],
            "last_exit_ts": last_exit,
        },
        grading_notes=(
            f"Full credit: the last visit, ending {iso_utc(last_exit)} UTC "
            f"(t_rel {last_exit - trail.ts[0]:.1f} s). Naming an earlier visit as "
            f"the last one scores 0.5. Times may be absolute UTC or relative to "
            f"the trail start {iso_utc(float(trail.ts[0]))} UTC. Claiming the "
            f"robot was never there is a hallucinated never."
        ),
    )


def _q2_case(objects: list[ObjectEntry]) -> CaseEntry:
    entry = next(
        (o for o in objects if o.name == Q2_PREFERRED_OBJECT),
        max(objects, key=lambda o: o.sightings),
    )
    x, y, _z = entry.last_position
    return CaseEntry(
        id="q2_last_seen",
        query=2,
        question=f"When did you last see a {entry.name}?",
        skill="last_seen",
        skill_args={"name": entry.name},
        expected={
            "name": entry.name,
            "last_ts": entry.last_ts,
            "last_position": entry.last_position,
            "last_room_id": entry.last_room_id,
            "sightings": entry.sightings,
        },
        grading_notes=(
            f"Full credit: last sighting at {iso_utc(entry.last_ts)} UTC near "
            f"({x}, {y}). An earlier real sighting time scores 0.5. Claiming it "
            f"was never seen is a hallucinated never."
        ),
    )


def _q3_case(rooms: RoomsEntry) -> CaseEntry:
    return CaseEntry(
        id="q3_room_count",
        query=3,
        question="How many rooms are there?",
        skill="get_scene",
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
        question=f"When did you last see a {trap.name} in {trap.room_id}?",
        skill="last_seen",
        skill_args={"name": trap.name, "in_node": trap.room_id},
        expected={
            "name": trap.name,
            "in_node": trap.room_id,
            "last_in_room_ts": trap.last_in_room_ts,
            "last_interval": [trap.last_interval[0], trap.last_interval[1]],
            "later_elsewhere_ts": trap.global_last_ts,
            "assignment_margin_m": round(trap.margin_m, 2),
        },
        grading_notes=(
            f"TRAP: the {trap.name} was seen elsewhere later, at "
            f"{iso_utc(trap.global_last_ts)} UTC — answering with that time scores "
            f"0.0. Full credit: the last in-room sighting at "
            f"{iso_utc(trap.last_in_room_ts)} UTC (t_rel "
            f"{trap.last_in_room_ts - trail_start:.1f} s). An earlier in-room "
            f"interval scores 0.5. Claiming it was never in {trap.room_id} is a "
            f"hallucinated never. Room ids refer to the seeded scene graph; if "
            f"the agent re-derived rooms mid-run, ids shifted — see "
            f"room_set_at_answer."
        ),
    )


def _q5_case(vocabulary: list[str], covered_room_ids: list[str]) -> CaseEntry | None:
    assert Q5_ABSENT_OBJECT not in vocabulary
    if not covered_room_ids:
        return None
    room_id = covered_room_ids[0]
    return CaseEntry(
        id="q5_never_in_room",
        query=5,
        question=f"Has a {Q5_ABSENT_OBJECT} ever been in {room_id}?",
        skill="last_seen",
        skill_args={"name": Q5_ABSENT_OBJECT, "in_node": room_id},
        expected={
            "name": Q5_ABSENT_OBJECT,
            "in_node": room_id,
            "sightings_matched": 0,
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


def _q6_case(objects: list[ObjectEntry]) -> CaseEntry:
    entry = next(
        (o for o in objects if o.name == Q2_PREFERRED_OBJECT),
        max(objects, key=lambda o: o.sightings),
    )
    x, y, _z = entry.last_position
    return CaseEntry(
        id="q6_where_is",
        query=6,
        question=f"Where is my {entry.name}?",
        skill="find",
        skill_args={"text": entry.name},
        expected={
            "name": entry.name,
            "room_id": entry.last_room_id,
            "position": entry.last_position,
            "position_tolerance_m": 1.0,
            "last_ts": entry.last_ts,
            "staleness_qualifier_required": True,
        },
        grading_notes=(
            f"A memory's 'where is' means 'where I last saw it'. Full credit: "
            f"{entry.last_room_id or 'the right place'} and/or a position within "
            f"~1 m of ({x}, {y}), WITH a staleness qualifier (e.g. 'as of "
            f"{iso_utc(entry.last_ts)} UTC' / 'when I last saw it'). An "
            f"unqualified present-tense claim of the right place scores 0.5. The "
            f"wrong room or a fabricated position scores 0.0. last_seen instead "
            f"of find is equally acceptable."
        ),
    )


def _q7_case(objects: list[ObjectEntry]) -> CaseEntry | None:
    entry = next(
        (o for o in objects if o.name == Q7_PREFERRED_OBJECT and o.last_room_id),
        next((o for o in objects if o.last_room_id), None),
    )
    if entry is None:
        return None
    return CaseEntry(
        id="q7_which_room_last_seen",
        query=7,
        question=f"Which room did you last see a {entry.name} in?",
        skill="last_seen",
        skill_args={"name": entry.name},
        expected={"name": entry.name, "room_id": entry.last_room_id, "last_ts": entry.last_ts},
        grading_notes=(
            f"One call answers this on the graph surface — the lineage in the "
            f"result IS the answer. Full credit: {entry.last_room_id}. A "
            f"different room the {entry.name} was really in earlier scores 0.5. "
            f"'I can't tell which room' is non-responsive (the first pass's "
            f"failure mode)."
        ),
    )


def _q8_case(sightings: list[Sighting], regions: list[RegionShape]) -> CaseEntry | None:
    rooms: dict[str, list[str]] = {}
    for _node_id, (name, room_id) in node_room_assignments(sightings, regions).items():
        if room_id:
            rooms.setdefault(room_id, []).append(name)
    if not rooms:
        return None
    room_id = max(sorted(rooms), key=lambda r: len(rooms[r]))
    return CaseEntry(
        id="q8_whats_in_room",
        query=8,
        question=f"What objects are in {room_id}?",
        skill="nodes_in",
        skill_args={"node_id": room_id},
        expected={"room_id": room_id, "object_names": sorted(rooms[room_id])},
        grading_notes=(
            "Full credit: names (or an accurate count) matching the reference "
            "list — deterministic containment, not eyeballing. The reference is "
            "node-level: one entry per object instance, parented by its latest "
            "sighting's room (the fold's own containment rule, recomputed). "
            "Extra objects that are really elsewhere, or missing most of the "
            "list, scores 0.5; a fabricated inventory scores 0.0."
        ),
    )


def _q9_case(trail: PoseTrail, regions: list[RegionShape]) -> CaseEntry | None:
    region = region_at(trail.xy[-1], regions)
    if region is None:
        return None
    return CaseEntry(
        id="q9_current_room",
        query=9,
        question="What room are you in right now?",
        skill="where_am_i",
        skill_args={},
        expected={"room_id": region.id, "ts": round(float(trail.ts[-1]), 3)},
        grading_notes=(
            f"Full credit: {region.id} (the room containing the end of the pose "
            f"trail). In replay 'now' = the end of the recording; an answer "
            f"anchored to an earlier pose scores 0.5."
        ),
    )


def _q10_case(regions: list[RegionShape], adjacency: dict[str, list[str]]) -> CaseEntry | None:
    corridor = next((r for r in regions if r.kind == "corridor" and adjacency.get(r.id)), None)
    if corridor is None:
        return None
    neighbors = sorted(adjacency[corridor.id])
    return CaseEntry(
        id="q10_rooms_on_corridor",
        query=10,
        question=f"Which rooms open onto {corridor.id}?",
        skill="adjacent",
        skill_args={"node_id": corridor.id},
        expected={"node_id": corridor.id, "neighbor_ids": neighbors},
        grading_notes=(
            f"Full credit: the doorway-adjacent set {neighbors} (order "
            f"irrelevant; naming most of them with none fabricated also "
            f"passes). Missing more than half scores 0.5; fabricated adjacency "
            f"scores 0.0. This was stored but unaskable on the first-pass "
            f"surface."
        ),
    )


def build_answer_key(
    recording: str,
    trail: PoseTrail,
    sightings: list[Sighting],
    vocabulary: list[str],
    regions: list[RegionShape],
    explored_fraction: float,
    source: str,
    covered_room_ids: list[str],
    adjacency: dict[str, list[str]],
    queries: tuple[int, ...] = ALL_QUERIES,
) -> AnswerKey:
    """Assemble the DRAFT answer key. Every entry starts unconfirmed.

    Cases that can't be built from the data (no sightings, no natural trap
    instance, no corridor with doorways, ...) are dropped with a warning
    rather than fabricated.
    """
    objects = object_entries(sightings, regions)
    rooms_entry = RoomsEntry(
        n_rooms=sum(1 for r in regions if r.kind == "room"),
        n_corridors=sum(1 for r in regions if r.kind == "corridor"),
        explored_fraction=round(explored_fraction, 3),
        source=source,
    )

    builders: dict[int, Callable[[], CaseEntry | None]] = {
        1: lambda: _q1_case(trail, regions),
        2: lambda: _q2_case(objects) if objects else None,
        3: lambda: _q3_case(rooms_entry),
        4: lambda: (
            _q4_case(traps, float(trail.ts[0]))
            if (traps := find_trap_instances(objects, sightings, regions))
            else None
        ),
        5: lambda: _q5_case(vocabulary, covered_room_ids),
        6: lambda: _q6_case(objects) if objects else None,
        7: lambda: _q7_case(objects),
        8: lambda: _q8_case(sightings, regions),
        9: lambda: _q9_case(trail, regions),
        10: lambda: _q10_case(regions, adjacency),
    }
    cases = []
    for query in queries:
        case = builders[query]()
        if case is None:
            logger.warning("Dropping case: not constructible", query=query, recording=recording)
        else:
            cases.append(case)

    return AnswerKey(
        recording=recording,
        trail_start_ts=round(float(trail.ts[0]), 3),
        trail_end_ts=round(float(trail.ts[-1]), 3),
        vocabulary=sorted(vocabulary),
        rooms=rooms_entry,
        objects=objects,
        cases=cases,
    )
