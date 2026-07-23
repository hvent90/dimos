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

"""Region-joined skills over the graph: trap case, honest negation, agent."""

from collections.abc import Callable, Iterator
from pathlib import Path
from typing import Any

import numpy as np
import pytest

from dimos.agents.skills.scene_memory import SceneMemorySkillContainer
from dimos.mapping.occupancy.room_segmentation import segment_rooms
from dimos.mapping.occupancy.room_store import RoomStore
from dimos.memory2.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.perception.scene_graph import BUILDING_ID, SceneGraph, Sighting

T0 = 1_000_000.0

# The synthetic map is two 4x4 m rooms joined by a doorway; segmentation
# deterministically labels the left room 1 and the right room 2, so the
# graph nodes are room_1 (left) and room_2 (right).
LEFT_ROOM = "room_1"
RIGHT_ROOM = "room_2"
LEFT_XY = (2.0, 2.0)
RIGHT_XY = (6.0, 2.0)

# The trap case: "box" seen in the left room during [T0+2, T0+4], then seen
# in the right room at T0+8 — "when did you last see box in the left room"
# must answer T0+4, not T0+8.
TRAP_SIGHTINGS = [
    Sighting(name="box", ts=T0 + 2.0, position=(*LEFT_XY, 0.1)),
    Sighting(name="box", ts=T0 + 3.0, position=(*LEFT_XY, 0.1)),
    Sighting(name="box", ts=T0 + 4.0, position=(*LEFT_XY, 0.1)),
    Sighting(name="box", ts=T0 + 8.0, position=(*RIGHT_XY, 0.1)),
]


def _two_room_grid() -> OccupancyGrid:
    cells = np.full((84, 166), 100, dtype=np.int16)
    cells[2:-2, 2:-2] = 0
    cells[:, 82:84] = 100
    cells[34:50, 82:84] = 0
    return OccupancyGrid(grid=cells.astype(np.int8), resolution=0.05, ts=900.0)


def _seed_scene(db: Path, sightings: list[Sighting], vocabulary: list[str]) -> None:
    """One derived room set + one scan pass over [T0, T0+10], via the graph."""
    with SceneGraph(db) as graph:
        with RoomStore(db) as rooms:
            rooms.save(segment_rooms(_two_room_grid()), source="test")
            room_set = rooms.latest()
        assert room_set is not None
        graph.apply_rooms(room_set)
        graph.fold_scan(
            sightings, t0=T0, t1=T0 + 10.0, vocabulary=vocabulary, source="test", frames=20
        )


def _write_trail(db: Path, points: list[tuple[float, float, float]]) -> None:
    with SqliteStore(path=str(db)) as store:
        stream = store.stream("odom", PoseStamped)
        for ts, x, y in points:
            pose = PoseStamped(ts=ts, position=(x, y, 0.0))
            stream.append(pose, ts=ts, pose=pose)


@pytest.fixture()
def make_container() -> Iterator[Callable[..., SceneMemorySkillContainer]]:
    started: list[SceneMemorySkillContainer] = []

    def make(**kwargs: Any) -> SceneMemorySkillContainer:
        module = SceneMemorySkillContainer(**kwargs)
        module.start()
        started.append(module)
        return module

    yield make
    for module in started:
        module.stop()


@pytest.fixture()
def trap_container(
    tmp_path: Path, make_container: Callable[..., SceneMemorySkillContainer]
) -> SceneMemorySkillContainer:
    db = tmp_path / "scene.db"
    _seed_scene(db, TRAP_SIGHTINGS, vocabulary=["box", "mug"])
    return make_container(sightings_db=str(db))


def test_trap_last_seen_in_region_is_not_global_last(
    trap_container: SceneMemorySkillContainer,
) -> None:
    result = trap_container.last_seen("box", in_node=LEFT_ROOM)
    assert result.success
    assert result.metadata["last_sighting"]["ts"] == T0 + 4.0
    assert result.metadata["last_sighting"]["room_id"] == LEFT_ROOM
    assert result.metadata["last_interval"] == [T0 + 2.0, T0 + 4.0]
    assert result.metadata["sightings_matched"] == 3
    assert result.metadata["later_elsewhere_ts"] == T0 + 8.0
    assert result.metadata["in_node"] == LEFT_ROOM
    assert "later seen outside" in result.message
    # The far-apart re-sighting is a second node; the in-region answer's
    # node payload is the left-room instance with full lineage.
    node = result.metadata["node"]
    assert node["id"] == "object_1"
    assert node["parent"] == LEFT_ROOM
    assert node["ancestors"] == [
        {"id": LEFT_ROOM, "layer": "room"},
        {"id": BUILDING_ID, "layer": "building"},
    ]
    # The global last-seen answer differs — that's the trap.
    global_last = trap_container.last_seen("box")
    assert global_last.metadata["last_sighting"]["ts"] == T0 + 8.0
    assert global_last.metadata["last_sighting"]["room_id"] == RIGHT_ROOM
    assert global_last.metadata["node"]["id"] == "object_2"
    assert "later_elsewhere_ts" not in global_last.metadata


def test_last_seen_in_other_region_is_the_later_sighting(
    trap_container: SceneMemorySkillContainer,
) -> None:
    result = trap_container.last_seen("box", in_node=RIGHT_ROOM)
    assert result.success
    assert result.metadata["last_sighting"]["ts"] == T0 + 8.0
    assert result.metadata["sightings_matched"] == 1
    assert "later_elsewhere_ts" not in result.metadata


def test_building_filter_matches_everything(
    trap_container: SceneMemorySkillContainer,
) -> None:
    result = trap_container.last_seen("box", in_node=BUILDING_ID)
    assert result.success
    assert result.metadata["sightings_matched"] == 4
    assert result.metadata["last_sighting"]["ts"] == T0 + 8.0


def test_ever_in_region_via_visits(trap_container: SceneMemorySkillContainer) -> None:
    # "Has a box ever been in the left room?" — the found answer carries the
    # full visit history, so first/last in-region times are one call.
    result = trap_container.last_seen("box", in_node=LEFT_ROOM)
    assert result.success
    assert result.metadata["visits"] == [[T0 + 2.0, T0 + 4.0]]


def test_never_answer_names_missing_vocabulary(
    trap_container: SceneMemorySkillContainer,
) -> None:
    result = trap_container.last_seen("unicorn", in_node=LEFT_ROOM)
    assert result.success
    assert result.metadata["sightings_matched"] == 0
    assert result.metadata["ever_in_vocabulary"] is False
    assert "never in any scan's vocabulary" in result.message


def test_seen_between_respects_region_and_window(
    trap_container: SceneMemorySkillContainer,
) -> None:
    result = trap_container.seen_between("box", T0 + 3.5, T0 + 9.0, in_node=LEFT_ROOM)
    assert result.success
    assert result.metadata["sightings_matched"] == 1
    assert result.metadata["last_sighting"]["ts"] == T0 + 4.0
    assert result.metadata["later_elsewhere_ts"] == T0 + 8.0

    nothing = trap_container.seen_between("box", T0 + 5.0, T0 + 7.0, in_node=LEFT_ROOM)
    assert nothing.success
    assert nothing.metadata["sightings_matched"] == 0


@pytest.fixture()
def coverage_container(
    tmp_path: Path, make_container: Callable[..., SceneMemorySkillContainer]
) -> SceneMemorySkillContainer:
    # Robot stays in the left room for the whole scan window; the only
    # sighting is also in the left room. The right room is never covered.
    db = tmp_path / "scene.db"
    _seed_scene(
        db,
        [Sighting(name="mug", ts=T0 + 5.0, position=(*LEFT_XY, 0.1))],
        vocabulary=["mug"],
    )
    trail_db = tmp_path / "trail.db"
    _write_trail(trail_db, [(T0 + i, *LEFT_XY) for i in range(11)])
    return make_container(sightings_db=str(db), trail_db=str(trail_db))


def test_never_in_uncovered_region_reports_weak_evidence(
    coverage_container: SceneMemorySkillContainer,
) -> None:
    result = coverage_container.last_seen("mug", in_node=RIGHT_ROOM)
    assert result.success
    assert result.metadata["sightings_matched"] == 0
    assert result.metadata["ever_in_vocabulary"] is True
    assert result.metadata["last_elsewhere_ts"] == T0 + 5.0
    assert result.metadata["coverage"] == {
        "scan_passes": 1,
        "passes_covering_region": 0,
        "region_last_scanned_ts": None,
    }
    assert "No scan pass is known to have covered" in result.message


def test_never_in_covered_region_reports_when_scanned(
    coverage_container: SceneMemorySkillContainer,
) -> None:
    result = coverage_container.last_seen("banana", in_node=LEFT_ROOM)
    assert result.success
    assert result.metadata["sightings_matched"] == 0
    assert result.metadata["ever_in_vocabulary"] is False
    # Last covered = the robot's last in-room trail sample inside the window.
    assert result.metadata["coverage"] == {
        "scan_passes": 1,
        "passes_covering_region": 1,
        "region_last_scanned_ts": T0 + 10.0,
    }
    assert f"1 of 1 scan pass(es) covered {LEFT_ROOM}" in result.message


def test_get_scene_per_room_coverage(
    coverage_container: SceneMemorySkillContainer,
) -> None:
    scene = coverage_container.get_scene()
    assert scene.success
    by_id = {r["id"]: r for r in scene.metadata["regions"]}
    assert by_id[LEFT_ROOM]["coverage"]["scan_passes_covering"] == 1
    assert by_id[LEFT_ROOM]["coverage"]["vocabulary"] == ["mug"]
    assert by_id[LEFT_ROOM]["objects"] == 1
    assert by_id[RIGHT_ROOM]["coverage"]["scan_passes_covering"] == 0
    assert by_id[RIGHT_ROOM]["coverage"]["last_covered_ts"] is None
    assert by_id[RIGHT_ROOM]["objects"] == 0
    assert scene.metadata["n_doorways"] == 1
    assert scene.metadata["agent"]["room"] == LEFT_ROOM


def test_coverage_from_sightings_alone_without_trail(
    tmp_path: Path, make_container: Callable[..., SceneMemorySkillContainer]
) -> None:
    # No pose trail configured: the left room still counts as covered
    # because a sighting landed there during the scan window.
    db = tmp_path / "scene.db"
    _seed_scene(
        db,
        [Sighting(name="mug", ts=T0 + 5.0, position=(*LEFT_XY, 0.1))],
        vocabulary=["mug"],
    )
    module = make_container(sightings_db=str(db))
    result = module.last_seen("banana", in_node=LEFT_ROOM)
    assert result.metadata["coverage"]["passes_covering_region"] == 1
    assert result.metadata["coverage"]["region_last_scanned_ts"] == T0 + 5.0
    uncovered = module.last_seen("banana", in_node=RIGHT_ROOM)
    assert uncovered.metadata["coverage"]["passes_covering_region"] == 0


def test_last_seen_in_region_never_sighted_anywhere(
    coverage_container: SceneMemorySkillContainer,
) -> None:
    result = coverage_container.last_seen("unicorn", in_node=LEFT_ROOM)
    assert result.success
    assert result.metadata["sightings_matched"] == 0
    assert result.metadata["ever_in_vocabulary"] is False
    assert result.metadata["known_names"] == ["mug"]
    assert "Never saw 'unicorn' in room_1" in result.message


def test_last_seen_in_region_sighted_only_elsewhere(
    coverage_container: SceneMemorySkillContainer,
) -> None:
    result = coverage_container.last_seen("mug", in_node=RIGHT_ROOM)
    assert result.success
    assert result.metadata["sightings_matched"] == 0
    assert result.metadata["last_elsewhere_ts"] == T0 + 5.0
    assert "Never saw 'mug' in room_2" in result.message


def test_wall_adjacent_sighting_snaps_to_nearest_room(
    tmp_path: Path, make_container: Callable[..., SceneMemorySkillContainer]
) -> None:
    # Objects are obstacles: their positions land in occupied cells just
    # outside the free-space room polygons. (-0.1, 2.0) is outside every
    # polygon but ~0.3 m from the left room — it must resolve to room_1.
    db = tmp_path / "scene.db"
    _seed_scene(
        db, [Sighting(name="shelf", ts=T0 + 3.0, position=(-0.1, 2.0, 0.4))], vocabulary=["shelf"]
    )
    module = make_container(sightings_db=str(db))
    in_left = module.last_seen("shelf", in_node=LEFT_ROOM)
    assert in_left.success
    assert in_left.metadata["last_sighting"]["ts"] == T0 + 3.0
    in_right = module.last_seen("shelf", in_node=RIGHT_ROOM)
    assert in_right.metadata["sightings_matched"] == 0


def test_far_sighting_stays_unassigned(
    tmp_path: Path, make_container: Callable[..., SceneMemorySkillContainer]
) -> None:
    db = tmp_path / "scene.db"
    _seed_scene(
        db, [Sighting(name="bird", ts=T0 + 3.0, position=(20.0, 20.0, 0.4))], vocabulary=["bird"]
    )
    module = make_container(sightings_db=str(db))
    result = module.last_seen("bird", in_node=LEFT_ROOM)
    assert result.metadata["sightings_matched"] == 0
    assert result.metadata["last_elsewhere_ts"] == T0 + 3.0
    # Unassigned objects still exist in the graph, parented to the building.
    found = module.find("bird")
    assert found.metadata["hits"][0]["parent"] == BUILDING_ID


def test_unknown_in_node_rejected(trap_container: SceneMemorySkillContainer) -> None:
    result = trap_container.last_seen("box", in_node="room_99")
    assert not result.success
    assert result.error_code == "INVALID_INPUT"
    assert "room_1" in result.message and "room_2" in result.message


def test_object_in_node_rejected(trap_container: SceneMemorySkillContainer) -> None:
    result = trap_container.last_seen("box", in_node="object_1")
    assert not result.success
    assert result.error_code == "INVALID_INPUT"
    assert "room" in result.message


def test_region_query_before_rooms_derived(
    tmp_path: Path, make_container: Callable[..., SceneMemorySkillContainer]
) -> None:
    module = make_container(sightings_db=str(tmp_path / "empty.db"))
    result = module.last_seen("box", in_node=LEFT_ROOM)
    assert not result.success
    assert result.error_code == "INVALID_INPUT"
    assert "rooms may not be derived yet" in result.message


def test_adjacent_rooms(trap_container: SceneMemorySkillContainer) -> None:
    result = trap_container.adjacent(LEFT_ROOM)
    assert result.success
    assert [n["node"]["id"] for n in result.metadata["neighbors"]] == [RIGHT_ROOM]
    doorway = result.metadata["neighbors"][0]
    assert doorway["doorway_width_m"] > 0
    assert abs(doorway["doorway_xy"][0] - 4.15) < 0.3
    unknown = trap_container.adjacent("room_99")
    assert not unknown.success
    assert unknown.error_code == "INVALID_INPUT"


@pytest.fixture()
def agent_container(
    tmp_path: Path, make_container: Callable[..., SceneMemorySkillContainer]
) -> SceneMemorySkillContainer:
    # 1 Hz walk: 5 s in the left room, 5 s in the right, 3 s back left,
    # ending with 2 s in the right room.
    db = tmp_path / "scene.db"
    _seed_scene(db, [], vocabulary=["mug"])
    points = []
    t = T0
    for x, seconds in [(LEFT_XY[0], 5), (RIGHT_XY[0], 5), (LEFT_XY[0], 3), (RIGHT_XY[0], 2)]:
        for _ in range(seconds):
            points.append((t, x, 2.0))
            t += 1.0
    trail_db = tmp_path / "trail.db"
    _write_trail(trail_db, points)
    return make_container(sightings_db=str(db), trail_db=str(trail_db))


def test_agent_last_seen_in_room(agent_container: SceneMemorySkillContainer) -> None:
    # "When were you last in the left room?" — two visits, the answer is the
    # second one's exit, with the same shape as object queries.
    result = agent_container.last_seen("agent", in_node=LEFT_ROOM)
    assert result.success
    assert result.metadata["visits"] == [[T0, T0 + 4.0], [T0 + 10.0, T0 + 12.0]]
    assert result.metadata["last_interval"] == [T0 + 10.0, T0 + 12.0]
    assert result.metadata["last_sighting"]["ts"] == T0 + 12.0
    assert result.metadata["last_sighting"]["room_id"] == LEFT_ROOM
    assert result.metadata["last_sighting"]["node_id"] == "agent_0"
    assert result.metadata["later_elsewhere_ts"] == T0 + 14.0
    node = result.metadata["node"]
    assert node["parent"] == RIGHT_ROOM  # where the trail ends, resolved lazily
    assert node["ancestors"][0] == {"id": RIGHT_ROOM, "layer": "room"}


def test_agent_never_in_unvisited_region(
    tmp_path: Path, make_container: Callable[..., SceneMemorySkillContainer]
) -> None:
    db = tmp_path / "scene.db"
    _seed_scene(db, [], vocabulary=["mug"])
    trail_db = tmp_path / "trail.db"
    _write_trail(trail_db, [(T0 + i, *LEFT_XY) for i in range(5)])
    module = make_container(sightings_db=str(db), trail_db=str(trail_db))
    result = module.last_seen("agent", in_node=RIGHT_ROOM)
    assert result.success
    assert result.metadata["sightings_matched"] == 0
    assert result.metadata["visits"] == []
    assert "full pose coverage" in result.message


def test_agent_seen_between_window(agent_container: SceneMemorySkillContainer) -> None:
    result = agent_container.seen_between("agent", T0, T0 + 6.0, in_node=LEFT_ROOM)
    assert result.success
    assert result.metadata["visits"] == [[T0, T0 + 4.0]]
    assert result.metadata["window"] == [T0, T0 + 6.0]


def test_where_am_i_resolves_room(agent_container: SceneMemorySkillContainer) -> None:
    result = agent_container.where_am_i()
    assert result.success
    node = result.metadata["node"]
    assert node["parent"] == RIGHT_ROOM
    assert node["ancestors"] == [
        {"id": RIGHT_ROOM, "layer": "room"},
        {"id": BUILDING_ID, "layer": "building"},
    ]
    earlier = agent_container.where_am_i(T0 + 6.0)
    assert earlier.metadata["node"]["parent"] == RIGHT_ROOM
    start = agent_container.where_am_i(T0)
    assert start.metadata["node"]["parent"] == LEFT_ROOM
