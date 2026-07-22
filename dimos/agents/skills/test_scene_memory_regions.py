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

"""Region-join skills: sightings x room polygons, with honest negation."""

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
from dimos.perception.sightings import Sighting, SightingsLog

T0 = 1_000_000.0

# The synthetic map is two 4x4 m rooms joined by a doorway; segmentation
# deterministically labels the left room 1 and the right room 2.
LEFT_ROOM_ID = 1
RIGHT_ROOM_ID = 2
LEFT_XY = (2.0, 2.0)
RIGHT_XY = (6.0, 2.0)
LEFT_POLY = [0.2, 0.2, 4.0, 0.2, 4.0, 4.0, 0.2, 4.0]

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
    """One derived room set + one scan pass over [T0, T0+10]."""
    with RoomStore(db) as rooms:
        rooms.save(segment_rooms(_two_room_grid()), source="test")
    with SightingsLog(db) as log:
        log.record_scan(
            sightings, t0=T0, t1=T0 + 10.0, vocabulary=vocabulary, source="test", frames=20
        )


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
    result = trap_container.last_seen_object_in_region("box", room_id=LEFT_ROOM_ID)
    assert result.success
    assert result.metadata["last_ts"] == T0 + 4.0
    assert result.metadata["last_interval"] == [T0 + 2.0, T0 + 4.0]
    assert result.metadata["in_region_count"] == 3
    assert result.metadata["later_elsewhere_ts"] == T0 + 8.0
    assert "later seen outside" in result.message
    # The global last-seen answer differs — that's the trap.
    global_last = trap_container.last_seen_object("box")
    assert global_last.metadata["last_ts"] == T0 + 8.0


def test_trap_case_with_raw_polygon(trap_container: SceneMemorySkillContainer) -> None:
    result = trap_container.last_seen_object_in_region("box", region=LEFT_POLY)
    assert result.success
    assert result.metadata["last_ts"] == T0 + 4.0
    assert result.metadata["later_elsewhere_ts"] == T0 + 8.0


def test_last_seen_in_other_region_is_the_later_sighting(
    trap_container: SceneMemorySkillContainer,
) -> None:
    result = trap_container.last_seen_object_in_region("box", room_id=RIGHT_ROOM_ID)
    assert result.success
    assert result.metadata["last_ts"] == T0 + 8.0
    assert result.metadata["in_region_count"] == 1
    assert "later_elsewhere_ts" not in result.metadata


def test_ever_in_region_yes(trap_container: SceneMemorySkillContainer) -> None:
    result = trap_container.object_ever_in_region("box", room_id=LEFT_ROOM_ID)
    assert result.success
    assert result.metadata["ever_seen_in_region"] is True
    assert result.metadata["first_ts"] == T0 + 2.0
    assert result.metadata["last_ts"] == T0 + 4.0
    assert result.metadata["in_region_count"] == 3
    assert result.message.startswith("Yes")


def test_never_answer_names_missing_vocabulary(
    trap_container: SceneMemorySkillContainer,
) -> None:
    result = trap_container.object_ever_in_region("unicorn", room_id=LEFT_ROOM_ID)
    assert result.success
    assert result.metadata["ever_seen_in_region"] is False
    assert result.metadata["ever_in_vocabulary"] is False
    assert "never in any scan's vocabulary" in result.message


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
    with SqliteStore(path=str(trail_db)) as store:
        stream = store.stream("odom", PoseStamped)
        for i in range(11):
            pose = PoseStamped(ts=T0 + i, position=(*LEFT_XY, 0.0))
            stream.append(pose, ts=T0 + i, pose=pose)
    return make_container(sightings_db=str(db), trail_db=str(trail_db))


def test_never_in_uncovered_region_reports_weak_evidence(
    coverage_container: SceneMemorySkillContainer,
) -> None:
    result = coverage_container.object_ever_in_region("mug", room_id=RIGHT_ROOM_ID)
    assert result.success
    assert result.metadata["ever_seen_in_region"] is False
    assert result.metadata["ever_in_vocabulary"] is True
    assert result.metadata["sightings_elsewhere"] == 1
    assert result.metadata["scan_passes"] == 1
    assert result.metadata["scan_passes_covering_region"] == 0
    assert result.metadata["region_last_scanned_ts"] is None
    assert result.metadata["rooms_with_scan_coverage"] == [LEFT_ROOM_ID]
    assert "No scan pass is known to have covered" in result.message


def test_never_in_covered_region_reports_when_scanned(
    coverage_container: SceneMemorySkillContainer,
) -> None:
    result = coverage_container.object_ever_in_region("banana", room_id=LEFT_ROOM_ID)
    assert result.success
    assert result.metadata["ever_seen_in_region"] is False
    assert result.metadata["ever_in_vocabulary"] is False
    assert result.metadata["scan_passes_covering_region"] == 1
    # Last covered = the robot's last in-room trail sample inside the window.
    assert result.metadata["region_last_scanned_ts"] == T0 + 10.0
    assert "1 of 1 scan pass(es) covered room 1" in result.message


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
    result = module.object_ever_in_region("mug", room_id=RIGHT_ROOM_ID)
    assert result.success
    assert result.metadata["rooms_with_scan_coverage"] == [LEFT_ROOM_ID]


def test_last_seen_in_region_never_sighted_anywhere(
    coverage_container: SceneMemorySkillContainer,
) -> None:
    result = coverage_container.last_seen_object_in_region("unicorn", room_id=LEFT_ROOM_ID)
    assert result.success
    assert result.metadata["in_region_count"] == 0
    assert result.metadata["ever_in_vocabulary"] is False
    assert result.metadata["known_names"] == ["mug"]
    assert "No sightings of 'unicorn' anywhere" in result.message


def test_last_seen_in_region_sighted_only_elsewhere(
    coverage_container: SceneMemorySkillContainer,
) -> None:
    result = coverage_container.last_seen_object_in_region("mug", room_id=RIGHT_ROOM_ID)
    assert result.success
    assert result.metadata["in_region_count"] == 0
    assert result.metadata["last_elsewhere_ts"] == T0 + 5.0
    assert "Never saw 'mug' in room 2" in result.message


def test_wall_adjacent_sighting_snaps_to_nearest_room(
    tmp_path: Path, make_container: Callable[..., SceneMemorySkillContainer]
) -> None:
    # Objects are obstacles: their positions land in occupied cells just
    # outside the free-space room polygons. (-0.1, 2.0) is outside every
    # polygon but ~0.3 m from the left room — it must resolve to room 1.
    db = tmp_path / "scene.db"
    _seed_scene(
        db, [Sighting(name="shelf", ts=T0 + 3.0, position=(-0.1, 2.0, 0.4))], vocabulary=["shelf"]
    )
    module = make_container(sightings_db=str(db))
    in_left = module.last_seen_object_in_region("shelf", room_id=LEFT_ROOM_ID)
    assert in_left.success
    assert in_left.metadata["last_ts"] == T0 + 3.0
    in_right = module.object_ever_in_region("shelf", room_id=RIGHT_ROOM_ID)
    assert in_right.metadata["ever_seen_in_region"] is False


def test_far_sighting_stays_unassigned(
    tmp_path: Path, make_container: Callable[..., SceneMemorySkillContainer]
) -> None:
    db = tmp_path / "scene.db"
    _seed_scene(
        db, [Sighting(name="bird", ts=T0 + 3.0, position=(20.0, 20.0, 0.4))], vocabulary=["bird"]
    )
    module = make_container(sightings_db=str(db))
    result = module.object_ever_in_region("bird", room_id=LEFT_ROOM_ID)
    assert result.metadata["ever_seen_in_region"] is False
    assert result.metadata["sightings_elsewhere"] == 1


def test_region_query_requires_room_id_or_polygon(
    trap_container: SceneMemorySkillContainer,
) -> None:
    result = trap_container.last_seen_object_in_region("box")
    assert not result.success
    assert result.error_code == "INVALID_INPUT"


def test_region_query_unknown_room_id(trap_container: SceneMemorySkillContainer) -> None:
    result = trap_container.object_ever_in_region("box", room_id=99)
    assert not result.success
    assert result.error_code == "INVALID_INPUT"
    assert "known ids: [1, 2]" in result.message


def test_region_query_before_rooms_derived(
    tmp_path: Path, make_container: Callable[..., SceneMemorySkillContainer]
) -> None:
    module = make_container(sightings_db=str(tmp_path / "empty.db"))
    result = module.last_seen_object_in_region("box", room_id=1)
    assert not result.success
    assert result.error_code == "INVALID_STATE"


def test_region_query_rejects_both_room_id_and_polygon(
    trap_container: SceneMemorySkillContainer,
) -> None:
    # Redundant parameters must be refused, not silently reinterpreted —
    # the mismatch produced mislabeled answers before this was rejected.
    result = trap_container.object_ever_in_region("box", room_id=1, region=LEFT_POLY)
    assert not result.success
    assert result.error_code == "INVALID_INPUT"
    assert "not both" in result.message


def test_region_query_malformed_polygon(trap_container: SceneMemorySkillContainer) -> None:
    result = trap_container.object_ever_in_region("box", region=[1.0, 2.0, 3.0])
    assert not result.success
    assert result.error_code == "INVALID_INPUT"
