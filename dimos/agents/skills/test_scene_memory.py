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

from collections.abc import Iterator
from pathlib import Path

import numpy as np
import pytest

from dimos.agents.skills.scene_memory import (
    SceneMemorySkillContainer,
    load_pose_trail,
    visit_intervals,
)
from dimos.memory2.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.perception.sightings import Sighting, SightingsLog

T0 = 1_000_000.0

# 1 Hz walk: 5 s at x=5 (inside region), 5 s at x=50 (outside), 3 s back at
# x=5 (inside again), ending outside at x=50.
_SEGMENTS = [(5.0, 5), (50.0, 5), (5.0, 3), (50.0, 2)]
REGION = [0.0, 0.0, 10.0, 0.0, 10.0, 10.0, 0.0, 10.0]  # square containing x=5, y=5


def _trail_points() -> list[tuple[float, float, float]]:
    points = []
    t = T0
    for x, seconds in _SEGMENTS:
        for _ in range(seconds):
            points.append((t, x, 5.0))
            t += 1.0
    return points


@pytest.fixture(scope="module")
def trail_db(tmp_path_factory: pytest.TempPathFactory) -> Path:
    db_path = tmp_path_factory.mktemp("scene_memory") / "trail.db"
    with SqliteStore(path=str(db_path)) as store:
        stream = store.stream("odom", PoseStamped)
        for ts, x, y in _trail_points():
            pose = PoseStamped(ts=ts, position=(x, y, 0.0))
            stream.append(pose, ts=ts, pose=pose)
    return db_path


@pytest.fixture()
def container(trail_db: Path) -> Iterator[SceneMemorySkillContainer]:
    module = SceneMemorySkillContainer(trail_db=str(trail_db))
    module.start()
    try:
        yield module
    finally:
        module.stop()


def test_visit_intervals_groups_and_splits() -> None:
    ts = np.array([0.0, 1.0, 2.0, 10.0, 11.0, 20.0])
    inside = np.array([True, True, False, True, True, True])
    assert visit_intervals(ts, inside, max_gap_s=2.0) == [(0.0, 1.0), (10.0, 11.0), (20.0, 20.0)]


def test_visit_intervals_bridges_small_gaps() -> None:
    ts = np.array([0.0, 1.0, 2.5, 4.0])
    inside = np.array([True, True, True, True])
    assert visit_intervals(ts, inside, max_gap_s=2.0) == [(0.0, 4.0)]


def test_visit_intervals_empty() -> None:
    assert visit_intervals(np.array([0.0, 1.0]), np.array([False, False])) == []


def test_load_pose_trail_from_pose_columns(trail_db: Path) -> None:
    trail = load_pose_trail(str(trail_db), ["go2_odom", "odom"])
    assert len(trail.ts) == 15
    assert trail.time_range() == (T0, T0 + 14.0)
    assert trail.xy[0].tolist() == [5.0, 5.0]
    assert trail.xy[-1].tolist() == [50.0, 5.0]


def test_load_pose_trail_from_payload(tmp_path: Path) -> None:
    # Rows without pose columns fall back to the PoseStamped payload.
    db_path = tmp_path / "no_pose_columns.db"
    with SqliteStore(path=str(db_path)) as store:
        stream = store.stream("odom", PoseStamped)
        stream.append(PoseStamped(ts=T0, position=(1.0, 2.0, 0.0)), ts=T0)
    trail = load_pose_trail(str(db_path), ["odom"])
    assert trail.xy[0].tolist() == [1.0, 2.0]


def test_load_pose_trail_missing_stream(trail_db: Path) -> None:
    with pytest.raises(LookupError, match="nonexistent"):
        load_pose_trail(str(trail_db), ["nonexistent"])


def test_robot_trail_info(container: SceneMemorySkillContainer) -> None:
    result = container.robot_trail_info()
    assert result.success
    assert result.metadata["start_ts"] == T0
    assert result.metadata["end_ts"] == T0 + 14.0
    assert result.metadata["samples"] == 15


def test_robot_position_at(container: SceneMemorySkillContainer) -> None:
    result = container.robot_position_at(T0 + 7.2)
    assert result.success
    assert result.metadata["ts"] == T0 + 7.0
    assert result.metadata["x"] == 50.0
    assert result.metadata["y"] == 5.0


def test_robot_position_at_outside_trail(container: SceneMemorySkillContainer) -> None:
    result = container.robot_position_at(T0 - 100.0)
    assert not result.success
    assert result.error_code == "INVALID_INPUT"


def test_robot_visits_to_region(container: SceneMemorySkillContainer) -> None:
    result = container.robot_visits_to_region(REGION)
    assert result.success
    # Two visits: t0..t0+4 and t0+10..t0+12; the last exit is the answer to
    # "when were you last in the region".
    assert result.metadata["visits"] == [[T0, T0 + 4.0], [T0 + 10.0, T0 + 12.0]]
    assert result.metadata["last_exit_ts"] == T0 + 12.0


def test_robot_visits_to_region_never(container: SceneMemorySkillContainer) -> None:
    result = container.robot_visits_to_region([100.0, 100.0, 101.0, 100.0, 101.0, 101.0])
    assert result.success
    assert result.metadata["visits"] == []
    assert "never" in result.message


def test_robot_visits_to_region_bad_polygon(container: SceneMemorySkillContainer) -> None:
    result = container.robot_visits_to_region([1.0, 2.0, 3.0])
    assert not result.success
    assert result.error_code == "INVALID_INPUT"


def test_missing_db_fails_cleanly(tmp_path: Path) -> None:
    module = SceneMemorySkillContainer(trail_db=str(tmp_path / "missing.db"))
    module.start()
    try:
        result = module.robot_trail_info()
        assert not result.success
        assert result.error_code == "NOT_CONFIGURED"
    finally:
        module.stop()


@pytest.fixture()
def sightings_db(tmp_path: Path) -> Path:
    db = tmp_path / "sightings.db"
    with SightingsLog(db) as log:
        log.record_scan(
            [
                Sighting(name="couch", ts=T0 + 5.0, position=(1.0, 2.0, 0.1)),
                Sighting(name="couch", ts=T0 + 9.0, position=(1.1, 2.0, 0.1)),
                Sighting(name="tv", ts=T0 + 7.0, position=(4.0, 0.5, 1.2)),
            ],
            t0=T0,
            t1=T0 + 10.0,
            vocabulary=["couch", "tv", "plant"],
            source="test",
            frames=12,
        )
    return db


def test_last_seen_object(sightings_db: Path) -> None:
    module = SceneMemorySkillContainer(sightings_db=str(sightings_db))
    module.start()
    try:
        result = module.last_seen_object("couch")
        assert result.success
        assert result.metadata["last_ts"] == T0 + 9.0
        assert result.metadata["position"] == [1.1, 2.0, 0.1]
        assert result.metadata["count"] == 2
    finally:
        module.stop()


def test_last_seen_object_in_vocabulary_but_never_seen(sightings_db: Path) -> None:
    # "plant" was looked for but never detected — the answer must say so
    # rather than fabricate a sighting.
    module = SceneMemorySkillContainer(sightings_db=str(sightings_db))
    module.start()
    try:
        result = module.last_seen_object("plant")
        assert result.success
        assert result.metadata["sightings"] == 0
        assert result.metadata["ever_in_vocabulary"] is True
        assert "never detected" in result.message
    finally:
        module.stop()


def test_last_seen_object_never_in_vocabulary(sightings_db: Path) -> None:
    module = SceneMemorySkillContainer(sightings_db=str(sightings_db))
    module.start()
    try:
        result = module.last_seen_object("fire extinguisher")
        assert result.success
        assert result.metadata["ever_in_vocabulary"] is False
        assert "never in any scan's vocabulary" in result.message
        assert result.metadata["known_names"] == ["couch", "tv"]
    finally:
        module.stop()


def test_scan_for_objects_requires_camera_config(trail_db: Path, tmp_path: Path) -> None:
    module = SceneMemorySkillContainer(trail_db=str(trail_db), sightings_db=str(tmp_path / "s.db"))
    module.start()
    try:
        result = module.scan_for_objects(["chair"])
        assert not result.success
        assert result.error_code == "NOT_CONFIGURED"
    finally:
        module.stop()


def test_scan_for_objects_rejects_empty_prompt(tmp_path: Path) -> None:
    module = SceneMemorySkillContainer(sightings_db=str(tmp_path / "s.db"))
    module.start()
    try:
        result = module.scan_for_objects(["  "])
        assert not result.success
        assert result.error_code == "INVALID_INPUT"
    finally:
        module.stop()


def _two_room_grid() -> "OccupancyGrid":
    cells = np.full((84, 166), 100, dtype=np.int16)
    cells[2:-2, 2:-2] = 0
    cells[:, 82:84] = 100
    cells[34:50, 82:84] = 0
    return OccupancyGrid(grid=cells.astype(np.int8), resolution=0.05, ts=777.0)


def test_derive_rooms_and_rooms_skills(tmp_path: Path) -> None:
    module = SceneMemorySkillContainer(sightings_db=str(tmp_path / "scene.db"))
    module.start()
    try:
        no_map = module.derive_rooms()
        assert not no_map.success
        assert no_map.error_code == "INVALID_STATE"

        module._on_costmap(_two_room_grid())
        derived = module.derive_rooms()
        assert derived.success
        assert derived.metadata["n_rooms"] == 2
        assert derived.metadata["n_corridors"] == 0
        assert derived.metadata["n_doorways"] == 1
        assert derived.metadata["derived_ts"] == 777.0
    finally:
        module.stop()
    # A fresh container instance answers rooms() from the persisted store.
    fresh = SceneMemorySkillContainer(sightings_db=str(tmp_path / "scene.db"))
    fresh.start()
    try:
        result = fresh.rooms()
        assert result.success
        assert [r["id"] for r in result.metadata["rooms"]] == [1, 2]
        assert result.metadata["n_doorways"] == 1
        assert "2 room(s)" in result.message
    finally:
        fresh.stop()


def test_rooms_before_derivation_fails_cleanly(tmp_path: Path) -> None:
    module = SceneMemorySkillContainer(sightings_db=str(tmp_path / "scene.db"))
    module.start()
    try:
        result = module.rooms()
        assert not result.success
        assert result.error_code == "INVALID_STATE"
    finally:
        module.stop()


def test_no_trail_db_outside_replay_fails_cleanly() -> None:
    # Without an explicit trail_db and outside replay mode, the skills must
    # refuse rather than answer from the default replay dataset.
    module = SceneMemorySkillContainer()
    module.start()
    try:
        assert not module.config.g.replay
        result = module.robot_trail_info()
        assert not result.success
        assert result.error_code == "NOT_CONFIGURED"
    finally:
        module.stop()
