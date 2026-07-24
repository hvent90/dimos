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

"""Core scene-memory skills: trail, find/near, last_seen, derive/persist."""

from collections.abc import Callable, Iterator
from pathlib import Path
from typing import Any

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
from dimos.perception.scene_graph import BUILDING_ID, SceneGraph, Sighting

T0 = 1_000_000.0

# 1 Hz walk: 5 s at x=5, 5 s at x=50, 3 s back at x=5, ending at x=50.
_SEGMENTS = [(5.0, 5), (50.0, 5), (5.0, 3), (50.0, 2)]


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


def test_get_scene_reports_time_range_and_honest_room_absence(
    trail_db: Path, tmp_path: Path, make_container: Callable[..., SceneMemorySkillContainer]
) -> None:
    module = make_container(trail_db=str(trail_db), sightings_db=str(tmp_path / "scene.db"))
    result = module.get_scene()
    assert result.success
    assert result.metadata["time_range"] == [T0, T0 + 14.0]
    assert result.metadata["regions"] == []
    assert result.metadata["agent"]["position"] == [50.0, 5.0, 0.0]
    assert "No rooms are derived yet" in result.message


def test_where_am_i_at_time(
    trail_db: Path, tmp_path: Path, make_container: Callable[..., SceneMemorySkillContainer]
) -> None:
    module = make_container(trail_db=str(trail_db), sightings_db=str(tmp_path / "scene.db"))
    result = module.where_am_i(T0 + 7.2)
    assert result.success
    assert result.metadata["ts"] == T0 + 7.0
    assert result.metadata["node"]["position"] == [50.0, 5.0, 0.0]
    assert result.metadata["node"]["id"] == "agent_0"
    # No rooms derived: containment is honestly absent, not invented.
    assert result.metadata["node"]["parent"] is None


def test_where_am_i_defaults_to_end_of_trail(
    trail_db: Path, tmp_path: Path, make_container: Callable[..., SceneMemorySkillContainer]
) -> None:
    module = make_container(trail_db=str(trail_db), sightings_db=str(tmp_path / "scene.db"))
    result = module.where_am_i()
    assert result.success
    assert result.metadata["ts"] == T0 + 14.0
    assert result.metadata["node"]["position"] == [50.0, 5.0, 0.0]


def test_where_am_i_outside_trail(
    trail_db: Path, tmp_path: Path, make_container: Callable[..., SceneMemorySkillContainer]
) -> None:
    module = make_container(trail_db=str(trail_db), sightings_db=str(tmp_path / "scene.db"))
    result = module.where_am_i(T0 - 100.0)
    assert not result.success
    assert result.error_code == "INVALID_INPUT"


def test_missing_trail_db_fails_cleanly(
    tmp_path: Path, make_container: Callable[..., SceneMemorySkillContainer]
) -> None:
    module = make_container(
        trail_db=str(tmp_path / "missing.db"), sightings_db=str(tmp_path / "scene.db")
    )
    result = module.where_am_i()
    assert not result.success
    assert result.error_code == "NOT_CONFIGURED"


def test_no_trail_db_outside_replay_fails_cleanly(
    tmp_path: Path, make_container: Callable[..., SceneMemorySkillContainer]
) -> None:
    # Without an explicit trail_db and outside replay mode, the skills must
    # refuse rather than answer from the default replay dataset.
    module = make_container(sightings_db=str(tmp_path / "scene.db"))
    assert not module.config.g.replay
    result = module.where_am_i()
    assert not result.success
    assert result.error_code == "NOT_CONFIGURED"


@pytest.fixture()
def seeded_db(tmp_path: Path) -> Path:
    """couch x2 + tv folded through the graph; 'plant' looked for, never seen."""
    db = tmp_path / "scene.db"
    with SceneGraph(db) as graph:
        graph.fold_scan(
            [
                Sighting(
                    name="couch",
                    ts=T0 + 5.0,
                    position=(1.0, 2.0, 0.1),
                    extent=(0.2, 1.6, 0.0, 1.8, 2.4, 0.7),
                ),
                Sighting(
                    name="couch",
                    ts=T0 + 9.0,
                    position=(1.1, 2.0, 0.1),
                    extent=(0.3, 1.7, 0.0, 2.0, 2.4, 0.6),
                ),
                Sighting(name="tv", ts=T0 + 7.0, position=(4.0, 0.5, 1.2)),
            ],
            t0=T0,
            t1=T0 + 10.0,
            vocabulary=["couch", "plant", "tv"],
            source="test",
            frames=12,
        )
    return db


def test_last_seen_object(
    seeded_db: Path, make_container: Callable[..., SceneMemorySkillContainer]
) -> None:
    module = make_container(sightings_db=str(seeded_db))
    result = module.last_seen("couch")
    assert result.success
    assert result.metadata["last_sighting"] == {
        "ts": T0 + 9.0,
        "position": [1.1, 2.0, 0.1],
        "room_id": None,
        "node_id": "object_1",
    }
    assert result.metadata["sightings_matched"] == 2
    assert result.metadata["in_node"] is None
    assert result.metadata["last_interval"] == [T0 + 5.0, T0 + 9.0]
    # The canonical node payload with lineage rides along.
    node = result.metadata["node"]
    assert node["id"] == "object_1"
    assert node["parent"] == BUILDING_ID
    assert node["ancestors"] == [{"id": BUILDING_ID, "layer": "building"}]


def test_last_seen_in_vocabulary_but_never_seen(
    seeded_db: Path, make_container: Callable[..., SceneMemorySkillContainer]
) -> None:
    # "plant" was looked for but never detected — the answer must say so
    # rather than fabricate a sighting.
    module = make_container(sightings_db=str(seeded_db))
    result = module.last_seen("plant")
    assert result.success
    assert result.metadata["sightings_matched"] == 0
    assert result.metadata["ever_in_vocabulary"] is True
    assert result.metadata["coverage"]["scan_passes"] == 1
    assert "was in the scan vocabulary" in result.message


def test_last_seen_never_in_vocabulary(
    seeded_db: Path, make_container: Callable[..., SceneMemorySkillContainer]
) -> None:
    module = make_container(sightings_db=str(seeded_db))
    result = module.last_seen("fire extinguisher")
    assert result.success
    assert result.metadata["ever_in_vocabulary"] is False
    assert "never in any scan's vocabulary" in result.message
    assert result.metadata["known_names"] == ["couch", "tv"]


def test_seen_between_window(
    seeded_db: Path, make_container: Callable[..., SceneMemorySkillContainer]
) -> None:
    module = make_container(sightings_db=str(seeded_db))
    result = module.seen_between("couch", T0, T0 + 6.0)
    assert result.success
    assert result.metadata["sightings_matched"] == 1
    assert result.metadata["last_sighting"]["ts"] == T0 + 5.0
    assert result.metadata["window"] == [T0, T0 + 6.0]

    empty = module.seen_between("couch", T0 + 10.0, T0 + 20.0)
    assert empty.success
    assert empty.metadata["sightings_matched"] == 0

    bad = module.seen_between("couch", T0 + 5.0, T0 + 1.0)
    assert not bad.success
    assert bad.error_code == "INVALID_INPUT"


def test_find_hit_and_miss(
    seeded_db: Path, make_container: Callable[..., SceneMemorySkillContainer]
) -> None:
    module = make_container(sightings_db=str(seeded_db))
    hit = module.find("couch")
    assert hit.success
    assert [h["id"] for h in hit.metadata["hits"]] == ["object_1"]
    assert hit.metadata["hits"][0]["ancestors"] == [{"id": BUILDING_ID, "layer": "building"}]
    # Object footprint = union of the two sighting AABBs; z-range rides along.
    assert hit.metadata["hits"][0]["extent"] == [0.2, 1.6, 2.0, 2.4]
    assert hit.metadata["hits"][0]["z_range"] == [0.0, 0.7]
    # The tv was folded without geometry — its extent stays honestly null.
    tv = module.find("tv")
    assert tv.metadata["hits"][0]["extent"] is None
    assert "z_range" not in tv.metadata["hits"][0]

    by_id = module.find("object_2")
    assert [h["id"] for h in by_id.metadata["hits"]] == ["object_2"]

    miss = module.find("unicorn")
    assert miss.success
    assert miss.metadata["hits"] == []
    assert miss.metadata["ever_in_vocabulary"] is False
    assert miss.metadata["known_names"] == ["couch", "tv"]

    empty = module.find("  ")
    assert not empty.success
    assert empty.error_code == "INVALID_INPUT"


def test_near(seeded_db: Path, make_container: Callable[..., SceneMemorySkillContainer]) -> None:
    module = make_container(sightings_db=str(seeded_db))
    # couch object_1 at (1.1, 2.0); tv object_2 at (4.0, 0.5) — 3.26 m apart.
    result = module.near(node_id="object_1", radius=4.0)
    assert result.success
    assert [h["id"] for h in result.metadata["hits"]] == ["object_2"]
    assert result.metadata["hits"][0]["distance_m"] == pytest.approx(3.26, abs=0.01)

    nothing = module.near(node_id="object_1", radius=1.0)
    assert nothing.metadata["hits"] == []

    by_xy = module.near(xy=[4.0, 0.5], radius=0.5)
    assert [h["id"] for h in by_xy.metadata["hits"]] == ["object_2"]

    bad = module.near(node_id="object_1", xy=[0.0, 0.0])
    assert not bad.success
    assert bad.error_code == "INVALID_INPUT"
    neither = module.near()
    assert not neither.success


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


def test_clearance_at_states_and_distance(
    tmp_path: Path, make_container: Callable[..., SceneMemorySkillContainer]
) -> None:
    module = make_container(sightings_db=str(tmp_path / "scene.db"))
    no_map = module.clearance_at(1.0, 1.0)
    assert not no_map.success
    assert no_map.error_code == "INVALID_STATE"

    module._on_costmap(_walled_grid())
    free = module.clearance_at(1.0, 1.0)
    assert free.success
    assert free.metadata["state"] == "free"
    # Nearest obstacle from (1.0, 1.0) is a wall at 0.1 m -> ~0.9 m away.
    assert free.metadata["clearance_m"] == pytest.approx(0.9, abs=0.08)

    assert module.clearance_at(2.0, 2.0).metadata["state"] == "occupied"
    assert module.clearance_at(2.0, 2.0).metadata["clearance_m"] == 0.0
    assert module.clearance_at(3.1, 2.0).metadata["state"] == "unknown"

    outside = module.clearance_at(9.0, 9.0)
    assert not outside.success
    assert outside.error_code == "INVALID_INPUT"


def test_nearest_free_escapes_the_block(
    tmp_path: Path, make_container: Callable[..., SceneMemorySkillContainer]
) -> None:
    module = make_container(sightings_db=str(tmp_path / "scene.db"))
    module._on_costmap(_walled_grid())
    result = module.nearest_free(2.0, 2.0, min_clearance=0.3)
    assert result.success
    assert result.metadata["found"] is True
    px, py = result.metadata["point"]
    assert result.metadata["clearance_m"] >= 0.3
    assert module.clearance_at(px, py).metadata["state"] == "free"
    # Just outside the 0.25 m-wide block plus the clearance band.
    assert result.metadata["distance_m"] < 0.7

    impossible = module.nearest_free(2.0, 2.0, min_clearance=5.0)
    assert impossible.success
    assert impossible.metadata["found"] is False


def test_raycast_outcomes(
    tmp_path: Path, make_container: Callable[..., SceneMemorySkillContainer]
) -> None:
    module = make_container(sightings_db=str(tmp_path / "scene.db"))
    module._on_costmap(_walled_grid())
    hit = module.raycast(1.0, 2.0, angle_deg=0.0)
    assert hit.success
    assert hit.metadata["outcome"] == "obstacle"
    assert hit.metadata["distance_m"] == pytest.approx(0.9, abs=0.06)

    wall = module.raycast(1.0, 2.0, angle_deg=180.0)
    assert wall.metadata["outcome"] == "obstacle"
    assert wall.metadata["distance_m"] == pytest.approx(0.9, abs=0.06)

    into_unknown = module.raycast(1.0, 1.5, angle_deg=0.0)
    assert into_unknown.metadata["outcome"] == "unknown"
    assert into_unknown.metadata["distance_m"] == pytest.approx(2.0, abs=0.06)

    clear = module.raycast(1.0, 1.0, angle_deg=90.0, max_range_m=1.0)
    assert clear.metadata["outcome"] == "max_range"
    assert clear.metadata["distance_m"] == 1.0


def test_free_space_near_ranks_and_spaces(
    tmp_path: Path, make_container: Callable[..., SceneMemorySkillContainer]
) -> None:
    module = make_container(sightings_db=str(tmp_path / "scene.db"))
    module._on_costmap(_walled_grid())
    result = module.free_space_near(2.0, 2.0, radius=1.0, min_clearance=0.3)
    assert result.success
    points = result.metadata["points"]
    assert points
    assert all(p["clearance_m"] >= 0.3 for p in points)
    assert all(p["distance_m"] <= 1.0 for p in points)
    clearances = [p["clearance_m"] for p in points]
    assert clearances == sorted(clearances, reverse=True)
    for i, a in enumerate(points):
        for b in points[i + 1 :]:
            assert np.hypot(a["x"] - b["x"], a["y"] - b["y"]) >= 0.4

    nothing = module.free_space_near(2.0, 2.0, radius=0.1, min_clearance=0.3)
    assert nothing.success
    assert nothing.metadata["points"] == []


def test_nodes_in_and_expand(
    seeded_db: Path, make_container: Callable[..., SceneMemorySkillContainer]
) -> None:
    module = make_container(sightings_db=str(seeded_db))
    listing = module.nodes_in(BUILDING_ID)
    assert listing.success
    assert listing.metadata["count"] == 2
    assert [c["id"] for c in listing.metadata["children"]] == ["object_1", "object_2"]

    same = module.expand(BUILDING_ID)
    assert same.metadata["children"] == listing.metadata["children"]

    unknown = module.expand("room_99")
    assert not unknown.success
    assert unknown.error_code == "INVALID_INPUT"


def _two_room_grid() -> OccupancyGrid:
    cells = np.full((84, 166), 100, dtype=np.int16)
    cells[2:-2, 2:-2] = 0
    cells[:, 82:84] = 100
    cells[34:50, 82:84] = 0
    return OccupancyGrid(grid=cells.astype(np.int8), resolution=0.05, ts=777.0)


def test_derive_rooms_and_restart_survival(
    tmp_path: Path, make_container: Callable[..., SceneMemorySkillContainer]
) -> None:
    db = tmp_path / "scene.db"
    module = make_container(sightings_db=str(db))
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
    assert derived.metadata["region_ids"] == ["room_1", "room_2"]

    # Re-deriving the unchanged map keeps the node ids stable.
    again = module.derive_rooms()
    assert again.success
    assert again.metadata["region_ids"] == ["room_1", "room_2"]
    assert "unchanged" in again.message

    # A fresh container answers from the persisted graph.
    fresh = make_container(sightings_db=str(db))
    scene = fresh.get_scene()
    assert scene.success
    assert [r["id"] for r in scene.metadata["regions"]] == ["room_1", "room_2"]
    assert scene.metadata["n_doorways"] == 1
    listing = fresh.nodes_in(BUILDING_ID)
    assert listing.metadata["count"] == 2


def test_scan_for_objects_requires_camera_config(
    trail_db: Path, tmp_path: Path, make_container: Callable[..., SceneMemorySkillContainer]
) -> None:
    module = make_container(trail_db=str(trail_db), sightings_db=str(tmp_path / "s.db"))
    result = module.scan_for_objects(["chair"])
    assert not result.success
    assert result.error_code == "NOT_CONFIGURED"


def test_scan_for_objects_rejects_empty_prompt(
    tmp_path: Path, make_container: Callable[..., SceneMemorySkillContainer]
) -> None:
    module = make_container(sightings_db=str(tmp_path / "s.db"))
    result = module.scan_for_objects(["  "])
    assert not result.success
    assert result.error_code == "INVALID_INPUT"
