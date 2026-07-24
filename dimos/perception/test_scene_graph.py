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

"""SceneGraph store: fold identity, containment, migration, persistence."""

from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest

from dimos.mapping.occupancy.room_segmentation import segment_rooms
from dimos.mapping.occupancy.room_store import RoomStore
from dimos.memory2.store.sqlite import SqliteStore
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.perception.scene_graph import (
    AGENT_ID,
    BUILDING_ID,
    FoldResult,
    SceneGraph,
    Sighting,
)

T0 = 1_000_000.0

# Two 4x4 m rooms joined by a doorway; segmentation deterministically labels
# the left room 1 and the right room 2 (asserted in the first-pass tests).
LEFT_XY = (2.0, 2.0)
RIGHT_XY = (6.0, 2.0)


def _two_room_grid(ts: float = 900.0) -> OccupancyGrid:
    cells = np.full((84, 166), 100, dtype=np.int16)
    cells[2:-2, 2:-2] = 0
    cells[:, 82:84] = 100
    cells[34:50, 82:84] = 0
    return OccupancyGrid(grid=cells.astype(np.int8), resolution=0.05, ts=ts)


def _seed_rooms(graph: SceneGraph, db: Path, grid_ts: float = 900.0) -> None:
    """Save a derivation record (evidence) and write it into the graph."""
    with RoomStore(db) as store:
        store.save(segment_rooms(_two_room_grid(grid_ts)), source="test")
        room_set = store.latest()
    assert room_set is not None
    graph.apply_rooms(room_set)


def _fold_couch_and_tv(graph: SceneGraph) -> FoldResult:
    return graph.fold_scan(
        [
            Sighting(name="couch", ts=T0 + 2.0, position=(*LEFT_XY, 0.1)),
            Sighting(name="couch", ts=T0 + 3.0, position=(2.1, 2.05, 0.1)),
            Sighting(name="tv", ts=T0 + 5.0, position=(*RIGHT_XY, 1.2)),
        ],
        t0=T0,
        t1=T0 + 10.0,
        vocabulary=["couch", "tv"],
        source="test",
        frames=10,
    )


def test_fold_creates_nodes_with_lineage(tmp_path: Path) -> None:
    db = tmp_path / "scene.db"
    with SceneGraph(db) as graph:
        _seed_rooms(graph, db)
        result = _fold_couch_and_tv(graph)
        assert result.appended_sightings == 3
        assert result.created_node_ids == ("object_1", "object_2")
        assert result.updated_node_ids == ()

        couch = graph.node("object_1")
        assert couch is not None
        assert couch.name == "couch"
        assert couch.sightings == 2
        assert couch.position == (2.1, 2.05, 0.1)  # latest sighting wins
        assert couch.first_seen_ts == T0 + 2.0
        assert couch.last_seen_ts == T0 + 3.0

        assert graph.parent_id("object_1") == "room_1"
        assert graph.parent_id("object_2") == "room_2"
        assert [n.id for n in graph.ancestors("object_1")] == ["room_1", BUILDING_ID]

        rows = graph.sightings("couch")
        assert [s.node_id for s in rows] == ["object_1", "object_1"]
        assert [s.room_id for s in rows] == ["room_1", "room_1"]


def test_identity_stable_across_rescans(tmp_path: Path) -> None:
    db = tmp_path / "scene.db"
    with SceneGraph(db) as graph:
        first = _fold_couch_and_tv(graph)
        assert first.created_node_ids == ("object_1", "object_2")
        # Overlapping vocabulary, one re-sighting near the couch + one new chair.
        second = graph.fold_scan(
            [
                Sighting(name="couch", ts=T0 + 12.0, position=(2.2, 2.1, 0.1)),
                Sighting(name="chair", ts=T0 + 13.0, position=(5.0, 1.0, 0.1)),
            ],
            t0=T0 + 11.0,
            t1=T0 + 14.0,
            vocabulary=["couch", "chair"],
            source="test",
            frames=5,
        )
        assert second.updated_node_ids == ("object_1",)
        assert second.created_node_ids == ("object_3",)
        couch = graph.node("object_1")
        assert couch is not None
        assert couch.sightings == 3
        assert couch.last_seen_ts == T0 + 12.0
        assert len(graph.nodes(layer="object")) == 3


def test_refold_same_scan_is_deduped(tmp_path: Path) -> None:
    db = tmp_path / "scene.db"
    with SceneGraph(db) as graph:
        _fold_couch_and_tv(graph)
        again = _fold_couch_and_tv(graph)
        assert again.appended_sightings == 0
        assert again.created_node_ids == ()
        assert again.updated_node_ids == ()
        couch = graph.node("object_1")
        assert couch is not None
        assert couch.sightings == 2
        assert len(graph.sightings()) == 3
        # The pass itself is still recorded — coverage counts even when empty.
        assert len(graph.scan_events()) == 2


def test_refold_with_jitter_and_new_track_ids_is_deduped(tmp_path: Path) -> None:
    # Track ids are per-detector-session counters and re-detected positions
    # jitter by centimetres, so a re-scan never reproduces exact rows.
    db = tmp_path / "scene.db"
    with SceneGraph(db) as graph:
        _fold_couch_and_tv(graph)
        again = graph.fold_scan(
            [
                Sighting(name="couch", ts=T0 + 2.0, position=(2.02, 1.97, 0.11), object_id="58"),
                Sighting(name="couch", ts=T0 + 3.0, position=(2.13, 2.08, 0.09), object_id="58"),
                Sighting(name="tv", ts=T0 + 5.0, position=(6.03, 2.01, 1.18), object_id="59"),
            ],
            t0=T0,
            t1=T0 + 10.0,
            vocabulary=["couch", "tv"],
            source="test",
            frames=10,
        )
        assert again.appended_sightings == 0
        assert again.created_node_ids == ()
        assert len(graph.sightings()) == 3


def test_same_name_far_apart_is_two_nodes(tmp_path: Path) -> None:
    db = tmp_path / "scene.db"
    with SceneGraph(db) as graph:
        result = graph.fold_scan(
            [
                Sighting(name="person", ts=T0 + 1.0, position=(-3.1, 0.1, 0.3)),
                Sighting(name="person", ts=T0 + 2.0, position=(-1.9, 6.6, 0.3)),
            ],
            t0=T0,
            t1=T0 + 5.0,
            vocabulary=["person"],
            source="test",
            frames=4,
        )
        assert result.created_node_ids == ("object_1", "object_2")
        assert [n.name for n in graph.nodes(layer="object")] == ["person", "person"]


def test_out_of_order_fold_keeps_latest_position(tmp_path: Path) -> None:
    db = tmp_path / "scene.db"
    with SceneGraph(db) as graph:
        graph.fold_scan(
            [Sighting(name="couch", ts=T0 + 20.0, position=(2.0, 2.0, 0.1))],
            t0=T0 + 15.0,
            t1=T0 + 25.0,
            vocabulary=["couch"],
            source="test",
            frames=3,
        )
        # An earlier window folded later must not regress the position cache.
        graph.fold_scan(
            [Sighting(name="couch", ts=T0 + 2.0, position=(2.3, 2.1, 0.1))],
            t0=T0,
            t1=T0 + 10.0,
            vocabulary=["couch"],
            source="test",
            frames=3,
        )
        couch = graph.node("object_1")
        assert couch is not None
        assert couch.position == (2.0, 2.0, 0.1)
        assert couch.first_seen_ts == T0 + 2.0
        assert couch.last_seen_ts == T0 + 20.0
        assert couch.sightings == 2


def test_extent_unions_across_sightings_and_persists(tmp_path: Path) -> None:
    db = tmp_path / "scene.db"
    with SceneGraph(db) as graph:
        graph.fold_scan(
            [
                Sighting(
                    name="couch",
                    ts=T0 + 1.0,
                    position=(2.0, 2.0, 0.3),
                    extent=(1.5, 1.8, 0.0, 2.5, 2.2, 0.6),
                ),
                Sighting(
                    name="couch",
                    ts=T0 + 2.0,
                    position=(2.1, 2.05, 0.3),
                    extent=(1.7, 1.9, 0.1, 2.8, 2.3, 0.5),
                ),
                # Extent-less producers must not erase accumulated bounds.
                Sighting(name="couch", ts=T0 + 3.0, position=(2.1, 2.0, 0.3)),
            ],
            t0=T0,
            t1=T0 + 5.0,
            vocabulary=["couch"],
            source="test",
            frames=3,
        )
    with SceneGraph(db) as graph:
        couch = graph.node("object_1")
        assert couch is not None
        footprint = couch.polygon()
        assert footprint[:, 0].min() == 1.5
        assert footprint[:, 1].min() == 1.8
        assert footprint[:, 0].max() == 2.8
        assert footprint[:, 1].max() == 2.3
        assert couch.metadata["z_range"] == [0.0, 0.6]
        rows = graph.sightings("couch")
        assert rows[0].extent == (1.5, 1.8, 0.0, 2.5, 2.2, 0.6)
        assert rows[2].extent is None


def test_same_frame_same_name_detections_both_kept(tmp_path: Path) -> None:
    # Two same-name detections in one frame share ts and a blank track id;
    # both must survive dedupe and become two far-apart nodes.
    db = tmp_path / "scene.db"
    with SceneGraph(db) as graph:
        result = graph.fold_scan(
            [
                Sighting(name="person", ts=T0 + 1.0, position=(-3.1, 0.1, 0.3)),
                Sighting(name="person", ts=T0 + 1.0, position=(4.0, 6.6, 0.3)),
            ],
            t0=T0,
            t1=T0 + 5.0,
            vocabulary=["person"],
            source="test",
            frames=1,
        )
        assert result.appended_sightings == 2
        assert result.created_node_ids == ("object_1", "object_2")


def test_fold_without_rooms_parents_to_building(tmp_path: Path) -> None:
    db = tmp_path / "scene.db"
    with SceneGraph(db) as graph:
        _fold_couch_and_tv(graph)
        assert graph.parent_id("object_1") == BUILDING_ID
        assert [n.id for n in graph.ancestors("object_1")] == [BUILDING_ID]
        assert all(s.room_id == "" for s in graph.sightings())


def test_rederivation_keeps_ids_for_same_place_regions(tmp_path: Path) -> None:
    db = tmp_path / "scene.db"
    with SceneGraph(db) as graph:
        _seed_rooms(graph, db)
        _fold_couch_and_tv(graph)
        assert graph.parent_id("object_1") == "room_1"

        # Same geometry re-derived: identity matching keeps the nodes —
        # no retirement, no id churn, the object's parent edge untouched.
        _seed_rooms(graph, db, grid_ts=901.0)
        assert [n.id for n in graph.nodes(layer="room")] == ["room_1", "room_2"]
        room_1 = graph.node("room_1")
        assert room_1 is not None
        assert not room_1.retired
        assert room_1.metadata["derived_ts"] == 901.0
        assert graph.parent_id("object_1") == "room_1"


def test_rederivation_reparents_when_geometry_changes(tmp_path: Path) -> None:
    db = tmp_path / "scene.db"
    with SceneGraph(db) as graph:
        _seed_rooms(graph, db)
        _fold_couch_and_tv(graph)
        assert graph.parent_id("object_1") == "room_1"

        # The dividing wall opens up: one big room whose centroid falls in
        # neither old polygon, so no identity carries — the old rooms
        # retire and the object reparents to the fresh node.
        open_grid = _two_room_grid(901.0)
        open_grid.grid[2:-2, 82:84] = 0
        with RoomStore(db) as store:
            store.save(segment_rooms(open_grid), source="test")
            room_set = store.latest()
        assert room_set is not None
        graph.apply_rooms(room_set)

        assert [n.id for n in graph.nodes(layer="room")] == ["room_3"]
        room_1 = graph.node("room_1")
        assert room_1 is not None
        assert room_1.retired

        assert graph.parent_id("object_1") == "room_3"
        live_parents = [e for e in graph.edges(kind="contains") if e.child_id == "object_1"]
        assert len(live_parents) == 1
        retired_parents = [
            e
            for e in graph.edges(kind="contains", include_retired=True)
            if e.child_id == "object_1" and e.retired
        ]
        assert [e.parent_id for e in retired_parents] == ["room_1"]
        # Historical sighting rows keep the room that contained them then.
        assert [s.room_id for s in graph.sightings("couch")] == ["room_1", "room_1"]


def test_adjacency_edges_from_doorways(tmp_path: Path) -> None:
    db = tmp_path / "scene.db"
    with SceneGraph(db) as graph:
        _seed_rooms(graph, db)
        neighbors = graph.adjacent_rooms("room_1")
        assert [n.id for n, _ in neighbors] == ["room_2"]
        doorway = neighbors[0][1]
        # The doorway sits in the wall gap at x ~= 4.15, y ~= 2.1.
        assert doorway["width_m"] > 0
        assert abs(doorway["xy"][0] - 4.15) < 0.3
        # Adjacency is symmetric even though the edge is stored once.
        assert [n.id for n, _ in graph.adjacent_rooms("room_2")] == ["room_1"]


def test_restart_survival_across_instances(tmp_path: Path) -> None:
    db = tmp_path / "scene.db"
    with SceneGraph(db) as graph:
        _seed_rooms(graph, db)
        _fold_couch_and_tv(graph)
    with SceneGraph(db) as fresh:
        couch = fresh.node("object_1")
        assert couch is not None
        assert couch.sightings == 2
        assert fresh.parent_id("object_1") == "room_1"
        assert len(fresh.sightings()) == 3
        assert fresh.ever_in_vocabulary("tv")


def test_restart_survival_across_processes(tmp_path: Path) -> None:
    db = tmp_path / "scene.db"
    build = (
        "from dimos.perception.scene_graph import SceneGraph, Sighting\n"
        f"with SceneGraph({str(db)!r}) as graph:\n"
        f"    graph.fold_scan([Sighting(name='couch', ts={T0 + 2.0}, position=(1.0, 2.0, 0.1))],\n"
        f"                    t0={T0}, t1={T0 + 10.0}, vocabulary=['couch'],\n"
        "                    source='test', frames=3)\n"
    )
    subprocess.run([sys.executable, "-c", build], check=True, timeout=120)
    with SceneGraph(db) as graph:
        couch = graph.node("object_1")
        assert couch is not None
        assert couch.name == "couch"
        assert graph.sightings("couch")[0].node_id == "object_1"


def _record_legacy_rows(
    db: Path, rows: list[tuple[str, float, tuple[float, float, float]]], vocabulary: list[str]
) -> None:
    """Write first-pass-format rows: no node_id/room_id tags anywhere."""
    with SqliteStore(path=str(db)) as store:
        sightings = store.stream("sightings", str)
        for name, ts, position in rows:
            sightings.append(
                name,
                ts=ts,
                pose=position,
                tags={"object_id": "", "source": "legacy", "vocabulary": vocabulary},
            )
        store.stream("scan_events", str).append(
            "legacy",
            ts=T0 + 10.0,
            tags={"t0": T0, "vocabulary": vocabulary, "frames": 10, "sightings": len(rows)},
        )


def _seed_legacy_db(db: Path) -> None:
    """A pre-graph DB: bare sighting rows + a stored room derivation."""
    with RoomStore(db) as store:
        store.save(segment_rooms(_two_room_grid()), source="legacy")
    _record_legacy_rows(
        db,
        [
            ("couch", T0 + 2.0, (*LEFT_XY, 0.1)),
            ("couch", T0 + 3.0, (2.1, 2.05, 0.1)),
            ("tv", T0 + 5.0, (*RIGHT_XY, 1.2)),
        ],
        vocabulary=["couch", "tv"],
    )


def test_migration_assigns_ids_and_rooms(tmp_path: Path) -> None:
    db = tmp_path / "legacy.db"
    _seed_legacy_db(db)
    with SceneGraph(db) as graph:
        migrated = graph.ensure_migrated()
        assert migrated == 3
        # Room nodes materialized from the stored derivation record.
        assert [n.id for n in graph.nodes(layer="room")] == ["room_1", "room_2"]
        couch = graph.node("object_1")
        assert couch is not None
        assert couch.name == "couch"
        assert couch.sightings == 2
        assert graph.parent_id("object_1") == "room_1"
        rows = graph.sightings("couch")
        assert [s.node_id for s in rows] == ["object_1", "object_1"]
        assert [s.room_id for s in rows] == ["room_1", "room_1"]
        # Scan events survive the migration untouched.
        events = graph.scan_events()
        assert len(events) == 1
        assert events[0].vocabulary == ("couch", "tv")


def test_migration_is_deterministic_and_idempotent(tmp_path: Path) -> None:
    db_a = tmp_path / "a.db"
    db_b = tmp_path / "b.db"
    _seed_legacy_db(db_a)
    _seed_legacy_db(db_b)
    with SceneGraph(db_a) as graph_a:
        graph_a.ensure_migrated()
        json_a = graph_a.to_json()
    with SceneGraph(db_b) as graph_b:
        graph_b.ensure_migrated()
        assert graph_b.to_json() == json_a
        # Second call is a no-op on an already-migrated DB.
        assert graph_b.ensure_migrated() == 0
        assert graph_b.to_json() == json_a


def test_migration_without_rooms_then_backfill(tmp_path: Path) -> None:
    db = tmp_path / "legacy.db"
    _record_legacy_rows(db, [("couch", T0 + 2.0, (*LEFT_XY, 0.1))], vocabulary=["couch"])
    with SceneGraph(db) as graph:
        assert graph.ensure_migrated() == 1
        assert graph.parent_id("object_1") == BUILDING_ID
        assert graph.sightings("couch")[0].room_id == ""
        # A later derivation re-parents the node and backfills the rows.
        _seed_rooms(graph, db)
        assert graph.parent_id("object_1") == "room_1"
        assert graph.sightings("couch")[0].room_id == "room_1"


def test_agent_node_updates(tmp_path: Path) -> None:
    db = tmp_path / "scene.db"
    with SceneGraph(db) as graph:
        graph.fold_scan(
            [Sighting(name="couch", ts=T0 + 2.0, position=(*LEFT_XY, 0.1))],
            t0=T0,
            t1=T0 + 10.0,
            vocabulary=["couch"],
            source="test",
            frames=3,
            agent_position=(1.0, 1.0, 0.0),
        )
        agent = graph.node(AGENT_ID)
        assert agent is not None
        assert agent.position == (1.0, 1.0, 0.0)
        assert agent.last_seen_ts == T0 + 10.0
        # The agent has no stored containment — it is resolved lazily.
        assert graph.parent_id(AGENT_ID) is None
        graph.fold_scan(
            [],
            t0=T0 + 11.0,
            t1=T0 + 20.0,
            vocabulary=["couch"],
            source="test",
            frames=3,
            agent_position=(3.0, 1.5, 0.0),
        )
        agent = graph.node(AGENT_ID)
        assert agent is not None
        assert agent.position == (3.0, 1.5, 0.0)
        assert agent.last_seen_ts == T0 + 20.0


def test_to_json_snapshot(tmp_path: Path) -> None:
    db = tmp_path / "scene.db"
    with SceneGraph(db) as graph:
        _seed_rooms(graph, db)
        _fold_couch_and_tv(graph)
        snapshot = graph.to_json()
    assert snapshot["metadata"]["counts"]["sightings"] == 3
    by_id = {n["id"]: n for n in snapshot["nodes"]}
    assert by_id["object_1"]["parent"] == "room_1"
    assert by_id["room_1"]["parent"] == BUILDING_ID
    assert by_id["room_1"]["extent"] is not None
    assert by_id["object_1"]["extent"] is None
    kinds = {(e["parent"], e["child"], e["kind"]) for e in snapshot["edges"] if not e["retired"]}
    assert ("room_1", "room_2", "adjacent") in kinds
    assert (BUILDING_ID, "room_1", "contains") in kinds


@pytest.mark.parametrize("bad_index", ["object_1"])
def test_unknown_node_lookups_are_none(tmp_path: Path, bad_index: str) -> None:
    db = tmp_path / "scene.db"
    with SceneGraph(db) as graph:
        assert graph.node(bad_index) is None
        assert graph.parent_id(bad_index) is None
        assert graph.ancestors(bad_index) == []
        assert graph.children(bad_index) == []
        assert graph.sightings() == []
