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

"""Room curation skills: view_map, rename/boundary/merge/split, persistence.

The invariants under test: agent edits survive automatic derivation (gating
+ identity matching), room ids stay stable across map growth, and view_map
pairs the rendered image with exact geometry.
"""

from collections.abc import Callable, Iterator
from pathlib import Path
from typing import Any

import numpy as np
import pytest

from dimos.agents.skills.scene_memory import SceneMemorySkillContainer
from dimos.agents.skills.scene_memory_rooms import MapViewResult
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.perception.scene_graph import SceneGraph, Sighting


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


class _CapturingOut:
    """Stand-in for a wired ``Out`` stream that records what was published."""

    def __init__(self, sink: list[Any]) -> None:
        self.transport = True
        self._sink = sink

    def publish(self, msg: Any) -> None:
        self._sink.append(msg)


def _two_room_grid(ts: float = 777.0) -> OccupancyGrid:
    """8.3 x 4.2 m: two rooms split by a wall at x ~4.1 with a doorway."""
    cells = np.full((84, 166), 100, dtype=np.int16)
    cells[2:-2, 2:-2] = 0
    cells[:, 82:84] = 100
    cells[34:50, 82:84] = 0
    return OccupancyGrid(grid=cells.astype(np.int8), resolution=0.05, ts=ts)


def _three_room_grid(ts: float = 900.0) -> OccupancyGrid:
    """The two-room map grown eastward by a third room (same left geometry)."""
    cells = np.full((84, 250), 100, dtype=np.int16)
    cells[2:-2, 2:-2] = 0
    cells[:, 82:84] = 100
    cells[34:50, 82:84] = 0
    cells[:, 164:166] = 100
    cells[34:50, 164:166] = 0
    return OccupancyGrid(grid=cells.astype(np.int8), resolution=0.05, ts=ts)


def _seed_objects(db: Path) -> None:
    """One couch in the west room, one fridge in the east room."""
    with SceneGraph(str(db)) as graph:
        graph.fold_scan(
            [
                Sighting(name="couch", ts=100.0, position=(1.0, 1.0, 0.0)),
                Sighting(name="fridge", ts=101.0, position=(6.0, 1.0, 0.0)),
            ],
            t0=99.0,
            t1=102.0,
            vocabulary=["couch", "fridge"],
            source="test",
            frames=2,
        )


def _room_at(db: Path, x: float, y: float) -> str:
    with SceneGraph(str(db)) as graph:
        return graph.assign_regions(np.asarray([[x, y]], dtype=np.float64))[0]


def _parent_of(db: Path, node_id: str) -> str | None:
    with SceneGraph(str(db)) as graph:
        return graph.parent_id(node_id)


def test_view_map_requires_grid(
    tmp_path: Path, make_container: Callable[..., SceneMemorySkillContainer]
) -> None:
    module = make_container(sightings_db=str(tmp_path / "scene.db"))
    result = module.view_map()
    assert not result.success
    assert result.error_code == "INVALID_STATE"


def test_view_map_returns_image_and_exact_geometry(
    tmp_path: Path, make_container: Callable[..., SceneMemorySkillContainer]
) -> None:
    db = tmp_path / "scene.db"
    _seed_objects(db)
    module = make_container(sightings_db=str(db))
    module._on_costmap(_two_room_grid())

    view = module.view_map()
    assert isinstance(view, MapViewResult)
    assert view.success
    assert view.image is not None and view.image.data.dtype == np.uint8
    blocks = view.agent_encode()
    assert [b["type"] for b in blocks] == ["text", "image_url"]
    assert blocks[1]["image_url"]["url"].startswith("data:image/jpeg;base64,")

    rooms = view.metadata["rooms"]
    assert len(rooms) == 2
    for room in rooms:
        assert len(room["polygon"]) >= 3
        assert room["origin"] == "derived"
    names = {o["name"] for o in view.metadata["objects"]}
    assert names == {"couch", "fridge"}

    zoomed = module.view_map(bounds=[0.0, 0.0, 2.0, 2.0])
    assert zoomed.success
    assert zoomed.metadata["view_bounds"] == [0.0, 0.0, 2.0, 2.0]
    assert zoomed.metadata["grid_step_m"] == 0.5

    bad = module.view_map(bounds=[3.0, 3.0, 1.0, 1.0])
    assert not bad.success
    assert bad.error_code == "INVALID_INPUT"
    outside = module.view_map(bounds=[50.0, 50.0, 60.0, 60.0])
    assert not outside.success
    assert outside.error_code == "INVALID_INPUT"


def test_rename_survives_map_growth(
    tmp_path: Path, make_container: Callable[..., SceneMemorySkillContainer]
) -> None:
    db = tmp_path / "scene.db"
    module = make_container(sightings_db=str(db))
    module._on_costmap(_two_room_grid())
    assert module.derive_rooms().success

    west = _room_at(db, 1.0, 1.0)
    renamed = module.rename_room(west, "kitchen")
    assert renamed.success

    missing = module.rename_room("room_99", "void")
    assert not missing.success
    assert missing.error_code == "INVALID_INPUT"

    # The map grows east; re-derivation must keep the west room's id + name.
    module._on_costmap(_three_room_grid())
    assert module.derive_rooms().success
    with SceneGraph(str(db)) as graph:
        regions = graph.regions()
        assert len(regions) == 3
        kept = graph.node(west)
        assert kept is not None and not kept.retired
        assert kept.name == "kitchen"
        assert _room_at(db, 1.0, 1.0) == west


def test_rename_reaches_the_viewer_marker(
    tmp_path: Path, make_container: Callable[..., SceneMemorySkillContainer]
) -> None:
    """A rename must show up in the viewer label, not just in the graph."""
    db = tmp_path / "scene.db"
    module = make_container(sightings_db=str(db))
    module._on_costmap(_two_room_grid())
    assert module.derive_rooms().success

    published: list[Any] = []
    module.scene_graph_markers = _CapturingOut(published)  # type: ignore[assignment]

    west = _room_at(db, 1.0, 1.0)
    assert module.rename_room(west, "kitchen").success

    assert published, "rename_room must republish the graph for the viewer"
    labels = {m.entity_id: m.label for m in published[-1].markers}
    assert labels[west] == "kitchen"


def test_set_room_boundary_moves_objects_and_gates_derivation(
    tmp_path: Path, make_container: Callable[..., SceneMemorySkillContainer]
) -> None:
    db = tmp_path / "scene.db"
    _seed_objects(db)
    module = make_container(sightings_db=str(db))
    module._on_costmap(_two_room_grid())
    assert module.derive_rooms().success

    # Stretch the lowest-index room over both objects: overlap ties break to
    # the lowest region index, so the other room's object must move into it.
    with SceneGraph(str(db)) as graph:
        target = graph.regions()[0].id
        stray = next(n.id for n in graph.nodes(layer="object") if graph.parent_id(n.id) != target)
    result = module.set_room_boundary(target, [0.2, 0.2, 7.0, 0.2, 7.0, 4.0, 0.2, 4.0])
    assert result.success
    assert result.metadata["objects_moved"] == 1
    assert _parent_of(db, stray) == target

    gated = module.derive_rooms()
    assert gated.success
    assert gated.metadata.get("kept_agent_edits") == [target]
    forced = module.derive_rooms(force=True)
    assert forced.success
    assert "kept_agent_edits" not in forced.metadata

    bad = module.set_room_boundary(target, [0.0, 0.0, 1.0])
    assert not bad.success
    assert bad.error_code == "INVALID_INPUT"


def test_merge_rooms_combines_and_split_room_divides(
    tmp_path: Path, make_container: Callable[..., SceneMemorySkillContainer]
) -> None:
    db = tmp_path / "scene.db"
    _seed_objects(db)
    module = make_container(sightings_db=str(db))
    module._on_costmap(_two_room_grid())
    assert module.derive_rooms().success

    merged = module.merge_rooms(["room_1", "room_2"], name="open plan")
    assert merged.success
    new_id = merged.metadata["merged_into"]
    assert merged.metadata["objects_moved"] >= 1
    assert _parent_of(db, "object_1") == new_id
    assert _parent_of(db, "object_2") == new_id
    with SceneGraph(str(db)) as graph:
        assert [r.id for r in graph.regions()] == [new_id]
        merged_node = graph.node(new_id)
        assert merged_node is not None and merged_node.name == "open plan"

    too_few = module.merge_rooms([new_id])
    assert not too_few.success
    assert too_few.error_code == "INVALID_INPUT"

    split = module.split_room(new_id, [4.1, 0.0, 4.1, 4.2], names=["west", "east"])
    assert split.success
    a_id, b_id = split.metadata["new_room_ids"]
    with SceneGraph(str(db)) as graph:
        assert {r.id for r in graph.regions()} == {a_id, b_id}
        by_id = {n.id: n for n in graph.regions()}
        assert {by_id[a_id].name, by_id[b_id].name} == {"west", "east"}
        # The halves are adjacent (a doorway edge joins them).
        neighbors = {node.id for node, _ in graph.adjacent_rooms(a_id)}
        assert b_id in neighbors
    assert _parent_of(db, "object_1") in (a_id, b_id)
    assert _parent_of(db, "object_1") != _parent_of(db, "object_2")

    misses = module.split_room(a_id, [20.0, 0.0, 20.0, 1.0])
    assert not misses.success
    assert misses.error_code == "INVALID_INPUT"


def test_merge_rejects_non_adjacent_rooms(
    tmp_path: Path, make_container: Callable[..., SceneMemorySkillContainer]
) -> None:
    # Two free areas separated by a solid 10-cell wall: no doorway at all.
    cells = np.full((84, 166), 100, dtype=np.int16)
    cells[2:-2, 2:76] = 0
    cells[2:-2, 86:-2] = 0
    grid = OccupancyGrid(grid=cells.astype(np.int8), resolution=0.05, ts=5.0)
    module = make_container(sightings_db=str(tmp_path / "scene.db"))
    module._on_costmap(grid)
    assert module.derive_rooms().success
    result = module.merge_rooms(["room_1", "room_2"])
    assert not result.success
    assert result.error_code == "INVALID_INPUT"
    assert "not contiguous" in result.message


def test_agent_edits_survive_restart(
    tmp_path: Path, make_container: Callable[..., SceneMemorySkillContainer]
) -> None:
    db = tmp_path / "scene.db"
    module = make_container(sightings_db=str(db))
    module._on_costmap(_two_room_grid())
    assert module.derive_rooms().success
    merged = module.merge_rooms(["room_1", "room_2"], name="studio")
    assert merged.success
    new_id = merged.metadata["merged_into"]

    fresh = make_container(sightings_db=str(db))
    fresh._on_costmap(_two_room_grid(ts=778.0))
    scene = fresh.get_scene()
    assert [r["id"] for r in scene.metadata["regions"]] == [new_id]
    gated = fresh.derive_rooms()
    assert gated.success
    assert gated.metadata.get("kept_agent_edits") == [new_id]
