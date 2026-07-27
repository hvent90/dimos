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

"""Scene-graph skills: derive rooms from the map and persist them."""

from collections.abc import Callable, Iterator
from pathlib import Path
from typing import Any

import numpy as np
import pytest

from dimos.agents.skills.scene_graph import SceneGraphSkillContainer
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.perception.scene_graph import SceneGraph


@pytest.fixture()
def make_container() -> Iterator[Callable[..., SceneGraphSkillContainer]]:
    started: list[SceneGraphSkillContainer] = []

    def make(**kwargs: Any) -> SceneGraphSkillContainer:
        module = SceneGraphSkillContainer(**kwargs)
        module.start()
        started.append(module)
        return module

    yield make
    for module in started:
        module.stop()


def _two_room_grid() -> OccupancyGrid:
    cells = np.full((84, 166), 100, dtype=np.int16)
    cells[2:-2, 2:-2] = 0
    cells[:, 82:84] = 100
    cells[34:50, 82:84] = 0
    return OccupancyGrid(grid=cells.astype(np.int8), resolution=0.05, ts=777.0)


def test_derive_rooms_and_restart_survival(
    tmp_path: Path, make_container: Callable[..., SceneGraphSkillContainer]
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

    # The derivation persists: a fresh reader sees both rooms and the doorway.
    with SceneGraph(db) as graph:
        assert [r.id for r in graph.regions()] == ["room_1", "room_2"]
        assert len(graph.edges(kind="adjacent")) == 1


def test_derive_rooms_honors_door_half_width(
    tmp_path: Path, make_container: Callable[..., SceneGraphSkillContainer]
) -> None:
    """A tiny half-width lets seeds bridge the 0.8 m doorway: rooms merge."""
    module = make_container(sightings_db=str(tmp_path / "s.db"), door_half_width_m=0.05)
    module._on_costmap(_two_room_grid())
    derived = module.derive_rooms()
    assert derived.success
    assert derived.metadata["n_rooms"] == 1
