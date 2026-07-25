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

"""Core scene-memory skills: occupancy-map clearance, free space, rays."""

from collections.abc import Callable, Iterator
from typing import Any

import numpy as np
import pytest

from dimos.agents.skills.scene_memory import SceneMemorySkillContainer
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid


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
    make_container: Callable[..., SceneMemorySkillContainer],
) -> None:
    module = make_container()
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
    make_container: Callable[..., SceneMemorySkillContainer],
) -> None:
    module = make_container()
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


def test_raycast_outcomes(make_container: Callable[..., SceneMemorySkillContainer]) -> None:
    module = make_container()
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
    make_container: Callable[..., SceneMemorySkillContainer],
) -> None:
    module = make_container()
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
