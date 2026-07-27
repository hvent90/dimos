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

"""Free-space skills as an adapter over the occupancy-map reads.

The geometry itself is covered in dimos/mapping/occupancy/test_clearance.py and
test_raycast.py. What is left to check here is the agent-facing half: no map
yet, off-map targets, rejected arguments, degrees becoming radians, and
metadata the model (and json.dumps) can actually consume.
"""

from collections.abc import Callable, Iterator
from typing import Any

import numpy as np
import pytest

from dimos.agents.skills.free_space import FreeSpaceSkillContainer
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid


@pytest.fixture()
def make_container() -> Iterator[Callable[..., FreeSpaceSkillContainer]]:
    started: list[FreeSpaceSkillContainer] = []

    def make(**kwargs: Any) -> FreeSpaceSkillContainer:
        module = FreeSpaceSkillContainer(**kwargs)
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


def test_every_skill_fails_before_a_map_arrives(
    make_container: Callable[..., FreeSpaceSkillContainer],
) -> None:
    module = make_container()
    results = [
        module.clearance_at(1.0, 1.0),
        module.nearest_free(1.0, 1.0),
        module.raycast(1.0, 1.0, angle_deg=0.0),
        module.free_space_near(1.0, 1.0),
    ]
    for result in results:
        assert not result.success
        assert result.error_code == "INVALID_STATE"


def test_off_map_targets_are_rejected_with_the_bounds(
    make_container: Callable[..., FreeSpaceSkillContainer],
) -> None:
    module = make_container()
    module._on_costmap(_walled_grid())

    for result in (module.clearance_at(9.0, 9.0), module.raycast(9.0, 9.0, angle_deg=0.0)):
        assert not result.success
        assert result.error_code == "INVALID_INPUT"
        assert "mapped area x [0.00, 4.00], y [0.00, 4.00]" in result.message


@pytest.mark.parametrize(
    "call",
    [
        lambda m: m.nearest_free(1.0, 1.0, min_clearance=0.0),
        lambda m: m.raycast(1.0, 1.0, angle_deg=0.0, max_range_m=0.0),
        lambda m: m.free_space_near(1.0, 1.0, radius=0.0),
        lambda m: m.free_space_near(1.0, 1.0, min_clearance=-1.0),
        lambda m: m.free_space_near(1.0, 1.0, max_results=0),
        lambda m: m.free_space_near(1.0, 1.0, max_results=51),
    ],
)
def test_bad_arguments_are_rejected_before_the_map_is_read(
    make_container: Callable[..., FreeSpaceSkillContainer],
    call: Callable[[FreeSpaceSkillContainer], Any],
) -> None:
    """Argument checks answer even with no map, and never raise out of the skill."""
    result = call(make_container())
    assert not result.success
    assert result.error_code == "INVALID_INPUT"


def test_clearance_at_reports_state_and_metric_clearance(
    make_container: Callable[..., FreeSpaceSkillContainer],
) -> None:
    module = make_container()
    module._on_costmap(_walled_grid())

    free = module.clearance_at(1.0, 1.0)
    assert free.success
    assert free.metadata["state"] == "free"
    assert free.metadata["clearance_m"] == pytest.approx(0.9, abs=0.08)

    assert module.clearance_at(2.0, 2.0).metadata["state"] == "occupied"
    assert module.clearance_at(3.1, 2.0).metadata["state"] == "unknown"


def test_raycast_converts_degrees_to_radians(
    make_container: Callable[..., FreeSpaceSkillContainer],
) -> None:
    module = make_container()
    module._on_costmap(_walled_grid())

    up = module.raycast(1.0, 1.0, angle_deg=90.0, max_range_m=1.0)
    assert up.success
    assert up.metadata["outcome"] == "max_range"
    # 90 degrees is +y: the endpoint moved in y and not in x.
    assert up.metadata["end"] == [pytest.approx(1.0), pytest.approx(2.0)]

    right = module.raycast(1.0, 2.0, angle_deg=0.0)
    assert right.metadata["outcome"] == "obstacle"
    assert right.metadata["distance_m"] == pytest.approx(0.9, abs=0.06)


def test_nearest_free_reports_absence_as_success(
    make_container: Callable[..., FreeSpaceSkillContainer],
) -> None:
    """'Nowhere qualifies' is an answer, not an error — the agent can act on it."""
    module = make_container()
    module._on_costmap(_walled_grid())

    found = module.nearest_free(2.0, 2.0, min_clearance=0.3)
    assert found.success
    assert found.metadata["found"] is True
    assert len(found.metadata["point"]) == 2

    missing = module.nearest_free(2.0, 2.0, min_clearance=5.0)
    assert missing.success
    assert missing.metadata["found"] is False


def test_free_space_near_metadata_shape(
    make_container: Callable[..., FreeSpaceSkillContainer],
) -> None:
    module = make_container()
    module._on_costmap(_walled_grid())

    result = module.free_space_near(2.0, 2.0, radius=1.0, min_clearance=0.3, max_results=3)
    assert result.success
    points = result.metadata["points"]
    assert 0 < len(points) <= 3
    assert all(p.keys() == {"x", "y", "clearance_m", "distance_m"} for p in points)

    empty = module.free_space_near(2.0, 2.0, radius=0.1, min_clearance=0.3)
    assert empty.success
    assert empty.metadata["points"] == []


def test_unbounded_clearance_is_reported_as_null_not_infinity(
    make_container: Callable[..., FreeSpaceSkillContainer],
) -> None:
    """An obstacle-free map measures inf, which json.dumps would emit as `Infinity`."""
    module = make_container()
    module._on_costmap(
        OccupancyGrid(grid=np.zeros((40, 40), dtype=np.int8), resolution=0.05, ts=900.0)
    )

    assert module.clearance_at(1.0, 1.0).metadata["clearance_m"] is None
    assert module.nearest_free(1.0, 1.0).metadata["clearance_m"] is None
    assert module.free_space_near(1.0, 1.0).metadata["points"][0]["clearance_m"] is None
