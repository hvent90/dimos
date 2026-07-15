# Copyright 2025-2026 Dimensional Inc.
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

import numpy as np
import pytest

from dimos.mapping.occupancy.gradient import gradient
from dimos.mapping.occupancy.path_resampling import simple_resample_path, smooth_resample_path
from dimos.mapping.occupancy.visualize_path import visualize_path
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.sensor_msgs.Image import Image
from dimos.navigation.replanning_a_star.min_cost_astar import min_cost_astar
from dimos.utils.data import get_data


@pytest.fixture
def costmap() -> OccupancyGrid:
    return gradient(OccupancyGrid(np.load(get_data("occupancy_simple.npy"))), max_distance=1.5)


@pytest.mark.parametrize("method", ["simple", "smooth"])
def test_resample_path(costmap: OccupancyGrid, method: str) -> None:
    start = Vector3(4.0, 2.0, 0)
    goal_pose = Pose(6.15, 10.0, 0, 0, 0, 0, 1)
    expected = Image.from_file(get_data(f"resample_path_{method}.png"))
    path = min_cost_astar(costmap, goal_pose.position, start, use_cpp=False)
    assert path is not None

    match method:
        case "simple":
            resampled = simple_resample_path(path, goal_pose, 0.1)
        case "smooth":
            resampled = smooth_resample_path(path, goal_pose, 0.1)
        case _:
            raise ValueError(f"Unknown resampling method: {method}")

    actual = visualize_path(costmap, resampled, 0.2, 0.4)
    np.testing.assert_array_equal(actual.data, expected.data)


def test_smooth_resample_preserves_stair_z() -> None:
    """DanLocalPlanner uses this; z=0 flattening killed MLS stair paths in rerun."""
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
    from dimos.msgs.nav_msgs.Path import Path

    poses = [
        PoseStamped(frame_id="world", position=[0.0, float(i), 0.25 * i], orientation=[0, 0, 0, 1])
        for i in range(11)
    ]
    path = Path(frame_id="world", poses=poses)
    goal = Pose(poses[-1].position, poses[-1].orientation)
    out = smooth_resample_path(path, goal, spacing=0.2, smoothing_window=5)
    zs = [p.z for p in out.poses]
    assert zs[0] == pytest.approx(0.0, abs=1e-5)
    assert zs[-1] == pytest.approx(2.5, abs=1e-5)
    assert max(zs) > 1.0
    assert min(zs) >= -0.05
