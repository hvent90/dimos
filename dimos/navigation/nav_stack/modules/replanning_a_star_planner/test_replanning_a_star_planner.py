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

"""No-sim smoke tests for ReplanningAStarPlanner pieces."""

from __future__ import annotations

import numpy as np

from dimos.mapping.occupancy.path_map import make_navigation_map
from dimos.msgs.nav_msgs.OccupancyGrid import CostValues
from dimos.navigation.nav_stack.modules.replanning_a_star_planner.costmap_builder import (
    HeightMapCostmap,
)
from dimos.navigation.nav_stack.modules.replanning_a_star_planner.replanning_a_star_planner import (
    ReplanningAStarPlanner,
    ReplanningAStarPlannerConfig,
    _distance_point_to_polyline,
)
from dimos.navigation.replanning_a_star.min_cost_astar import min_cost_astar


def test_module_class_exists() -> None:
    """Module class is importable and has the expected nav_stack ports."""
    assert hasattr(ReplanningAStarPlanner, "terrain_map_ext")
    assert hasattr(ReplanningAStarPlanner, "terrain_map")
    assert hasattr(ReplanningAStarPlanner, "goal")
    assert hasattr(ReplanningAStarPlanner, "stop_movement")
    assert hasattr(ReplanningAStarPlanner, "way_point")
    assert hasattr(ReplanningAStarPlanner, "goal_path")
    assert hasattr(ReplanningAStarPlanner, "costmap_cloud")


def test_config_defaults() -> None:
    cfg = ReplanningAStarPlannerConfig()
    assert cfg.cell_size > 0
    assert cfg.obstacle_height_threshold > 0
    assert cfg.max_replan_attempts >= 1
    assert cfg.lookahead_distance > 0


def test_costmap_ingest_empty() -> None:
    cm = HeightMapCostmap(cell_size=0.1, obstacle_height_threshold=0.15)
    cm.ingest(np.zeros((0, 3), dtype=np.float32), ground_z=0.0)
    assert cm.observed_count == 0


def test_costmap_ingest_marks_observed_free_and_occupied() -> None:
    cm = HeightMapCostmap(cell_size=0.5, obstacle_height_threshold=0.3)
    # Ground-level points only — should be observed but FREE.
    ground_points = np.array(
        [
            [0.25, 0.25, 0.0],
            [0.75, 0.25, 0.0],
            [1.25, 0.25, 0.0],
        ],
        dtype=np.float32,
    )
    # One tall point — should be OCCUPIED.
    tall_point = np.array([[0.25, 0.75, 0.5]], dtype=np.float32)

    cm.ingest(ground_points, ground_z=0.0)
    cm.ingest(tall_point, ground_z=0.0)

    grid = cm.to_occupancy_grid(center_x=0.5, center_y=0.5, radius=1.0)
    assert grid.width > 0
    assert grid.height > 0

    # The tall_point cell at world ~ (0.25, 0.75) should be OCCUPIED.
    g_tall = grid.world_to_grid((0.25, 0.75))
    assert grid.grid[int(g_tall.y), int(g_tall.x)] == int(CostValues.OCCUPIED)

    # Ground point cells should be FREE.
    g_free = grid.world_to_grid((0.25, 0.25))
    assert grid.grid[int(g_free.y), int(g_free.x)] == int(CostValues.FREE)

    # An untouched cell should be UNKNOWN — pick a corner well inside the window.
    g_unk = grid.world_to_grid((-0.25, -0.25))
    assert grid.grid[int(g_unk.y), int(g_unk.x)] == int(CostValues.UNKNOWN)


def test_costmap_window_expands_to_include_extra_points() -> None:
    cm = HeightMapCostmap(cell_size=0.5, obstacle_height_threshold=0.3)
    cm.ingest(np.array([[0.0, 0.0, 0.0]], dtype=np.float32), ground_z=0.0)
    grid = cm.to_occupancy_grid(center_x=0.0, center_y=0.0, radius=1.0, extra_points=[(20.0, 0.0)])
    # Window now covers (-1..21) in x — much wider than 2*radius alone.
    assert grid.width >= int(22.0 / 0.5)


def test_distance_point_to_polyline() -> None:
    path = [(0.0, 0.0), (10.0, 0.0)]
    # On the line
    assert _distance_point_to_polyline(5.0, 0.0, path) == 0.0
    # 1 m above midpoint
    assert _distance_point_to_polyline(5.0, 1.0, path) == 1.0
    # Past the endpoint — clamps to endpoint
    assert _distance_point_to_polyline(11.0, 0.0, path) == 1.0
    # Empty path
    assert _distance_point_to_polyline(0.0, 0.0, []) == float("inf")


def test_end_to_end_plan_on_costmap() -> None:
    """Build a U-shaped obstacle, plan around it via the same pipeline the
    module uses (HeightMapCostmap → make_navigation_map → min_cost_astar).
    """
    cm = HeightMapCostmap(cell_size=0.2, obstacle_height_threshold=0.3)
    # Mark a 4-meter rectangle near origin as observed (so it isn't UNKNOWN)
    xs = np.arange(-3.0, 3.0, 0.2)
    ys = np.arange(-3.0, 3.0, 0.2)
    XX, YY = np.meshgrid(xs, ys)
    ground = np.stack([XX.ravel(), YY.ravel(), np.zeros_like(XX.ravel())], axis=1)
    cm.ingest(ground.astype(np.float32), ground_z=0.0)

    # Vertical wall at x=0, y in [-1..1]
    wall_y = np.arange(-1.0, 1.0, 0.05)
    wall = np.stack([np.zeros_like(wall_y), wall_y, np.full_like(wall_y, 1.0)], axis=1)
    cm.ingest(wall.astype(np.float32), ground_z=0.0)

    binary = cm.to_occupancy_grid(center_x=0.0, center_y=0.0, radius=3.0)
    costmap = make_navigation_map(binary, 0.2, strategy="simple", gradient_strategy="voronoi")

    # Plan from left of wall to right of wall, going around it.
    path = min_cost_astar(costmap, goal=(2.0, 0.0), start=(-2.0, 0.0))
    assert path is not None
    assert len(path.poses) > 0
    # End within ~1 cell of the goal
    last = path.poses[-1]
    assert abs(last.x - 2.0) <= 0.5
    assert abs(last.y - 0.0) <= 0.5
