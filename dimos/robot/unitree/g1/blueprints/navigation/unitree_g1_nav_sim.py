#!/usr/bin/env python3
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

"""G1 nav stack on the pimsim (Babylon + Havok) browser sim.

Previously this drove the Unity sim via ``UnityBridgeModule``; it now runs on
the in-repo pimsim. ``build_babylon_nav`` supplies the same contract the nav
stack consumed from Unity — /odometry, /lidar registered scan, and the
map->body TF — and the stack's nav_cmd_vel flows back to the browser base.

Open ``http://localhost:8091/`` (or run a headless browser) to tick the in-browser
physics, then publish a goal to ``/clicked_point``.
"""

from __future__ import annotations

import os
from typing import Any

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.experimental.pimsim.blueprints._factory import build_babylon_nav
from dimos.navigation.nav_stack.main import nav_stack_rerun_config
from dimos.robot.unitree.g1.config import G1, G1_LOCAL_PLANNER_PRECOMPUTED_PATHS
from dimos.robot.unitree.g1.g1_rerun import g1_static_robot
from dimos.visualization.vis_module import vis_module

nav_config: dict[str, Any] = dict(
    planner="simple",
    vehicle_height=G1.height_clearance,
    max_speed=2.0,  # m/s, higher than real robot defaults
    terrain_analysis={
        "ground_height_threshold": 0.05,
        "min_relative_z": -1.5,
    },
    terrain_map_ext={
        "decay_time": 120,
    },
    local_planner={
        "paths_dir": str(G1_LOCAL_PLANNER_PRECOMPUTED_PATHS),
        "min_relative_z": -1.5,
        "freeze_ang": 180.0,
        "obstacle_height_threshold": 0.02,
        "publish_free_paths": True,  # turn off visual for better runtime performance
    },
    path_follower={
        # these effect smoothness quite a bit
        "max_acceleration": 2.0,
        "max_yaw_rate": 60.0,
    },
)

# Load a real scene (not the empty flat floor) and spawn on a road within it.
# Override the scene with DIMOS_PIMSIM_SCENE (name or path to a scene.meta.json).
_SCENE = os.getenv("DIMOS_PIMSIM_SCENE", "cyberpunk-city")

unitree_g1_nav_sim = autoconnect(
    build_babylon_nav(
        _SCENE,
        vehicle_height=G1.height_clearance,
        nav_config=nav_config,
        load_visual=True,
    ),
    vis_module(
        viewer_backend=global_config.viewer,
        rerun_config=nav_stack_rerun_config(
            {
                "static": {
                    "world/tf/robot": g1_static_robot,
                },
            },
            # Rate-limit heavy point cloud topics to prevent rerun crashing
            vis_throttle=0.1,
        ),
    ),
).global_config(n_workers=8, robot_model="unitree_g1", simulation=True)
