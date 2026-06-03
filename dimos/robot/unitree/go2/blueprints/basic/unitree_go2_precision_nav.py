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

"""Unitree Go2 precision-nav blueprint — stock-hardware planning →
precision controller.

End-to-end composition: the operator clicks a goal in the rerun viewer,
the planner builds a path, and the coord's ``precision_follower`` task
(``PrecisionPathFollowerTask``) follows it with a live corridor
half-width tunable via the existing ``KeyboardTeleop`` 0-9 keys.

Stock Go2 hardware (L1 lidar, GO2Connection-published PoseStamped odom)
is sufficient — no Mid360 / FastLio2 required. The composition mirrors
the working smart-tier Go2 nav blueprint
([dimos.robot.unitree.go2.blueprints.smart.unitree_go2.unitree_go2](dimos/robot/unitree/go2/blueprints/smart/unitree_go2.py))
but swaps the final actuation seam: instead of ``MovementManager`` mixing
``nav_cmd_vel`` into ``cmd_vel`` and driving GO2Connection directly,
``ReplanningAStarPlanner.path`` flows into ``ControlCoordinator.path``,
the coord broadcasts ``set_path(path, odom)`` to ``precision_follower``,
and the precision controller drives the robot via the coord's tick loop.

Composition:

- ``unitree_go2_coordinator`` — GO2Connection + ControlCoordinator
  (already declares ``path_follower`` and ``precision_follower`` tasks
  and the ``path: In[Path]`` + ``e_max: In[Float32]`` stream ports).
- ``vis_module`` — rerun viewer + ``RerunWebSocketServer`` (provides
  the click-to-goal source) + websocket vis.
- ``KeyboardTeleop`` with ``disable_movement=True`` — 0-9 keys publish
  ``e_max`` only; WASD/QE Twist generation is disabled.
- ``VoxelGridMapper`` — raw ``lidar: In[PointCloud2]`` (from
  GO2Connection) → ``global_map: Out[PointCloud2]``.
- ``CostMapper`` — voxel ``global_map`` → ``global_costmap:
  Out[OccupancyGrid]``.
- ``ReplanningAStarPlanner`` — directly accepts GO2Connection's
  ``odom: In[PoseStamped]`` (no SLAM step needed) and
  ``clicked_point: In[PointStamped]`` from the rerun server. Emits
  ``path: Out[Path]`` which the coord consumes.


Wiring (all by-port-name, no remappings):

    GO2Connection.lidar (PointCloud2)        -> VoxelGridMapper.lidar
    VoxelGridMapper.global_map (PointCloud2) -> CostMapper.global_map
    CostMapper.global_costmap (OccupancyGrid)-> ReplanningAStarPlanner.global_costmap
    GO2Connection.odom (PoseStamped)         -> ReplanningAStarPlanner.odom
    RerunWebSocketServer.clicked_point       -> ReplanningAStarPlanner.clicked_point
    ReplanningAStarPlanner.path (Path)       -> ControlCoordinator.path
    KeyboardTeleop.e_max (Float32)           -> ControlCoordinator.e_max

``ReplanningAStarPlanner.nav_cmd_vel`` is intentionally left unwired —
the precision controller, not the planner, drives the robot. The planner
is only used as a path source.

Operator flow::

    dimos run unitree-go2-precision-nav

Open rerun → click a point on the map → robot drives the planned path.
Press 0-9 in the pygame window to dial the corridor half-width
(0.0-0.9 m) live; ``PrecisionPathFollowerTask`` re-solves the velocity
profile on each keypress and atomically swaps the per-waypoint cap.
"""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.core.transport import LCMTransport
from dimos.mapping.costmapper import CostMapper
from dimos.mapping.voxels import VoxelGridMapper
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Path import Path
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.std_msgs.Float32 import Float32
from dimos.navigation.costmap_precision_governor.module import CostmapPrecisionGovernor
from dimos.navigation.replanning_a_star.module import ReplanningAStarPlanner
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import (
    rerun_config as _basic_rerun_config,
)
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_coordinator import (
    unitree_go2_coordinator,
    unitree_go2_coordinator_rage,
)
from dimos.robot.unitree.keyboard_teleop import KeyboardTeleop
from dimos.visualization.vis_module import vis_module

_rerun_config = {**_basic_rerun_config, "memory_limit": "4GB"}


def _make(coord):
    return (
        autoconnect(
            coord,
            vis_module(viewer_backend=global_config.viewer, rerun_config=_rerun_config),
            KeyboardTeleop.blueprint(
                publish_only_when_active=True,
                # WASD/QE acts as a safety override: while held it drives
                # the coord's vel_go2 task (priority 20) which preempts
                # the path/precision follower (priority 10). Release the
                # keys → vel_go2 times out (200 ms) and goes inactive →
                # path follower resumes. 0-9 still drives e_max.
                disable_movement=False,
            ),
            VoxelGridMapper.blueprint(emit_every=5),
            # Inflate by ~half the Go2's long axis (~0.70 m / 2). The
            # planner downstream also inflates by half global_config.robot_width
            # (default 0.30 m → 0.165 m), which is shaped for a circular
            # robot, not a long one. Stacking these gives ~0.50 m total
            # envelope so the planner stops carving paths through corners.
            CostMapper.blueprint(inflation_radius_m=0.35),
            ReplanningAStarPlanner.blueprint(),
            CostmapPrecisionGovernor.blueprint(),
        )
        .transports(
            {
                ("e_max", Float32): LCMTransport("/e_max", Float32),
                ("path", Path): LCMTransport("/precision_nav/path", Path),
                ("cmd_vel", Twist): LCMTransport("/cmd_vel", Twist),
                ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
                ("odom", PoseStamped): LCMTransport("/go2/odom", PoseStamped),
            }
        )
        .global_config(n_workers=10, robot_model="unitree_go2")
    )


unitree_go2_precision_nav = _make(unitree_go2_coordinator)
# Rage variant — pair with a rage-mode artifact so the precision
# follower's plant model + envelope match the gait it's tracking.
unitree_go2_precision_nav_rage = _make(unitree_go2_coordinator_rage)

__all__ = ["unitree_go2_precision_nav", "unitree_go2_precision_nav_rage"]
