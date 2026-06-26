#!/usr/bin/env python3
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

"""3D navigation on Go2 with voxel-grid mapping, MLS planning, and holonomic
trajectory control over WebRTC.

Decouples planning from control: ``MLSPlannerNative`` owns route safety and
emits the ``Path`` (empty when nothing ahead is traversable), while
``DanHolonomicTC`` follows that path with the holonomic tracking law. Compared
to ``unitree_go2_nav_3d`` this swaps ``PointLio`` + ``RayTracingVoxelMap`` for
``VoxelGridMapper``, adds ``PoseOdomRelay``, and replaces ``BasicPathFollower``
with ``DanHolonomicTC``.
"""

from typing import Any

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.mapping.voxels import VoxelGridMapper
from dimos.navigation.holonomic_trajectory_controller.module import DanHolonomicTC
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.nav_3d.mls_planner.goal_relay import GoalRelay, PoseOdomRelay
from dimos.navigation.nav_3d.mls_planner.mls_planner_native import MLSPlannerNative
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import rerun_config
from dimos.robot.unitree.go2.connection import GO2Connection
from dimos.visualization.vis_module import vis_module

voxel_size = 0.08
# Height of the head-mounted lidar above the ground while standing.
go2_lidar_height = 0.5


def _render_global_map(msg: Any) -> Any:
    return msg.to_rerun()


def _render_path(msg: Any) -> Any:
    # The planner emits an empty path when it finds no route to the goal.
    # Logging those would blank the line, so drop them and keep the last path.
    if len(msg.poses) == 0:
        return None
    return msg


_nav_rerun_config = {
    **rerun_config,
    "max_hz": {
        **rerun_config["max_hz"],
        "world/global_map": 1.0,
    },
    "memory_limit": "2096MB",
    "visual_override": {
        **rerun_config["visual_override"],
        "world/global_map": _render_global_map,
        "world/path": _render_path,
        "world/surface_map": None,
        "world/nodes": None,
        "world/node_edges": None,
    },
}

unitree_go2_mls_htc = autoconnect(
    vis_module(viewer_backend=global_config.viewer, rerun_config=_nav_rerun_config),
    # "mcf" for stair traversal
    GO2Connection.blueprint(motion_mode="mcf"),
    PoseOdomRelay.blueprint(),
    VoxelGridMapper.blueprint(
        voxel_size=voxel_size,
        frame_id="world",
        carve_columns=False,
        emit_every=1,
    ),
    MLSPlannerNative.blueprint(
        world_frame="world",
        voxel_size=voxel_size,
        robot_height=go2_lidar_height,
        wall_clearance_m=0.2,
        wall_buffer_m=0.75,
        wall_buffer_weight=100.0,
        step_threshold_m=0.16,
        step_penalty_weight=1.0,
        viz_publish_hz=0.0,
    ),
    GoalRelay.blueprint(),
    DanHolonomicTC.blueprint(),
    MovementManager.blueprint(),
).global_config(n_workers=10, robot_model="unitree_go2", obstacle_avoidance=False)

__all__ = ["unitree_go2_mls_htc"]
