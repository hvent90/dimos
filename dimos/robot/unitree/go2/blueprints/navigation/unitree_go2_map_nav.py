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

"""Virtual map navigation: load a recording as a frozen map, perfect odom plant.

Unlike ``unitree-go2-mls-htc --replay``, this does **not** play bag odometry.
``cmd_vel`` (teleop or holonomic follower) moves a simulated pose; floor Z snaps
to the static map under the body so teleop and click-nav both climb stairs.

Works with Go2 L1 bags (``lidar`` in world) and Mid360 Point-LIO bags
(``pointlio_lidar`` in ``mid360_link``, registered via ``tf``).

Usage::

    uv run dimos --replay-db=go2_synthetic_stairs --viewer=rerun run unitree-go2-map-nav
    uv run dimos --replay-db=go2_bigoffice --viewer=rerun run unitree-go2-map-nav
    uv run dimos --replay-db=mid360_athens_stairs --viewer=rerun run unitree-go2-map-nav

Muddy non-SLAM bags (e.g. ``go2_bigoffice``): offline PGO cleans the map, then
caches ``{db}.map_nav_pgo.pc2.lcm`` beside the dataset for fast reloads::

    uv run dimos --replay-db=go2_bigoffice --viewer=rerun --map-pgo run unitree-go2-map-nav

    # Optional prebuild (same PGO as map-nav cache; also writes cwd/{stem}.pc2.lcm):
    #   uv run dimos map global go2_bigoffice --pgo --export --no-gui

Agent / MCP (milestones + click knobs on :9990)::

    uv run dimos --replay-db=go2_bigoffice --viewer=rerun \
      --map-pgo --map-for-agent --map-milestones 20 run unitree-go2-map-nav

    # MCP tools: list_milestones, go_to_milestone, go_to_point,
    # follow_milestones, stop_navigation

Controls (same as live Go2 nav):
  - ``MovementManager.tele_cmd_vel`` from Rerun / web dashboard (vis_module)
  - Click in Rerun 3D -> goal -> MLS -> holonomic follow
  - Teleop cancels nav so you can stop a follow
  - With ``--map-for-agent``: MCP skills publish the same click path
"""

from __future__ import annotations

from typing import Any

from dimos.agents.mcp.mcp_server import McpServer
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.navigation.dannav.holonomic_tc.module import DanHolonomicTC
from dimos.navigation.dannav.local_planner.module import DanLocalPlanner
from dimos.navigation.map_nav.map_nav_agent import MapNavAgent
from dimos.navigation.map_nav.map_nav_plant import MapNavPlant
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.nav_3d.mls_planner.goal_relay import GoalRelay
from dimos.navigation.nav_3d.mls_planner.mls_planner_native import MLSPlannerNative
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import rerun_config
from dimos.visualization.vis_module import vis_module

voxel_size = 0.08
body_height_m = 0.31
stair_step_threshold_m = 0.4


# Rerun visual_override: log the frozen cloud once. Plant still bursts on LCM so
# late MLS/bridge subscribers catch the map; returning None stops re-ingest spam.
_global_map_viz_state = {"logged": False}


def _render_global_map(msg: Any) -> Any:
    if _global_map_viz_state["logged"]:
        return None
    _global_map_viz_state["logged"] = True
    return msg.to_rerun()


def _render_path(msg: Any) -> Any:
    if len(msg.poses) == 0:
        return None
    # Default Path.to_rerun is green (0, 255, 128); red reads better on the map cloud.
    return msg.to_rerun(color=(255, 48, 48), radii=0.08)


def _render_milestones(msg: Any) -> Any:
    """Labeled milestone markers ``m1``..``mN`` in Rerun."""
    if len(msg.poses) == 0:
        return None
    import rerun as rr

    positions = [[p.x, p.y, p.z] for p in msg.poses]
    labels = [f"m{i + 1}" for i in range(len(positions))]
    return rr.Points3D(
        positions=positions,
        labels=labels,
        show_labels=True,
        radii=[0.25] * len(positions),
        colors=[[255, 64, 64]] * len(positions),
    )


# def _render_mls_nodes(msg: Any) -> Any:
#     return msg.to_rerun()


def _render_mls_node_edges(msg: Any) -> Any:
    # Default LineSegments3D.to_rerun lifts by 1.7 m for old 2D overlays; keep on surface.
    return msg.to_rerun(z_offset=0.0)


def _render_odom(msg: Any) -> Any:
    """Keep odom TF and show a floating ``marvin`` label above the dog."""
    import rerun as rr

    label_z = msg.z + 0.35
    return [
        ("world/odom", msg.to_rerun()),
        (
            "world/robot_label",
            rr.Points3D(
                positions=[[msg.x, msg.y, label_z]],
                labels=["marvin"],
                show_labels=True,
                radii=[0.001],
                colors=[[255, 64, 64]],
            ),
        ),
    ]


_nav_rerun_config = {
    **rerun_config,
    "max_hz": {
        **rerun_config["max_hz"],
        # Same as unitree_go2_mls_htc / unitree_go2_basic: 0 = no bridge throttle.
        "world/global_map": 0,
        "world/color_image": 0,
        "world/milestones": 1,
    },
    "memory_limit": "8192MB",
    "visual_override": {
        **rerun_config["visual_override"],
        "world/global_map": _render_global_map,
        "world/planner_path": None,
        "world/path": _render_path,
        "world/milestones": _render_milestones,
        "world/surface_map": None,
        # Nodes include wall/ceiling samples; edges are the walkable graph on the surface.
        "world/nodes": None,
        "world/node_edges": _render_mls_node_edges if global_config.map_for_agent else None,
        "world/odom": _render_odom,
    },
}

_modules: list[Any] = [
    vis_module(viewer_backend=global_config.viewer, rerun_config=_nav_rerun_config),
    MapNavPlant.blueprint(
        voxel_size=voxel_size,
        body_height_m=body_height_m,
        frame_id="world",
        max_step_m=stair_step_threshold_m,
    ),
    MLSPlannerNative.blueprint(
        world_frame="world",
        voxel_size=voxel_size,
        robot_height=body_height_m,
        surface_closing_radius=0.5,
        wall_clearance_m=0.2,
        wall_buffer_m=0.75,
        wall_buffer_weight=100.0,
        step_threshold_m=stair_step_threshold_m,
        step_penalty_weight=1.0,
        # Publish nodes so MapNavAgent can sample milestone XYZ from them.
        viz_publish_hz=1.0 if global_config.map_for_agent else 0.0,
        node_spacing_m=0.75 if global_config.map_for_agent else 1.0,
    ).remappings(
        [
            (MLSPlannerNative, "path", "planner_path"),
            (MLSPlannerNative, "start_pose", "odom"),
        ]
    ),
    GoalRelay.blueprint(),
    DanLocalPlanner.blueprint(resample_spacing_m=0.0),
    DanHolonomicTC.blueprint(run_profile="walk"),
    # tele_cmd_vel comes from vis_module (RerunWebSocketServer / WebsocketVisModule)
    MovementManager.blueprint(),
]

if global_config.map_for_agent:
    _modules.extend(
        [
            MapNavAgent.blueprint(
                enabled=True,
                n_milestones=global_config.map_milestones,
                frame_id="world",
                body_height_m=body_height_m,
            ),
            McpServer.blueprint(),
        ]
    )

unitree_go2_map_nav = autoconnect(*_modules).global_config(
    n_workers=8, robot_model="unitree_go2", obstacle_avoidance=False
)
