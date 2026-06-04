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

"""pimsim box + FAR nav stack, wired for cross-wall routing.

Composes the babylon box sim with the nav stack (TerrainAnalysis, LocalPlanner,
PathFollower, PGO, SimplePlanner) plus the adapters pimsim needs:
- ``PoseStampedToOdometry``: ``/odom`` -> ``/odometry``
- ``OdomTfBroadcaster``: TF ``map -> body`` for SimplePlanner
- ``MovementManager``: relays ``/clicked_point`` -> the planner goal

Runs on an open cooked floor so a spawned wall is the only obstacle. The
registered ``babylon-nav`` includes the rerun viewer; run it with rerun web::

    dimos run babylon-nav --rerun-web

then drive a goal/walls (PimSimClient) with a HeadlessBrowser connected.
"""

from __future__ import annotations

from pathlib import Path

from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.global_config import global_config
from dimos.core.transport import LCMTransport
from dimos.experimental.pimsim.blueprints.babylon_smoketest import build_babylon_sim
from dimos.experimental.pimsim.odometry_adapter import OdomTfBroadcaster, PoseStampedToOdometry
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.nav_stack.main import create_nav_stack
from dimos.simulation.scene_assets.cook import cook_scene_package
from dimos.simulation.scene_assets.mesh_scene import SceneMeshAlignment
from dimos.simulation.scene_assets.spec import BrowserVisualSpec, MujocoSceneSpec
from dimos.visualization.vis_module import vis_module

FLOOR_SCENE_DIR = Path.home() / ".cache" / "dimos" / "scene_packages" / "pimsim_flat_floor"
WAYPOINT_THRESHOLD_M = 0.6


def ensure_flat_floor_scene() -> str:
    """Cook (once) a 40x40 m flat floor scene; return its scene.meta.json path."""
    meta = FLOOR_SCENE_DIR / "scene.meta.json"
    if meta.exists():
        return str(meta)
    import trimesh

    floor = trimesh.creation.box(extents=[40.0, 40.0, 0.1])
    floor.apply_translation([0.0, 0.0, -0.05])
    glb = FLOOR_SCENE_DIR.parent / "pimsim_flat_floor.glb"
    glb.parent.mkdir(parents=True, exist_ok=True)
    trimesh.Scene(floor).export(str(glb))
    package = cook_scene_package(
        glb,
        output_dir=FLOOR_SCENE_DIR,
        alignment=SceneMeshAlignment(scale=1.0, y_up=False),
        visual_spec=BrowserVisualSpec(optimizer="copy"),
        mujoco_spec=MujocoSceneSpec(enabled=False),
    )
    return str(package.metadata_path)


def build_babylon_nav(scene: str | None = None, *, with_vis: bool = False) -> Blueprint:
    """pimsim sim (open floor) + odom/TF adapters + FAR nav stack."""
    sim = build_babylon_sim(scene or ensure_flat_floor_scene())
    odom_adapter = PoseStampedToOdometry.blueprint().transports(
        {
            ("pose", PoseStamped): LCMTransport("/odom", PoseStamped),
            ("odometry", Odometry): LCMTransport("/odometry", Odometry),
        }
    )
    tf_broadcaster = OdomTfBroadcaster.blueprint().transports(
        {("pose", PoseStamped): LCMTransport("/odom", PoseStamped)}
    )
    nav_stack = create_nav_stack(
        planner="simple",
        vehicle_height=0.40,
        max_speed=0.8,
        waypoint_threshold=WAYPOINT_THRESHOLD_M,
    ).transports({("registered_scan", PointCloud2): LCMTransport("/lidar", PointCloud2)})
    movement_manager = MovementManager.blueprint()
    parts = [sim, odom_adapter, tf_broadcaster, nav_stack, movement_manager]
    if with_vis:
        parts.append(vis_module(global_config.viewer))
    return (
        autoconnect(*parts)
        .remappings([(MovementManager, "way_point", "_mgr_way_point_unused")])
        .global_config(simulation=True)
    )


# The trailing .global_config keeps the module-level assignment recognizable to
# the all_blueprints generator (which detects blueprint-method call chains).
babylon_nav = build_babylon_nav(with_vis=True).global_config(simulation=True)

__all__ = ["babylon_nav", "build_babylon_nav", "ensure_flat_floor_scene"]
