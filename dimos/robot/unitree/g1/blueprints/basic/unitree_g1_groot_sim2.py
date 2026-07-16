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

"""G1 GR00T whole-body control on the sim2 MuJoCo backend."""

from typing import Any, cast

from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.control.tasks.g1_groot_wbc_task.g1_groot_wbc_task import (
    ARM_DEFAULT_POSE,
    G1_GROOT_KD,
    G1_GROOT_KP,
    g1_arms,
    g1_legs_waist,
)
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.hardware.whole_body.spec import WholeBodyConfig
from dimos.mapping.costmapper import CostMapper
from dimos.mapping.pointclouds.occupancy import HeightCostConfig
from dimos.mapping.voxels import VoxelGridMapper
from dimos.msgs.nav_msgs.Path import Path as NavPath
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.replanning_a_star.module import ReplanningAStarPlanner
from dimos.robot.unitree.g1.config import G1
from dimos.robot.unitree.g1.g1_rerun import (
    G1_RERUN_ROOT,
    g1_costmap,
    g1_urdf_joint_state,
    g1_urdf_static_robot,
)
from dimos.robot.unitree.g1.sim2_profile import (
    G1_GROOT_SIM2_LEGACY_MJCF,
    g1_groot_sim2_robot,
)
from dimos.sim2.backend.mujoco import MujocoBackend, MujocoBackendConfig
from dimos.sim2.control.profile import sim_hardware
from dimos.sim2.module import SimModule
from dimos.sim2.spec import ExecutionConfig, ExecutionMode, SimConfig, WorldSpec
from dimos.simulation.scenes.catalog import resolve_scene_package
from dimos.utils.data import LfsPath
from dimos.visualization.rerun.scene_package import scene_package_static_entities
from dimos.visualization.vis_module import vis_module

_SIM_ID = "main"
_GROOT_MODEL_DIR = LfsPath("groot")
_SCENE_ARGUMENT = global_config.scene_package or "office"
_SCENE = resolve_scene_package(_SCENE_ARGUMENT)
_G1 = g1_groot_sim2_robot()
_G1_HARDWARE = sim_hardware(
    _G1,
    sim_id=_SIM_ID,
    wb_config=WholeBodyConfig(kp=tuple(G1_GROOT_KP), kd=tuple(G1_GROOT_KD)),
)
assert G1.height_clearance is not None and G1.width_clearance is not None
_NAV_STACK = autoconnect(
    VoxelGridMapper.blueprint(emit_every=1),
    CostMapper.blueprint(
        config=HeightCostConfig(
            resolution=0.05,
            can_pass_under=G1.height_clearance + 0.2,
            can_climb=0.10,
        ),
        initial_safe_radius_meters=G1.width_clearance + 0.6,
    ),
    ReplanningAStarPlanner.blueprint(
        robot_width=G1.width_clearance,
        robot_rotation_diameter=0.8,
    ),
    MovementManager.blueprint(),
)


def _rerun_blueprint() -> Any:
    import rerun as rr
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Spatial3DView(
            origin="world",
            name="G1 GR00T sim2",
            background=rrb.Background(kind="SolidColor", color=[0, 0, 0]),
            line_grid=rrb.LineGrid3D(
                plane=rr.components.Plane3D.XY.with_distance(0.0),
            ),
        ),
        rrb.TimePanel(state="collapsed"),
    )


def _nav_path(path: NavPath) -> Any:
    return path.to_rerun(z_offset=0.3)


_STATIC_RERUN_ENTITIES: dict[str, Any] = {
    G1_RERUN_ROOT: g1_urdf_static_robot(G1_RERUN_ROOT),
}
_STATIC_RERUN_ENTITIES.update(scene_package_static_entities(_SCENE_ARGUMENT))
_RERUN_CONFIG = {
    "blueprint": _rerun_blueprint,
    "visual_override": {
        "world/world_state": None,
        "world/world_manifest": None,
        "world/coordinator_joint_state": g1_urdf_joint_state(G1_RERUN_ROOT),
        "world/global_costmap": g1_costmap,
        "world/navigation_costmap": g1_costmap,
        "world/path": _nav_path,
    },
    "max_hz": {
        "world/coordinator_joint_state": 20.0,
        "world/global_map": 1.0,
        "world/global_costmap": 2.0,
        "world/navigation_costmap": 2.0,
        "world/path": 0,
    },
    "static": _STATIC_RERUN_ENTITIES,
    "memory_limit": "1GB",
}

unitree_g1_groot_sim2 = (
    autoconnect(
        SimModule.blueprint(
            sim=SimConfig(
                sim_id=_SIM_ID,
                backend=MujocoBackend(
                    MujocoBackendConfig(
                        model_path=G1_GROOT_SIM2_LEGACY_MJCF if _SCENE is None else None,
                        asset_loader="dimos.simulation.mujoco.model:get_assets",
                    )
                ),
                robots=(_G1,),
                world=WorldSpec(
                    scene=_SCENE,
                    revision=_SCENE.package_dir.name
                    if _SCENE is not None
                    else "g1-groot-legacy-v1",
                ),
                execution=ExecutionConfig(
                    mode=ExecutionMode.LOCKSTEP,
                    physics_dt=0.005,
                    control_decimation=4,
                ),
            )
        ),
        ControlCoordinator.blueprint(
            tick_rate=50.0,
            clock="sim",
            sim_id=_SIM_ID,
            hardware=[_G1_HARDWARE],
            tasks=[
                TaskConfig(
                    name="groot_wbc",
                    type="g1_groot_wbc",
                    joint_names=g1_legs_waist,
                    priority=50,
                    auto_start=True,
                    params={
                        "model_path": _GROOT_MODEL_DIR,
                        "hardware_id": "g1",
                        "auto_arm": True,
                        "auto_dry_run": False,
                        "default_ramp_seconds": 0.0,
                        "decimation": 1,
                    },
                ),
                TaskConfig(
                    name="servo_arms",
                    type="servo",
                    joint_names=g1_arms,
                    priority=10,
                    auto_start=True,
                    params={"default_positions": ARM_DEFAULT_POSE},
                ),
            ],
        ),
        _NAV_STACK,
        vis_module(
            viewer_backend=global_config.viewer,
            rerun_config=_RERUN_CONFIG,
        ),
    )
    .remappings(
        cast(
            "Any",
            [
                (VoxelGridMapper, "lidar", "pointcloud"),
                (ControlCoordinator, "twist_command", "cmd_vel"),
            ],
        )
    )
    .global_config(simulation="mujoco", robot_model="unitree_g1", n_workers=8)
)
