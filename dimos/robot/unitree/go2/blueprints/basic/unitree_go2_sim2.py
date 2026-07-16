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

"""Fast kinematic Go2 stacks for path-planning and integration tests."""

from typing import Any, cast

from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.coordination.blueprints import autoconnect
from dimos.mapping.costmapper import CostMapper
from dimos.mapping.voxels import VoxelGridMapper
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.replanning_a_star.module import ReplanningAStarPlanner
from dimos.robot.unitree.go2.sim2_profile import go2_kinematic_sim2_robot
from dimos.sim2.backend.kinematic import KinematicBackend
from dimos.sim2.control.profile import sim_hardware
from dimos.sim2.module import SimModule
from dimos.sim2.spec import ExecutionConfig, ExecutionMode, SimConfig, WorldSpec
from dimos.simulation.scenes.catalog import resolve_scene_package
from dimos.simulation.sensors.scene_lidar import SceneLidarConfig, scene_lidar_blueprint

_SIM_ID = "main"
_GO2 = go2_kinematic_sim2_robot()
_GO2_HARDWARE = sim_hardware(_GO2, sim_id=_SIM_ID)
_GO2_TASK = TaskConfig(
    name="vel_go2",
    type="velocity",
    joint_names=list(_GO2.joint_names),
    priority=10,
    auto_start=True,
)
_NAV_SCENE = resolve_scene_package("office")
assert _NAV_SCENE is not None
_GO2_NAV = go2_kinematic_sim2_robot(portable_lidar=True)
_GO2_NAV_HARDWARE = sim_hardware(_GO2_NAV, sim_id=_SIM_ID)
_GO2_NAV_LIDAR = _GO2_NAV.sensors[0]
_GO2_NAV_LIDAR_CONFIG = SceneLidarConfig.for_scene(
    _NAV_SCENE,
    _GO2_NAV_LIDAR,
    scan_model="mid360",
    point_rate=200_000,
    sensor_z=0.25,
    support_floor=True,
    support_floor_z=0.0,
    support_floor_size=100.0,
)

unitree_go2_sim2 = autoconnect(
    SimModule.blueprint(
        sim=SimConfig(
            sim_id=_SIM_ID,
            backend=KinematicBackend(),
            robots=(_GO2,),
            world=WorldSpec(revision="go2-kinematic-v1"),
            execution=ExecutionConfig(
                mode=ExecutionMode.LIVE,
                physics_dt=0.01,
                control_decimation=2,
            ),
        )
    ),
    ControlCoordinator.blueprint(
        tick_rate=50.0,
        hardware=[_GO2_HARDWARE],
        tasks=[_GO2_TASK],
    ),
).global_config(simulation="kinematic", robot_model="unitree_go2", obstacle_avoidance=False)

unitree_go2_sim2_lockstep = autoconnect(
    SimModule.blueprint(
        sim=SimConfig(
            sim_id=_SIM_ID,
            backend=KinematicBackend(),
            robots=(_GO2,),
            world=WorldSpec(revision="go2-kinematic-v1"),
            execution=ExecutionConfig(
                mode=ExecutionMode.LOCKSTEP,
                physics_dt=0.01,
                control_decimation=2,
            ),
        )
    ),
    ControlCoordinator.blueprint(
        tick_rate=50.0,
        clock="sim",
        sim_id=_SIM_ID,
        hardware=[_GO2_HARDWARE],
        tasks=[_GO2_TASK],
    ),
).global_config(simulation="kinematic", robot_model="unitree_go2", obstacle_avoidance=False)

unitree_go2_sim2_nav = (
    autoconnect(
        SimModule.blueprint(
            sim=SimConfig(
                sim_id=_SIM_ID,
                backend=KinematicBackend(),
                robots=(_GO2_NAV,),
                world=WorldSpec(
                    scene=_NAV_SCENE,
                    revision=_NAV_SCENE.package_dir.name,
                ),
                execution=ExecutionConfig(
                    mode=ExecutionMode.LIVE,
                    physics_dt=0.01,
                    control_decimation=2,
                ),
            )
        ),
        ControlCoordinator.blueprint(
            tick_rate=50.0,
            hardware=[_GO2_NAV_HARDWARE],
            tasks=[_GO2_TASK],
        ),
        scene_lidar_blueprint(_GO2_NAV_LIDAR_CONFIG),
        VoxelGridMapper.blueprint(emit_every=1),
        CostMapper.blueprint(),
        ReplanningAStarPlanner.blueprint(robot_width=0.4, robot_rotation_diameter=0.6),
        MovementManager.blueprint(),
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
    .global_config(
        simulation="kinematic",
        robot_model="unitree_go2",
        obstacle_avoidance=False,
        n_workers=7,
    )
)
