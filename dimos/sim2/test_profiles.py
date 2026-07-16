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

from __future__ import annotations

import numpy as np
import pytest

from dimos.control.coordinator import ControlCoordinator
from dimos.mapping.costmapper import CostMapper
from dimos.mapping.voxels import VoxelGridMapper
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.replanning_a_star.module import ReplanningAStarPlanner
from dimos.robot.manipulators.xarm.blueprints.sim2 import xarm7_sim2
from dimos.robot.manipulators.xarm.sim2_profile import (
    xarm7_sim2_robot,
    xarm7_sim2_world,
)
from dimos.robot.unitree.g1.blueprints.basic.unitree_g1_groot_sim2 import (
    _RERUN_CONFIG,
    unitree_g1_groot_sim2,
)
from dimos.robot.unitree.g1.g1_rerun import G1_RERUN_ROOT
from dimos.robot.unitree.g1.sim2_profile import g1_groot_sim2_robot
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_sim2 import (
    unitree_go2_sim2,
    unitree_go2_sim2_lockstep,
    unitree_go2_sim2_nav,
)
from dimos.robot.unitree.go2.sim2_profile import go2_kinematic_sim2_robot
from dimos.sim2.backend.mujoco import MujocoBackend, MujocoBackendConfig
from dimos.sim2.control.profile import sim_hardware
from dimos.sim2.module import SimModule
from dimos.sim2.spec import ExecutionMode, SensorImplementation, WorldSpec
from dimos.simulation.sensors.scene_lidar import SceneLidarModule


def _kwargs(blueprint, module_type):
    return next(atom.kwargs for atom in blueprint.blueprints if atom.module is module_type)


def test_robot_profiles_derive_matching_generic_hardware() -> None:
    for robot in (
        go2_kinematic_sim2_robot(),
        xarm7_sim2_robot(),
        g1_groot_sim2_robot(),
    ):
        hardware = sim_hardware(
            robot,
            gripper="gripper" in robot.capabilities,
        )
        assert hardware.hardware_id == robot.robot_id
        assert hardware.joints == list(robot.joint_names)
        assert hardware.adapter_type == "sim"
        assert hardware.adapter_kwargs == {"sim_id": "main", "robot_id": robot.robot_id}


def test_parallel_blueprints_select_expected_runtime_and_clock() -> None:
    live_sim = _kwargs(unitree_go2_sim2, SimModule)["sim"]
    live_coordinator = _kwargs(unitree_go2_sim2, ControlCoordinator)
    lockstep_sim = _kwargs(unitree_go2_sim2_lockstep, SimModule)["sim"]
    lockstep_coordinator = _kwargs(unitree_go2_sim2_lockstep, ControlCoordinator)

    assert live_sim.execution.mode == ExecutionMode.LIVE
    assert live_coordinator.get("clock", "wall") == "wall"
    assert lockstep_sim.execution.mode == ExecutionMode.LOCKSTEP
    assert lockstep_coordinator["clock"] == "sim"
    assert _kwargs(xarm7_sim2, SimModule)["sim"].robots[0].robot_id == "arm"
    g1_sim = _kwargs(unitree_g1_groot_sim2, SimModule)["sim"]
    g1_coordinator = _kwargs(unitree_g1_groot_sim2, ControlCoordinator)
    assert g1_sim.robots[0].dof == 29
    assert g1_sim.execution.mode == ExecutionMode.LOCKSTEP
    assert g1_sim.execution.physics_dt == 0.005
    assert g1_sim.execution.control_decimation == 4
    assert g1_coordinator["clock"] == "sim"


def test_g1_rerun_uses_odom_robot_hierarchy_and_static_scene() -> None:
    assert G1_RERUN_ROOT == "world/odom/g1"
    assert G1_RERUN_ROOT in _RERUN_CONFIG["static"]
    assert "world/scene" in _RERUN_CONFIG["static"]
    assert _RERUN_CONFIG["visual_override"]["world/world_state"] is None


def test_go2_nav_profile_uses_portable_lidar_and_complete_nav_stack() -> None:
    sim = _kwargs(unitree_go2_sim2_nav, SimModule)["sim"]
    lidar = _kwargs(unitree_go2_sim2_nav, SceneLidarModule)
    module_types = {atom.module for atom in unitree_go2_sim2_nav.blueprints}

    assert sim.world.scene is not None
    assert sim.robots[0].sensors[0].implementation == SensorImplementation.PORTABLE
    assert lidar["sensor_id"] == sim.robots[0].sensors[0].sensor_id
    assert VoxelGridMapper in module_types
    assert CostMapper in module_types
    assert ReplanningAStarPlanner in module_types
    assert MovementManager in module_types


@pytest.mark.mujoco
def test_xarm_profile_resolves_real_model_and_world_entities() -> None:
    robot = xarm7_sim2_robot()
    backend = MujocoBackend()
    try:
        handles = backend.load(xarm7_sim2_world(), (robot,), physics_dt=0.002)
        observation = backend.observe(handles[robot.robot_id])
        backend.apply_action(
            handles[robot.robot_id],
            {
                "command_mode": np.array([0], dtype=np.int32),
                "enabled": np.array([1], dtype=np.uint8),
                "position": observation["position"],
                "velocity": np.zeros(robot.dof),
                "effort": np.zeros(robot.dof),
                "velocity_scale": np.array([1.0]),
                "gripper": np.array([0.4]),
            },
        )
        backend.step(0.002)

        assert observation["position"].shape == (7,)
        assert {state.entity_id for state in backend.entity_states()} == {
            "arm",
            "apple",
            "orange",
            "cup",
        }
    finally:
        backend.close()


@pytest.mark.mujoco
def test_g1_profile_resolves_real_model_and_imu() -> None:
    robot = g1_groot_sim2_robot()
    backend = MujocoBackend(
        MujocoBackendConfig(asset_loader="dimos.simulation.mujoco.model:get_assets")
    )
    try:
        handles = backend.load(
            WorldSpec(revision="g1-profile-test"),
            (robot,),
            physics_dt=0.002,
        )
        observation = backend.observe(handles[robot.robot_id])

        assert observation["position"].shape == (29,)
        assert observation["imu_quaternion"].shape == (4,)
        assert observation["imu_gyroscope"].shape == (3,)
        assert observation["root_position"][2] == pytest.approx(0.793)
    finally:
        backend.close()
