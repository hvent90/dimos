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

"""Factory functions for manipulation planning components."""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

from dimos.manipulation.planning.kinematics.config import (
    DrakeOptimizationKinematicsConfig,
    JacobianKinematicsConfig,
    ManipulationKinematicsConfig,
    PinkKinematicsConfig,
    kinematics_config_from_name,
)
from dimos.manipulation.planning.planners.config import (
    ManipulationPlannerConfig,
    RRTConnectPlannerConfig,
    VampPlannerConfig,
    planner_config_from_name,
)
from dimos.manipulation.planning.world.config import (
    DrakeWorldConfig,
    ManipulationWorldConfig,
    VampWorldConfig,
    world_config_from_name,
)

if TYPE_CHECKING:
    from dimos.manipulation.planning.spec.protocols import KinematicsSpec, PlannerSpec, WorldSpec


def create_world(
    backend: str = "drake",
    config: ManipulationWorldConfig | None = None,
    enable_viz: bool = False,
    **kwargs: Any,
) -> WorldSpec:
    """Create a world instance from a backend name or typed world config."""
    if config is None:
        config = world_config_from_name(backend)

    if isinstance(config, DrakeWorldConfig):
        from dimos.manipulation.planning.world.drake_world import DrakeWorld

        return DrakeWorld(enable_viz=enable_viz, **kwargs)
    if isinstance(config, VampWorldConfig):
        from dimos.manipulation.planning.world.vamp_world import VampWorld

        return VampWorld(config=config, **kwargs)
    raise TypeError(f"Unsupported world config: {type(config).__name__}")


def create_kinematics(
    name: str = "jacobian",
    config: ManipulationKinematicsConfig | None = None,
    **kwargs: Any,
) -> KinematicsSpec:
    """Create IK solver from a backend name or typed kinematics config."""
    if config is None:
        config = kinematics_config_from_name(name)

    if isinstance(config, JacobianKinematicsConfig):
        from dimos.manipulation.planning.kinematics.jacobian_ik import JacobianIK

        return JacobianIK(**kwargs)
    elif isinstance(config, DrakeOptimizationKinematicsConfig):
        from dimos.manipulation.planning.kinematics.drake_optimization_ik import (
            DrakeOptimizationIK,
        )

        return DrakeOptimizationIK(**kwargs)
    elif isinstance(config, PinkKinematicsConfig):
        from dimos.manipulation.planning.kinematics.pink_ik import PinkIK

        return PinkIK(config, **kwargs)
    else:
        raise TypeError(f"Unsupported kinematics config: {type(config).__name__}")


def create_planner(
    name: str = "rrt_connect",
    config: ManipulationPlannerConfig | None = None,
    **kwargs: Any,
) -> PlannerSpec:
    """Create motion planner from a backend name or typed planner config."""
    if config is None:
        config = planner_config_from_name(name)

    if isinstance(config, RRTConnectPlannerConfig):
        from dimos.manipulation.planning.planners.rrt_planner import RRTConnectPlanner

        return RRTConnectPlanner(
            step_size=config.step_size,
            connect_step_size=config.connect_step_size,
            goal_tolerance=config.goal_tolerance,
            collision_step_size=config.collision_step_size,
            **kwargs,
        )
    if isinstance(config, VampPlannerConfig):
        from dimos.manipulation.planning.planners.vamp_planner import VampPlanner

        return VampPlanner(config=config, **kwargs)
    raise TypeError(f"Unsupported planner config: {type(config).__name__}")


def validate_planning_stack_config(
    world: ManipulationWorldConfig,
    planner: ManipulationPlannerConfig,
    kinematics: ManipulationKinematicsConfig,
) -> None:
    """Validate that selected world, planner, and kinematics backends can pair."""
    if isinstance(planner, VampPlannerConfig) and not isinstance(world, VampWorldConfig):
        raise ValueError("VAMP planner requires world backend 'vamp'")
    if isinstance(world, VampWorldConfig) and not isinstance(planner, VampPlannerConfig):
        raise ValueError("VAMP world backend requires planner backend 'vamp'")
    if isinstance(kinematics, DrakeOptimizationKinematicsConfig) and not isinstance(
        world, DrakeWorldConfig
    ):
        raise ValueError("Drake optimization kinematics requires world backend 'drake'")


def create_planning_stack(
    robot_config: Any,
    enable_viz: bool = False,
    world: ManipulationWorldConfig | None = None,
    planner_name: str = "rrt_connect",
    planner: ManipulationPlannerConfig | None = None,
    kinematics_name: str = "jacobian",
    kinematics: ManipulationKinematicsConfig | None = None,
) -> tuple[WorldSpec, KinematicsSpec, PlannerSpec, str]:
    """Create complete planning stack. Returns (world, kinematics, planner, robot_id)."""
    world_config = world if world is not None else DrakeWorldConfig()
    planner_config = planner if planner is not None else planner_config_from_name(planner_name)
    kinematics_config = (
        kinematics if kinematics is not None else kinematics_config_from_name(kinematics_name)
    )
    validate_planning_stack_config(world_config, planner_config, kinematics_config)

    world_backend = create_world(config=world_config, enable_viz=enable_viz)
    kinematics_solver = create_kinematics(name=kinematics_name, config=kinematics_config)
    planner_backend = create_planner(name=planner_name, config=planner_config)

    robot_id = world_backend.add_robot(robot_config)
    world_backend.finalize()

    return world_backend, kinematics_solver, planner_backend, robot_id
