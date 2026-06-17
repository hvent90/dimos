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

"""VAMP-native joint-space planner adapter."""

from __future__ import annotations

from itertools import pairwise
import time
from typing import Any

import numpy as np

from dimos.manipulation.planning.planners.config import VampPlannerConfig
from dimos.manipulation.planning.spec.enums import PlanningStatus
from dimos.manipulation.planning.spec.models import PlanningResult, WorldRobotID
from dimos.manipulation.planning.spec.protocols import WorldSpec
from dimos.manipulation.planning.world.vamp_world import VampWorld
from dimos.msgs.sensor_msgs.JointState import JointState


class VampPlanner:
    """Joint-space planner adapter for VAMP robot modules."""

    def __init__(self, config: VampPlannerConfig) -> None:
        self.config = config

    def plan_joint_path(
        self,
        world: WorldSpec,
        robot_id: WorldRobotID,
        start: JointState,
        goal: JointState,
        timeout: float = 10.0,
    ) -> PlanningResult:
        """Plan a VAMP-native joint-space path."""
        start_time = time.time()
        if not isinstance(world, VampWorld):
            raise ValueError("VampPlanner requires VampWorld")
        if not world.is_finalized:
            return _failure(PlanningStatus.NO_SOLUTION, "World must be finalized before planning")
        if robot_id not in world.get_robot_ids():
            return _failure(PlanningStatus.NO_SOLUTION, f"Robot '{robot_id}' not found")

        if not world.check_config_collision_free(robot_id, start):
            return _failure(PlanningStatus.COLLISION_AT_START, "Start configuration is invalid")
        if not world.check_config_collision_free(robot_id, goal):
            return _failure(PlanningStatus.COLLISION_AT_GOAL, "Goal configuration is invalid")

        robot_name = _robot_name(world)
        robot_module, planner_func, plan_settings, simplify_settings = (
            world.vamp_module.configure_robot_and_planner_with_kwargs(
                robot_name,
                self.config.algorithm,
                max_iterations=_timeout_to_iteration_budget(timeout),
            )
        )
        sampler = robot_module.halton()
        result = planner_func(
            list(start.position),
            list(goal.position),
            world.environment,
            plan_settings,
            sampler,
        )
        if not bool(getattr(result, "solved", False)):
            return _failure(
                PlanningStatus.NO_SOLUTION,
                "VAMP planner did not find a path",
                planning_time=time.time() - start_time,
                iterations=int(getattr(result, "iterations", 0)),
            )

        path_source = result.path
        if self.config.simplify:
            simplified = robot_module.simplify(
                path_source, world.environment, simplify_settings, sampler
            )
            if bool(getattr(simplified, "solved", True)):
                path_source = simplified.path

        path = _path_to_joint_states(
            path_source, start.name or world.get_robot_config(robot_id).joint_names
        )
        if self.config.validate_path and not _validate_path(world, robot_id, path):
            return _failure(
                PlanningStatus.NO_SOLUTION,
                "VAMP returned a path that failed native validation",
                planning_time=time.time() - start_time,
            )
        return PlanningResult(
            status=PlanningStatus.SUCCESS,
            path=path,
            planning_time=time.time() - start_time,
            path_length=_path_length(path),
            iterations=int(getattr(result, "iterations", 0)),
            message="VAMP planning succeeded",
        )

    def get_name(self) -> str:
        """Get planner name."""
        return f"VAMP/{self.config.algorithm}"


def _robot_name(world: VampWorld) -> str:
    artifact = world.config.artifact
    robot = getattr(artifact, "robot", None)
    if isinstance(robot, str):
        return robot
    return world.robot_module.__name__.split(".")[-1]


def _timeout_to_iteration_budget(timeout: float) -> int:
    return max(1, int(timeout * 1000))


def _path_to_joint_states(path_source: Any, joint_names: list[str]) -> list[JointState]:
    path_array = _path_to_array(path_source)
    return [JointState(name=joint_names, position=row.astype(float).tolist()) for row in path_array]


def _path_to_array(path_source: Any) -> np.ndarray:
    if hasattr(path_source, "numpy"):
        return np.asarray(path_source.numpy(), dtype=np.float64)
    return np.asarray(path_source, dtype=np.float64)


def _validate_path(world: VampWorld, robot_id: WorldRobotID, path: list[JointState]) -> bool:
    if not path:
        return False
    return all(
        world.check_edge_collision_free(robot_id, before, after) for before, after in pairwise(path)
    )


def _path_length(path: list[JointState]) -> float:
    if len(path) < 2:
        return 0.0
    total = 0.0
    for before, after in pairwise(path):
        q_before = np.array(before.position, dtype=np.float64)
        q_after = np.array(after.position, dtype=np.float64)
        total += float(np.linalg.norm(q_after - q_before))
    return total


def _failure(
    status: PlanningStatus,
    message: str,
    planning_time: float = 0.0,
    iterations: int = 0,
) -> PlanningResult:
    return PlanningResult(
        status=status,
        planning_time=planning_time,
        iterations=iterations,
        message=message,
    )
