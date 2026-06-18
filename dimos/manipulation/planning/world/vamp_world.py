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

"""VAMP-native WorldSpec implementation."""

from __future__ import annotations

from contextlib import contextmanager
from copy import deepcopy
from dataclasses import dataclass
from itertools import pairwise
import time
from typing import TYPE_CHECKING, Any

import numpy as np
from scipy.spatial.transform import Rotation as R

from dimos.manipulation.planning.planners.config import VampPlannerConfig
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.enums import ObstacleType, PlanningStatus
from dimos.manipulation.planning.spec.models import Obstacle, PlanningResult, WorldRobotID
from dimos.manipulation.planning.spec.protocols import WorldSpec
from dimos.manipulation.planning.vamp.errors import UnsupportedWorldCapabilityError
from dimos.manipulation.planning.vamp.loader import load_vamp_robot_module
from dimos.manipulation.planning.world.config import VampWorldConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.utils.transform_utils import matrix_to_pose

if TYPE_CHECKING:
    from collections.abc import Generator

    from numpy.typing import NDArray


@dataclass
class _VampContext:
    joint_states: dict[WorldRobotID, JointState]


class VampWorld(WorldSpec):
    """World adapter for VAMP-native robot artifacts and validity checking."""

    def __init__(self, config: VampWorldConfig) -> None:
        self.config = config
        self._vamp_module, self._robot_module = load_vamp_robot_module(config.artifact)
        self._environment = self._vamp_module.Environment()
        self._robots: dict[WorldRobotID, RobotModelConfig] = {}
        self._live_joint_states: dict[WorldRobotID, JointState] = {}
        self._obstacles: dict[str, Obstacle] = {}
        self._robot_counter = 0
        self._finalized = False

    def add_robot(self, config: RobotModelConfig) -> WorldRobotID:
        """Add a robot to the VAMP world."""
        if self._finalized:
            raise RuntimeError("Cannot add robot after world is finalized")
        if self._robots:
            raise ValueError("VAMP world currently supports one robot per world")
        self._robot_counter += 1
        robot_id = f"robot_{self._robot_counter}"
        self._robots[robot_id] = config
        home_positions = config.home_joints or [0.0] * len(config.joint_names)
        self._live_joint_states[robot_id] = JointState(
            name=config.joint_names,
            position=home_positions,
        )
        return robot_id

    def get_robot_ids(self) -> list[WorldRobotID]:
        """Get all robot IDs."""
        return list(self._robots)

    def get_robot_config(self, robot_id: WorldRobotID) -> RobotModelConfig:
        """Get robot configuration."""
        return self._robots[robot_id]

    def get_joint_limits(
        self, robot_id: WorldRobotID
    ) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        """Get joint limits from config or conservative defaults."""
        config = self._robots[robot_id]
        if config.joint_limits_lower is not None and config.joint_limits_upper is not None:
            return (
                np.array(config.joint_limits_lower, dtype=np.float64),
                np.array(config.joint_limits_upper, dtype=np.float64),
            )
        n_joints = len(config.joint_names)
        return (np.full(n_joints, -np.pi), np.full(n_joints, np.pi))

    def add_obstacle(self, obstacle: Obstacle) -> str:
        """Add an obstacle and rebuild the VAMP environment."""
        self._obstacles[obstacle.name] = obstacle
        self._rebuild_environment()
        return obstacle.name

    def remove_obstacle(self, obstacle_id: str) -> bool:
        """Remove an obstacle."""
        existed = obstacle_id in self._obstacles
        self._obstacles.pop(obstacle_id, None)
        if existed:
            self._rebuild_environment()
        return existed

    def update_obstacle_pose(self, obstacle_id: str, pose: PoseStamped) -> bool:
        """Update an obstacle pose."""
        if obstacle_id not in self._obstacles:
            return False
        self._obstacles[obstacle_id].pose = pose
        self._rebuild_environment()
        return True

    def clear_obstacles(self) -> None:
        """Remove all obstacles."""
        self._obstacles.clear()
        self._rebuild_environment()

    def get_obstacles(self) -> list[Obstacle]:
        """Get all obstacles."""
        return list(self._obstacles.values())

    def finalize(self) -> None:
        """Finalize the VAMP world."""
        self._finalized = True

    @property
    def is_finalized(self) -> bool:
        """Check if the world is finalized."""
        return self._finalized

    def get_live_context(self) -> _VampContext:
        """Get the live VAMP context."""
        return _VampContext(self._live_joint_states)

    @contextmanager
    def scratch_context(self) -> Generator[_VampContext, None, None]:
        """Get a scratch context with copied joint states."""
        yield _VampContext(deepcopy(self._live_joint_states))

    def sync_from_joint_state(self, robot_id: WorldRobotID, joint_state: JointState) -> None:
        """Sync live state from a joint-state message."""
        self._live_joint_states[robot_id] = self._normalize_joint_state(robot_id, joint_state)

    def set_joint_state(
        self, ctx: _VampContext, robot_id: WorldRobotID, joint_state: JointState
    ) -> None:
        """Set robot joint state in a context."""
        ctx.joint_states[robot_id] = self._normalize_joint_state(robot_id, joint_state)

    def get_joint_state(self, ctx: _VampContext, robot_id: WorldRobotID) -> JointState:
        """Get robot joint state from a context."""
        return ctx.joint_states[robot_id]

    def is_collision_free(self, ctx: _VampContext, robot_id: WorldRobotID) -> bool:
        """Check if current configuration is valid according to VAMP."""
        return self._validate_state(ctx.joint_states[robot_id], check_bounds=True)

    def get_min_distance(self, ctx: _VampContext, robot_id: WorldRobotID) -> float:
        """Minimum distance is not exposed by VAMP's Python API."""
        raise UnsupportedWorldCapabilityError("vamp", "minimum distance query")

    def check_config_collision_free(self, robot_id: WorldRobotID, joint_state: JointState) -> bool:
        """Check a joint state using VAMP native validation."""
        return self._validate_state(
            self._normalize_joint_state(robot_id, joint_state), check_bounds=True
        )

    def check_edge_collision_free(
        self,
        robot_id: WorldRobotID,
        start: JointState,
        end: JointState,
        step_size: float = 0.05,
    ) -> bool:
        """Check an edge using VAMP native motion validation."""
        del step_size
        start_state = self._normalize_joint_state(robot_id, start)
        end_state = self._normalize_joint_state(robot_id, end)
        result = self._robot_module.validate_motion(
            list(start_state.position),
            list(end_state.position),
            self._environment,
            True,
        )
        return bool(result)

    def get_ee_pose(self, ctx: _VampContext, robot_id: WorldRobotID) -> PoseStamped:
        """Get end-effector pose from VAMP eefk."""
        joint_state = ctx.joint_states[robot_id]
        transform = np.asarray(
            self._robot_module.eefk(list(joint_state.position)), dtype=np.float64
        )
        pose = matrix_to_pose(transform)
        return PoseStamped(position=pose.position, orientation=pose.orientation, frame_id="world")

    def get_link_pose(
        self, ctx: _VampContext, robot_id: WorldRobotID, link_name: str
    ) -> NDArray[np.float64]:
        """Return EE pose only when the requested link is the configured EE link."""
        config = self._robots[robot_id]
        if link_name != config.end_effector_link:
            raise UnsupportedWorldCapabilityError("vamp", f"link pose for '{link_name}'")
        joint_state = ctx.joint_states[robot_id]
        return np.asarray(self._robot_module.eefk(list(joint_state.position)), dtype=np.float64)

    def get_jacobian(self, ctx: _VampContext, robot_id: WorldRobotID) -> NDArray[np.float64]:
        """VAMP's Python API does not expose a Jacobian."""
        raise UnsupportedWorldCapabilityError("vamp", "end-effector Jacobian")

    def plan_joint_path(
        self,
        planner_config: VampPlannerConfig,
        robot_id: WorldRobotID,
        start: JointState,
        goal: JointState,
        timeout: float = 10.0,
    ) -> PlanningResult:
        """Plan a VAMP-native joint-space path inside the VAMP world adapter."""
        start_time = time.time()
        if not self.is_finalized:
            return _failure(PlanningStatus.NO_SOLUTION, "World must be finalized before planning")
        if robot_id not in self.get_robot_ids():
            return _failure(PlanningStatus.NO_SOLUTION, f"Robot '{robot_id}' not found")

        if not self.check_config_collision_free(robot_id, start):
            return _failure(PlanningStatus.COLLISION_AT_START, "Start configuration is invalid")
        if not self.check_config_collision_free(robot_id, goal):
            return _failure(PlanningStatus.COLLISION_AT_GOAL, "Goal configuration is invalid")

        robot_module, planner_func, plan_settings, simplify_settings = (
            self._vamp_module.configure_robot_and_planner_with_kwargs(
                self._robot_name(),
                planner_config.algorithm,
                max_iterations=_timeout_to_iteration_budget(timeout),
            )
        )
        sampler = robot_module.halton()
        result = planner_func(
            list(start.position),
            list(goal.position),
            self._environment,
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
        if planner_config.simplify:
            simplified = robot_module.simplify(
                path_source, self._environment, simplify_settings, sampler
            )
            if bool(getattr(simplified, "solved", True)):
                path_source = simplified.path

        path = _path_to_joint_states(
            path_source, start.name or self.get_robot_config(robot_id).joint_names
        )
        if planner_config.validate_path and not self._validate_path(robot_id, path):
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

    def _normalize_joint_state(self, robot_id: WorldRobotID, joint_state: JointState) -> JointState:
        config = self._robots[robot_id]
        positions = list(joint_state.position[: len(config.joint_names)])
        names = list(joint_state.name[: len(positions)]) if joint_state.name else config.joint_names
        return JointState(name=names, position=positions)

    def _robot_name(self) -> str:
        robot = getattr(self.config.artifact, "robot", None)
        if isinstance(robot, str):
            return robot
        return self._robot_module.__name__.split(".")[-1]

    def _validate_path(self, robot_id: WorldRobotID, path: list[JointState]) -> bool:
        if not path:
            return False
        return all(
            self.check_edge_collision_free(robot_id, before, after)
            for before, after in pairwise(path)
        )

    def _validate_state(self, joint_state: JointState, check_bounds: bool) -> bool:
        return bool(
            self._robot_module.validate(
                list(joint_state.position),
                self._environment,
                check_bounds,
            )
        )

    def _rebuild_environment(self) -> None:
        self._environment = self._vamp_module.Environment()
        for obstacle in self._obstacles.values():
            self._add_obstacle_to_environment(obstacle)

    def _add_obstacle_to_environment(self, obstacle: Obstacle) -> None:
        center = [obstacle.pose.position.x, obstacle.pose.position.y, obstacle.pose.position.z]
        euler_xyz = (
            R.from_quat(
                [
                    obstacle.pose.orientation.x,
                    obstacle.pose.orientation.y,
                    obstacle.pose.orientation.z,
                    obstacle.pose.orientation.w,
                ]
            )
            .as_euler("xyz")
            .tolist()
        )
        if obstacle.obstacle_type == ObstacleType.SPHERE:
            self._environment.add_sphere(self._vamp_module.Sphere(center, obstacle.dimensions[0]))
        elif obstacle.obstacle_type == ObstacleType.BOX:
            half_extents = [dimension / 2.0 for dimension in obstacle.dimensions]
            self._environment.add_cuboid(self._vamp_module.Cuboid(center, euler_xyz, half_extents))
        elif obstacle.obstacle_type == ObstacleType.CYLINDER:
            self._environment.add_capsule(
                self._vamp_module.Cylinder(
                    center,
                    euler_xyz,
                    obstacle.dimensions[0],
                    obstacle.dimensions[1],
                )
            )
        else:
            raise UnsupportedWorldCapabilityError("vamp", f"{obstacle.obstacle_type.name} obstacle")


def _timeout_to_iteration_budget(timeout: float) -> int:
    return max(1, int(timeout * 1000))


def _path_to_joint_states(path_source: Any, joint_names: list[str]) -> list[JointState]:
    path_array = _path_to_array(path_source)
    return [JointState(name=joint_names, position=row.astype(float).tolist()) for row in path_array]


def _path_to_array(path_source: Any) -> np.ndarray:
    if hasattr(path_source, "numpy"):
        return np.asarray(path_source.numpy(), dtype=np.float64)
    return np.asarray(path_source, dtype=np.float64)


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
