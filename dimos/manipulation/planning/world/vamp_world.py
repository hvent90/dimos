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
from types import ModuleType
from typing import TYPE_CHECKING, Any

import numpy as np
from scipy.spatial.transform import Rotation as R

from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.enums import ObstacleType
from dimos.manipulation.planning.spec.models import Obstacle, WorldRobotID
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

    @property
    def vamp_module(self) -> ModuleType:
        """The imported VAMP package module."""
        return self._vamp_module

    @property
    def robot_module(self) -> ModuleType:
        """The loaded VAMP robot module for this world."""
        return self._robot_module

    @property
    def environment(self) -> Any:
        """The current VAMP environment object."""
        return self._environment

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

    def _normalize_joint_state(self, robot_id: WorldRobotID, joint_state: JointState) -> JointState:
        config = self._robots[robot_id]
        positions = list(joint_state.position[: len(config.joint_names)])
        names = list(joint_state.name[: len(positions)]) if joint_state.name else config.joint_names
        return JointState(name=names, position=positions)

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
