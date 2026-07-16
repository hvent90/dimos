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

"""Deterministic pose-integrating backend for velocity-controlled robots."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any

import numpy as np
from numpy.typing import NDArray

from dimos.sim2.backend.base import RobotHandle, SensorSample
from dimos.sim2.spec import (
    ControlInterface,
    EntityDescriptor,
    EntityState,
    SimRobotSpec,
    SpawnPose,
    WorldSpec,
)


@dataclass
class _KinematicRobot:
    spec: SimRobotSpec
    pose: NDArray[np.float64]
    z: float
    velocity: NDArray[np.float64]
    enabled: bool = True


class KinematicBackend:
    def __init__(self) -> None:
        self._robots: dict[str, _KinematicRobot] = {}
        self._scene_entities: dict[str, EntityState] = {}
        self._next_wall_id = 1

    @property
    def capabilities(self) -> frozenset[ControlInterface]:
        return frozenset({ControlInterface.TWIST_BASE})

    def load(
        self,
        world: WorldSpec,
        robots: tuple[SimRobotSpec, ...],
        physics_dt: float,
    ) -> dict[str, RobotHandle]:
        del world, physics_dt
        self._robots = {}
        self._scene_entities = {}
        self._next_wall_id = 1
        handles: dict[str, RobotHandle] = {}
        for spec in robots:
            if spec.control_interface not in self.capabilities:
                raise ValueError(
                    f"kinematic backend does not support {spec.control_interface.value} "
                    f"for robot '{spec.robot_id}'"
                )
            if spec.dof not in (2, 3):
                raise ValueError("kinematic twist bases require 2 or 3 velocity DOFs")
            yaw = _yaw_from_quaternion(spec.spawn.quaternion_xyzw)
            robot = _KinematicRobot(
                spec=spec,
                pose=np.array([spec.spawn.position[0], spec.spawn.position[1], yaw]),
                z=spec.spawn.position[2],
                velocity=np.zeros(spec.dof, dtype=np.float64),
            )
            self._robots[spec.robot_id] = robot
            handles[spec.robot_id] = RobotHandle(
                robot_id=spec.robot_id,
                control_interface=spec.control_interface,
                dof=spec.dof,
                backend_data=robot,
            )
        return handles

    def reset(self, seed: int | None = None) -> None:
        del seed
        for robot in self._robots.values():
            robot.pose[:] = (
                robot.spec.spawn.position[0],
                robot.spec.spawn.position[1],
                _yaw_from_quaternion(robot.spec.spawn.quaternion_xyzw),
            )
            robot.z = robot.spec.spawn.position[2]
            robot.velocity.fill(0.0)
            robot.enabled = True

    def apply_action(self, handle: RobotHandle, action: dict[str, Any]) -> None:
        robot = self._robot(handle)
        robot.enabled = bool(np.asarray(action["enabled"])[0])
        if robot.enabled:
            robot.velocity[:] = np.asarray(action["velocities"], dtype=np.float64)
        else:
            robot.velocity.fill(0.0)

    def step(self, dt: float) -> None:
        for robot in self._robots.values():
            if robot.spec.dof == 3:
                vx, vy, wz = robot.velocity
            else:
                vx, wz = robot.velocity
                vy = 0.0
            yaw = robot.pose[2]
            robot.pose[0] += (math.cos(yaw) * vx - math.sin(yaw) * vy) * dt
            robot.pose[1] += (math.sin(yaw) * vx + math.cos(yaw) * vy) * dt
            robot.pose[2] = _wrap_angle(yaw + wz * dt)

    def observe(self, handle: RobotHandle) -> dict[str, Any]:
        robot = self._robot(handle)
        odometry = robot.pose.copy()
        if robot.spec.dof == 2:
            odometry = odometry[[0, 2]]
        return {
            "enabled": np.array([robot.enabled], dtype=np.uint8),
            "velocities": robot.velocity.copy(),
            "odometry": odometry,
        }

    def set_robot_pose(self, handle: RobotHandle, pose: SpawnPose) -> None:
        robot = self._robot(handle)
        robot.pose[:] = (
            pose.position[0],
            pose.position[1],
            _yaw_from_quaternion(pose.quaternion_xyzw),
        )
        robot.z = pose.position[2]
        robot.velocity.fill(0.0)
        robot.enabled = True

    def add_wall(
        self,
        x1: float,
        y1: float,
        x2: float,
        y2: float,
        height: float,
        thickness: float,
    ) -> EntityDescriptor:
        length = math.hypot(x2 - x1, y2 - y1)
        if length <= 0.0:
            raise ValueError("wall endpoints must be distinct")
        if height <= 0.0 or thickness <= 0.0:
            raise ValueError("wall height and thickness must be positive")
        entity_id = f"wall-{self._next_wall_id}"
        self._next_wall_id += 1
        yaw = math.atan2(y2 - y1, x2 - x1)
        self._scene_entities[entity_id] = EntityState(
            entity_id=entity_id,
            position=((x1 + x2) / 2.0, (y1 + y2) / 2.0, height / 2.0),
            quaternion_xyzw=_quaternion_from_yaw(yaw),
        )
        return EntityDescriptor(
            entity_id=entity_id,
            kind="kinematic",
            backend_name=entity_id,
            shape_hint="box",
            extents=(length, thickness, height),
        )

    def entity_states(self) -> tuple[EntityState, ...]:
        robots = tuple(
            EntityState(
                entity_id=robot.spec.robot_id,
                position=(float(robot.pose[0]), float(robot.pose[1]), robot.z),
                quaternion_xyzw=_quaternion_from_yaw(float(robot.pose[2])),
                linear_velocity=(
                    float(robot.velocity[0]),
                    float(robot.velocity[1]) if robot.spec.dof == 3 else 0.0,
                    0.0,
                ),
                angular_velocity=(0.0, 0.0, float(robot.velocity[-1])),
            )
            for robot in self._robots.values()
        )
        return robots + tuple(self._scene_entities.values())

    def sensor_samples(self, sim_time: float) -> tuple[SensorSample, ...]:
        del sim_time
        return ()

    def close(self) -> None:
        self._robots.clear()
        self._scene_entities.clear()

    @staticmethod
    def _robot(handle: RobotHandle) -> _KinematicRobot:
        if not isinstance(handle.backend_data, _KinematicRobot):
            raise ValueError(f"invalid kinematic handle for robot '{handle.robot_id}'")
        return handle.backend_data


def _yaw_from_quaternion(quaternion: tuple[float, float, float, float]) -> float:
    x, y, z, w = quaternion
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def _wrap_angle(value: float) -> float:
    return (value + math.pi) % (2.0 * math.pi) - math.pi
