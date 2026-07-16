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

"""MuJoCo implementation of the sim2 world backend."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, replace
import importlib
import math
from pathlib import Path
from typing import Any, cast

import mujoco
import numpy as np

from dimos.sim2.backend.base import RobotHandle, SensorSample
from dimos.sim2.spec import (
    ControlInterface,
    EntityState,
    RaycastLidarSpec,
    SensorImplementation,
    SimRobotSpec,
    SpawnPose,
    WorldSpec,
    scene_entity_descriptors,
)
from dimos.simulation.engines.robot_sim_binding import (
    RobotSimBinding,
    RobotSimSpec,
    mjcf_joint_names_from_hardware,
    resolve_robot_sim_binding,
)
from dimos.simulation.mujoco.scene_package_entity_composer import (
    add_scene_package_entities_to_spec,
    find_scene_package_entity_spawn_penetrators,
)
from dimos.simulation.utils.xml_parser import build_joint_mappings
from dimos.utils.logging_config import setup_logger

_MANIP_POSITION = 0
_MANIP_VELOCITY = 1
_MANIP_EFFORT = 2

logger = setup_logger()


@dataclass(frozen=True)
class MujocoBackendConfig:
    model_path: Path | None = None
    assets: dict[str, bytes] | None = None
    asset_loader: Callable[[], dict[str, bytes]] | str | None = None
    composed_binary_key: str | None = None
    composed_robot: str | None = None
    composed_entity_policy: str | None = None
    add_floor_without_scene: bool = True


@dataclass
class _MujocoRobot:
    spec: SimRobotSpec
    binding: RobotSimBinding
    enabled: bool = True
    whole_body_action: dict[str, np.ndarray[Any, Any]] | None = None
    gripper_actuator_id: int | None = None
    gripper_joint_id: int | None = None


@dataclass
class _MujocoLidar:
    robot: _MujocoRobot
    spec: RaycastLidarSpec
    camera_ids: tuple[int, ...]
    ray_directions_camera: tuple[np.ndarray[Any, Any], ...]
    geom_groups: np.ndarray[Any, Any]
    last_sample_time: float = -math.inf


class MujocoBackend:
    def __init__(self, config: MujocoBackendConfig | None = None) -> None:
        self.config = config or MujocoBackendConfig()
        self._model: mujoco.MjModel | None = None
        self._data: mujoco.MjData | None = None
        self._robots: dict[str, _MujocoRobot] = {}
        self._world_bodies: dict[str, int] = {}
        self._lidars: list[_MujocoLidar] = []

    @property
    def capabilities(self) -> frozenset[ControlInterface]:
        return frozenset({ControlInterface.MANIPULATOR, ControlInterface.WHOLE_BODY})

    def load(
        self,
        world: WorldSpec,
        robots: tuple[SimRobotSpec, ...],
        physics_dt: float,
    ) -> dict[str, RobotHandle]:
        model_path, model_prefixes = self._resolve_model(world, robots)
        if model_path is not None:
            self._model = self._load_model(model_path)
        assert self._model is not None
        self._model.opt.timestep = physics_dt
        self._data = mujoco.MjData(self._model)
        mappings = build_joint_mappings(
            None
            if model_path is None
            or model_path.suffix.lower() == ".mjb"
            or self.config.assets
            or self.config.asset_loader
            else model_path,
            self._model,
        )

        self._world_bodies = {}
        world_entities = list(world.entities)
        if world.scene is not None:
            known_ids = {entity.entity_id for entity in world_entities}
            world_entities.extend(
                entity
                for entity in scene_entity_descriptors(world.scene)
                if entity.entity_id not in known_ids
            )
        for entity in world_entities:
            backend_name = entity.backend_name or entity.entity_id
            body_id = int(mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_BODY, backend_name))
            if body_id < 0:
                raise ValueError(
                    f"tracked entity '{entity.entity_id}' body '{backend_name}' was not found"
                )
            self._world_bodies[entity.entity_id] = body_id

        handles: dict[str, RobotHandle] = {}
        self._robots = {}
        for spec in robots:
            if spec.control_interface not in self.capabilities:
                raise ValueError(
                    f"MuJoCo backend does not expose {spec.control_interface.value} "
                    f"for robot '{spec.robot_id}'"
                )
            binding_spec = spec.backend_options.get("mujoco_spec")
            if binding_spec is None:
                binding_spec = RobotSimSpec(
                    robot_id=spec.robot_id,
                    hardware_joints=spec.joint_names,
                    model_joint_names=mjcf_joint_names_from_hardware(spec.joint_names),
                )
            if not isinstance(binding_spec, RobotSimSpec):
                raise TypeError("backend_options['mujoco_spec'] must be RobotSimSpec")
            model_prefix = model_prefixes.get(spec.robot_id)
            if model_prefix is not None:
                binding_spec = replace(binding_spec, model_prefix=model_prefix)
            binding = resolve_robot_sim_binding(self._model, binding_spec, mappings)
            robot = _MujocoRobot(spec=spec, binding=binding)
            robot.gripper_actuator_id = self._optional_id(
                mujoco.mjtObj.mjOBJ_ACTUATOR,
                spec.backend_options.get("gripper_actuator_name"),
            )
            robot.gripper_joint_id = self._optional_id(
                mujoco.mjtObj.mjOBJ_JOINT,
                spec.backend_options.get("gripper_joint_name"),
            )
            if "gripper" in spec.capabilities and robot.gripper_actuator_id is None:
                raise ValueError(
                    f"robot '{spec.robot_id}' declares a gripper without gripper_actuator_name"
                )
            self._robots[spec.robot_id] = robot
            handles[spec.robot_id] = RobotHandle(
                robot_id=spec.robot_id,
                control_interface=spec.control_interface,
                dof=spec.dof,
                backend_data=robot,
            )
        self._configure_lidars()
        self.reset()
        return handles

    def reset(self, seed: int | None = None) -> None:
        del seed
        model, data = self._require_model()
        if model.nkey > 0:
            mujoco.mj_resetDataKeyframe(model, data, 0)
        else:
            mujoco.mj_resetData(model, data)  # type: ignore[attr-defined]
        for robot in self._robots.values():
            self._apply_spawn(robot)
            positions = robot.spec.backend_options.get("reset_joint_positions")
            if positions is not None:
                if len(positions) != robot.spec.dof:
                    raise ValueError(
                        f"robot '{robot.spec.robot_id}' reset positions do not match DOF"
                    )
                for address, value in zip(
                    robot.binding.joint_qpos_adrs,
                    positions,
                    strict=True,
                ):
                    data.qpos[address] = float(value)
            robot.enabled = True
            robot.whole_body_action = None
        for lidar in self._lidars:
            lidar.last_sample_time = -math.inf
        data.ctrl[:] = 0.0
        mujoco.mj_forward(model, data)

    def set_robot_pose(self, handle: RobotHandle, pose: SpawnPose) -> None:
        model, data = self._require_model()
        robot = self._robot(handle)
        qpos_address = robot.binding.root_qpos_adr
        qvel_address = robot.binding.root_qvel_adr
        if qpos_address is None:
            raise NotImplementedError(
                f"robot '{robot.spec.robot_id}' has no free root and cannot be respawned"
            )
        x, y, z, w = pose.quaternion_xyzw
        data.qpos[qpos_address : qpos_address + 3] = pose.position
        data.qpos[qpos_address + 3 : qpos_address + 7] = (w, x, y, z)
        if qvel_address is not None:
            data.qvel[qvel_address : qvel_address + 6] = 0.0
        robot.enabled = True
        robot.whole_body_action = None
        mujoco.mj_forward(model, data)

    def apply_action(self, handle: RobotHandle, action: dict[str, Any]) -> None:
        _, data = self._require_model()
        robot = self._robot(handle)
        robot.enabled = bool(np.asarray(action["enabled"])[0])
        if not robot.enabled:
            for actuator_id in robot.binding.actuator_ids:
                data.ctrl[actuator_id] = 0.0
            return

        if robot.spec.control_interface == ControlInterface.WHOLE_BODY:
            robot.whole_body_action = {
                name: np.asarray(action[name]).copy()
                for name in ("position", "velocity", "kp", "kd", "effort")
            }
            self._apply_whole_body_pd(robot)
            return

        mode = int(np.asarray(action["command_mode"])[0])
        if mode == _MANIP_POSITION:
            values = np.asarray(action["position"])
        elif mode == _MANIP_VELOCITY:
            values = np.asarray(action["velocity"])
        elif mode == _MANIP_EFFORT:
            values = np.asarray(action["effort"])
        else:
            raise ValueError(f"unknown manipulator command mode {mode}")
        self._write_actuators(robot, values)
        if robot.gripper_actuator_id is not None:
            self._write_gripper(robot, float(np.asarray(action["gripper"])[0]))

    def step(self, dt: float) -> None:
        model, data = self._require_model()
        if not math.isclose(float(model.opt.timestep), dt, rel_tol=0.0, abs_tol=1e-12):
            raise ValueError(f"MuJoCo timestep is {model.opt.timestep}, runtime requested {dt}")
        for robot in self._robots.values():
            if robot.enabled and robot.whole_body_action is not None:
                self._apply_whole_body_pd(robot)
        mujoco.mj_step(model, data)

    def observe(self, handle: RobotHandle) -> dict[str, Any]:
        model, data = self._require_model()
        robot = self._robot(handle)
        binding = robot.binding
        position = data.qpos[list(binding.joint_qpos_adrs)].copy()
        velocity = data.qvel[list(binding.joint_qvel_adrs)].copy()
        effort = data.qfrc_actuator[list(binding.joint_qvel_adrs)].copy()
        if robot.spec.control_interface == ControlInterface.MANIPULATOR:
            gripper = 0.0
            if robot.gripper_joint_id is not None:
                gripper = float(data.qpos[int(model.jnt_qposadr[robot.gripper_joint_id])])
            return {
                "position": position,
                "velocity": velocity,
                "effort": effort,
                "gripper": np.array([gripper]),
                "enabled": np.array([robot.enabled], dtype=np.uint8),
                "error_code": np.array([0], dtype=np.int32),
            }

        root_position, root_quaternion, root_linear_velocity, root_angular_velocity = (
            self._root_state(robot)
        )
        imu_quaternion = self._sensor_or_default(
            binding.imu_quat_slice,
            root_quaternion[[3, 0, 1, 2]],
        )
        imu_gyro = self._sensor_or_default(binding.imu_gyro_slice, root_angular_velocity)
        imu_accel = self._sensor_or_default(binding.imu_accel_slice, np.zeros(3))
        return {
            "position": position,
            "velocity": velocity,
            "effort": effort,
            "imu_quaternion": imu_quaternion,
            "imu_gyroscope": imu_gyro,
            "imu_accelerometer": imu_accel,
            "imu_rpy": _rpy_from_xyzw(root_quaternion),
            "root_position": root_position,
            "root_quaternion": root_quaternion,
            "root_linear_velocity": root_linear_velocity,
            "root_angular_velocity": root_angular_velocity,
            "enabled": np.array([robot.enabled], dtype=np.uint8),
        }

    def entity_states(self) -> tuple[EntityState, ...]:
        robots = tuple(
            EntityState(
                entity_id=robot.spec.robot_id,
                position=tuple(self._root_state(robot)[0].tolist()),
                quaternion_xyzw=tuple(self._root_state(robot)[1].tolist()),
                linear_velocity=tuple(self._root_state(robot)[2].tolist()),
                angular_velocity=tuple(self._root_state(robot)[3].tolist()),
            )
            for robot in self._robots.values()
        )
        world_entities = tuple(
            self._body_entity_state(entity_id, body_id)
            for entity_id, body_id in self._world_bodies.items()
        )
        return robots + world_entities

    def sensor_samples(self, sim_time: float) -> tuple[SensorSample, ...]:
        samples: list[SensorSample] = []
        for lidar in self._lidars:
            if sim_time - lidar.last_sample_time < 1.0 / lidar.spec.rate_hz:
                continue
            lidar.last_sample_time = sim_time
            points = self._raycast_lidar(lidar)
            samples.append(
                SensorSample(
                    sensor_id=lidar.spec.sensor_id,
                    robot_id=lidar.robot.spec.robot_id,
                    frame_id=lidar.spec.frame_id,
                    payload={"points": points, "voxel_size": lidar.spec.voxel_size},
                )
            )
        return tuple(samples)

    def close(self) -> None:
        self._lidars.clear()
        self._robots.clear()
        self._world_bodies.clear()
        self._data = None
        self._model = None

    def _resolve_model(
        self,
        world: WorldSpec,
        robots: tuple[SimRobotSpec, ...],
    ) -> tuple[Path | None, dict[str, str | None]]:
        if self.config.model_path is not None:
            path = Path(self.config.model_path).expanduser().resolve()
            if not path.exists():
                raise FileNotFoundError(f"MuJoCo model not found: {path}")
            return path, {robot.robot_id: None for robot in robots}

        if world.scene is not None:
            composed = world.scene.mujoco_composed_binary_path(
                self.config.composed_binary_key,
                robot=self.config.composed_robot,
                entity_policy=self.config.composed_entity_policy,
            )
            if composed is not None:
                path = Path(composed).expanduser().resolve()
                if not path.exists():
                    raise FileNotFoundError(f"composed MuJoCo model not found: {path}")
                return path, {robot.robot_id: None for robot in robots}
            return self._compose_model(world, robots)

        paths = [
            Path(robot.model_path).expanduser().resolve()
            for robot in robots
            if robot.model_path is not None
        ]
        if len(robots) > 1 or any(
            robot.backend_options.get("mujoco_meshdir") is not None for robot in robots
        ):
            return self._compose_model(world, robots)
        if len(paths) != 1:
            raise ValueError("a MuJoCo robot must declare exactly one model_path")
        path = paths[0]
        path = path.expanduser().resolve()
        if not path.exists():
            raise FileNotFoundError(f"MuJoCo model not found: {path}")
        return path, {robots[0].robot_id: None}

    def _compose_model(
        self,
        world: WorldSpec,
        robots: tuple[SimRobotSpec, ...],
    ) -> tuple[None, dict[str, str | None]]:
        if world.scene is not None:
            if world.scene.mujoco_scene_path is None:
                raise ValueError(
                    f"scene package has no MuJoCo scene artifact: {world.scene.metadata_path}"
                )
            scene_spec = mujoco.MjSpec.from_file(str(world.scene.mujoco_scene_path))
        else:
            scene_spec = mujoco.MjSpec()
            if self.config.add_floor_without_scene:
                scene_spec.worldbody.add_geom(
                    name="sim2-floor",
                    type=mujoco.mjtGeom.mjGEOM_PLANE,
                    size=[0.0, 0.0, 0.05],
                    group=2,
                )

        prefixes: dict[str, str | None] = {}
        for index, robot in enumerate(robots):
            if robot.model_path is None:
                raise ValueError(f"robot '{robot.robot_id}' has no MuJoCo model_path")
            robot_spec = mujoco.MjSpec.from_file(str(robot.model_path))
            meshdir = robot.backend_options.get("mujoco_meshdir")
            if meshdir is not None:
                robot_spec.meshdir = str(meshdir)
            if index == 0:
                scene_spec.option.timestep = robot_spec.option.timestep

            x, y, z = robot.spawn.position
            qx, qy, qz, qw = robot.spawn.quaternion_xyzw
            frame = scene_spec.worldbody.add_frame(
                pos=[float(x), float(y), float(z)],
                quat=[float(qw), float(qx), float(qy), float(qz)],
            )
            prefix = f"{robot.robot_id}-" if len(robots) > 1 else None
            scene_spec.attach(robot_spec, prefix=prefix, frame=frame)
            prefixes[robot.robot_id] = prefix

        if world.scene is not None and world.scene.entities:
            add_scene_package_entities_to_spec(scene_spec, world.scene.entities)
        model = scene_spec.compile()
        if world.scene is not None and world.scene.entities:
            penetrators = find_scene_package_entity_spawn_penetrators(model)
            if penetrators:
                logger.warning(
                    "sim2 scene entities spawn in deep contact; welding static",
                    count=len(penetrators),
                    samples=sorted(penetrators)[:20],
                )
                scene_spec = self._compose_with_static_entities(world, robots, penetrators)
                model = scene_spec.compile()
        self._model = model
        return None, prefixes

    def _compose_with_static_entities(
        self,
        world: WorldSpec,
        robots: tuple[SimRobotSpec, ...],
        force_static: frozenset[str],
    ) -> mujoco.MjSpec:
        assert world.scene is not None and world.scene.mujoco_scene_path is not None
        scene_spec = mujoco.MjSpec.from_file(str(world.scene.mujoco_scene_path))
        for index, robot in enumerate(robots):
            assert robot.model_path is not None
            robot_spec = mujoco.MjSpec.from_file(str(robot.model_path))
            meshdir = robot.backend_options.get("mujoco_meshdir")
            if meshdir is not None:
                robot_spec.meshdir = str(meshdir)
            if index == 0:
                scene_spec.option.timestep = robot_spec.option.timestep
            x, y, z = robot.spawn.position
            qx, qy, qz, qw = robot.spawn.quaternion_xyzw
            frame = scene_spec.worldbody.add_frame(
                pos=[float(x), float(y), float(z)],
                quat=[float(qw), float(qx), float(qy), float(qz)],
            )
            prefix = f"{robot.robot_id}-" if len(robots) > 1 else None
            scene_spec.attach(robot_spec, prefix=prefix, frame=frame)
        add_scene_package_entities_to_spec(
            scene_spec,
            world.scene.entities,
            force_static=force_static,
        )
        return scene_spec

    def _load_model(self, path: Path) -> mujoco.MjModel:
        if path.suffix.lower() == ".mjb":
            if self.config.assets or self.config.asset_loader:
                raise ValueError("cannot inject assets into a MuJoCo binary")
            return cast(
                "mujoco.MjModel",
                mujoco.MjModel.from_binary_path(str(path)),  # type: ignore[attr-defined]
            )
        assets = self.config.assets
        if assets is None and self.config.asset_loader is not None:
            loader = self.config.asset_loader
            if isinstance(loader, str):
                module_name, separator, attribute = loader.partition(":")
                if not separator:
                    raise ValueError("asset_loader must use the 'module:function' format")
                loader = getattr(importlib.import_module(module_name), attribute)
            assets = loader()
        if assets is not None:
            return mujoco.MjModel.from_xml_string(path.read_text(), assets=assets)
        return mujoco.MjModel.from_xml_path(str(path))

    def _apply_spawn(self, robot: _MujocoRobot) -> None:
        _, data = self._require_model()
        qpos_address = robot.binding.root_qpos_adr
        if qpos_address is None:
            return
        x, y, z = robot.spec.spawn.position
        qx, qy, qz, qw = robot.spec.spawn.quaternion_xyzw
        data.qpos[qpos_address : qpos_address + 7] = [x, y, z, qw, qx, qy, qz]
        if robot.binding.root_qvel_adr is not None:
            data.qvel[robot.binding.root_qvel_adr : robot.binding.root_qvel_adr + 6] = 0.0

    def _write_actuators(self, robot: _MujocoRobot, values: np.ndarray[Any, Any]) -> None:
        _, data = self._require_model()
        if len(values) != robot.spec.dof:
            raise ValueError(f"action size does not match robot '{robot.spec.robot_id}' DOF")
        data.ctrl[list(robot.binding.actuator_ids)] = values

    def _apply_whole_body_pd(self, robot: _MujocoRobot) -> None:
        _, data = self._require_model()
        action = robot.whole_body_action
        if action is None:
            return
        q = data.qpos[list(robot.binding.joint_qpos_adrs)]
        dq = data.qvel[list(robot.binding.joint_qvel_adrs)]
        torque = (
            action["kp"] * (action["position"] - q)
            + action["kd"] * (action["velocity"] - dq)
            + action["effort"]
        )
        self._write_actuators(robot, torque)

    def _configure_lidars(self) -> None:
        model, _ = self._require_model()
        self._lidars = []
        for robot in self._robots.values():
            prefix = robot.binding.model_prefix or ""
            for spec in robot.spec.sensors:
                if spec.implementation != SensorImplementation.NATIVE:
                    continue
                if not spec.camera_names:
                    raise ValueError(
                        f"native lidar '{spec.sensor_id}' on '{robot.spec.robot_id}' "
                        "requires at least one MuJoCo camera name"
                    )
                camera_ids: list[int] = []
                directions: list[np.ndarray[Any, Any]] = []
                for camera_name in spec.camera_names:
                    camera_id = self._resolve_model_object_id(
                        mujoco.mjtObj.mjOBJ_CAMERA,
                        camera_name,
                        prefix,
                    )
                    if camera_id is None:
                        raise ValueError(
                            f"native lidar '{spec.sensor_id}' camera not found: "
                            f"{prefix}{camera_name}"
                        )
                    camera_ids.append(camera_id)
                    directions.append(
                        _camera_ray_directions(
                            spec.width,
                            spec.height,
                            float(model.cam_fovy[camera_id]),
                        )
                    )
                geom_groups = np.zeros(6, dtype=np.uint8)
                if spec.geom_groups:
                    for group in spec.geom_groups:
                        if 0 <= group < len(geom_groups):
                            geom_groups[group] = 1
                else:
                    geom_groups[:] = 1
                self._lidars.append(
                    _MujocoLidar(
                        robot=robot,
                        spec=spec,
                        camera_ids=tuple(camera_ids),
                        ray_directions_camera=tuple(directions),
                        geom_groups=geom_groups,
                    )
                )

    def _resolve_model_object_id(
        self,
        object_type: int,
        name: str,
        prefix: str,
    ) -> int | None:
        model, _ = self._require_model()
        raw = name.lstrip("/")
        prefixed = f"{prefix}{raw}"
        candidates = tuple(dict.fromkeys((name, raw, f"/{raw}", prefixed, f"/{prefixed}")))
        matches = {
            int(object_id)
            for candidate in candidates
            if (object_id := mujoco.mj_name2id(model, object_type, candidate)) >= 0
        }
        if len(matches) > 1:
            matched_names = sorted(
                mujoco.mj_id2name(model, object_type, object_id) or str(object_id)
                for object_id in matches
            )
            raise ValueError(f"ambiguous MuJoCo object '{name}': {matched_names}")
        return next(iter(matches), None)

    def _raycast_lidar(self, lidar: _MujocoLidar) -> np.ndarray[Any, Any]:
        model, data = self._require_model()
        arrays: list[np.ndarray[Any, Any]] = []
        body_exclude = lidar.robot.binding.root_body_id
        for camera_id, directions_camera in zip(
            lidar.camera_ids,
            lidar.ray_directions_camera,
            strict=True,
        ):
            origin = data.cam_xpos[camera_id].copy()
            camera_matrix = data.cam_xmat[camera_id].reshape(3, 3).copy()
            directions_world = directions_camera @ camera_matrix.T
            ray_count = directions_world.shape[0]
            geom_ids = np.full(ray_count, -1, dtype=np.int32)
            distances = np.full(ray_count, -1.0, dtype=np.float64)
            mujoco.mj_multiRay(  # type: ignore[attr-defined]
                model,
                data,
                origin,
                directions_world.ravel(),
                lidar.geom_groups,
                1,
                body_exclude if body_exclude is not None else -1,
                geom_ids,
                distances,
                None,
                ray_count,
                lidar.spec.max_range,
            )
            valid = (distances >= lidar.spec.min_range) & (distances <= lidar.spec.max_range)
            valid &= np.abs(directions_camera[:, 1] * distances) <= lidar.spec.max_height
            if not np.any(valid):
                continue
            points = origin + directions_world[valid] * distances[valid, None]
            if lidar.spec.robot_exclusion_radius > 0.0:
                root_position = self._root_state(lidar.robot)[0]
                keep = (
                    np.linalg.norm(points[:, :2] - root_position[:2], axis=1)
                    >= lidar.spec.robot_exclusion_radius
                )
                points = points[keep]
            if points.size:
                arrays.append(points.astype(np.float32))
        if not arrays:
            return np.empty((0, 3), dtype=np.float32)
        return np.vstack(arrays)

    def _write_gripper(self, robot: _MujocoRobot, target: float) -> None:
        model, data = self._require_model()
        actuator_id = robot.gripper_actuator_id
        joint_id = robot.gripper_joint_id
        if actuator_id is None or joint_id is None:
            return
        joint_low, joint_high = model.jnt_range[joint_id]
        control_low, control_high = model.actuator_ctrlrange[actuator_id]
        clamped = max(float(joint_low), min(float(joint_high), target))
        fraction = (
            0.0 if joint_high == joint_low else (clamped - joint_low) / (joint_high - joint_low)
        )
        if robot.spec.backend_options.get("gripper_reversed", False):
            fraction = 1.0 - fraction
        data.ctrl[actuator_id] = control_low + fraction * (control_high - control_low)

    def _body_entity_state(self, entity_id: str, body_id: int) -> EntityState:
        _, data = self._require_model()
        quaternion_wxyz = data.xquat[body_id]
        spatial_velocity = data.cvel[body_id]
        return EntityState(
            entity_id=entity_id,
            position=(
                float(data.xpos[body_id][0]),
                float(data.xpos[body_id][1]),
                float(data.xpos[body_id][2]),
            ),
            quaternion_xyzw=(
                float(quaternion_wxyz[1]),
                float(quaternion_wxyz[2]),
                float(quaternion_wxyz[3]),
                float(quaternion_wxyz[0]),
            ),
            linear_velocity=(
                float(spatial_velocity[3]),
                float(spatial_velocity[4]),
                float(spatial_velocity[5]),
            ),
            angular_velocity=(
                float(spatial_velocity[0]),
                float(spatial_velocity[1]),
                float(spatial_velocity[2]),
            ),
        )

    def _root_state(
        self,
        robot: _MujocoRobot,
    ) -> tuple[
        np.ndarray[Any, Any], np.ndarray[Any, Any], np.ndarray[Any, Any], np.ndarray[Any, Any]
    ]:
        _, data = self._require_model()
        if robot.binding.root_qpos_adr is None:
            return (
                np.asarray(robot.spec.spawn.position, dtype=np.float64),
                np.asarray(robot.spec.spawn.quaternion_xyzw, dtype=np.float64),
                np.zeros(3),
                np.zeros(3),
            )
        qpos = robot.binding.root_qpos_adr
        qvel = robot.binding.root_qvel_adr
        quaternion_wxyz = data.qpos[qpos + 3 : qpos + 7]
        return (
            data.qpos[qpos : qpos + 3].copy(),
            quaternion_wxyz[[1, 2, 3, 0]].copy(),
            data.qvel[qvel : qvel + 3].copy() if qvel is not None else np.zeros(3),
            data.qvel[qvel + 3 : qvel + 6].copy() if qvel is not None else np.zeros(3),
        )

    def _sensor_or_default(
        self,
        sensor_slice: slice | None,
        default: np.ndarray[Any, Any],
    ) -> np.ndarray[Any, Any]:
        _, data = self._require_model()
        return data.sensordata[sensor_slice].copy() if sensor_slice is not None else default.copy()

    def _optional_id(self, object_type: Any, name: Any) -> int | None:
        if name is None:
            return None
        model, _ = self._require_model()
        identifier = int(mujoco.mj_name2id(model, object_type, str(name)))
        if identifier < 0:
            raise ValueError(f"MuJoCo object '{name}' not found")
        return identifier

    def _require_model(self) -> tuple[mujoco.MjModel, mujoco.MjData]:
        if self._model is None or self._data is None:
            raise RuntimeError("MuJoCo backend is not loaded")
        return self._model, self._data

    @staticmethod
    def _robot(handle: RobotHandle) -> _MujocoRobot:
        if not isinstance(handle.backend_data, _MujocoRobot):
            raise ValueError(f"invalid MuJoCo handle for robot '{handle.robot_id}'")
        return handle.backend_data


def _rpy_from_xyzw(quaternion: np.ndarray[Any, Any]) -> np.ndarray[Any, Any]:
    x, y, z, w = quaternion
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return np.array([roll, pitch, yaw], dtype=np.float64)


def _camera_ray_directions(
    width: int,
    height: int,
    fovy_degrees: float,
) -> np.ndarray[Any, Any]:
    fovy = math.radians(fovy_degrees)
    focal = height / (2.0 * math.tan(fovy / 2.0))
    ys, xs = np.mgrid[0:height, 0:width]
    x = (xs + 0.5 - width / 2.0) / focal
    y = -(ys + 0.5 - height / 2.0) / focal
    z = -np.ones_like(x)
    directions = np.stack((x, y, z), axis=-1).reshape(-1, 3).astype(np.float64)
    return cast(
        "np.ndarray[Any, Any]",
        directions / np.linalg.norm(directions, axis=1, keepdims=True),
    )
