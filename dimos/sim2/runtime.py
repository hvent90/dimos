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

"""Simulation clock, backend, and robot-channel orchestration."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import asdict, dataclass
import threading
import time
from typing import Any
import uuid

import numpy as np

from dimos.sim2.backend.base import (
    RobotHandle,
    RobotPoseBackend,
    SceneAuthoringBackend,
    SensorSample,
    SimBackend,
)
from dimos.sim2.ipc.abi import ChannelDescriptor, make_channel_descriptor
from dimos.sim2.ipc.channel import FrameMetadata, RobotChannel
from dimos.sim2.ipc.control import SimControlServer
from dimos.sim2.ipc.registry import (
    SimRegistry,
    control_socket_path,
    shared_memory_name,
)
from dimos.sim2.spec import (
    ControlInterface,
    EntityDescriptor,
    ExecutionMode,
    SimConfig,
    SpawnPose,
    WorldManifest,
    WorldStateFrame,
    scene_entity_descriptors,
)


@dataclass(frozen=True)
class RuntimeStatus:
    sim_id: str
    generation: str
    episode_id: int
    physics_tick: int
    control_tick: int
    sim_time: float
    running: bool
    fault: str | None


class SimRuntime:
    def __init__(
        self,
        config: SimConfig,
        registry: SimRegistry | None = None,
        world_state_callback: Callable[[WorldStateFrame], None] | None = None,
        observation_callback: Callable[[str, dict[str, Any], FrameMetadata], None] | None = None,
        sensor_callback: Callable[[SensorSample, FrameMetadata], None] | None = None,
    ) -> None:
        if not isinstance(config.backend, SimBackend):
            raise TypeError("SimConfig.backend must implement SimBackend")
        self.config = config
        self.backend = config.backend
        self.registry = registry or SimRegistry()
        self._world_state_callback = world_state_callback
        self._observation_callback = observation_callback
        self._sensor_callback = sensor_callback
        self.generation = uuid.uuid4().hex
        self._handles: dict[str, RobotHandle] = {}
        self._descriptors: dict[str, ChannelDescriptor] = {}
        self._channels: dict[str, RobotChannel] = {}
        self._server: SimControlServer | None = None
        self._episode_id = 0
        self._physics_tick = 0
        self._control_tick = 0
        self._sim_time = 0.0
        self._last_action_sequence: dict[str, int] = {}
        self._last_action_wall_time: dict[str, float] = {}
        self._sensor_sequences: dict[str, int] = {}
        self._authored_entities: dict[str, EntityDescriptor] = {}
        self._last_world_state: WorldStateFrame | None = None
        self._fault: str | None = None
        self._controller_seen = False
        self._running = False
        self._stop = threading.Event()
        self._run_condition = threading.Condition()
        self._mutation_lock = threading.RLock()
        self._thread: threading.Thread | None = None

    @property
    def channels(self) -> dict[str, RobotChannel]:
        return dict(self._channels)

    @property
    def world_state(self) -> WorldStateFrame | None:
        return self._last_world_state

    @property
    def world_manifest(self) -> WorldManifest:
        world_entities = list(self.config.world.entities)
        if self.config.world.scene is not None:
            known_ids = {entity.entity_id for entity in world_entities}
            world_entities.extend(
                entity
                for entity in scene_entity_descriptors(self.config.world.scene)
                if entity.entity_id not in known_ids
            )
        entities = (
            tuple(
                EntityDescriptor(
                    entity_id=robot.robot_id,
                    kind="dynamic",
                    backend_name=robot.robot_id,
                )
                for robot in self.config.robots
            )
            + tuple(world_entities)
            + tuple(self._authored_entities.values())
        )
        ids = [entity.entity_id for entity in entities]
        if len(ids) != len(set(ids)):
            raise ValueError("robot and world entity IDs must be globally unique")
        return WorldManifest(
            scene_revision=self.config.world.revision,
            frame_id="world",
            entities=entities,
        )

    def start(self) -> None:
        if self._channels:
            raise RuntimeError("sim2 runtime is already started")
        unsupported = [
            robot.robot_id
            for robot in self.config.robots
            if robot.control_interface not in self.backend.capabilities
        ]
        if unsupported:
            raise ValueError(f"backend lacks required capabilities for robots: {unsupported}")
        self._handles = self.backend.load(
            self.config.world,
            self.config.robots,
            self.config.execution.physics_dt,
        )
        if set(self._handles) != {robot.robot_id for robot in self.config.robots}:
            raise ValueError("backend returned handles for the wrong robot set")

        socket_path = control_socket_path(
            self.registry.run_id,
            self.config.sim_id,
            self.generation,
        )
        self._server = SimControlServer(socket_path)
        self._server.start()
        try:
            for robot in self.config.robots:
                descriptor = make_channel_descriptor(
                    sim_id=self.config.sim_id,
                    robot_id=robot.robot_id,
                    generation=self.generation,
                    shm_name=shared_memory_name(
                        self.registry.run_id,
                        self.config.sim_id,
                        robot.robot_id,
                        self.generation,
                    ),
                    control_interface=robot.control_interface,
                    dof=robot.dof,
                    capabilities=tuple(sorted(robot.capabilities)),
                    physics_dt=self.config.execution.physics_dt,
                    control_decimation=self.config.execution.control_decimation,
                )
                self._descriptors[robot.robot_id] = descriptor
                self._channels[robot.robot_id] = RobotChannel.create(descriptor)
            self.registry.publish(
                self.config.sim_id,
                self.generation,
                self._descriptors,
                socket_path=str(socket_path),
            )
            self.reset()
            if self.config.execution.autostart:
                self.run()
        except Exception:
            self.close()
            raise

    def reset(self, seed: int | None = None) -> WorldStateFrame:
        with self._mutation_lock:
            was_running = self._running
            self.pause()
            self.backend.reset(seed)
            frame = self._begin_episode()
            if was_running:
                self.run()
            return frame

    def respawn(self, robot_id: str, pose: SpawnPose, seed: int | None = None) -> WorldStateFrame:
        if not isinstance(self.backend, RobotPoseBackend):
            raise NotImplementedError(
                f"{type(self.backend).__name__} does not support robot respawn"
            )
        try:
            handle = self._handles[robot_id]
        except KeyError as error:
            raise ValueError(f"unknown robot '{robot_id}'") from error
        with self._mutation_lock:
            was_running = self._running
            self.pause()
            self.backend.reset(seed)
            self.backend.set_robot_pose(handle, pose)
            frame = self._begin_episode()
            if was_running:
                self.run()
            return frame

    def add_wall(
        self,
        x1: float,
        y1: float,
        x2: float,
        y2: float,
        height: float = 1.5,
        thickness: float = 0.1,
    ) -> EntityDescriptor:
        if not isinstance(self.backend, SceneAuthoringBackend):
            raise NotImplementedError(
                f"{type(self.backend).__name__} has a compiled world and does not support "
                "runtime wall authoring"
            )
        with self._mutation_lock:
            descriptor = self.backend.add_wall(x1, y1, x2, y2, height, thickness)
            if descriptor.entity_id in {item.entity_id for item in self.world_manifest.entities}:
                raise ValueError(f"duplicate authored entity ID '{descriptor.entity_id}'")
            self._authored_entities[descriptor.entity_id] = descriptor
            self._publish_world_state(include_sensors=False)
            return descriptor

    def _begin_episode(self) -> WorldStateFrame:
        self._episode_id += 1
        self._physics_tick = 0
        self._control_tick = 0
        self._sim_time = 0.0
        self._fault = None
        self._last_action_sequence.clear()
        self._last_action_wall_time.clear()
        self._sensor_sequences.clear()
        for channel in self._channels.values():
            channel.reset_frames(self._episode_id)
            channel.set_lifecycle("ready")
        self._publish_observations(reset=True)
        assert self._last_world_state is not None
        return self._last_world_state

    def run(self) -> None:
        if self._fault is not None:
            raise RuntimeError(f"cannot run faulted simulation: {self._fault}")
        with self._run_condition:
            self._running = True
            self._run_condition.notify_all()
        if self._thread is None or not self._thread.is_alive():
            self._stop.clear()
            self._thread = threading.Thread(target=self._loop, name="sim2-runtime", daemon=True)
            self._thread.start()

    def pause(self) -> None:
        with self._run_condition:
            self._running = False

    def step(self, control_ticks: int = 1) -> WorldStateFrame:
        if control_ticks < 1:
            raise ValueError("control_ticks must be at least 1")
        if self._running:
            raise RuntimeError("pause the simulation before calling step")
        with self._mutation_lock:
            for _ in range(control_ticks):
                self._advance_control_interval()
        assert self._last_world_state is not None
        return self._last_world_state

    def status(self) -> RuntimeStatus:
        return RuntimeStatus(
            sim_id=self.config.sim_id,
            generation=self.generation,
            episode_id=self._episode_id,
            physics_tick=self._physics_tick,
            control_tick=self._control_tick,
            sim_time=self._sim_time,
            running=self._running,
            fault=self._fault,
        )

    def describe(self) -> dict[str, Any]:
        return {
            "status": asdict(self.status()),
            "world_manifest": {
                "scene_revision": self.world_manifest.scene_revision,
                "frame_id": self.world_manifest.frame_id,
                "entities": [entity.to_wire() for entity in self.world_manifest.entities],
            },
            "channels": {
                robot_id: descriptor.to_dict() for robot_id, descriptor in self._descriptors.items()
            },
        }

    def _loop(self) -> None:
        while not self._stop.is_set():
            with self._run_condition:
                self._run_condition.wait_for(lambda: self._running or self._stop.is_set())
            if self._stop.is_set():
                return
            started = time.perf_counter()
            try:
                with self._mutation_lock:
                    self._advance_control_interval()
            except Exception as exc:
                self._fault = str(exc)
                self._running = False
                for channel in self._channels.values():
                    channel.set_lifecycle("faulted")
                continue
            target = self.config.execution.control_dt / self.config.execution.realtime_factor
            remaining = target - (time.perf_counter() - started)
            if remaining > 0.0:
                self._stop.wait(remaining)

    def _advance_control_interval(self) -> None:
        if self.config.execution.mode == ExecutionMode.LOCKSTEP:
            assert self._server is not None
            if not self._controller_seen:
                if not self._server.wait_for_controller():
                    raise RuntimeError("simulation stopped before a controller connected")
                self._controller_seen = True
            if not self._server.wait_for_action(
                self._episode_id,
                self._control_tick,
                self.config.execution.action_timeout,
            ):
                raise TimeoutError(
                    f"timed out waiting for action at episode={self._episode_id}, "
                    f"control_tick={self._control_tick}"
                )

        actions = self._read_actions()
        for robot_id, handle in self._handles.items():
            self.backend.apply_action(handle, actions[robot_id])
        for _ in range(self.config.execution.control_decimation):
            self.backend.step(self.config.execution.physics_dt)
            self._physics_tick += 1
            self._sim_time = self._physics_tick * self.config.execution.physics_dt
        self._control_tick += 1
        self._publish_observations()

    def _read_actions(self) -> dict[str, dict[str, Any]]:
        now = time.monotonic()
        actions: dict[str, dict[str, Any]] = {}
        for robot in self.config.robots:
            frame = self._channels[robot.robot_id].read_action()
            valid = frame is not None and frame.metadata.episode_id == self._episode_id
            if self.config.execution.mode == ExecutionMode.LOCKSTEP:
                valid = (
                    valid
                    and frame is not None
                    and frame.metadata.control_tick == self._control_tick
                )
            if valid and frame is not None:
                previous_sequence = self._last_action_sequence.get(robot.robot_id, 0)
                if frame.metadata.sequence > previous_sequence:
                    self._last_action_sequence[robot.robot_id] = frame.metadata.sequence
                    self._last_action_wall_time[robot.robot_id] = now
                last_time = self._last_action_wall_time.get(robot.robot_id)
                if self.config.execution.mode == ExecutionMode.LIVE and (
                    last_time is None or now - last_time > self.config.execution.action_timeout
                ):
                    actions[robot.robot_id] = self._safe_action(
                        robot.control_interface,
                        robot.dof,
                    )
                else:
                    actions[robot.robot_id] = frame.values
                continue
            last_time = self._last_action_wall_time.get(robot.robot_id)
            if self.config.execution.mode == ExecutionMode.LOCKSTEP:
                raise RuntimeError(
                    f"missing action frame for robot '{robot.robot_id}' at "
                    f"control_tick={self._control_tick}"
                )
            if last_time is None or now - last_time > self.config.execution.action_timeout:
                actions[robot.robot_id] = self._safe_action(robot.control_interface, robot.dof)
            elif frame is not None:
                actions[robot.robot_id] = frame.values
            else:
                actions[robot.robot_id] = self._safe_action(robot.control_interface, robot.dof)
        return actions

    def _publish_observations(self, *, reset: bool = False) -> None:
        for robot_id, handle in self._handles.items():
            channel = self._channels[robot_id]
            observation = self.backend.observe(handle)
            metadata = FrameMetadata(
                sequence=0,
                episode_id=self._episode_id,
                physics_tick=self._physics_tick,
                control_tick=self._control_tick,
                sim_time=self._sim_time,
                applied_action_sequence=self._last_action_sequence.get(robot_id, 0),
            )
            channel.publish_observation(observation, metadata)
            if self._observation_callback is not None:
                self._observation_callback(robot_id, observation, metadata)
        self._publish_world_state(include_sensors=True)
        if self._server is not None:
            self._server.publish_observation(
                episode_id=self._episode_id,
                control_tick=self._control_tick,
                sim_time=self._sim_time,
                control_dt=self.config.execution.control_dt,
                reset=reset,
            )

    def _publish_world_state(self, *, include_sensors: bool) -> None:
        self._last_world_state = WorldStateFrame(
            episode_id=self._episode_id,
            physics_tick=self._physics_tick,
            control_tick=self._control_tick,
            sim_time=self._sim_time,
            scene_revision=self.config.world.revision,
            entities=tuple(self.backend.entity_states()),
        )
        if self._world_state_callback is not None:
            self._world_state_callback(self._last_world_state)
        if include_sensors and self._sensor_callback is not None:
            for sample in self.backend.sensor_samples(self._sim_time):
                sequence = self._sensor_sequences.get(sample.sensor_id, 0) + 1
                self._sensor_sequences[sample.sensor_id] = sequence
                self._sensor_callback(
                    sample,
                    FrameMetadata(
                        sequence=sequence,
                        episode_id=self._episode_id,
                        physics_tick=self._physics_tick,
                        control_tick=self._control_tick,
                        sim_time=self._sim_time,
                    ),
                )

    @staticmethod
    def _safe_action(interface: ControlInterface, dof: int) -> dict[str, Any]:
        zeros = np.zeros(dof, dtype=np.float64)
        if interface == ControlInterface.TWIST_BASE:
            return {"enabled": np.array([0], dtype=np.uint8), "velocities": zeros}
        if interface == ControlInterface.MANIPULATOR:
            return {
                "command_mode": np.array([0], dtype=np.int32),
                "enabled": np.array([0], dtype=np.uint8),
                "position": zeros.copy(),
                "velocity": zeros.copy(),
                "effort": zeros.copy(),
                "velocity_scale": np.array([0.0]),
                "gripper": np.array([0.0]),
            }
        return {
            "enabled": np.array([0], dtype=np.uint8),
            "position": zeros.copy(),
            "velocity": zeros.copy(),
            "kp": zeros.copy(),
            "kd": zeros.copy(),
            "effort": zeros.copy(),
        }

    def close(self) -> None:
        self._stop.set()
        with self._run_condition:
            self._running = False
            self._run_condition.notify_all()
        if self._server is not None:
            self._server.close()
            self._server = None
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        for channel in self._channels.values():
            channel.set_lifecycle("closed")
            channel.close()
            channel.unlink()
        self._channels.clear()
        self.registry.remove(self.config.sim_id, self.generation)
        self.backend.close()

    def __enter__(self) -> SimRuntime:
        self.start()
        return self

    def __exit__(self, *_: Any) -> None:
        self.close()
