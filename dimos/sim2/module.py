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

"""The single DimOS Module that owns a sim2 runtime."""

from __future__ import annotations

from collections.abc import Callable
import math
import threading
import time
from typing import Any, cast

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.sim2.backend.base import SensorSample
from dimos.sim2.ipc.channel import FrameMetadata
from dimos.sim2.runtime import SimRuntime
from dimos.sim2.spec import SensorReady, SimConfig, SpawnPose, WorldManifest, WorldStateFrame


class SimModuleConfig(ModuleConfig):
    sim: SimConfig


class SimModule(Module):
    dedicated_worker = True
    manifest_publish_interval = 1.0

    config: SimModuleConfig

    world_state: Out[WorldStateFrame]
    world_manifest: Out[WorldManifest]
    odom: Out[PoseStamped]
    imu: Out[Imu]
    pointcloud: Out[PointCloud2]
    goal_request: Out[PoseStamped]
    sensor_ready: In[SensorReady]

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._runtime: SimRuntime | None = None
        self._sensor_status: dict[str, SensorReady] = {}
        self._sensor_condition = threading.Condition()
        self._sensor_unsubscribe: Callable[[], None] | None = None
        self._last_manifest_sim_time: float | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self._last_manifest_sim_time = None
        if self.sensor_ready._transport is not None:
            self._sensor_unsubscribe = self.sensor_ready.subscribe(self._on_sensor_ready)
        self._runtime = SimRuntime(
            self.config.sim,
            world_state_callback=self._on_world_state,
            observation_callback=self._on_observation,
            sensor_callback=self._on_sensor_sample,
        )
        self._runtime.start()
        self._publish_world_manifest(0.0, force=True)

    @rpc
    def describe(self) -> dict[str, Any]:
        return self._require_runtime().describe()

    @rpc
    def status(self) -> dict[str, Any]:
        return cast("dict[str, Any]", self._require_runtime().describe()["status"])

    @rpc
    def reset(self, seed: int | None = None) -> WorldStateFrame:
        with self._sensor_condition:
            self._sensor_status.clear()
        return self._require_runtime().reset(seed)

    @rpc
    def respawn_at(
        self,
        x: float,
        y: float,
        z: float = 0.0,
        yaw: float = 0.0,
        robot_id: str = "",
    ) -> WorldStateFrame:
        selected_robot = robot_id or self.config.sim.primary_robot
        robot = next(
            (item for item in self.config.sim.robots if item.robot_id == selected_robot),
            None,
        )
        if robot is None:
            raise ValueError(f"unknown robot '{selected_robot}'")
        pose = SpawnPose(
            position=(x, y, robot.spawn.position[2] + z),
            quaternion_xyzw=(0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)),
        )
        return self._require_runtime().respawn(selected_robot, pose)

    @rpc
    def set_agent_position(self, x: float, y: float, z: float = 0.0) -> None:
        self.respawn_at(x, y, z)

    @rpc
    def add_wall(
        self,
        x1: float,
        y1: float,
        x2: float,
        y2: float,
        height: float = 1.5,
        thickness: float = 0.1,
    ) -> None:
        self._require_runtime().add_wall(x1, y1, x2, y2, height, thickness)
        self._publish_world_manifest(self._require_runtime().status().sim_time, force=True)

    @rpc
    def publish_goal(self, x: float, y: float) -> None:
        self.goal_request.publish(
            PoseStamped(
                position=(x, y, 0.0),
                orientation=(0.0, 0.0, 0.0, 1.0),
                frame_id="world",
            )
        )

    @rpc
    def pause(self) -> None:
        self._require_runtime().pause()

    @rpc
    def run(self) -> None:
        self._require_runtime().run()

    @rpc
    def step(
        self,
        control_ticks: int = 1,
        await_sensors: list[str] | None = None,
        sensor_timeout: float = 5.0,
    ) -> WorldStateFrame:
        frame = self._require_runtime().step(control_ticks)
        required = frozenset(await_sensors or ())
        if required:
            self._wait_for_sensors(required, frame, sensor_timeout)
        return frame

    @rpc
    def stop(self) -> None:
        if self._sensor_unsubscribe is not None:
            self._sensor_unsubscribe()
            self._sensor_unsubscribe = None
        if self._runtime is not None:
            self._runtime.close()
            self._runtime = None
        super().stop()

    def _on_sensor_ready(self, status: SensorReady) -> None:
        with self._sensor_condition:
            current = self._sensor_status.get(status.sensor_id)
            if current is None or (status.episode_id, status.source_tick, status.sequence) > (
                current.episode_id,
                current.source_tick,
                current.sequence,
            ):
                self._sensor_status[status.sensor_id] = status
            self._sensor_condition.notify_all()

    def _on_world_state(self, frame: WorldStateFrame) -> None:
        self._publish_world_manifest(frame.sim_time)
        self.world_state.publish(frame)
        state = next(
            (
                entity
                for entity in frame.entities
                if entity.entity_id == self.config.sim.primary_robot
            ),
            None,
        )
        if state is None:
            return
        self.odom.publish(
            PoseStamped(
                ts=frame.sim_time if frame.sim_time > 0.0 else 1e-12,
                frame_id=state.frame_id,
                position=Vector3(state.position),
                orientation=Quaternion(state.quaternion_xyzw),
            )
        )

    def _publish_world_manifest(self, sim_time: float, *, force: bool = False) -> None:
        last = self._last_manifest_sim_time
        if not force and last is not None and sim_time >= last:
            if sim_time - last < self.manifest_publish_interval:
                return
        runtime = self._runtime
        if runtime is None:
            return
        self.world_manifest.publish(runtime.world_manifest)
        self._last_manifest_sim_time = sim_time

    def _on_observation(
        self,
        robot_id: str,
        observation: dict[str, Any],
        metadata: FrameMetadata,
    ) -> None:
        if robot_id != self.config.sim.primary_robot or "imu_quaternion" not in observation:
            return
        w, x, y, z = (float(item) for item in observation["imu_quaternion"])
        self.imu.publish(
            Imu(
                orientation=Quaternion(x, y, z, w),
                angular_velocity=Vector3(observation["imu_gyroscope"]),
                linear_acceleration=Vector3(observation["imu_accelerometer"]),
                frame_id=f"{robot_id}/imu",
                ts=metadata.sim_time,
            )
        )

    def _on_sensor_sample(self, sample: SensorSample, metadata: FrameMetadata) -> None:
        if sample.robot_id != self.config.sim.primary_robot:
            return
        points = sample.payload.get("points")
        if points is None:
            return
        cloud = PointCloud2.from_numpy(
            points,
            frame_id=sample.frame_id,
            timestamp=metadata.sim_time,
        )
        voxel_size = float(sample.payload.get("voxel_size", 0.0))
        if voxel_size > 0.0 and len(cloud) > 0:
            cloud = cloud.voxel_downsample(voxel_size)
        self.pointcloud.publish(cloud)
        self._on_sensor_ready(
            SensorReady(
                sensor_id=sample.sensor_id,
                episode_id=metadata.episode_id,
                source_tick=metadata.physics_tick,
                sim_time=metadata.sim_time,
                sequence=metadata.sequence,
            )
        )

    def _wait_for_sensors(
        self,
        sensor_ids: frozenset[str],
        frame: WorldStateFrame,
        timeout: float,
    ) -> None:
        deadline = time.monotonic() + timeout

        def ready_for(sensor_id: str) -> bool:
            status = self._sensor_status.get(sensor_id)
            return (
                status is not None
                and status.episode_id == frame.episode_id
                and status.source_tick >= frame.physics_tick
            )

        def ready() -> bool:
            return all(ready_for(sensor_id) for sensor_id in sensor_ids)

        with self._sensor_condition:
            if not self._sensor_condition.wait_for(
                ready, timeout=max(0.0, deadline - time.monotonic())
            ):
                missing = sorted(sensor_id for sensor_id in sensor_ids if not ready_for(sensor_id))
                raise TimeoutError(
                    f"sensors did not reach episode={frame.episode_id}, "
                    f"physics_tick={frame.physics_tick}: {missing}"
                )

    def _require_runtime(self) -> SimRuntime:
        if self._runtime is None:
            raise RuntimeError("SimModule is not started")
        return self._runtime
