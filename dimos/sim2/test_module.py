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

from dimos.sim2.backend.kinematic import KinematicBackend
from dimos.sim2.ipc.channel import FrameMetadata
from dimos.sim2.module import SimModule
from dimos.sim2.spec import (
    ControlInterface,
    EntityState,
    ExecutionConfig,
    SceneControl,
    SensorReady,
    SimConfig,
    SimRobotSpec,
    WorldStateFrame,
)


def test_sim_module_owns_runtime_and_tracks_sensor_ticks() -> None:
    module = SimModule(
        sim=SimConfig(
            sim_id="module-test",
            backend=KinematicBackend(),
            robots=(SimRobotSpec("base", ControlInterface.TWIST_BASE, 3),),
            execution=ExecutionConfig(autostart=False),
        )
    )
    frames = []
    manifests = []
    odometry = []
    unsubscribe = module.world_state.subscribe(frames.append)
    unsubscribe_manifest = module.world_manifest.subscribe(manifests.append)
    unsubscribe_odom = module.odom.subscribe(odometry.append)
    module.start()
    try:
        frame = module.step()
        module._on_sensor_ready(
            SensorReady(
                sensor_id="lidar",
                episode_id=frame.episode_id,
                source_tick=frame.physics_tick,
                sim_time=frame.sim_time,
                sequence=1,
            )
        )

        module._wait_for_sensors(frozenset({"lidar"}), frame, timeout=0.0)

        assert len(frames) == 2
        assert frames[-1] == frame
        assert [entity.entity_id for entity in manifests[0].entities] == ["base"]
        assert odometry[-1].frame_id == "world"
        assert odometry[-1].ts == pytest.approx(frame.sim_time)
        assert module.status()["control_tick"] == 1
    finally:
        unsubscribe()
        unsubscribe_manifest()
        unsubscribe_odom()
        module.stop()


def test_sim_module_publishes_typed_imu_from_primary_robot_observation() -> None:
    module = SimModule(
        sim=SimConfig(
            sim_id="imu-test",
            backend=KinematicBackend(),
            robots=(SimRobotSpec("base", ControlInterface.TWIST_BASE, 3),),
            execution=ExecutionConfig(autostart=False),
        )
    )
    messages = []
    unsubscribe = module.imu.subscribe(messages.append)
    try:
        module._on_observation(
            "base",
            {
                "imu_quaternion": np.array([1.0, 0.0, 0.0, 0.0]),
                "imu_gyroscope": np.array([1.0, 2.0, 3.0]),
                "imu_accelerometer": np.array([0.0, 0.0, 9.81]),
            },
            FrameMetadata(0, 1, 4, 1, 0.02),
        )

        assert len(messages) == 1
        assert messages[0].orientation.to_tuple() == (0.0, 0.0, 0.0, 1.0)
        assert messages[0].angular_velocity.to_list() == [1.0, 2.0, 3.0]
        assert messages[0].ts == pytest.approx(0.02)
    finally:
        unsubscribe()
        module.stop()


def test_sim_module_rejects_stale_sensor_sample() -> None:
    module = SimModule(
        sim=SimConfig(
            sim_id="sensor-test",
            backend=KinematicBackend(),
            robots=(SimRobotSpec("base", ControlInterface.TWIST_BASE, 3),),
            execution=ExecutionConfig(autostart=False),
        )
    )
    module.start()
    try:
        frame = module.step()
        module._on_sensor_ready(
            SensorReady("lidar", frame.episode_id, frame.physics_tick - 1, 0.0, 1)
        )

        with pytest.raises(TimeoutError, match="lidar"):
            module._wait_for_sensors(frozenset({"lidar"}), frame, timeout=0.0)
    finally:
        module.stop()


def test_external_sensor_contracts_have_versioned_lcm_wire_format() -> None:
    frame = WorldStateFrame(
        episode_id=2,
        physics_tick=20,
        control_tick=4,
        sim_time=0.04,
        scene_revision="office-123",
        entities=(
            EntityState(
                entity_id="robot",
                position=(1.0, 2.0, 3.0),
                quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
            ),
        ),
    )
    ready = SensorReady("lidar", 2, 20, 0.04, 7)

    assert WorldStateFrame.lcm_decode(frame.lcm_encode()) == frame
    assert SensorReady.lcm_decode(ready.lcm_encode()) == ready


def test_sim_module_exposes_backend_neutral_scenario_verbs() -> None:
    module = SimModule(
        sim=SimConfig(
            sim_id="scenario-test",
            backend=KinematicBackend(),
            robots=(SimRobotSpec("base", ControlInterface.TWIST_BASE, 3),),
            execution=ExecutionConfig(autostart=False),
        )
    )
    manifests = []
    goals = []
    unsubscribe_manifest = module.world_manifest.subscribe(manifests.append)
    unsubscribe_goal = module.goal_request.subscribe(goals.append)
    module.start()
    try:
        module.set_agent_position(1.0, 2.0, 0.1)
        module.add_wall(0.0, 0.0, 2.0, 0.0)
        module.publish_goal(4.0, 5.0)

        state = module._require_runtime().world_state
        assert isinstance(module, SceneControl)
        assert state is not None
        assert state.entities[0].position == pytest.approx((1.0, 2.0, 0.1))
        assert [item.entity_id for item in manifests[-1].entities] == ["base", "wall-1"]
        assert goals[0].position.to_tuple() == (4.0, 5.0, 0.0)
        assert goals[0].frame_id == "world"
    finally:
        unsubscribe_manifest()
        unsubscribe_goal()
        module.stop()
