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

from pathlib import Path
import threading
import time

import numpy as np
import pytest

from dimos.sim2.backend.kinematic import KinematicBackend
from dimos.sim2.control.tick_source import SimTickSource
from dimos.sim2.ipc.channel import FrameMetadata
from dimos.sim2.ipc.control import SimControlClient
from dimos.sim2.ipc.registry import SimRegistry
from dimos.sim2.runtime import SimRuntime
from dimos.sim2.spec import (
    ControlInterface,
    ExecutionConfig,
    ExecutionMode,
    SimConfig,
    SimRobotSpec,
    SpawnPose,
)


def _config(mode: ExecutionMode, robots: tuple[SimRobotSpec, ...]) -> SimConfig:
    return SimConfig(
        sim_id="test",
        backend=KinematicBackend(),
        robots=robots,
        execution=ExecutionConfig(
            mode=mode,
            physics_dt=0.01,
            control_decimation=2,
            autostart=False,
            action_timeout=1.0,
        ),
    )


def _base(robot_id: str = "go2") -> SimRobotSpec:
    return SimRobotSpec(
        robot_id=robot_id,
        control_interface=ControlInterface.TWIST_BASE,
        dof=3,
        joint_names=(f"{robot_id}/vx", f"{robot_id}/vy", f"{robot_id}/wz"),
    )


@pytest.fixture
def live_runtime(tmp_path: Path) -> SimRuntime:
    runtime = SimRuntime(
        _config(ExecutionMode.LIVE, (_base(),)),
        registry=SimRegistry(run_id="runtime-live", root=tmp_path),
    )
    runtime.start()
    try:
        yield runtime
    finally:
        runtime.close()


def test_live_runtime_applies_action_before_advancing(live_runtime: SimRuntime) -> None:
    channel = live_runtime.channels["go2"]
    channel.publish_action(
        {"enabled": np.array([1], dtype=np.uint8), "velocities": np.array([1.0, 0.0, 0.0])},
        FrameMetadata(0, 1, 0, 0, 0.0),
    )

    world = live_runtime.step(5)
    observation = channel.read_observation()

    assert world.control_tick == 5
    assert world.physics_tick == 10
    assert world.sim_time == pytest.approx(0.1)
    assert world.entities[0].position == pytest.approx((0.1, 0.0, 0.0))
    assert observation is not None
    assert observation.metadata.applied_action_sequence == 1
    assert observation.values["odometry"] == pytest.approx([0.1, 0.0, 0.0])


def test_lockstep_requires_action_for_exact_tick(tmp_path: Path) -> None:
    runtime = SimRuntime(
        _config(ExecutionMode.LOCKSTEP, (_base(),)),
        registry=SimRegistry(run_id="runtime-lockstep", root=tmp_path),
    )
    runtime.start()
    client = SimControlClient(runtime.registry.resolve_socket("test"))
    try:
        initial = client.next_observation(timeout=1.0)
        assert initial["control_tick"] == 0
        channel = runtime.channels["go2"]
        channel.publish_action(
            {
                "enabled": np.array([1], dtype=np.uint8),
                "velocities": np.array([0.5, 0.0, 0.0]),
            },
            FrameMetadata(0, 1, 0, 0, 0.0),
        )
        client.action_ready(1, 0)

        world = runtime.step()
        event = client.next_observation(timeout=1.0)

        assert world.control_tick == 1
        assert world.entities[0].position == pytest.approx((0.01, 0.0, 0.0))
        assert event["episode_id"] == 1
        assert event["control_tick"] == 1
    finally:
        client.close()
        runtime.close()


def test_sim_tick_source_receives_initial_observation_and_acknowledges_it(tmp_path: Path) -> None:
    runtime = SimRuntime(
        _config(ExecutionMode.LOCKSTEP, (_base(),)),
        registry=SimRegistry(run_id="runtime-source", root=tmp_path),
    )
    runtime.start()
    source = SimTickSource("test", registry=runtime.registry)
    source.start()
    stop = threading.Event()
    try:
        tick = source.wait_next(stop)
        assert tick is not None
        assert tick.episode_id == 1
        assert tick.tick == 0
        assert tick.t_now == 0.0
        assert tick.reset

        channel = runtime.channels["go2"]
        channel.publish_action(
            {"enabled": np.array([1], dtype=np.uint8), "velocities": np.zeros(3)},
            FrameMetadata(0, 1, 0, 0, 0.0),
        )
        source.complete(tick)
        assert runtime.step().control_tick == 1
    finally:
        source.stop()
        runtime.close()


def test_multi_robot_channels_do_not_cross_talk(tmp_path: Path) -> None:
    runtime = SimRuntime(
        _config(ExecutionMode.LIVE, (_base("first"), _base("second"))),
        registry=SimRegistry(run_id="runtime-multi", root=tmp_path),
    )
    runtime.start()
    try:
        first = runtime.channels["first"]
        second = runtime.channels["second"]
        first.publish_action(
            {"enabled": np.array([1], dtype=np.uint8), "velocities": np.array([1.0, 0.0, 0.0])},
            FrameMetadata(0, 1, 0, 0, 0.0),
        )
        second.publish_action(
            {"enabled": np.array([1], dtype=np.uint8), "velocities": np.array([0.0, 1.0, 0.0])},
            FrameMetadata(0, 1, 0, 0, 0.0),
        )

        world = runtime.step()

        entities = {entity.entity_id: entity for entity in world.entities}
        manifest_ids = {entity.entity_id for entity in runtime.world_manifest.entities}
        assert entities["first"].position == pytest.approx((0.02, 0.0, 0.0))
        assert entities["second"].position == pytest.approx((0.0, 0.02, 0.0))
        assert manifest_ids == {"first", "second"}
    finally:
        runtime.close()


def test_respawn_starts_new_episode_at_requested_pose(tmp_path: Path) -> None:
    runtime = SimRuntime(
        _config(ExecutionMode.LIVE, (_base(),)),
        registry=SimRegistry(run_id="runtime-respawn", root=tmp_path),
    )
    runtime.start()
    try:
        runtime.run()
        frame = runtime.respawn(
            "go2",
            SpawnPose(
                position=(2.0, -1.0, 0.4),
                quaternion_xyzw=(0.0, 0.0, np.sin(0.25), np.cos(0.25)),
            ),
        )

        assert frame.episode_id == 2
        assert frame.physics_tick == 0
        assert frame.control_tick == 0
        assert runtime.status().running
        assert frame.entities[0].position == pytest.approx((2.0, -1.0, 0.4))
        assert frame.entities[0].quaternion_xyzw == pytest.approx(
            (0.0, 0.0, np.sin(0.25), np.cos(0.25))
        )
    finally:
        runtime.close()


def test_kinematic_wall_authoring_updates_manifest_and_world_state(tmp_path: Path) -> None:
    runtime = SimRuntime(
        _config(ExecutionMode.LIVE, (_base(),)),
        registry=SimRegistry(run_id="runtime-wall", root=tmp_path),
    )
    runtime.start()
    try:
        wall = runtime.add_wall(0.0, 0.0, 3.0, 4.0, height=2.0, thickness=0.2)

        state = next(
            item for item in runtime.world_state.entities if item.entity_id == wall.entity_id
        )
        descriptor = next(
            item for item in runtime.world_manifest.entities if item.entity_id == wall.entity_id
        )
        assert descriptor.shape_hint == "box"
        assert descriptor.extents == pytest.approx((5.0, 0.2, 2.0))
        assert state.position == pytest.approx((1.5, 2.0, 1.0))
        assert runtime.status().control_tick == 0
    finally:
        runtime.close()


def test_live_runtime_stops_reusing_an_action_after_timeout(tmp_path: Path) -> None:
    runtime = SimRuntime(
        SimConfig(
            sim_id="timeout",
            backend=KinematicBackend(),
            robots=(_base(),),
            execution=ExecutionConfig(
                mode=ExecutionMode.LIVE,
                physics_dt=0.01,
                control_decimation=2,
                autostart=False,
                action_timeout=0.01,
            ),
        ),
        registry=SimRegistry(run_id="runtime-timeout", root=tmp_path),
    )
    runtime.start()
    try:
        channel = runtime.channels["go2"]
        channel.publish_action(
            {
                "enabled": np.array([1], dtype=np.uint8),
                "velocities": np.array([1.0, 0.0, 0.0]),
            },
            FrameMetadata(0, 1, 0, 0, 0.0),
        )
        first = runtime.step()
        time.sleep(0.02)
        second = runtime.step()

        assert first.entities[0].position == pytest.approx((0.02, 0.0, 0.0))
        assert second.entities[0].position == first.entities[0].position
    finally:
        runtime.close()
