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

import threading
import uuid

import numpy as np
import pytest

from dimos.sim2.ipc.abi import make_channel_descriptor
from dimos.sim2.ipc.channel import FrameMetadata, RobotChannel
from dimos.sim2.spec import ControlInterface


@pytest.fixture
def whole_body_channel() -> RobotChannel:
    descriptor = make_channel_descriptor(
        sim_id="test",
        robot_id="g1",
        generation=uuid.uuid4().hex,
        shm_name=f"dms2_test_{uuid.uuid4().hex[:12]}",
        control_interface=ControlInterface.WHOLE_BODY,
        dof=29,
        physics_dt=0.002,
        control_decimation=10,
    )
    channel = RobotChannel.create(descriptor)
    try:
        yield channel
    finally:
        channel.set_lifecycle("closed")
        channel.close()
        channel.unlink()


def _whole_body_action(dof: int, value: float) -> dict[str, np.ndarray]:
    return {
        "enabled": np.array([1], dtype=np.uint8),
        "position": np.full(dof, value),
        "velocity": np.full(dof, value),
        "kp": np.full(dof, value),
        "kd": np.full(dof, value),
        "effort": np.full(dof, value),
    }


def test_channel_publishes_one_coherent_action_frame(whole_body_channel: RobotChannel) -> None:
    metadata = FrameMetadata(
        sequence=0,
        episode_id=3,
        physics_tick=40,
        control_tick=4,
        sim_time=0.08,
    )

    sequence = whole_body_channel.publish_action(_whole_body_action(29, 7.0), metadata)
    frame = whole_body_channel.read_action()

    assert sequence == 1
    assert frame is not None
    assert frame.metadata.sequence == 1
    assert frame.metadata.episode_id == 3
    assert frame.metadata.control_tick == 4
    assert np.array_equal(frame.values["position"], np.full(29, 7.0))
    assert np.array_equal(frame.values["effort"], np.full(29, 7.0))


def test_channel_rejects_partial_or_wrong_shape_frames(whole_body_channel: RobotChannel) -> None:
    values = _whole_body_action(29, 1.0)
    del values["kp"]

    with pytest.raises(ValueError, match="missing=.*kp"):
        whole_body_channel.publish_action(values, FrameMetadata(0, 1, 0, 0, 0.0))

    values = _whole_body_action(29, 1.0)
    values["kd"] = np.zeros(28)
    with pytest.raises(ValueError, match="field 'kd'.*expected"):
        whole_body_channel.publish_action(values, FrameMetadata(0, 1, 0, 0, 0.0))


def test_channel_reset_invalidates_old_frames(whole_body_channel: RobotChannel) -> None:
    whole_body_channel.publish_action(
        _whole_body_action(29, 1.0),
        FrameMetadata(0, 1, 0, 0, 0.0),
    )

    whole_body_channel.reset_frames(2)

    assert whole_body_channel.episode_id == 2
    assert whole_body_channel.read_action() is None


def test_channel_concurrent_frames_are_not_torn(whole_body_channel: RobotChannel) -> None:
    iterations = 2_000
    finished = threading.Event()
    failures: list[str] = []

    def write_frames() -> None:
        for value in range(1, iterations + 1):
            whole_body_channel.publish_action(
                _whole_body_action(29, float(value)),
                FrameMetadata(0, 1, value, value, value * 0.02),
            )
        finished.set()

    writer = threading.Thread(target=write_frames)
    writer.start()
    while not finished.is_set():
        frame = whole_body_channel.read_action()
        if frame is None:
            continue
        expected = frame.values["position"][0]
        for name in ("position", "velocity", "kp", "kd", "effort"):
            if not np.all(frame.values[name] == expected):
                failures.append(name)
                finished.set()
                break
    writer.join(timeout=5.0)

    assert not writer.is_alive()
    assert failures == []
