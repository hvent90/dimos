# Copyright 2026 Dimensional Inc.
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

"""Unit tests for the live LeRobot policy module."""

from __future__ import annotations

from collections.abc import Callable, Iterator
from threading import Event
import time
from typing import Any

import numpy as np
import pytest
import pytest_mock

from dimos.learning.lerobot_policy import LeRobotPolicyModule
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.protocol.rpc.pubsubrpc import LCMRPC

JOINTS = [f"arm/joint{i}" for i in range(1, 7)] + ["arm/gripper"]


class FakeBackend:
    def __init__(self, action: np.ndarray[Any, Any]) -> None:
        self.action = action
        self.called = Event()
        self.reset_count = 0
        self.image: np.ndarray[Any, Any] | None = None
        self.state: np.ndarray[Any, Any] | None = None
        self.task = ""
        self.robot_type = ""

    def reset(self) -> None:
        self.reset_count += 1

    def predict(
        self,
        image: np.ndarray[Any, Any],
        state: np.ndarray[Any, Any],
        *,
        task: str,
        robot_type: str,
    ) -> np.ndarray[Any, Any]:
        self.image = image
        self.state = state
        self.task = task
        self.robot_type = robot_type
        self.called.set()
        return self.action


class CapturingOutput:
    def __init__(self) -> None:
        self.messages: list[JointState] = []
        self.published = Event()

    def publish(self, message: JointState) -> None:
        self.messages.append(message)
        self.published.set()


@pytest.fixture
def make_module(
    mocker: pytest_mock.MockerFixture,
) -> Iterator[Callable[[FakeBackend], tuple[LeRobotPolicyModule, CapturingOutput, Event]]]:
    mocker.patch("dimos.core.module.get_loop", return_value=(mocker.MagicMock(), None))
    mocker.patch.object(LCMRPC, "__init__", return_value=None)
    mocker.patch.object(LCMRPC, "serve_module_rpc", return_value=None)
    mocker.patch.object(LCMRPC, "start", return_value=None)
    mocker.patch.object(LCMRPC, "stop", return_value=None)

    built: list[LeRobotPolicyModule] = []

    def _make(backend: FakeBackend) -> tuple[LeRobotPolicyModule, CapturingOutput, Event]:
        mocker.patch("dimos.learning.lerobot_policy._load_policy_backend", return_value=backend)
        module = LeRobotPolicyModule(
            policy_path="checkpoint",
            joint_names=JOINTS,
            fps=50.0,
            task="default task",
            robot_type="galaxea_a1z",
        )
        output = CapturingOutput()
        finished = Event()
        module.joint_command = output  # type: ignore[assignment]
        mocker.patch.object(module, "start_tool")
        mocker.patch.object(module, "tool_update")
        mocker.patch.object(module, "stop_tool", side_effect=lambda _name: finished.set())
        module.build()
        built.append(module)
        return module, output, finished

    yield _make
    for module in built:
        module.stop()


def _provide_observation(module: LeRobotPolicyModule) -> tuple[np.ndarray[Any, Any], list[float]]:
    # BGR input verifies that the module supplies the RGB convention used by DataPrep.
    bgr = np.zeros((4, 5, 3), dtype=np.uint8)
    bgr[..., 0] = 10
    bgr[..., 1] = 20
    bgr[..., 2] = 30
    positions = [float(i) / 10 for i in range(len(JOINTS))]
    now = time.time()
    module._on_color_image(Image(data=bgr, format=ImageFormat.BGR, ts=now))
    module._on_joint_state(JointState(ts=now, name=JOINTS, position=positions))
    return bgr, positions


def test_policy_observation_and_action_use_canonical_order(
    make_module: Callable[[FakeBackend], tuple[LeRobotPolicyModule, CapturingOutput, Event]],
) -> None:
    action = np.arange(len(JOINTS), dtype=np.float32) / 20
    backend = FakeBackend(action)
    module, output, _finished = make_module(backend)
    bgr, positions = _provide_observation(module)

    result = module.execute_learned_policy(duration=1.0, task="pick up cube")

    assert "started" in result.lower()
    assert output.published.wait(1.0), "policy did not publish a command"
    assert backend.reset_count == 1
    assert backend.task == "pick up cube"
    assert backend.robot_type == "galaxea_a1z"
    assert backend.image is not None
    np.testing.assert_array_equal(backend.image, bgr[..., ::-1])
    np.testing.assert_allclose(backend.state, positions)
    assert output.messages[0].name == JOINTS
    np.testing.assert_allclose(output.messages[0].position, action)
    module.stop_learned_policy()


def test_invalid_policy_action_stops_without_publishing(
    make_module: Callable[[FakeBackend], tuple[LeRobotPolicyModule, CapturingOutput, Event]],
) -> None:
    backend = FakeBackend(np.zeros(len(JOINTS) - 1, dtype=np.float32))
    module, output, finished = make_module(backend)
    _provide_observation(module)

    module.execute_learned_policy(duration=1.0)

    assert backend.called.wait(1.0), "policy was not invoked"
    assert finished.wait(1.0), "policy thread did not stop after invalid output"
    assert output.messages == []
    status = module.policy_status()
    assert status["running"] is False
    assert "expected (7,)" in status["last_error"]


def test_policy_refuses_to_start_without_live_observations(
    make_module: Callable[[FakeBackend], tuple[LeRobotPolicyModule, CapturingOutput, Event]],
) -> None:
    backend = FakeBackend(np.zeros(len(JOINTS), dtype=np.float32))
    module, output, _finished = make_module(backend)

    result = module.execute_learned_policy(duration=1.0)

    assert "no camera image" in result
    assert backend.reset_count == 0
    assert output.messages == []
    assert module.policy_status()["running"] is False
