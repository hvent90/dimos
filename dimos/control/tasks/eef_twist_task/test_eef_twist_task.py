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

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray
import pytest

from dimos.control.task import CoordinatorState, JointStateSnapshot
from dimos.control.tasks.eef_twist_task.eef_twist_task import EEFTwistTask, EEFTwistTaskConfig
from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped


@dataclass
class FakePose:
    translation: NDArray[np.float64]
    rotation: NDArray[np.float64]

    def copy(self) -> FakePose:
        return FakePose(self.translation.copy(), self.rotation.copy())


class FakeIK:
    nq = 3

    def __init__(self) -> None:
        self.solve_calls: list[FakePose] = []
        self.solution = np.array([0.01, 0.02, 0.03], dtype=np.float64)

    def forward_kinematics(self, q_current: NDArray[np.float64]) -> FakePose:
        return FakePose(q_current.copy(), np.eye(3, dtype=np.float64))

    def solve(
        self, pose: FakePose, q_init: NDArray[np.float64]
    ) -> tuple[NDArray[np.float64], bool, float]:
        del q_init
        self.solve_calls.append(pose.copy())
        return self.solution.copy(), True, 0.0


@pytest.fixture
def fake_ik(mocker) -> FakeIK:
    ik = FakeIK()
    mocker.patch(
        "dimos.control.tasks.eef_twist_task.eef_twist_task.PinocchioIK.from_model_path",
        return_value=ik,
    )
    return ik


@pytest.fixture
def task(fake_ik: FakeIK) -> EEFTwistTask:
    return EEFTwistTask(
        "eef",
        EEFTwistTaskConfig(
            joint_names=["arm/joint1", "arm/joint2", "arm/joint3"],
            model_path="fake.urdf",
            ee_joint_id=3,
            timeout=0.3,
            max_joint_delta_deg=15.0,
        ),
    )


def _state(t_now: float, positions: list[float], dt: float = 0.01) -> CoordinatorState:
    return CoordinatorState(
        joints=JointStateSnapshot(
            joint_positions={f"arm/joint{i + 1}": value for i, value in enumerate(positions)}
        ),
        t_now=t_now,
        dt=dt,
    )


def _twist(x: float = 0.1) -> TwistStamped:
    return TwistStamped(frame_id="eef", linear=[x, 0.0, 0.0], angular=[0.0, 0.0, 0.0])


def test_zero_twist_reissues_last_commanded_target(task: EEFTwistTask, fake_ik: FakeIK) -> None:
    assert task.on_ee_twist_command(_twist(1.0), t_now=1.0)
    first = task.compute(_state(1.01, [0.0, 0.0, 0.0]))
    assert first is not None

    assert task.on_ee_twist_command(_twist(0.0), t_now=1.02)
    held = task.compute(_state(1.03, [0.5, 0.5, 0.5]))

    assert held is not None
    assert held.positions == first.positions
    assert len(fake_ik.solve_calls) == 1
    assert task.is_active()


def test_initial_zero_twist_seeds_hold_from_measured_state(task: EEFTwistTask) -> None:
    assert task.on_ee_twist_command(_twist(0.0), t_now=1.0)

    held = task.compute(_state(1.01, [0.4, 0.5, 0.6]))

    assert held is not None
    assert held.positions == [0.4, 0.5, 0.6]


def test_nonzero_twist_integrates_from_last_commanded_pose(
    task: EEFTwistTask, fake_ik: FakeIK
) -> None:
    assert task.on_ee_twist_command(_twist(1.0), t_now=1.0)
    first = task.compute(_state(1.01, [0.0, 0.0, 0.0]))
    assert first is not None

    fake_ik.solution = np.array([0.51, 0.0, 0.0], dtype=np.float64)
    second = task.compute(_state(1.04, [0.5, 0.0, 0.0]))

    assert second is not None
    assert second.positions == [0.51, 0.0, 0.0]
    assert fake_ik.solve_calls[1].translation[0] == pytest.approx(
        fake_ik.solve_calls[0].translation[0] + 0.01
    )


def test_timeout_clears_hold_target_before_next_command(
    task: EEFTwistTask, fake_ik: FakeIK
) -> None:
    assert task.on_ee_twist_command(_twist(), t_now=1.0)
    assert task.compute(_state(1.01, [0.0, 0.0, 0.0])) is not None
    assert task.compute(_state(1.5, [0.5, 0.0, 0.0])) is None
    assert not task.is_active()

    fake_ik.solution = np.array([1.01, 0.0, 0.0], dtype=np.float64)
    assert task.on_ee_twist_command(_twist(), t_now=2.0)
    output = task.compute(_state(2.01, [1.0, 0.0, 0.0]))
    assert output is not None
    assert output.positions == [1.01, 0.0, 0.0]
