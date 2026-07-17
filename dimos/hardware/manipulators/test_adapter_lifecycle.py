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

import sys
from types import SimpleNamespace

import pytest
from typing_extensions import override

from dimos.hardware.manipulators.a750.adapter import A750Adapter
from dimos.hardware.manipulators.openarm.adapter import OpenArmAdapter
from dimos.hardware.manipulators.piper import adapter as piper_adapter
from dimos.hardware.manipulators.piper.adapter import PiperAdapter


class _PiperSdk:
    def __init__(self) -> None:
        self.actions: list[str] = []
        self.gripper_position = 0

    def EnablePiper(self) -> bool:
        self.actions.append("enable")
        return True

    def ConnectPort(self, **_: object) -> None:
        self.actions.append("connect")

    def GetArmStatus(self) -> object:
        return object()

    def MotionCtrl_2(self, **_: object) -> None:
        self.actions.append("position_mode")

    def MotionCtrl_1(self, ctrl_mode: int, move_mode: int, move_spd_rate_ctrl: int) -> None:
        self.actions.append(f"motion1:{ctrl_mode},{move_mode},{move_spd_rate_ctrl}")

    def JointCtrl(self, *joints: int) -> None:
        self.actions.append(f"joints:{','.join(str(joint) for joint in joints)}")

    def GetArmJointMsgs(self) -> object:
        class JointState:
            joint_1 = 0
            joint_2 = 0
            joint_3 = 0
            joint_4 = 0
            joint_5 = 0
            joint_6 = 0

        class JointMessages:
            joint_state = JointState()

        return JointMessages()

    def GripperCtrl(self, position: int, speed: int, code: int, set_zero: int) -> None:
        self.gripper_position = position
        self.actions.append(f"gripper:{position},{speed},{code},{set_zero}")

    def GetArmGripperMsgs(self) -> object:
        class GripperState:
            grippers_angle = self.gripper_position

        class GripperMessages:
            gripper_state = GripperState()

        return GripperMessages()

    def DisablePiper(self) -> None:
        self.actions.append("disable")

    def DisconnectPort(self) -> None:
        self.actions.append("disconnect")


class _ResetOnceFailingPiperSdk(_PiperSdk):
    def __init__(self) -> None:
        super().__init__()
        self._reset_attempts = 0

    def MotionCtrl_1(self, ctrl_mode: int, move_mode: int, move_spd_rate_ctrl: int) -> None:
        self._reset_attempts += 1
        if self._reset_attempts == 1:
            self.actions.append("reset-failed")
            raise RuntimeError("transient reset failure")
        super().MotionCtrl_1(ctrl_mode, move_mode, move_spd_rate_ctrl)


class _LifecyclePiperAdapter(PiperAdapter):
    def use_sdk(self, sdk: _PiperSdk) -> None:
        self._sdk: _PiperSdk | None
        self._sdk = sdk


def test_piper_lifecycle_enables_then_disables() -> None:
    sdk = _PiperSdk()
    adapter = _LifecyclePiperAdapter()
    adapter.use_sdk(sdk)

    assert adapter.activate()
    assert adapter.deactivate()
    assert sdk.actions == [
        "enable",
        "position_mode",
        "motion1:1,0,0",
    ]


def test_piper_disconnect_gracefully_stops_before_disabling() -> None:
    sdk = _PiperSdk()
    adapter = _LifecyclePiperAdapter()
    adapter.use_sdk(sdk)

    assert adapter.activate()
    adapter.disconnect()

    assert sdk.actions == [
        "enable",
        "position_mode",
        "position_mode",
        "joints:0,0,0,0,0,0",
        "gripper:0,1000,2,0",
        "disable",
        "disconnect",
    ]


def test_piper_explicit_stop_uses_motion_ctrl_1() -> None:
    sdk = _PiperSdk()
    adapter = _LifecyclePiperAdapter()
    adapter.use_sdk(sdk)

    assert adapter.write_stop()
    assert sdk.actions == ["motion1:1,0,0"]


def test_piper_connect_initializes_recovery_enable_zero_pose_and_gripper(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    sdk = _PiperSdk()
    sleeps: list[float] = []
    monkeypatch.setitem(
        sys.modules, "piper_sdk", SimpleNamespace(C_PiperInterface_V2=lambda **_: sdk)
    )
    monkeypatch.setattr(piper_adapter.time, "sleep", sleeps.append)

    adapter = PiperAdapter()

    assert adapter.connect()
    assert sdk.actions == [
        "connect",
        "motion1:2,0,0",
        "motion1:2,0,0",
        "position_mode",
        "joints:0,0,0,0,0,0",
        "gripper:0,1000,1,0",
    ]
    assert sleeps == [0.025, 0.5, 0.5, 1.0]


def test_piper_connect_reset_failure_cleans_up_without_zero(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    sdk = _ResetOnceFailingPiperSdk()
    monkeypatch.setitem(
        sys.modules, "piper_sdk", SimpleNamespace(C_PiperInterface_V2=lambda **_: sdk)
    )

    adapter = PiperAdapter()

    assert not adapter.connect()
    assert sdk.actions == ["connect", "reset-failed", "disconnect"]


def test_piper_connect_does_not_enable_during_startup(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    sdk = _PiperSdk()
    monkeypatch.setitem(
        sys.modules, "piper_sdk", SimpleNamespace(C_PiperInterface_V2=lambda **_: sdk)
    )
    monkeypatch.setattr(piper_adapter.time, "sleep", lambda _: None)

    adapter = PiperAdapter()

    assert adapter.connect()
    assert "enable" not in sdk.actions


def test_piper_connect_joint_failure_cleans_up_without_gripper(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    class FailingJointSdk(_PiperSdk):
        def JointCtrl(self, *joints: int) -> None:
            raise RuntimeError("joint command failed")

    sdk = FailingJointSdk()
    monkeypatch.setitem(
        sys.modules, "piper_sdk", SimpleNamespace(C_PiperInterface_V2=lambda **_: sdk)
    )
    monkeypatch.setattr(piper_adapter.time, "sleep", lambda _: None)

    adapter = PiperAdapter()

    assert not adapter.connect()
    assert "disconnect" in sdk.actions
    assert "joints:0,0,0,0,0,0" not in sdk.actions


def test_piper_gripper_uses_millimeter_units_and_clamps() -> None:
    sdk = _PiperSdk()
    adapter = _LifecyclePiperAdapter()
    adapter.use_sdk(sdk)

    assert adapter.write_gripper_position(0.1)
    assert sdk.gripper_position == 80_000
    assert adapter.read_gripper_position() == 0.08
    assert sdk.actions == [
        "gripper:0,1000,2,0",
        "gripper:0,1000,1,0",
        "gripper:80000,1000,1,0",
    ]


class _OpenArmLifecycle:
    def __init__(self) -> None:
        self.actions: list[str] = []

    def enable_all(self) -> None:
        self.actions.append("enable")

    def disable_all(self) -> None:
        self.actions.append("disable")


class _LifecycleOpenArmAdapter(OpenArmAdapter):
    def __init__(self, lifecycle: _OpenArmLifecycle) -> None:
        super().__init__()
        self._lifecycle: _OpenArmLifecycle
        self._lifecycle = lifecycle

    @override
    def read_joint_positions(self) -> list[float]:
        return [0.0] * 7

    @override
    def _compute_gravity_torques(self, q: list[float]) -> list[float]:
        return [0.0] * len(q)

    @override
    def write_enable(self, enable: bool) -> bool:
        if enable:
            self._lifecycle.enable_all()
        else:
            self._lifecycle.disable_all()
        return True

    @override
    def write_stop(self) -> bool:
        self._lifecycle.actions.append("hold")
        return True


def test_openarm_lifecycle_enables_then_holds_and_disables() -> None:
    lifecycle = _OpenArmLifecycle()
    adapter = _LifecycleOpenArmAdapter(lifecycle)

    assert adapter.activate()
    assert adapter.deactivate()
    assert lifecycle.actions == ["enable", "hold", "disable"]


class _A750Robot:
    def __init__(self) -> None:
        self.actions: list[str] = []

    def start_control_loop(self) -> None:
        self.actions.append("start")

    def stop_control_loop(self) -> None:
        self.actions.append("stop")


class _LifecycleA750Adapter(A750Adapter):
    def use_robot(self, robot: _A750Robot) -> None:
        self._robot: _A750Robot | None
        self._connected: bool
        self._robot = robot
        self._connected = True


def test_a750_lifecycle_starts_then_stops_control_loop() -> None:
    robot = _A750Robot()
    adapter = _LifecycleA750Adapter()
    adapter.use_robot(robot)

    assert adapter.activate()
    assert adapter.deactivate()
    assert robot.actions == ["start", "stop"]
