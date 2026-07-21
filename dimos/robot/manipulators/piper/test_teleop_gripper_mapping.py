# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

from typing import Any
from unittest.mock import MagicMock

from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.control.hardware_interface import ConnectedHardware
from dimos.control.task import ControlMode, CoordinatorState, JointStateSnapshot
from dimos.hardware.manipulators.spec import ManipulatorAdapter
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.piper.blueprints.teleop import (
    coordinator_teleop_piper,
    keyboard_teleop_piper,
)
from dimos.robot.manipulators.xarm.config import xarm6_hardware, xarm7_hardware
from dimos.teleop.keyboard.keyboard_teleop_module import KeyboardTeleopModule


class _JointCommandStream:
    def __init__(self) -> None:
        self._callbacks: list[Any] = []

    def subscribe(self, callback: Any, *_: Any) -> Any:
        self._callbacks.append(callback)
        return lambda: self._callbacks.remove(callback)

    def publish(self, message: JointState) -> None:
        for callback in self._callbacks:
            callback(message)

    def stop(self) -> None:
        self._callbacks.clear()


def _keyboard_tasks() -> list[TaskConfig]:
    coordinator_kwargs = next(
        atom.kwargs
        for atom in keyboard_teleop_piper.blueprints
        if atom.module is ControlCoordinator
    )
    return coordinator_kwargs["tasks"]


def test_piper_keyboard_uses_07m_open_position() -> None:
    keyboard_kwargs = next(
        atom.kwargs
        for atom in keyboard_teleop_piper.blueprints
        if atom.module is KeyboardTeleopModule
    )

    assert keyboard_kwargs == {}
    hardware = next(
        atom.kwargs["hardware"][0]
        for atom in keyboard_teleop_piper.blueprints
        if atom.module is ControlCoordinator
    )
    assert (hardware.gripper_closed_position, hardware.gripper_open_position) == (0.0, 0.07)


def test_xarm_mock_factories_keep_normalized_mapping() -> None:
    for hardware in (
        xarm6_hardware(
            gripper=True,
            gripper_open_position=0.85,
            gripper_closed_position=0.0,
            mock_without_address=True,
        ),
        xarm7_hardware(
            gripper=True,
            gripper_open_position=0.85,
            gripper_closed_position=0.0,
            mock_without_address=True,
        ),
    ):
        assert (hardware.gripper_closed_position, hardware.gripper_open_position) == (0.0, 0.85)


def test_piper_quest_gripper_maps_neutral_open_and_full_closed() -> None:
    coordinator_kwargs = next(
        atom.kwargs
        for atom in coordinator_teleop_piper.blueprints
        if atom.module is ControlCoordinator
    )
    hardware = coordinator_kwargs["hardware"][0]
    task = coordinator_kwargs["tasks"][0]

    assert hardware.gripper_open_position == 0.07
    assert hardware.gripper_closed_position == 0.0
    assert task.params["gripper_open_pos"] == 1.0
    assert task.params["gripper_closed_pos"] == 0.0


def test_piper_keyboard_and_quest_normalized_endpoints_reach_adapter() -> None:
    component = next(
        atom.kwargs["hardware"][0]
        for atom in keyboard_teleop_piper.blueprints
        if atom.module is ControlCoordinator
    )
    adapter = MagicMock(spec=ManipulatorAdapter)
    adapter.read_joint_positions.return_value = [0.0] * 6
    adapter.read_gripper_position.return_value = 0.0
    adapter.set_control_mode.return_value = True
    adapter.write_joint_positions.return_value = True
    adapter.write_gripper_position.return_value = True
    hardware = ConnectedHardware(adapter, component)

    hardware.write_command({"arm/gripper": 0.0}, ControlMode.POSITION)
    hardware.write_command({"arm/gripper": 1.0}, ControlMode.POSITION)

    assert adapter.write_gripper_position.call_args_list == [
        ((0.0,), {}),
        ((0.07,), {}),
    ]


def test_piper_keyboard_has_high_priority_gripper_servo() -> None:
    servo = next(task for task in _keyboard_tasks() if task.name == "servo_gripper")

    assert servo.type == "servo"
    assert servo.joint_names == ["arm/gripper"]
    assert servo.priority > next(
        task.priority for task in _keyboard_tasks() if task.type == "eef_twist"
    )
    assert servo.params == {"timeout": 0.0, "default_positions": [0.0]}


def test_piper_keyboard_joint_commands_reach_gripper_servo() -> None:
    servo_config = next(task for task in _keyboard_tasks() if task.name == "servo_gripper")
    coordinator = ControlCoordinator(tasks=[servo_config], publish_joint_state=False)
    stream = _JointCommandStream()
    coordinator.joint_command.transport = stream  # type: ignore[assignment]
    try:
        coordinator.start()
        servo = coordinator.get_task("servo_gripper")
        assert servo is not None

        stream.publish(JointState({"name": ["arm/gripper"], "position": [1.0]}))
        opened = servo.compute(CoordinatorState(JointStateSnapshot({}), t_now=1.0, dt=0.01))
        assert opened is not None
        assert opened.joint_names == ["arm/gripper"]
        assert opened.positions == [1.0]

        stream.publish(JointState({"name": ["arm/gripper"], "position": [0.0]}))
        closed = servo.compute(CoordinatorState(JointStateSnapshot({}), t_now=2.0, dt=0.01))
        assert closed is not None
        assert closed.positions == [0.0]
    finally:
        coordinator.stop()
