# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.control.task import CoordinatorState, JointStateSnapshot
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.piper.blueprints.teleop import (
    coordinator_teleop_piper,
    keyboard_teleop_piper,
)
from dimos.teleop.keyboard.keyboard_teleop_module import KeyboardTeleopModule


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

    assert keyboard_kwargs["gripper_open_position"] == 0.07


def test_piper_quest_gripper_maps_neutral_open_and_full_closed() -> None:
    coordinator_kwargs = next(
        atom.kwargs
        for atom in coordinator_teleop_piper.blueprints
        if atom.module is ControlCoordinator
    )
    task = coordinator_kwargs["tasks"][0]

    assert task.params["gripper_open_pos"] == 0.07
    assert task.params["gripper_closed_pos"] == 0.0


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
    coordinator = ControlCoordinator(tasks=[servo_config])
    try:
        coordinator._setup_from_config()
        servo = coordinator._tasks["servo_gripper"]

        coordinator._on_joint_command(JointState({"name": ["arm/gripper"], "position": [0.015]}))
        opened = servo.compute(CoordinatorState(JointStateSnapshot({}), t_now=1.0, dt=0.01))
        assert opened is not None
        assert opened.joint_names == ["arm/gripper"]
        assert opened.positions == [0.015]

        coordinator._on_joint_command(JointState({"name": ["arm/gripper"], "position": [0.0]}))
        closed = servo.compute(CoordinatorState(JointStateSnapshot({}), t_now=2.0, dt=0.01))
        assert closed is not None
        assert closed.positions == [0.0]
    finally:
        coordinator.stop()
