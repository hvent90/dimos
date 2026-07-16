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
from typing import Any

import numpy as np

from dimos.hardware.drive_trains.spec import TwistBaseAdapter
from dimos.hardware.manipulators.spec import ManipulatorAdapter
from dimos.hardware.whole_body.spec import MotorCommand, WholeBodyAdapter
from dimos.sim2.backend.base import RobotHandle
from dimos.sim2.control.adapters.manipulator.adapter import SimManipulatorAdapter
from dimos.sim2.control.adapters.twist_base.adapter import SimTwistBaseAdapter
from dimos.sim2.control.adapters.whole_body.adapter import SimWholeBodyAdapter
from dimos.sim2.ipc.registry import SimRegistry
from dimos.sim2.runtime import SimRuntime
from dimos.sim2.spec import (
    ControlInterface,
    ExecutionConfig,
    SimConfig,
    SimRobotSpec,
    WorldSpec,
)


class ProtocolBackend:
    def __init__(self) -> None:
        self.states: dict[str, dict[str, Any]] = {}

    @property
    def capabilities(self) -> frozenset[ControlInterface]:
        return frozenset(ControlInterface)

    def load(
        self,
        world: WorldSpec,
        robots: tuple[SimRobotSpec, ...],
        physics_dt: float,
    ) -> dict[str, RobotHandle]:
        del world, physics_dt
        handles: dict[str, RobotHandle] = {}
        for robot in robots:
            handles[robot.robot_id] = RobotHandle(
                robot.robot_id,
                robot.control_interface,
                robot.dof,
            )
            self.states[robot.robot_id] = self._initial_state(robot)
        return handles

    def reset(self, seed: int | None = None) -> None:
        del seed

    def apply_action(self, handle: RobotHandle, action: dict[str, Any]) -> None:
        state = self.states[handle.robot_id]
        if handle.control_interface == ControlInterface.TWIST_BASE:
            state["velocities"] = action["velocities"].copy()
            state["enabled"] = action["enabled"].copy()
        elif handle.control_interface == ControlInterface.MANIPULATOR:
            mode = int(action["command_mode"][0])
            key = ("position", "velocity", "effort")[mode]
            state[key] = action[key].copy()
            state["enabled"] = action["enabled"].copy()
            state["gripper"] = action["gripper"].copy()
        else:
            state["position"] = action["position"].copy()
            state["velocity"] = action["velocity"].copy()
            state["effort"] = action["effort"].copy()

    def step(self, dt: float) -> None:
        del dt

    def observe(self, handle: RobotHandle) -> dict[str, Any]:
        return {name: value.copy() for name, value in self.states[handle.robot_id].items()}

    def entity_states(self) -> tuple[Any, ...]:
        return ()

    def sensor_samples(self, sim_time: float) -> tuple[Any, ...]:
        del sim_time
        return ()

    def close(self) -> None:
        pass

    @staticmethod
    def _initial_state(robot: SimRobotSpec) -> dict[str, Any]:
        zeros = np.zeros(robot.dof)
        if robot.control_interface == ControlInterface.TWIST_BASE:
            return {
                "enabled": np.array([1], dtype=np.uint8),
                "velocities": zeros.copy(),
                "odometry": zeros.copy(),
            }
        if robot.control_interface == ControlInterface.MANIPULATOR:
            return {
                "position": zeros.copy(),
                "velocity": zeros.copy(),
                "effort": zeros.copy(),
                "gripper": np.array([0.0]),
                "enabled": np.array([1], dtype=np.uint8),
                "error_code": np.array([0], dtype=np.int32),
            }
        return {
            "position": zeros.copy(),
            "velocity": zeros.copy(),
            "effort": zeros.copy(),
            "imu_quaternion": np.array([1.0, 0.0, 0.0, 0.0]),
            "imu_gyroscope": np.zeros(3),
            "imu_accelerometer": np.zeros(3),
            "imu_rpy": np.zeros(3),
            "root_position": np.zeros(3),
            "root_quaternion": np.array([0.0, 0.0, 0.0, 1.0]),
            "root_linear_velocity": np.zeros(3),
            "root_angular_velocity": np.zeros(3),
            "enabled": np.array([1], dtype=np.uint8),
        }


def _runtime(tmp_path: Path) -> tuple[SimRuntime, SimRegistry]:
    registry = SimRegistry(run_id="adapter-test", root=tmp_path)
    runtime = SimRuntime(
        SimConfig(
            sim_id="main",
            backend=ProtocolBackend(),
            robots=(
                SimRobotSpec("base", ControlInterface.TWIST_BASE, 3),
                SimRobotSpec(
                    "arm",
                    ControlInterface.MANIPULATOR,
                    7,
                    capabilities=frozenset({"gripper"}),
                ),
                SimRobotSpec("body", ControlInterface.WHOLE_BODY, 4),
            ),
            execution=ExecutionConfig(autostart=False),
        ),
        registry=registry,
    )
    runtime.start()
    return runtime, registry


def test_generic_adapters_implement_existing_hardware_protocols(tmp_path: Path) -> None:
    runtime, registry = _runtime(tmp_path)
    base = SimTwistBaseAdapter(dof=3, hardware_id="base", registry=registry)
    arm = SimManipulatorAdapter(dof=7, hardware_id="arm", registry=registry)
    body = SimWholeBodyAdapter(dof=4, hardware_id="body", registry=registry)
    try:
        assert isinstance(base, TwistBaseAdapter)
        assert isinstance(arm, ManipulatorAdapter)
        assert isinstance(body, WholeBodyAdapter)
        assert base.connect()
        assert arm.connect()
        assert body.connect()
    finally:
        base.disconnect()
        arm.disconnect()
        body.disconnect()
        runtime.close()


def test_generic_adapters_exchange_complete_protocol_commands(tmp_path: Path) -> None:
    runtime, registry = _runtime(tmp_path)
    base = SimTwistBaseAdapter(dof=3, hardware_id="base", registry=registry)
    arm = SimManipulatorAdapter(dof=7, hardware_id="arm", registry=registry)
    body = SimWholeBodyAdapter(dof=4, hardware_id="body", registry=registry)
    try:
        assert base.connect() and arm.connect() and body.connect()
        assert base.write_velocities([1.0, 2.0, 3.0])
        assert arm.write_joint_positions([0.5] * 7)
        assert arm.write_gripper_position(0.04)
        assert body.write_motor_commands(
            [MotorCommand(q=0.25, dq=0.1, kp=10.0, kd=1.0, tau=0.2) for _ in range(4)]
        )

        runtime.step()

        assert base.read_velocities() == [1.0, 2.0, 3.0]
        assert arm.read_joint_positions() == [0.5] * 7
        assert arm.read_gripper_position() == 0.04
        assert [state.q for state in body.read_motor_states()] == [0.25] * 4
        assert [state.dq for state in body.read_motor_states()] == [0.1] * 4
        assert [state.tau for state in body.read_motor_states()] == [0.2] * 4
    finally:
        base.disconnect()
        arm.disconnect()
        body.disconnect()
        runtime.close()
