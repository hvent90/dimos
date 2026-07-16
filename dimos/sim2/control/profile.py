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

"""ControlCoordinator hardware declarations derived from sim2 robot specs."""

from __future__ import annotations

from typing import Any

from dimos.control.components import HardwareComponent, HardwareType
from dimos.hardware.whole_body.spec import WholeBodyConfig
from dimos.sim2.spec import ControlInterface, SimRobotSpec

_HARDWARE_TYPES = {
    ControlInterface.TWIST_BASE: HardwareType.BASE,
    ControlInterface.MANIPULATOR: HardwareType.MANIPULATOR,
    ControlInterface.WHOLE_BODY: HardwareType.WHOLE_BODY,
}


def sim_hardware(
    robot: SimRobotSpec,
    *,
    sim_id: str = "main",
    auto_enable: bool = True,
    gripper: bool = False,
    wb_config: WholeBodyConfig | None = None,
    adapter_kwargs: dict[str, Any] | None = None,
) -> HardwareComponent:
    """Make the coordinator side of a robot's sim2 control contract."""
    if not robot.joint_names:
        raise ValueError(f"robot '{robot.robot_id}' must declare coordinator joint names")
    if gripper and "gripper" not in robot.capabilities:
        raise ValueError(f"robot '{robot.robot_id}' does not declare the gripper capability")
    if wb_config is not None and robot.control_interface != ControlInterface.WHOLE_BODY:
        raise ValueError("wb_config is only valid for whole-body robots")

    resolved_adapter_kwargs: dict[str, Any] = {
        "sim_id": sim_id,
        "robot_id": robot.robot_id,
    }
    if adapter_kwargs:
        resolved_adapter_kwargs.update(adapter_kwargs)
    return HardwareComponent(
        hardware_id=robot.robot_id,
        hardware_type=_HARDWARE_TYPES[robot.control_interface],
        joints=list(robot.joint_names),
        adapter_type="sim",
        auto_enable=auto_enable,
        gripper_joints=[f"{robot.robot_id}/gripper"] if gripper else [],
        adapter_kwargs=resolved_adapter_kwargs,
        wb_config=wb_config,
    )
