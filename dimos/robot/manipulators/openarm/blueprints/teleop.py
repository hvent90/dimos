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

"""OpenArm teleop blueprints."""

from __future__ import annotations

from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.coordination.blueprints import autoconnect
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.robot.manipulators.common.blueprints import cartesian_ik_task
from dimos.robot.manipulators.openarm.config import (
    LEFT_CAN,
    OPENARM_ADAPTER_KWARGS,
    OPENARM_V10_FK_MODEL,
    RIGHT_CAN,
    openarm_hardware,
    openarm_model_config,
    openarm_single_hardware,
    openarm_single_model_config,
)
from dimos.teleop.keyboard.keyboard_teleop_module import KeyboardTeleopModule
from dimos.teleop.openarm_mini.adapter import OpenArmMiniTeleopAdapter
from dimos.teleop.openarm_mini.config import OpenArmMiniTeleopConfig
from dimos.teleop.openarm_mini.teleop_module import OpenArmMiniTeleopModule
from dimos.teleop.openarm_mini.viser_visualizer import OpenArmJointStateViserModule
from dimos.teleop.runtime.teleop_module import TeleopModule

_teleop_hw = openarm_single_hardware()

keyboard_teleop_openarm_mock = autoconnect(
    KeyboardTeleopModule.blueprint(
        model_path=OPENARM_V10_FK_MODEL,
        ee_joint_id=7,
        joint_names=_teleop_hw.joints,
    ),
    ControlCoordinator.blueprint(
        hardware=[_teleop_hw],
        tasks=[cartesian_ik_task(_teleop_hw, model_path=OPENARM_V10_FK_MODEL, ee_joint_id=7)],
    ),
    ManipulationModule.blueprint(
        robots=[openarm_single_model_config()],
        visualization={"backend": "meshcat"},
    ),
)

_teleop_real_hw = openarm_single_hardware(adapter_type="openarm", address=LEFT_CAN)

keyboard_teleop_openarm = autoconnect(
    KeyboardTeleopModule.blueprint(
        model_path=OPENARM_V10_FK_MODEL,
        ee_joint_id=7,
        joint_names=_teleop_real_hw.joints,
    ),
    ControlCoordinator.blueprint(
        hardware=[_teleop_real_hw],
        tasks=[
            cartesian_ik_task(
                _teleop_real_hw,
                model_path=OPENARM_V10_FK_MODEL,
                ee_joint_id=7,
            )
        ],
    ),
    ManipulationModule.blueprint(
        robots=[openarm_single_model_config()],
        visualization={"backend": "meshcat"},
    ),
)

_openarm_mini_left_hw = openarm_hardware(
    side="left",
    address=LEFT_CAN,
    adapter_type="openarm",
    adapter_kwargs=OPENARM_ADAPTER_KWARGS,
)
_openarm_mini_right_hw = openarm_hardware(
    side="right",
    address=RIGHT_CAN,
    adapter_type="openarm",
    adapter_kwargs=OPENARM_ADAPTER_KWARGS,
)


def _servo_task(hw_name: str, joint_names: list[str]) -> TaskConfig:
    return TaskConfig(
        name=f"servo_{hw_name}",
        type="servo",
        joint_names=joint_names,
        priority=10,
    )


openarm_mini_teleop_openarm = autoconnect(
    TeleopModule.blueprint(adapter=OpenArmMiniTeleopAdapter()),
    ControlCoordinator.blueprint(
        hardware=[_openarm_mini_left_hw, _openarm_mini_right_hw],
        tasks=[
            _servo_task(_openarm_mini_left_hw.hardware_id, _openarm_mini_left_hw.joints),
            _servo_task(_openarm_mini_right_hw.hardware_id, _openarm_mini_right_hw.joints),
        ],
    ),
)

openarm_mini_left_teleop_viser = autoconnect(
    OpenArmMiniTeleopModule.blueprint(
        openarm_mini=OpenArmMiniTeleopConfig(enabled_sides=("left",))
    ),
    OpenArmJointStateViserModule.blueprint(
        robot=openarm_model_config("left"),
        robot_id="openarm_left",
    ),
)
