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

from pathlib import Path
from typing import cast

import pytest

from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.control.tasks.cartesian_ik_task.pink_control_ik import PinkControlIKConfig
from dimos.core.coordination.blueprints import Blueprint
from dimos.manipulation.manipulation_module import ManipulationModule, ManipulationModuleConfig
from dimos.manipulation.visualization.config import NoManipulationVisualizationConfig
from dimos.robot.manipulators.a1z.blueprints.teleop import keyboard_teleop_a1z
from dimos.robot.manipulators.a750.blueprints.teleop import keyboard_teleop_a750
from dimos.robot.manipulators.common.blueprints import eef_twist_task, planner
from dimos.robot.manipulators.common.topics import EEF_TWIST_TASK_NAME
from dimos.robot.manipulators.openarm.blueprints.teleop import (
    keyboard_teleop_openarm,
    keyboard_teleop_openarm_mock,
)
from dimos.robot.manipulators.piper.blueprints.teleop import (
    coordinator_cartesian_ik_mock,
    coordinator_cartesian_ik_piper,
    keyboard_teleop_piper,
)
from dimos.robot.manipulators.piper.config import PIPER_MODEL_PATH
from dimos.robot.manipulators.xarm.blueprints.basic import (
    dual_xarm6_planner,
    xarm6_planner_only,
    xarm7_planner_coordinator,
)
from dimos.robot.manipulators.xarm.blueprints.teleop import (
    keyboard_teleop_xarm6,
    keyboard_teleop_xarm7,
)
from dimos.robot.manipulators.xarm.config import make_xarm7_model_config, make_xarm_hardware
from dimos.teleop.keyboard.keyboard_teleop_module import KeyboardTeleopModule


def _module_kwargs(blueprint: Blueprint, module_type: type) -> dict[str, object]:
    return next(atom.kwargs for atom in blueprint.blueprints if atom.module is module_type)


def _manipulation_kwargs(blueprint: Blueprint) -> dict[str, object]:
    return _module_kwargs(blueprint, ManipulationModule)


def _manipulation_config(blueprint: Blueprint) -> ManipulationModuleConfig:
    return ManipulationModuleConfig(**_manipulation_kwargs(blueprint))


def _coordinator_tasks(blueprint: Blueprint) -> list[TaskConfig]:
    return cast("list[TaskConfig]", _module_kwargs(blueprint, ControlCoordinator)["tasks"])


def test_planner_helper_defaults_to_no_visualization() -> None:
    blueprint = planner(robots=[make_xarm7_model_config(name="arm", add_gripper=True)])

    kwargs = _manipulation_kwargs(blueprint)
    config = ManipulationModuleConfig(**kwargs)

    assert "visualization" not in kwargs
    assert isinstance(config.visualization, NoManipulationVisualizationConfig)


def test_planner_helper_preserves_explicit_visualization() -> None:
    blueprint = planner(
        robots=[make_xarm7_model_config(name="arm", add_gripper=True)],
        visualization={"backend": "meshcat"},
    )

    assert _manipulation_kwargs(blueprint)["visualization"] == {"backend": "meshcat"}


def test_xarm_planner_blueprints_default_to_no_visualization() -> None:
    for blueprint in (xarm6_planner_only, dual_xarm6_planner, xarm7_planner_coordinator):
        config = _manipulation_config(blueprint)

        assert isinstance(config.visualization, NoManipulationVisualizationConfig)


def test_eef_twist_task_helper_requires_pink_robot_model() -> None:
    hardware = make_xarm_hardware("arm", 6, adapter_type="mock")

    with pytest.raises(ValueError, match="authoritative RobotModelConfig"):
        eef_twist_task(hardware, model_path=Path("fake.urdf"), ee_joint_id=6)


@pytest.mark.parametrize(
    "blueprint",
    [
        pytest.param(keyboard_teleop_xarm6, id="xarm6"),
        pytest.param(keyboard_teleop_xarm7, id="xarm7"),
        pytest.param(keyboard_teleop_piper, id="piper"),
        pytest.param(keyboard_teleop_openarm_mock, id="openarm-mock"),
        pytest.param(keyboard_teleop_openarm, id="openarm"),
        pytest.param(keyboard_teleop_a750, id="a750"),
        pytest.param(keyboard_teleop_a1z, id="a1z"),
    ],
)
def test_manipulator_keyboard_blueprint_uses_eef_twist_and_light_keyboard_kwargs(
    blueprint: Blueprint,
) -> None:
    keyboard_kwargs = _module_kwargs(blueprint, KeyboardTeleopModule)
    coordinator_tasks = _coordinator_tasks(blueprint)
    eef_twist_tasks = [task for task in coordinator_tasks if task.type == "eef_twist"]

    assert keyboard_kwargs == {}
    assert [task.name for task in eef_twist_tasks] == [EEF_TWIST_TASK_NAME]
    assert all(task.type != "cartesian_ik" for task in coordinator_tasks)


@pytest.mark.parametrize(
    "blueprint",
    [
        pytest.param(keyboard_teleop_xarm6, id="xarm6"),
        pytest.param(keyboard_teleop_xarm7, id="xarm7"),
        pytest.param(keyboard_teleop_piper, id="piper"),
        pytest.param(keyboard_teleop_openarm_mock, id="openarm-mock"),
        pytest.param(keyboard_teleop_openarm, id="openarm"),
        pytest.param(keyboard_teleop_a750, id="a750"),
        pytest.param(keyboard_teleop_a1z, id="a1z"),
    ],
)
def test_shipped_eef_twist_blueprints_use_pink_with_named_models(
    blueprint: Blueprint,
) -> None:
    task = next(task for task in _coordinator_tasks(blueprint) if task.type == "eef_twist")
    control_ik = task.params["control_ik"]

    assert control_ik["backend"] == "pink"
    assert control_ik["robot_model"]["end_effector_link"]
    assert task.params["ee_joint_id"] is None
    assert not str(task.params["model_path"]).endswith((".xml", ".mjcf"))


def test_piper_pink_task_uses_xacro_and_gripper_base() -> None:
    blueprints = (
        keyboard_teleop_piper,
        coordinator_cartesian_ik_mock,
        coordinator_cartesian_ik_piper,
    )
    for blueprint in blueprints:
        task = next(
            task
            for task in _coordinator_tasks(blueprint)
            if task.type in ("eef_twist", "cartesian_ik")
        )
        control_ik = task.params["control_ik"]
        assert task.params["model_path"] == PIPER_MODEL_PATH
        assert control_ik["backend"] == "pink"
        assert control_ik["robot_model"]["model_path"] == str(PIPER_MODEL_PATH)
        assert control_ik["robot_model"]["end_effector_link"] == "gripper_base"
        assert task.params["ee_joint_id"] is None
        assert "self_collision_enabled" not in control_ik

        reconstructed = PinkControlIKConfig.model_validate(control_ik)
        assert reconstructed.robot_model is not None
        assert reconstructed.robot_model.model_path == PIPER_MODEL_PATH
        assert reconstructed.robot_model.end_effector_link == "gripper_base"
