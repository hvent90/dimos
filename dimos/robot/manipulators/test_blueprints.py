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

from typing import Any

from dimos.control.coordinator import ControlCoordinator
from dimos.core.coordination.blueprints import Blueprint
from dimos.core.global_config import global_config
from dimos.manipulation.manipulation_module import ManipulationModule, ManipulationModuleConfig
from dimos.manipulation.visualization.config import NoManipulationVisualizationConfig
from dimos.robot.manipulators.common.blueprints import planner
from dimos.robot.manipulators.openarm.blueprints.teleop import (
    openarm_mini_left_teleop_viser,
    openarm_mini_teleop_openarm,
)
from dimos.robot.manipulators.xarm.blueprints.basic import (
    dual_xarm6_planner,
    xarm6_planner_only,
    xarm7_planner_coordinator,
)
from dimos.robot.manipulators.xarm.config import make_xarm7_model_config
from dimos.teleop.openarm_mini.adapter import OpenArmMiniTeleopAdapter
from dimos.teleop.openarm_mini.teleop_module import OpenArmMiniTeleopModule
from dimos.teleop.openarm_mini.viser_visualizer import OpenArmJointStateViserModule
from dimos.teleop.quest.blueprints import teleop_quest_go2, teleop_quest_xarm7
from dimos.teleop.quest.quest_extensions import ArmTeleopModule, Go2TeleopModule
from dimos.teleop.runtime.teleop_module import TeleopModule


def _manipulation_kwargs(blueprint: Blueprint) -> dict[str, Any]:
    return next(atom.kwargs for atom in blueprint.blueprints if atom.module is ManipulationModule)


def _manipulation_config(blueprint: Blueprint) -> ManipulationModuleConfig:
    return ManipulationModuleConfig(**_manipulation_kwargs(blueprint))


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


def test_openarm_mini_teleop_blueprint_wires_joint_commands() -> None:
    teleop_atom = next(
        atom for atom in openarm_mini_teleop_openarm.blueprints if atom.module is TeleopModule
    )
    coordinator_atom = next(
        atom for atom in openarm_mini_teleop_openarm.blueprints if atom.module is ControlCoordinator
    )

    assert isinstance(teleop_atom.kwargs["adapter"], OpenArmMiniTeleopAdapter)
    assert any(
        stream.name == "joint_command" and stream.direction == "out"
        for stream in teleop_atom.streams
    )
    assert any(
        stream.name == "joint_command" and stream.direction == "in"
        for stream in coordinator_atom.streams
    )
    assert [task.type for task in coordinator_atom.kwargs["tasks"]] == ["servo", "servo"]
    assert [hardware.hardware_id for hardware in coordinator_atom.kwargs["hardware"]] == [
        "left_arm",
        "right_arm",
    ]


def test_openarm_mini_left_teleop_viser_blueprint_is_visualization_only() -> None:
    teleop_atom = next(
        atom
        for atom in openarm_mini_left_teleop_viser.blueprints
        if atom.module is OpenArmMiniTeleopModule
    )
    visualizer_atom = next(
        atom
        for atom in openarm_mini_left_teleop_viser.blueprints
        if atom.module is OpenArmJointStateViserModule
    )

    assert teleop_atom.kwargs["openarm_mini"].enabled_sides == ("left",)
    assert visualizer_atom.kwargs["robot_id"] == "openarm_left"
    assert visualizer_atom.kwargs["robot"].joint_names == [
        f"openarm_left_joint{i}" for i in range(1, 8)
    ]
    assert any(
        stream.name == "joint_command" and stream.direction == "out"
        for stream in teleop_atom.streams
    )
    assert any(
        stream.name == "joint_command" and stream.direction == "in"
        for stream in visualizer_atom.streams
    )
    assert all(
        atom.module is not ControlCoordinator for atom in openarm_mini_left_teleop_viser.blueprints
    )
    assert all("hardware" not in atom.kwargs for atom in openarm_mini_left_teleop_viser.blueprints)

    for atom in (teleop_atom, visualizer_atom):
        atom.module.resolve_config({**atom.kwargs, "g": global_config})

    resolved = OpenArmMiniTeleopModule.resolve_config(
        {
            **teleop_atom.kwargs,
            "openarm_mini": {
                "port_left": "/dev/ttyACM0",
            },
            "g": global_config,
        }
    )
    assert resolved.openarm_mini.port_left == "/dev/ttyACM0"
    assert resolved.openarm_mini.enabled_sides == ("left",)


def test_existing_quest_teleop_blueprints_still_use_quest_modules() -> None:
    assert any(atom.module is ArmTeleopModule for atom in teleop_quest_xarm7.blueprints)
    assert any(atom.module is Go2TeleopModule for atom in teleop_quest_go2.blueprints)
    assert all(atom.module is not TeleopModule for atom in teleop_quest_xarm7.blueprints)
