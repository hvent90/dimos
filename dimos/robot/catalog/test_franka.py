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

"""Tests for the Franka Panda robot catalog."""

from __future__ import annotations

from dimos.control.components import HardwareType
from dimos.robot.catalog.franka import (
    FRANKA_PANDA_FK_MODEL,
    FRANKA_PANDA_JOINT_LIMITS_LOWER,
    FRANKA_PANDA_JOINT_LIMITS_UPPER,
    FRANKA_PANDA_JOINT_NAMES,
    FRANKA_PANDA_MODEL,
    FRANKA_PANDA_SRDF,
    franka_panda,
)


def _lfs_filename(path: object) -> object:
    return object.__getattribute__(path, "_lfs_filename")


def test_franka_panda_catalog_defaults_to_mock_control() -> None:
    """The Panda catalog config is mock-control first."""
    config = franka_panda()

    assert config.name == "panda"
    assert config.adapter_type == "mock"
    assert config.address is None
    assert config.joint_names == FRANKA_PANDA_JOINT_NAMES
    assert config.end_effector_link == "panda_hand"
    assert config.base_link == "panda_link0"
    assert config.joint_limits_lower == FRANKA_PANDA_JOINT_LIMITS_LOWER
    assert config.joint_limits_upper == FRANKA_PANDA_JOINT_LIMITS_UPPER


def test_franka_panda_uses_lfs_backed_model_and_srdf_paths() -> None:
    """Panda URDF/SRDF resources follow the repo LFS-backed data pattern."""
    assert _lfs_filename(FRANKA_PANDA_MODEL) == "franka_description/urdf/panda.urdf.xacro"
    assert _lfs_filename(FRANKA_PANDA_FK_MODEL) == "franka_description/urdf/panda.urdf"
    assert _lfs_filename(FRANKA_PANDA_SRDF) == "franka_description/srdf/panda.srdf"


def test_franka_panda_hardware_component_uses_mock_adapter_and_prefixed_joints() -> None:
    """RobotConfig conversion feeds ControlCoordinator mock hardware."""
    config = franka_panda()

    component = config.to_hardware_component()

    assert component.hardware_id == "panda"
    assert component.hardware_type == HardwareType.MANIPULATOR
    assert component.adapter_type == "mock"
    assert component.joints == [f"panda/{joint}" for joint in FRANKA_PANDA_JOINT_NAMES]
    assert component.adapter_kwargs["initial_positions"] == config.home_joints


def test_franka_panda_robot_model_config_preserves_vamp_joint_order() -> None:
    """Manipulation robot model config keeps the official Panda joint order."""
    config = franka_panda()

    model = config.to_robot_model_config()

    assert model.name == "panda"
    assert _lfs_filename(model.model_path) == _lfs_filename(FRANKA_PANDA_MODEL)
    assert model.joint_names == FRANKA_PANDA_JOINT_NAMES
    assert model.joint_limits_lower == FRANKA_PANDA_JOINT_LIMITS_LOWER
    assert model.joint_limits_upper == FRANKA_PANDA_JOINT_LIMITS_UPPER
    assert model.joint_name_mapping == {
        f"panda/{joint}": joint for joint in FRANKA_PANDA_JOINT_NAMES
    }


def test_franka_panda_task_config_supports_mock_coordinator_benchmark_path() -> None:
    """Catalog output can build the same mock coordinator task shape as xArm/Piper."""
    config = franka_panda()

    task = config.to_task_config(
        task_type="cartesian_ik",
        task_name="cartesian_ik_panda",
        model_path=FRANKA_PANDA_FK_MODEL,
        ee_joint_id=config.dof,
    )

    assert task.name == "cartesian_ik_panda"
    assert task.type == "cartesian_ik"
    assert task.joint_names == [f"panda/{joint}" for joint in FRANKA_PANDA_JOINT_NAMES]
    assert _lfs_filename(task.params["model_path"]) == _lfs_filename(FRANKA_PANDA_FK_MODEL)
    assert task.params["ee_joint_id"] == 7
