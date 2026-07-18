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

"""Small blueprint helpers shared by manipulator stacks."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any

from dimos.control.components import HardwareComponent
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.coordination.blueprints import Blueprint
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.robot.manipulators.common.topics import (
    CARTESIAN_IK_TASK_NAME,
    COORDINATOR_FRAME_ID,
    DEFAULT_TRAJECTORY_TASK_NAME,
    EEF_TWIST_TASK_NAME,
    trajectory_task_name,
)


def trajectory_task(
    hardware: HardwareComponent,
    *,
    name: str | None = None,
    priority: int = 10,
) -> TaskConfig:
    return TaskConfig(
        name=name or trajectory_task_name(hardware.hardware_id),
        type="trajectory",
        joint_names=hardware.joints,
        priority=priority,
    )


def _resolve_control_ik(
    hardware: HardwareComponent,
    robot_model: RobotModelConfig,
    control_ik: Mapping[str, object] | None,
) -> dict[str, object]:
    coordinator_joints = robot_model.get_coordinator_joint_names()
    if hardware.joints != coordinator_joints:
        raise ValueError("hardware joints must match RobotModelConfig coordinator joints")
    payload = dict(control_ik or {})
    payload["robot_model"] = _serialize_robot_model(robot_model)
    return payload


def _serialize_robot_model(robot_model: RobotModelConfig) -> dict[str, object]:
    """Serialize the authoritative robot model without runtime-only objects."""
    base_pose = robot_model.base_pose
    return {
        "name": robot_model.name,
        "model_path": str(robot_model.model_path),
        "base_pose": {
            "ts": float(base_pose.ts),
            "frame_id": base_pose.frame_id,
            "position": [base_pose.position.x, base_pose.position.y, base_pose.position.z],
            "orientation": [
                base_pose.orientation.x,
                base_pose.orientation.y,
                base_pose.orientation.z,
                base_pose.orientation.w,
            ],
        },
        "joint_names": list(robot_model.joint_names),
        "end_effector_link": robot_model.end_effector_link,
        "base_link": robot_model.base_link,
        "package_paths": {name: str(path) for name, path in robot_model.package_paths.items()},
        "joint_limits_lower": robot_model.joint_limits_lower,
        "joint_limits_upper": robot_model.joint_limits_upper,
        "velocity_limits": robot_model.velocity_limits,
        "auto_convert_meshes": robot_model.auto_convert_meshes,
        "xacro_args": dict(robot_model.xacro_args),
        "collision_exclusion_pairs": list(robot_model.collision_exclusion_pairs),
        "max_velocity": robot_model.max_velocity,
        "max_acceleration": robot_model.max_acceleration,
        "joint_name_mapping": dict(robot_model.joint_name_mapping),
        "coordinator_task_name": robot_model.coordinator_task_name,
        "gripper_hardware_id": robot_model.gripper_hardware_id,
        "tf_extra_links": list(robot_model.tf_extra_links),
        "home_joints": robot_model.home_joints,
        "pre_grasp_offset": robot_model.pre_grasp_offset,
    }


def cartesian_ik_task(
    hardware: HardwareComponent,
    *,
    name: str = CARTESIAN_IK_TASK_NAME,
    priority: int = 10,
    control_ik: Mapping[str, object] | None = None,
    robot_model: RobotModelConfig,
) -> TaskConfig:
    resolved_control_ik = _resolve_control_ik(hardware, robot_model, control_ik)
    return TaskConfig(
        name=name,
        type="cartesian_ik",
        joint_names=hardware.joints,
        priority=priority,
        params={
            "control_ik": resolved_control_ik,
        },
    )


def eef_twist_task(
    hardware: HardwareComponent,
    *,
    name: str = EEF_TWIST_TASK_NAME,
    priority: int = 10,
    control_ik: Mapping[str, object] | None = None,
    robot_model: RobotModelConfig,
) -> TaskConfig:
    resolved_control_ik = _resolve_control_ik(hardware, robot_model, control_ik)
    return TaskConfig(
        name=name,
        type="eef_twist",
        joint_names=hardware.joints,
        priority=priority,
        params={
            "control_ik": resolved_control_ik,
        },
    )


def teleop_ik_task(
    hardware: HardwareComponent,
    *,
    model_path: Path,
    ee_joint_id: int,
    hand: str,
    name: str,
    priority: int = 10,
    params: dict[str, Any] | None = None,
) -> TaskConfig:
    task_params: dict[str, Any] = {
        "model_path": model_path,
        "ee_joint_id": ee_joint_id,
        "hand": hand,
    }
    if params:
        task_params.update(params)
    return TaskConfig(
        name=name,
        type="teleop_ik",
        joint_names=hardware.joints,
        priority=priority,
        params=task_params,
    )


def coordinator(
    *,
    hardware: Sequence[HardwareComponent] = (),
    tasks: Sequence[TaskConfig] = (),
    tick_rate: float = 100.0,
    publish_joint_state: bool = True,
    joint_state_frame_id: str = COORDINATOR_FRAME_ID,
) -> Blueprint:
    return ControlCoordinator.blueprint(
        tick_rate=tick_rate,
        publish_joint_state=publish_joint_state,
        joint_state_frame_id=joint_state_frame_id,
        hardware=list(hardware),
        tasks=list(tasks),
    )


def planner(
    *,
    robots: Sequence[RobotModelConfig],
    planning_timeout: float = 10.0,
    visualization: dict[str, Any] | None = None,
    **kwargs: Any,
) -> Blueprint:
    module_kwargs: dict[str, Any] = {
        "robots": list(robots),
        "planning_timeout": planning_timeout,
        **kwargs,
    }
    if visualization is not None:
        module_kwargs["visualization"] = visualization
    return ManipulationModule.blueprint(**module_kwargs)


def default_trajectory_task_name(hardware_id: str) -> str:
    if hardware_id == "arm":
        return DEFAULT_TRAJECTORY_TASK_NAME
    return trajectory_task_name(hardware_id)
