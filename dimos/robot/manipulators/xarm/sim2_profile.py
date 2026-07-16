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

"""xArm model bindings for sim2."""

from dimos.control.components import make_joints
from dimos.robot.manipulators.xarm.config import XARM7_SIM_PATH
from dimos.sim2.spec import (
    ControlInterface,
    EntityDescriptor,
    SimRobotSpec,
    WorldSpec,
)
from dimos.simulation.engines.robot_sim_binding import RobotSimSpec


def xarm7_sim2_robot(robot_id: str = "arm") -> SimRobotSpec:
    joints = tuple(make_joints(robot_id, 7))
    return SimRobotSpec(
        robot_id=robot_id,
        control_interface=ControlInterface.MANIPULATOR,
        dof=len(joints),
        joint_names=joints,
        model_path=XARM7_SIM_PATH,
        capabilities=frozenset({"gripper"}),
        backend_options={
            "mujoco_spec": RobotSimSpec(
                robot_id=robot_id,
                hardware_joints=joints,
                model_joint_names=tuple(f"joint{index}" for index in range(1, 8)),
                model_actuator_names=tuple(f"act{index}" for index in range(1, 8)),
            ),
            "gripper_actuator_name": "gripper",
            "gripper_joint_name": "left_driver_joint",
            "gripper_reversed": True,
        },
    )


def xarm7_sim2_world() -> WorldSpec:
    return WorldSpec(
        revision="xarm7-demo-v1",
        entities=tuple(
            EntityDescriptor(entity_id=name, backend_name=name, kind="dynamic")
            for name in ("apple", "orange", "cup")
        ),
    )
