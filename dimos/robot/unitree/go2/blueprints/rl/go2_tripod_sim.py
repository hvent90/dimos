#!/usr/bin/env python3
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

"""Go2 tripod RL policy in MuJoCo + Quest teleop (joysticks + FR arm).

Loads the trained 3-leg velocity policy ([data/2026-05-28_11-42-04/model_1200.pt])
and walks the Go2 in MuJoCo. The FR ("right arm") is owned by a
QuestJointTask gated by the right primary (A) button; the policy controls
the other 9 leg joints.

Quest controller mapping:
    left thumbstick   -> twist linear (drive)
    right thumbstick X -> twist angular.z (yaw)
    right A (hold)    -> engage FR-leg pose tracking
    right hand pose   -> FR_hip / FR_thigh / FR_calf deltas

Connect the Quest at https://<host>:8443/teleop (cert auth).

Usage:
    dimos run go2-tripod-sim
"""

from __future__ import annotations

from dimos.control.components import HardwareComponent, HardwareType
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.transport import LCMTransport
from dimos.hardware.whole_body.spec import WholeBodyConfig
from dimos.learning.inference.obs_builder import GO2_JOINT_ORDER
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.teleop.quest.quest_extensions import JoystickTwistTeleopModule
from dimos.teleop.quest.quest_types import Buttons


_DEFAULT_POLICY = "data/2026-05-28_11-42-04/model_1200.pt"
_DEFAULT_MJCF = "data/go2_mjlab/xmls/scene_go2.xml"

# PD gains from training env (hip=20/1, thigh=20/1, calf=40/2).
_KP = (20.0, 20.0, 40.0,
       20.0, 20.0, 40.0,
       20.0, 20.0, 40.0,
       20.0, 20.0, 40.0)
_KD = (1.0, 1.0, 2.0,
       1.0, 1.0, 2.0,
       1.0, 1.0, 2.0,
       1.0, 1.0, 2.0)

_HW = "go2"
_joints = [f"{_HW}/{j}" for j in GO2_JOINT_ORDER]

# FR leg's three joints (in mjlab/training order). These are owned by the
# Quest teleop task; the RL policy masks them.
_FR_JOINTS = [f"{_HW}/FR_hip_joint", f"{_HW}/FR_thigh_joint", f"{_HW}/FR_calf_joint"]

# Rest pose for FR while teleop is active: matches training default
# (hip +0.1, thigh 0.9, calf -1.8). Operator deltas perturb around this.
_FR_REST = [0.1, 0.9, -1.8]

# Task name the Quest module stamps into right-controller PoseStamped frame_id.
# The coordinator routes the PoseStamped to the task with this exact name.
_FR_TASK_NAME = "fr_teleop"


go2_tripod_sim = (
    autoconnect(
        ControlCoordinator.blueprint(
            tick_rate=100,
            hardware=[
                HardwareComponent(
                    hardware_id=_HW,
                    hardware_type=HardwareType.WHOLE_BODY,
                    joints=_joints,
                    adapter_type="sim_mujoco_go2",
                    adapter_kwargs={
                        "mjcf_path": _DEFAULT_MJCF,
                        "render": True,
                    },
                    wb_config=WholeBodyConfig(kp=_KP, kd=_KD),
                ),
            ],
            tasks=[
                # RL walking policy - masks FR so the QuestJointTask owns it.
                TaskConfig(
                    name="rl_walk_go2",
                    type="rl_policy_go2",
                    joint_names=_joints,
                    priority=10,
                    auto_start=True,
                    params={
                        "policy_path": _DEFAULT_POLICY,
                        "hardware_id": _HW,
                        "inference_period": 0.02,
                        "mask_fr": True,
                        "device": "cpu",
                    },
                ),
                # FR-leg joint-space teleop. Receives PoseStamped via the
                # coordinator's cartesian_command routing (frame_id matches
                # this task name). Higher priority than rl_walk_go2 so it
                # wins per-joint arbitration on FR_*.
                TaskConfig(
                    name=_FR_TASK_NAME,
                    type="quest_joint",
                    joint_names=_FR_JOINTS,
                    priority=20,
                    auto_start=True,
                    params={
                        "rest_pose": _FR_REST,
                        "command_timeout": 0.3,
                        # Controller -> FR leg joint mapping. Controller pose is
                        # already in robot frame (X=fwd, Y=left, Z=up - see
                        # teleop_transforms.VR_TO_ROBOT_FRAME). 3x3 row-major:
                        # rows = controller (x, y, z), cols = (hip, thigh, calf).
                        #   x fwd        -> thigh pitch fwd
                        #   y left       -> hip rotates inward (-)
                        #   z up         -> calf extends (- = straighter leg)
                        "axis_map": [
                            0.0, -3.0,  0.0,   # x -> -thigh  (push fwd  -> paw back)
                            3.0,  0.0,  0.0,   # y ->  hip    (controller left -> paw right)
                            0.0,  0.0, -3.0,   # z -> -calf   (up -> paw up)
                        ],
                    },
                ),
            ],
        ),
        JoystickTwistTeleopModule.blueprint(
            linear_speed=1.0,
            angular_speed=0.8,
            # Route the right controller PoseStamped to the FR task.
            task_names={"right": _FR_TASK_NAME},
        ),
    )
    .transports(
        {
            # Joystick twist -> coordinator twist_command -> RLPolicyTask.set_velocity_command.
            ("twist_command", Twist): LCMTransport("/cmd_vel", Twist),
            ("cmd_vel", Twist): LCMTransport("/cmd_vel", Twist),
            # Right controller pose -> coordinator cartesian_command -> QuestJointTask.
            ("cartesian_command", PoseStamped): LCMTransport(
                "/cartesian_command", PoseStamped
            ),
            ("right_controller_output", PoseStamped): LCMTransport(
                "/cartesian_command", PoseStamped
            ),
            # Buttons - coordinator forwards to all tasks.
            ("buttons", Buttons): LCMTransport("/buttons", Buttons),
            ("joint_state", JointState): LCMTransport(
                "/coordinator/joint_state", JointState
            ),
        }
    )
)


__all__ = ["go2_tripod_sim"]
