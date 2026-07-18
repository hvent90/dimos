#!/usr/bin/env python3
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

"""Teleop blueprints for testing and deployment.

Single sim/real blueprints — pass `--simulation` to run inside MuJoCo, omit for real
hardware. The underlying coordinator blueprints branch on `global_config.simulation`.
"""

from dimos.constants import DEFAULT_CAPACITY_COLOR_IMAGE
from dimos.control.components import make_gripper_joints
from dimos.control.coordinator import ControlCoordinator
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.transport import LCMTransport, pSHMTransport
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.Image import Image
from dimos.robot.manipulators.common.blueprints import teleop_ik_task
from dimos.robot.manipulators.common.mixed import coordinator_teleop_dual
from dimos.robot.manipulators.common.sim import mujoco_if_sim
from dimos.robot.manipulators.piper.blueprints.teleop import coordinator_teleop_piper
from dimos.robot.manipulators.xarm.blueprints.teleop import (
    coordinator_teleop_xarm6,
    coordinator_teleop_xarm7,
)
from dimos.robot.manipulators.xarm.config import (
    XARM6_FK_MODEL,
    XARM6_SIM_PATH,
    xarm6_hardware,
)
from dimos.robot.unitree.go2.connection import GO2Connection
from dimos.teleop.quest.quest_extensions import (
    ArmTeleopModule,
    DrawArmTeleopModule,
    Go2TeleopModule,
    VideoArmTeleopModule,
)
from dimos.teleop.utils.trajectory_replay_module import TrajectoryReplayModule
from dimos.visualization.vis_module import vis_module

# Arm teleop with press-and-hold engage (has rerun viz)
teleop_quest_rerun = autoconnect(
    ArmTeleopModule.blueprint(),
    vis_module("rerun"),
).transports(
    {
        ("left_controller_output", PoseStamped): LCMTransport("/teleop/left_delta", PoseStamped),
        ("right_controller_output", PoseStamped): LCMTransport("/teleop/right_delta", PoseStamped),
    }
)


# XArm7 teleop (sim with --simulation, real otherwise): right controller -> xarm7
teleop_quest_xarm7 = autoconnect(
    ArmTeleopModule.blueprint(task_names={"right": "teleop_xarm"}),
    coordinator_teleop_xarm7,
).remappings([(ArmTeleopModule, "right_controller_output", "coordinator_cartesian_command")])


# XArm7 teleop + camera streaming into the Quest scene as a panel.
teleop_quest_xarm7_video = (
    autoconnect(
        VideoArmTeleopModule.blueprint(task_names={"right": "teleop_xarm"}),
        coordinator_teleop_xarm7,
    )
    .remappings(
        [(VideoArmTeleopModule, "right_controller_output", "coordinator_cartesian_command")]
    )
    .transports(
        {
            ("color_image", Image): LCMTransport("/teleop/color_image", Image),
        }
    )
)


# Piper teleop (sim with --simulation, real otherwise): left controller -> piper arm
teleop_quest_piper = autoconnect(
    ArmTeleopModule.blueprint(task_names={"left": "teleop_piper"}),
    coordinator_teleop_piper,
).remappings([(ArmTeleopModule, "left_controller_output", "coordinator_cartesian_command")])


# XArm6 teleop (sim with --simulation, real otherwise): right controller -> xarm6
teleop_quest_xarm6 = autoconnect(
    ArmTeleopModule.blueprint(task_names={"right": "teleop_xarm"}),
    coordinator_teleop_xarm6,
).remappings([(ArmTeleopModule, "right_controller_output", "coordinator_cartesian_command")])


# XArm6 teleop + VR draw-a-line trajectory record/replay.
#
# TrajectoryReplayModule is spliced between the Quest module and the coordinator:
# it forwards live teleop through untouched (so X/A press-and-hold teleop still
# works), and adds a record/replay gesture — hold right trigger to sketch a delta
# trajectory in the air (arm stays still), press right B to replay it onto the
# arm at half speed.
#
# Uses a dedicated coordinator with a looser per-tick joint-delta safety limit
# (30° vs the default 5°): replayed trajectories can command larger EE jumps
# between ticks than smooth live teleop, and the tighter clamp rejects them.
#
# Own hardware handle so it doesn't share the module-level xarm6 teleop hardware.
_xarm6_vrtraj_hw = xarm6_hardware("arm", gripper=True)
#
# Wiring (all four crossing edges are remapped to unique intermediate names so
# nothing double-connects; ports sharing an (effective_name, type) share a bus):
#   Quest right_controller_output ─→ traj controller_in  (via "traj_controller_in")
#   Quest teleop_buttons          ─→ traj buttons_in      (via "traj_buttons_in")
#   traj cartesian_out            ─→ coordinator_cartesian_command
#   traj buttons_out              ─→ coordinator teleop_buttons
coordinator_teleop_xarm6_vrtraj = autoconnect(
    ControlCoordinator.blueprint(
        hardware=[_xarm6_vrtraj_hw],
        tasks=[
            teleop_ik_task(
                _xarm6_vrtraj_hw,
                model_path=XARM6_FK_MODEL,
                ee_joint_id=6,
                hand="right",
                name="teleop_xarm",
                params={
                    "gripper_joint": make_gripper_joints("arm")[0],
                    "gripper_open_pos": 0.85,
                    "gripper_closed_pos": 0.0,
                    "max_joint_delta_deg": 30.0,
                },
            ),
        ],
    ),
    *mujoco_if_sim(XARM6_SIM_PATH, len(_xarm6_vrtraj_hw.joints)),
)

teleop_quest_xarm6_vrtraj = autoconnect(
    DrawArmTeleopModule.blueprint(task_names={"right": "teleop_xarm"}),
    TrajectoryReplayModule.blueprint(),
    coordinator_teleop_xarm6_vrtraj,
).remappings(
    [
        (DrawArmTeleopModule, "right_controller_output", "traj_controller_in"),
        (TrajectoryReplayModule, "controller_in", "traj_controller_in"),
        (DrawArmTeleopModule, "teleop_buttons", "traj_buttons_in"),
        (TrajectoryReplayModule, "buttons_in", "traj_buttons_in"),
        (TrajectoryReplayModule, "cartesian_out", "coordinator_cartesian_command"),
        (TrajectoryReplayModule, "buttons_out", "teleop_buttons"),
        # Replay progress → back to the teleop module, which relays it to the
        # headset over /ws so the web app can consume the drawn line.
        (TrajectoryReplayModule, "replay_progress", "traj_progress"),
        (DrawArmTeleopModule, "replay_progress", "traj_progress"),
    ]
)


# Dual arm teleop: right -> piper, left -> xarm6 (TeleopIK, real-only)
teleop_quest_dual = autoconnect(
    ArmTeleopModule.blueprint(task_names={"right": "teleop_piper", "left": "teleop_xarm"}),
    coordinator_teleop_dual,
).remappings(
    [
        (ArmTeleopModule, "right_controller_output", "coordinator_cartesian_command"),
        (ArmTeleopModule, "left_controller_output", "coordinator_cartesian_command"),
    ]
)


# Go2 quadruped: thumbstick velocity teleop + camera streamed to the headset.
teleop_quest_go2 = (
    autoconnect(
        Go2TeleopModule.blueprint(),
        GO2Connection.blueprint(),
    )
    .transports(
        {
            ("cmd_vel", Twist): LCMTransport("/cmd_vel", Twist),
            ("color_image", Image): pSHMTransport(
                "color_image", default_capacity=DEFAULT_CAPACITY_COLOR_IMAGE
            ),
        }
    )
    .global_config(robot_model="unitree_go2")
)
