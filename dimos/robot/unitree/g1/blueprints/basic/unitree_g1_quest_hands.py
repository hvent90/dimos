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

"""Unitree G1 GR00T WBC + Quest hands-only teleop.

The groot WBC stack plus the Quest module: hold both index triggers and
the robot's arms track your hands through the dual-arm IK task (joint
targets flow out the coordinator's joint_command JointState stream).
Quest thumbstick locomotion is disconnected — the robot balances in
place; the only headset-driven motion is the arms.

The pygame panel is included as the operator console (real hardware boots
unarmed + dry-run): Enter = arm (10 s ramp), K = disarm, Space = e-stop.
Its WASD keys can still walk the robot, but that stays with the operator
at the keyboard, not the headset.

Usage:
    dimos --simulation mujoco run unitree-g1-quest-hands  # sim
    dimos run unitree-g1-quest-hands                      # real hardware
"""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.unitree.g1.blueprints.basic.unitree_g1_groot_wbc import unitree_g1_groot_wbc
from dimos.robot.unitree.g1.g1_groot_wbc_teleop import G1GrootWbcTeleop
from dimos.robot.unitree.g1.quest_teleop import G1QuestTeleopModule

unitree_g1_quest_hands = autoconnect(
    unitree_g1_groot_wbc,
    G1QuestTeleopModule.blueprint(),
    G1GrootWbcTeleop.blueprint(),
).remappings(
    [
        (G1QuestTeleopModule, "left_controller_output", "coordinator_cartesian_command"),
        (G1QuestTeleopModule, "right_controller_output", "coordinator_cartesian_command"),
        # Park the thumbstick twists on an unconsumed stream: hands only.
        (G1QuestTeleopModule, "cmd_vel", "quest_cmd_vel_unused"),
        (G1GrootWbcTeleop, "cmd_vel", "tele_cmd_vel"),
    ]
)
