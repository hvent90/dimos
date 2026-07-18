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

"""Unitree G1 GR00T WBC + Quest teleop + WASD arming panel, in one launch.

The groot WBC stack (Quest thumbsticks walk the robot, both triggers engage
hand tracking through the dual-arm IK task) plus the :class:`G1GrootWbcTeleop`
pygame panel. On real hardware the panel is the arming UI (Enter = dry-run
off + activate, 10 s ramp) and doubles as a WASD fallback if the headset
drops; its twists go through the MovementManager's tele_cmd_vel arbitration.

Unlike ``unitree-g1-teleop`` this skips episode recording, and the RealSense
(operator view in the headset) is only added when a device is actually
plugged in, so the demo boots on a machine without one.

Usage:
    dimos run unitree-g1-teleop-demo                      # real hardware
    dimos --simulation mujoco run unitree-g1-teleop-demo  # sim rehearsal
"""

from __future__ import annotations

from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.global_config import global_config
from dimos.robot.unitree.g1.blueprints.basic.unitree_g1_groot_wbc import unitree_g1_groot_wbc
from dimos.robot.unitree.g1.g1_groot_wbc_teleop import G1GrootWbcTeleop
from dimos.robot.unitree.g1.quest_teleop import G1QuestTeleopModule


def _realsense_present() -> bool:
    try:
        import pyrealsense2 as rs  # type: ignore[import-untyped]

        return len(rs.context().devices) > 0
    except Exception:
        return False


def _camera_if_present() -> tuple[Blueprint, ...]:
    """The groot MuJoCo sim exposes no color camera, and off-sim the module
    fails to start (killing the whole launch) when no device is plugged in."""
    if global_config.simulation or not _realsense_present():
        return ()
    from dimos.robot.unitree.g1.blueprints.basic.unitree_g1_teleop import (
        DedicatedRealSenseCamera,
    )

    return (DedicatedRealSenseCamera.blueprint(enable_pointcloud=False),)


unitree_g1_teleop_demo = autoconnect(
    unitree_g1_groot_wbc,
    G1QuestTeleopModule.blueprint(),
    G1GrootWbcTeleop.blueprint(),
    *_camera_if_present(),
).remappings(
    [
        (G1QuestTeleopModule, "left_controller_output", "coordinator_cartesian_command"),
        (G1QuestTeleopModule, "right_controller_output", "coordinator_cartesian_command"),
        (G1GrootWbcTeleop, "cmd_vel", "tele_cmd_vel"),
    ]
)
