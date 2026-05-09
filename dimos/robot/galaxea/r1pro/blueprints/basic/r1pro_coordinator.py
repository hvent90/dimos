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

"""R1 Pro ControlCoordinator: R1ProConnection Module + transport_lcm bridges.

Mirrors ``unitree_g1_coordinator.py``. Whole-body upper-body (18-DOF) goes
through ``TransportWholeBodyAdapter``; the holonomic chassis (3-DOF) goes
through ``TransportTwistAdapter``. No R1Pro-specific adapter code — same
shape as the G1 wiring.

Usage:
    dimos run r1pro-coordinator
"""

from __future__ import annotations

from dimos.control.components import HardwareComponent, HardwareType, make_twist_base_joints
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.transport import LCMTransport
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.robot.galaxea.r1pro.connection import R1PRO_UPPER_BODY_JOINTS, R1ProConnection

_chassis_joints = make_twist_base_joints("chassis")

r1pro_coordinator = (
    autoconnect(
        R1ProConnection.blueprint(),
        ControlCoordinator.blueprint(
            tick_rate=100,
            hardware=[
                HardwareComponent(
                    hardware_id="r1pro",
                    hardware_type=HardwareType.WHOLE_BODY,
                    joints=R1PRO_UPPER_BODY_JOINTS,
                    adapter_type="transport_lcm",
                ),
                HardwareComponent(
                    hardware_id="chassis",
                    hardware_type=HardwareType.BASE,
                    joints=_chassis_joints,
                    adapter_type="transport_lcm",
                ),
            ],
            tasks=[
                TaskConfig(
                    name="servo_r1pro",
                    type="servo",
                    joint_names=R1PRO_UPPER_BODY_JOINTS,
                    priority=10,
                ),
                TaskConfig(
                    name="vel_chassis",
                    type="velocity",
                    joint_names=_chassis_joints,
                    priority=10,
                ),
            ],
        ),
    )
    # Module's `cmd_vel`/`odom` collide with the chassis transport adapter's
    # /{hw}/cmd_vel + /{hw}/odom topics — rename so the adapter (hw_id="chassis")
    # owns the canonical names.
    .remappings(
        [
            (R1ProConnection, "cmd_vel", "chassis_cmd_vel"),
            (R1ProConnection, "odom", "chassis_odom"),
        ]
    )
    .transports(
        {
            # WholeBody bridge (hw_id="r1pro"). TransportWholeBodyAdapter
            # subscribes /{hw}/imu — only one IMU goes there.
            ("motor_states", JointState): LCMTransport("/r1pro/motor_states", JointState),
            ("imu_chassis", Imu): LCMTransport("/r1pro/imu", Imu),
            ("imu_torso", Imu): LCMTransport("/r1pro/imu_torso", Imu),
            ("motor_command", MotorCommandArray): LCMTransport(
                "/r1pro/motor_command", MotorCommandArray
            ),
            # Twist bridge (hw_id="chassis").
            ("chassis_cmd_vel", Twist): LCMTransport("/chassis/cmd_vel", Twist),
            ("chassis_odom", PoseStamped): LCMTransport("/chassis/odom", PoseStamped),
            # Sensor pass-throughs — downstream consumers (rerun bridge,
            # perception modules, etc.) attach to these topics directly.
            ("head_color", Image): LCMTransport("/r1pro/head_color", Image),
            ("head_depth", Image): LCMTransport("/r1pro/head_depth", Image),
            ("chassis_front_left", Image): LCMTransport("/r1pro/chassis_front_left", Image),
            ("chassis_front_right", Image): LCMTransport("/r1pro/chassis_front_right", Image),
            ("chassis_left", Image): LCMTransport("/r1pro/chassis_left", Image),
            ("chassis_right", Image): LCMTransport("/r1pro/chassis_right", Image),
            ("chassis_rear", Image): LCMTransport("/r1pro/chassis_rear", Image),
            ("lidar", PointCloud2): LCMTransport("/r1pro/lidar", PointCloud2),
            ("wrist_left_color", Image): LCMTransport("/r1pro/wrist_left_color", Image),
            ("wrist_left_depth", Image): LCMTransport("/r1pro/wrist_left_depth", Image),
            ("wrist_right_color", Image): LCMTransport("/r1pro/wrist_right_color", Image),
            ("wrist_right_depth", Image): LCMTransport("/r1pro/wrist_right_depth", Image),
            # ControlCoordinator outs.
            ("joint_state", JointState): LCMTransport(
                "/coordinator/joint_state", JointState
            ),
            ("joint_command", JointState): LCMTransport("/r1pro/joint_command", JointState),
        }
    )
)


__all__ = ["r1pro_coordinator"]
