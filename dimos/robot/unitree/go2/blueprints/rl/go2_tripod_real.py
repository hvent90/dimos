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

"""Go2 tripod RL policy on REAL hardware via Go2WholeBodyConnection.

Real-hardware counterpart to [go2_tripod_sim]. Same RLPolicyTask, same
joint conventions, swap the sim MuJoCo adapter for the DDS-backed Unitree
connection.

Safety differences vs sim:
- RL task is `auto_start=False` and starts disarmed. Arm via
  `coordinator.set_activated(True)` RPC after the operator confirms the
  robot is in a stable stance.
- `activation_ramp_seconds=1.5` smoothly blends from the robot's pose at
  arming toward the policy's target over 1.5s, protecting against a lurch
  if the legs aren't at the default standing pose when armed.
- kp/kd match the training env (hip=20/1, thigh=20/1, calf=40/2) so the
  closed-loop dynamics match what the policy saw during PPO rollouts.

Usage:
    ROBOT_INTERFACE=eth0 dimos run go2-tripod-real

The robot will load the policy, hold safe-stop, and wait. To start walking,
either bring the robot to a tripod stance manually (or with Unitree's stand
command, then release sport mode) and arm via the coordinator RPC.

TODO (deferred): no built-in stand-up sequence yet. Operator is responsible
for putting the robot at a safe standing pose before arming. See
[data/notes/go2_rl_stand_gains.md] for the design notes on Unitree's
`_targetPos_1` (kp=50/kd=3.5) stand pose and the three-phase gain ramp
plan when we come back to this.
"""

from __future__ import annotations

import os

from dimos.control.components import HardwareComponent, HardwareType, make_quadruped_joints
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.transport import LCMTransport
from dimos.hardware.whole_body.spec import WholeBodyConfig
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.robot.unitree.go2.wholebody_connection import Go2WholeBodyConnection

_HW = "go2"
_DEFAULT_POLICY = "data/2026-05-28_11-42-04/model_1200.pt"

# Training env PD gains (hip=20/1, thigh=20/1, calf=40/2 per leg).
# Wire order is FR/FL/RR/RL but the pattern is per-leg-symmetric so the
# tuple is identical regardless of leg ordering.
_KP = (20.0, 20.0, 40.0, 20.0, 20.0, 40.0, 20.0, 20.0, 40.0, 20.0, 20.0, 40.0)
_KD = (1.0, 1.0, 2.0, 1.0, 1.0, 2.0, 1.0, 1.0, 2.0, 1.0, 1.0, 2.0)

_joints = make_quadruped_joints(_HW)


go2_tripod_real = autoconnect(
    # DDS connection to the real Go2 - rt/lowstate (sub) + rt/lowcmd (pub).
    Go2WholeBodyConnection.blueprint(
        release_sport_mode=True,
        network_interface=os.getenv("ROBOT_INTERFACE", ""),
    ),
    ControlCoordinator.blueprint(
        # SLOW BRING-UP: 1 Hz tick = one full policy output per second.
        # Gives the operator a clear window to abort if a commanded pose
        # looks dangerous. The Go2WholeBodyConnection still publishes the
        # held target at 500 Hz via its own publish loop, so PD tracking
        # stays tight between coordinator ticks - the robot just holds
        # the last commanded pose for ~1s before the next policy update.
        # Bump back to 500 once you trust the policy on this robot.
        # Tick rate ladder for hardware bring-up:
        #   1   Hz - first life: see every commanded pose, 1s abort window
        #   5   Hz - smoother gait, can still observe per tick
        #   10  Hz - real-time feel, hard to abort visually
        #   50  Hz - training rate; full policy speed
        # Start at the bottom of the ladder you've verified, bump up after
        # observing the robot is stable + self-balancing at the current rate.
        tick_rate=5,
        hardware=[
            HardwareComponent(
                hardware_id=_HW,
                hardware_type=HardwareType.WHOLE_BODY,
                joints=_joints,
                adapter_type="transport_lcm",
                wb_config=WholeBodyConfig(kp=_KP, kd=_KD),
            ),
        ],
        tasks=[
            # RL walking policy. Starts disarmed - arm with
            # set_activated(True) once the operator confirms safe stance.
            TaskConfig(
                name="rl_walk_go2",
                type="rl_policy_go2",
                joint_names=_joints,
                priority=10,
                # auto_start=True is safe now because Go2WholeBodyConnection
                # gates _on_motor_command on _low_cmd_ready - early commands
                # from the coordinator are dropped on the floor until the
                # startup hold seed has run. So even if the policy starts
                # emitting before "Sport mode released", the publish loop
                # keeps holding the seeded pose and the policy's commands
                # only take effect once the connection is fully booted.
                auto_start=True,
                params={
                    "policy_path": _DEFAULT_POLICY,
                    "hardware_id": _HW,
                    # inference_period <= 1/tick_rate means the policy runs
                    # every tick. 0.02 is the training rate (50 Hz); using
                    # the smaller value here means "always run".
                    "inference_period": 0.02,
                    "mask_fr": False,
                    "device": "cpu",
                    # Lifecycle timing in WALL-CLOCK seconds (independent of
                    # tick_rate). At 5+ Hz the ramp is naturally smooth so we
                    # don't need it long - just enough for the body to settle
                    # from sport-stand pose to the policy's expected default.
                    #   pre_ramp_hold:  observation window before motion (2s)
                    #   activation_ramp: blend to policy target (3s)
                    #   post_ramp_hold:  observe at policy target (2s)
                    #   then: live policy
                    "pre_ramp_hold_seconds": 2.0,
                    "activation_ramp_seconds": 3.0,
                    "post_ramp_hold_seconds": 2.0,
                    # DISABLED for now (0.0 = off). Earlier run showed
                    # silently capping the command causes wind-up: the
                    # policy stores last_action assuming full tracking,
                    # the obs's last_actions term drifts out-of-dist,
                    # the policy starts asking for ±2 rad deltas. Fix
                    # later by also clamping _last_action (reconcile
                    # with what we actually sent), but for now trust
                    # the policy fully.
                    "max_joint_delta_rad": 0.0,
                },
            ),
        ],
    ),
).transports(
    {
        # DDS bridge ports (Go2WholeBodyConnection <-> coordinator's
        # transport_lcm adapter).
        ("motor_states", JointState): LCMTransport("/go2/motor_states", JointState),
        ("imu", Imu): LCMTransport("/go2/imu", Imu),
        ("motor_command", MotorCommandArray): LCMTransport("/go2/motor_command", MotorCommandArray),
        # Operator twist input.
        ("twist_command", Twist): LCMTransport("/cmd_vel", Twist),
        ("cmd_vel", Twist): LCMTransport("/cmd_vel", Twist),
        # Coordinator -> downstream consumers.
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
        ("joint_command", JointState): LCMTransport("/go2/joint_command", JointState),
    }
)


__all__ = ["go2_tripod_real"]
