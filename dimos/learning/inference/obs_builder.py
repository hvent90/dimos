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

"""Observation construction for the Go2 velocity-tracking RL policy.

Verified against `model_1200.pt`: the actor consumes a 47-D vector with
7 terms in this exact order:

    base_ang_vel       (3)
    projected_gravity  (3)
    command            (3)  vx, vy, wz  (heading folded into wz upstream)
    phase              (2)  sin, cos    (period 0.6 s)
    joint_pos_rel      (12) q - default
    joint_vel_rel      (12) dq - 0
    last_actions       (12) actor's previous output

Default pose lifted from the training env's init_state (mjlab Go2 spec):
hips ±0.1 (not 0 as in scene_go2.xml keyframe), thigh 0.9, calf -1.8.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable

import numpy as np


# mjlab's articulation joint ordering (verified via robot.joint_names).
# NOT the MJCF actuator order - mjlab reorders to FL, FR, RL, RR.
GO2_JOINT_ORDER: tuple[str, ...] = (
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
)

# Default joint positions from the training env's init_state, in the order above.
# Verified against env.default_joint_pos: FL hip=-0.1, FR hip=+0.1, RL hip=-0.1, RR hip=+0.1.
GO2_DEFAULT_POSE: tuple[float, ...] = (
    -0.1,  0.9, -1.8,   # FL
     0.1,  0.9, -1.8,   # FR
    -0.1,  0.9, -1.8,   # RL
     0.1,  0.9, -1.8,   # RR
)


@dataclass
class TwistCommand:
    """Operator-level command. wz already incorporates heading control."""
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0


HeightScanFn = Callable[[], np.ndarray]
"""Unused for this policy (height_scan was critic-only) but kept for the
shape of the abstraction - other RL policies may need it."""


@dataclass
class Go2VelocityObsBuilder:
    """Stateful builder for the 47-D observation vector.

    Owns the phase clock and `last_action` cache between ticks.
    """

    phase_period: float = 0.6
    last_action: np.ndarray = field(default_factory=lambda: np.zeros(12, dtype=np.float32))
    _phase_t: float = 0.0
    _default_pose: np.ndarray = field(default_factory=lambda: np.array(GO2_DEFAULT_POSE, dtype=np.float32))

    @property
    def obs_dim(self) -> int:
        return 47

    def reset(self) -> None:
        self._phase_t = 0.0
        self.last_action[:] = 0.0

    def step_phase(self, dt: float) -> None:
        self._phase_t += dt

    def build(
        self,
        joint_positions: np.ndarray,   # (12,)
        joint_velocities: np.ndarray,  # (12,)
        base_ang_vel: np.ndarray,      # (3,)
        projected_gravity: np.ndarray, # (3,)
        command: TwistCommand,
    ) -> np.ndarray:
        if joint_positions.shape != (12,):
            raise ValueError(f"joint_positions shape {joint_positions.shape}")
        if joint_velocities.shape != (12,):
            raise ValueError(f"joint_velocities shape {joint_velocities.shape}")

        # Phase term matches src.tasks.velocity.mdp.observations.phase:
        # zero when L2 command norm < 0.1, else sin/cos of normalized phase.
        cmd_l2 = math.sqrt(command.vx * command.vx + command.vy * command.vy + command.wz * command.wz)
        if cmd_l2 < 0.1:
            phase_sin, phase_cos = 0.0, 0.0
        else:
            global_phase = (self._phase_t % self.phase_period) / self.phase_period
            theta = 2.0 * math.pi * global_phase
            phase_sin, phase_cos = math.sin(theta), math.cos(theta)

        out = np.empty(47, dtype=np.float32)
        out[0:3]   = base_ang_vel
        out[3:6]   = projected_gravity
        out[6:9]   = (command.vx, command.vy, command.wz)
        out[9:11]  = (phase_sin, phase_cos)
        out[11:23] = joint_positions - self._default_pose
        out[23:35] = joint_velocities  # joint_vel_rel: default vel is 0
        out[35:47] = self.last_action
        return out

    def cache_action(self, action: np.ndarray) -> None:
        if action.shape != (12,):
            raise ValueError(f"action shape {action.shape}")
        self.last_action[:] = action


def projected_gravity_from_quat(quat_wxyz: tuple[float, float, float, float]) -> np.ndarray:
    """Rotate world gravity (0, 0, -1) into body frame via inverse of `quat`.

    Quaternion is (w, x, y, z) - the convention MuJoCo's framequat returns.
    Returns shape (3,) float32.
    """
    w, x, y, z = quat_wxyz
    # Inverse rotation of (0, 0, -1) by quat. For a unit quat, q^-1 = (w, -x, -y, -z).
    # Use the standard formula: v' = q^-1 * v * q.
    # Closed form for v=(0,0,-1):
    gx = 2.0 * (x * z - w * y) * -1.0
    gy = 2.0 * (y * z + w * x) * -1.0
    gz = (1.0 - 2.0 * (x * x + y * y)) * -1.0
    return np.array([gx, gy, gz], dtype=np.float32)


__all__ = [
    "GO2_DEFAULT_POSE",
    "GO2_JOINT_ORDER",
    "Go2VelocityObsBuilder",
    "HeightScanFn",
    "TwistCommand",
    "projected_gravity_from_quat",
]
