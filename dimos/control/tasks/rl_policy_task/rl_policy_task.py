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

"""ControlTask wrapping an rsl_rl PPO policy for the Go2 velocity tracker.

Runs the actor inside the 100Hz tick loop, subsampled to its training rate
(50Hz default). Emits 12-joint SERVO_POSITION targets each tick (or 9 if
`mask_fr=True` for the held-up tripod variant).
"""

from __future__ import annotations

import threading
from dataclasses import dataclass, field
from typing import Any

import numpy as np
from pydantic import Field

from dimos.control.task import (
    BaseControlTask,
    ControlMode,
    CoordinatorState,
    JointCommandOutput,
    ResourceClaim,
)
from dimos.learning.inference.obs_builder import (
    GO2_DEFAULT_POSE,
    GO2_JOINT_ORDER,
    Go2VelocityObsBuilder,
    TwistCommand,
    projected_gravity_from_quat,
)
from dimos.learning.policy.rl_policy import RslRlPolicy
from dimos.protocol.service.spec import BaseConfig
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


# Joint short-names (no hardware prefix) the held-up FR leg occupies.
FR_JOINT_SHORTNAMES: tuple[str, ...] = (
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
)


@dataclass
class RLPolicyTaskConfig:
    """Per-instance config (built in `create_task` from TaskConfig.params)."""

    joint_names: list[str]
    policy_path: str
    hardware_id: str = "go2"
    inference_period: float = 0.02
    mask_fr: bool = False
    priority: int = 10
    device: str = "cpu"


class RLPolicyTask(BaseControlTask):
    """Reactive rsl_rl PPO actor running in the tick loop."""

    def __init__(self, name: str, config: RLPolicyTaskConfig) -> None:
        self._name = name
        self._config = config
        self._policy = RslRlPolicy.load(config.policy_path, device=config.device)
        if self._policy.config.obs_dim != 47 or self._policy.config.action_dim != 12:
            raise ValueError(
                f"Policy shape mismatch: expected 47->12, got "
                f"{self._policy.config.obs_dim}->{self._policy.config.action_dim}"
            )
        self._obs_builder = Go2VelocityObsBuilder()
        self._default_pose = np.array(GO2_DEFAULT_POSE, dtype=np.float32)

        self._command = TwistCommand()
        self._last_inference_t = -1.0
        self._last_action = np.zeros(12, dtype=np.float32)
        self._lock = threading.Lock()
        self._active = True

        # Pre-compute fully qualified joint names for our claim + outputs.
        self._prefixed_joints = [f"{config.hardware_id}/{j}" for j in GO2_JOINT_ORDER]
        self._fr_indices: tuple[int, ...] = tuple(
            i for i, j in enumerate(GO2_JOINT_ORDER) if j in FR_JOINT_SHORTNAMES
        )

        logger.info(
            f"RLPolicyTask {name} loaded {config.policy_path} "
            f"(mask_fr={config.mask_fr}, joints={len(self._claimed_joints())})"
        )

    @property
    def name(self) -> str:
        return self._name

    def claim(self) -> ResourceClaim:
        return ResourceClaim(
            joints=frozenset(self._claimed_joints()),
            priority=self._config.priority,
            mode=ControlMode.SERVO_POSITION,
        )

    def is_active(self) -> bool:
        with self._lock:
            return self._active

    def compute(self, state: CoordinatorState) -> JointCommandOutput | None:
        with self._lock:
            if not self._active:
                return None
            command = TwistCommand(self._command.vx, self._command.vy, self._command.wz)

        # Pull joint state in GO2_JOINT_ORDER.
        q = np.empty(12, dtype=np.float32)
        dq = np.empty(12, dtype=np.float32)
        for i, prefixed in enumerate(self._prefixed_joints):
            pos = state.joints.get_position(prefixed)
            vel = state.joints.get_velocity(prefixed)
            if pos is None or vel is None:
                return None  # Joint state not ready yet.
            q[i] = pos
            dq[i] = vel

        imu = state.imu.get(self._config.hardware_id)
        if imu is None:
            return None
        proj_g = projected_gravity_from_quat(imu.quaternion)
        ang_vel = np.array(imu.gyroscope, dtype=np.float32)

        self._obs_builder.step_phase(state.dt)

        # Subsample inference to inference_period; reuse last_action otherwise.
        do_infer = (
            self._last_inference_t < 0.0
            or (state.t_now - self._last_inference_t) >= self._config.inference_period
        )
        if do_infer:
            obs = self._obs_builder.build(q, dq, ang_vel, proj_g, command)
            action = self._policy.act(obs)
            self._obs_builder.cache_action(action)
            self._last_action = action.astype(np.float32, copy=False)
            self._last_inference_t = state.t_now

        # Apply action term scale (training: JointPositionAction.scale=0.25).
        # last_action is stored RAW so the obs's last_actions term matches what
        # the training env's action_manager.action stores.
        target_q = self._default_pose + 0.25 * self._last_action

        # Mask FR if requested.
        if self._config.mask_fr:
            keep = [i for i in range(12) if i not in self._fr_indices]
            joint_names = [self._prefixed_joints[i] for i in keep]
            positions = [float(target_q[i]) for i in keep]
        else:
            joint_names = list(self._prefixed_joints)
            positions = [float(x) for x in target_q]

        return JointCommandOutput(
            joint_names=joint_names,
            positions=positions,
            mode=ControlMode.SERVO_POSITION,
        )

    def on_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        # Keep computing internally so last_actions stays continuous in-distribution.
        logger.debug(f"RLPolicyTask {self._name} preempted by {by_task} on {joints}")

    # --- Public command setters (called by the blueprint's input wiring) -----

    def set_velocity_command(self, vx: float, vy: float, wz: float, t_now: float) -> None:
        """Coordinator twist routing hook. Signature is fixed by `_on_twist_command`.

        See coordinator.py:516 - any task exposing this method gets twist updates.
        """
        with self._lock:
            self._command = TwistCommand(float(vx), float(vy), float(wz))

    def start(self) -> None:
        with self._lock:
            self._active = True

    def stop(self) -> None:
        with self._lock:
            self._active = False

    # --- Internals -----------------------------------------------------------

    def _claimed_joints(self) -> list[str]:
        if self._config.mask_fr:
            return [
                self._prefixed_joints[i]
                for i in range(12)
                if i not in self._fr_indices
            ]
        return list(self._prefixed_joints)


class RLPolicyTaskParams(BaseConfig):
    """Schema for TaskConfig.params - validated in `create_task`."""

    policy_path: str = Field(..., description="Path to rsl_rl checkpoint (.pt)")
    hardware_id: str = "go2"
    inference_period: float = 0.02
    mask_fr: bool = False
    device: str = "cpu"


def create_task(cfg: Any, hardware: Any) -> RLPolicyTask:
    params = RLPolicyTaskParams.model_validate(cfg.params)
    return RLPolicyTask(
        cfg.name,
        RLPolicyTaskConfig(
            joint_names=cfg.joint_names,
            policy_path=params.policy_path,
            hardware_id=params.hardware_id,
            inference_period=params.inference_period,
            mask_fr=params.mask_fr,
            priority=cfg.priority,
            device=params.device,
        ),
    )


__all__ = ["RLPolicyTask", "RLPolicyTaskConfig", "create_task"]
