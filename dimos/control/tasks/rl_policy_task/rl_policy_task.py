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

"""ControlTask wrapping an MLP policy for the Go2 velocity tracker.

Runs the actor inside the 100Hz tick loop, subsampled to its training rate
(50Hz default). Emits 12-joint SERVO_POSITION targets each tick (or 9 if
`mask_fr=True` for the held-up tripod variant).
"""

from __future__ import annotations

from dataclasses import dataclass
import threading
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
    MJLAB_TO_WIRE,
    WIRE_TO_MJLAB,
    Go2VelocityObsBuilder,
    TwistCommand,
    projected_gravity_from_quat,
)
from dimos.learning.policy.rl_policy import MLPPolicy
from dimos.protocol.service.spec import BaseConfig
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


# Joint short-names (no hardware prefix) the held-up FR leg occupies.
# Uses wire convention - no '_joint' suffix - matching GO2_JOINT_ORDER.
FR_JOINT_SHORTNAMES: tuple[str, ...] = (
    "FR_hip",
    "FR_thigh",
    "FR_calf",
)

# Numpy index arrays for the wire <-> mjlab permutation. Materialized once.
_WIRE_TO_MJLAB = np.array(WIRE_TO_MJLAB, dtype=np.int64)
_MJLAB_TO_WIRE = np.array(MJLAB_TO_WIRE, dtype=np.int64)


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
    # Smooth blend from the joint state at activation toward the policy's
    # target over this many seconds. 0 = no ramp (sim). 1.5 = real hardware
    # default - protects against a lurch when arming on a non-standing robot.
    activation_ramp_seconds: float = 0.0


class RLPolicyTask(BaseControlTask):
    """Reactive MLP actor running in the tick loop."""

    def __init__(self, name: str, config: RLPolicyTaskConfig) -> None:
        self._name = name
        self._config = config
        self._policy = MLPPolicy.load(config.policy_path, device=config.device)
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
        # Inactive by default. Coordinator calls start() iff TaskConfig.auto_start
        # is True (see coordinator.py:199); otherwise the operator arms via
        # set_activated(True) / arm() once safety preconditions are met.
        self._active = False
        # Set on activation (start/arm). Joint pos at the moment we go active,
        # in WIRE order. Blended with the policy's target during the ramp.
        self._activation_t: float = -1.0
        self._ramp_origin_wire: np.ndarray | None = None

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

        # Pull joint state in wire order (GO2_JOINT_ORDER = FR, FL, RR, RL).
        q_wire = np.empty(12, dtype=np.float32)
        dq_wire = np.empty(12, dtype=np.float32)
        for i, prefixed in enumerate(self._prefixed_joints):
            pos = state.joints.get_position(prefixed)
            vel = state.joints.get_velocity(prefixed)
            if pos is None or vel is None:
                return None  # Joint state not ready yet.
            q_wire[i] = pos
            dq_wire[i] = vel

        imu = state.imu.get(self._config.hardware_id)
        if imu is None:
            return None
        proj_g = projected_gravity_from_quat(imu.quaternion)
        ang_vel = np.array(imu.gyroscope, dtype=np.float32)

        self._obs_builder.step_phase(state.dt)

        # Permute wire -> mjlab for the policy's internal view.
        q_mjlab = q_wire[_WIRE_TO_MJLAB]
        dq_mjlab = dq_wire[_WIRE_TO_MJLAB]

        # Subsample inference to inference_period; reuse last_action otherwise.
        do_infer = (
            self._last_inference_t < 0.0
            or (state.t_now - self._last_inference_t) >= self._config.inference_period
        )
        if do_infer:
            obs = self._obs_builder.build(q_mjlab, dq_mjlab, ang_vel, proj_g, command)
            action_mjlab = self._policy.act(obs)
            self._obs_builder.cache_action(action_mjlab)
            self._last_action = action_mjlab.astype(np.float32, copy=False)
            self._last_inference_t = state.t_now

        # Apply action term scale (training: JointPositionAction.scale=0.25).
        # last_action is stored RAW (mjlab order) so the obs's last_actions
        # term matches the training env's action_manager.action.
        target_q_mjlab = self._default_pose + 0.25 * self._last_action

        # Permute mjlab -> wire for hardware output.
        target_q_wire = target_q_mjlab[_MJLAB_TO_WIRE]

        # Activation ramp: on the first compute() after arming, snapshot the
        # robot's current joint pos and blend toward the policy target over
        # `activation_ramp_seconds`. Protects against a lurch when the robot
        # was folded / not at default pose when armed.
        if self._activation_t < 0.0:
            self._activation_t = state.t_now
            self._ramp_origin_wire = q_wire.copy()
        ramp_s = self._config.activation_ramp_seconds
        if ramp_s > 0.0 and self._ramp_origin_wire is not None:
            alpha = min(1.0, (state.t_now - self._activation_t) / ramp_s)
            target_q_wire = (1.0 - alpha) * self._ramp_origin_wire + alpha * target_q_wire

        # Mask FR if requested. _fr_indices is computed in wire order.
        if self._config.mask_fr:
            keep = [i for i in range(12) if i not in self._fr_indices]
            joint_names = [self._prefixed_joints[i] for i in keep]
            positions = [float(target_q_wire[i]) for i in keep]
        else:
            joint_names = list(self._prefixed_joints)
            positions = [float(x) for x in target_q_wire]

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
            self._activation_t = -1.0  # trigger fresh ramp on next compute()
            self._ramp_origin_wire = None

    def stop(self) -> None:
        with self._lock:
            self._active = False
            self._activation_t = -1.0
            self._ramp_origin_wire = None

    # arm/disarm are the coordinator's set_activated() hooks (coordinator.py:529).
    def arm(self) -> None:
        self.start()

    def disarm(self) -> None:
        self.stop()

    # --- Internals -----------------------------------------------------------

    def _claimed_joints(self) -> list[str]:
        if self._config.mask_fr:
            return [self._prefixed_joints[i] for i in range(12) if i not in self._fr_indices]
        return list(self._prefixed_joints)


class RLPolicyTaskParams(BaseConfig):
    """Schema for TaskConfig.params - validated in `create_task`."""

    policy_path: str = Field(..., description="Path to MLP actor checkpoint (.pt)")
    hardware_id: str = "go2"
    inference_period: float = 0.02
    mask_fr: bool = False
    device: str = "cpu"
    activation_ramp_seconds: float = 0.0


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
            activation_ramp_seconds=params.activation_ramp_seconds,
        ),
    )
    # auto_start is handled by the coordinator: it calls task.start() iff
    # TaskConfig.auto_start is True (see coordinator.py:199). The task starts
    # inactive (self._active=False); start() / arm() flip it active and reset
    # the ramp.


__all__ = ["RLPolicyTask", "RLPolicyTaskConfig", "create_task"]
