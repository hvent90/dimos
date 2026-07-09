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

"""Quadruped velocity-tracking RL policy task (mjlab-trained, height scan).

Runs a single-frame ONNX locomotion policy inside the coordinator tick
loop, following the rsl_rl / mjlab velocity-task observation contract:

    [0:3]     base_lin_vel          # body frame (velocimeter)
    [3:6]     base_ang_vel          # body frame (gyro)
    [6:9]     projected_gravity     # gravity in body frame
    [9:21]    q - default_q         # 12 joints, policy order
    [21:33]   dq                    # 12 joints
    [33:45]   last_action           # previous raw policy output
    [45:48]   command               # [vx, vy, wz]
    [48:48+N] height_scan * 0.2     # N-ray terrain grid, misses = 5.0 m

Action: ``target_q = default_q + action * action_scale`` emitted as
SERVO_POSITION. The observation layout, scan grid, defaults, and scales
are part of the trained policy's contract - do not change them without
retraining (they are also embedded in the exported ONNX metadata).

The policy needs two observations a ``WholeBodyAdapter`` doesn't carry:
body-frame base linear velocity and the terrain height scan. Adapters
that can provide them (``SimMujocoQuadrupedAdapter``; later, a hardware
adapter backed by state estimation + elevation mapping) expose
``read_base_lin_vel()`` / ``read_height_scan(n)``; see
``QuadrupedPolicyExtras``.
"""

from __future__ import annotations

from dataclasses import dataclass
import threading
from typing import TYPE_CHECKING, Any, Protocol, runtime_checkable

import numpy as np
from numpy.typing import NDArray
import onnxruntime as ort  # type: ignore[import-untyped]

from dimos.control.hardware_interface import ConnectedWholeBody
from dimos.control.task import (
    BaseControlTask,
    ControlMode,
    CoordinatorState,
    JointCommandOutput,
    ResourceClaim,
)
from dimos.protocol.service.spec import BaseConfig
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from pathlib import Path

    from dimos.hardware.whole_body.spec import WholeBodyAdapter

logger = setup_logger()

_NUM_JOINTS = 12

# Go1 policy contract (mjlab go1_velocity run go1_heightscan_dr_4096),
# in canonical MJCF joint order: FR, FL, RR, RL x (hip, thigh, calf).
GO1_POLICY_JOINT_SUFFIXES: tuple[str, ...] = (
    "FR_hip",
    "FR_thigh",
    "FR_calf",
    "FL_hip",
    "FL_thigh",
    "FL_calf",
    "RR_hip",
    "RR_thigh",
    "RR_calf",
    "RL_hip",
    "RL_thigh",
    "RL_calf",
)
GO1_DEFAULT_POSITIONS: tuple[float, ...] = (
    0.1,
    0.9,
    -1.8,
    -0.1,
    0.9,
    -1.8,
    0.1,
    0.9,
    -1.8,
    -0.1,
    0.9,
    -1.8,
)
# 0.25 * effort_limit / stiffness per actuator group (hip/thigh vs calf).
_HIP_THIGH_SCALE = 0.3727530387083568
_CALF_SCALE = 0.24850202580557115
GO1_ACTION_SCALE: tuple[float, ...] = (
    _HIP_THIGH_SCALE,
    _HIP_THIGH_SCALE,
    _CALF_SCALE,
) * 4
# MJCF <position> actuator gains the policy was trained against. The sim
# bakes these into the robot XML; they are surfaced here so blueprints can
# pass matching values to hardware adapters that execute PD on-board.
GO1_KP: tuple[float, ...] = (15.89524265323492, 15.89524265323492, 35.76429596977857) * 4
GO1_KD: tuple[float, ...] = (1.0119225759919113, 1.0119225759919113, 2.2768257959818006) * 4


def make_go1_joints(hardware_id: str = "go1") -> list[str]:
    """Hardware joint names in policy order (e.g. ``go1/FR_hip``)."""
    return [f"{hardware_id}/{suffix}" for suffix in GO1_POLICY_JOINT_SUFFIXES]


@runtime_checkable
class QuadrupedPolicyExtras(Protocol):
    """Adapter extras this policy needs beyond ``WholeBodyAdapter``."""

    def read_base_lin_vel(self) -> tuple[float, float, float]: ...

    def read_height_scan(self, n_rays: int) -> list[float]: ...


def _preferred_onnx_providers() -> list[str]:
    available = ort.get_available_providers()
    providers: list[str] = []
    if "CUDAExecutionProvider" in available:
        providers.append("CUDAExecutionProvider")
    providers.append("CPUExecutionProvider")
    return providers


@dataclass(frozen=True)
class QuadrupedVelocityTaskConfig:
    policy_onnx: Path | str
    joint_names: tuple[str, ...]
    priority: int
    default_positions: tuple[float, ...] = GO1_DEFAULT_POSITIONS
    action_scale: tuple[float, ...] = GO1_ACTION_SCALE
    # Height-scan grid contract: 1.6 x 1.0 m at 0.1 m resolution = 17 x 11.
    height_scan_rays: int = 187
    height_scan_scale: float = 0.2
    height_scan_miss: float = 5.0
    # Clamp scan heights before feeding the policy. Training terrain was
    # continuous, so raw miss values (5.0 = "bottomless cliff") are far
    # out of distribution and make the policy crouch-freeze or collapse
    # the moment part of the grid hangs over a platform edge. 1.0 m keeps
    # edge readings inside the depth range the policy saw in training
    # while still signalling "big drop, don't step there".
    height_scan_clip: float = 1.0
    # Per-axis gain from the stack's cmd_vel convention to the policy's
    # command units, applied in the obs (same pattern as
    # G1GrootWBCTaskConfig.cmd_scale). 2.0 maps the nav planner's and
    # teleop client's standard magnitudes onto the range this policy
    # tracks well (measured stable to ~1.9 m/s / 1.9 rad/s).
    cmd_scale: tuple[float, float, float] = (2.0, 2.0, 2.0)
    # Inference every N coordinator ticks (blueprint pairs a 50 Hz tick
    # with decimation 1 to match the trained 50 Hz policy rate).
    decimation: int = 1
    auto_arm: bool = False
    # Zero the command if none arrives within this window.
    timeout: float = 1.0


class QuadrupedVelocityTask(BaseControlTask):
    """See module docstring for the observation/action contract."""

    def __init__(
        self,
        name: str,
        config: QuadrupedVelocityTaskConfig,
        adapter: WholeBodyAdapter,
    ) -> None:
        if len(config.joint_names) != _NUM_JOINTS:
            raise ValueError(
                f"QuadrupedVelocityTask '{name}' requires exactly {_NUM_JOINTS} "
                f"joint names, got {len(config.joint_names)}"
            )
        if len(config.default_positions) != _NUM_JOINTS:
            raise ValueError(f"QuadrupedVelocityTask '{name}' needs {_NUM_JOINTS} defaults")
        if len(config.action_scale) != _NUM_JOINTS:
            raise ValueError(f"QuadrupedVelocityTask '{name}' needs {_NUM_JOINTS} action scales")
        if config.decimation < 1:
            raise ValueError(f"QuadrupedVelocityTask '{name}' requires decimation >= 1")
        if not isinstance(adapter, QuadrupedPolicyExtras):
            raise TypeError(
                f"QuadrupedVelocityTask '{name}' requires an adapter exposing "
                f"read_base_lin_vel()/read_height_scan() (got {type(adapter).__name__}). "
                f"In sim, use adapter_type='sim_mujoco_quadruped' with "
                f"MujocoSimModule enable_height_scan=True."
            )

        self._name = name
        self._config = config
        self._adapter = adapter
        self._joint_names_list = list(config.joint_names)
        self._joint_names_set = frozenset(config.joint_names)

        providers = _preferred_onnx_providers()
        self._session = ort.InferenceSession(str(config.policy_onnx), providers=providers)
        self._input_name = self._session.get_inputs()[0].name
        obs_dim = int(self._session.get_inputs()[0].shape[-1])
        expected = 48 + config.height_scan_rays
        if obs_dim != expected:
            raise ValueError(
                f"QuadrupedVelocityTask '{name}': ONNX expects obs dim {obs_dim}, "
                f"but the configured contract produces {expected} "
                f"(48 + {config.height_scan_rays} scan rays)"
            )
        self._obs = np.zeros((1, obs_dim), dtype=np.float32)
        logger.info(
            "QuadrupedVelocityTask loaded ONNX policy",
            task=name,
            policy=str(config.policy_onnx),
            obs_dim=obs_dim,
            providers=self._session.get_providers(),
        )

        self._default_q = np.asarray(config.default_positions, dtype=np.float32)
        self._action_scale = np.asarray(config.action_scale, dtype=np.float32)
        self._cmd_scale = np.asarray(config.cmd_scale, dtype=np.float32)

        self._last_action = np.zeros(_NUM_JOINTS, dtype=np.float32)
        self._tick_count = 0
        self._last_targets: list[float] | None = None

        # Last-known-good caches (same rationale as G1GrootWBCTask: a
        # transiently missing joint must not read as "at zero" to the
        # policy).
        self._cached_q = self._default_q.copy()
        self._cached_dq = np.zeros(_NUM_JOINTS, dtype=np.float32)
        self._state_seen = False

        self._active = False
        self._armed = False

        self._cmd_lock = threading.Lock()
        self._cmd = np.zeros(3, dtype=np.float32)
        self._last_cmd_time: float = 0.0

    @property
    def name(self) -> str:
        return self._name

    def claim(self) -> ResourceClaim:
        return ResourceClaim(
            joints=self._joint_names_set,
            priority=self._config.priority,
            mode=ControlMode.SERVO_POSITION,
        )

    def is_active(self) -> bool:
        return self._active

    # Lifecycle

    def start(self) -> None:
        self._active = True
        self._armed = False
        self._last_targets = None
        self._reset_policy_state()
        with self._cmd_lock:
            self._cmd[:] = 0.0
            self._last_cmd_time = 0.0
        logger.info("QuadrupedVelocityTask started", task=self._name)
        if self._config.auto_arm:
            self.arm()

    def stop(self) -> None:
        self._active = False
        self._armed = False
        self._last_targets = None
        logger.info("QuadrupedVelocityTask stopped", task=self._name)

    def arm(self) -> bool:
        """Hand control to the ONNX policy (no ramp - the sim spawns at
        the policy's default pose)."""
        if not self._active:
            logger.warning("QuadrupedVelocityTask arm() before start(); ignoring", task=self._name)
            return False
        if self._armed:
            return False
        self._reset_policy_state()
        self._armed = True
        logger.info("QuadrupedVelocityTask armed", task=self._name)
        return True

    def disarm(self) -> bool:
        self._armed = False
        logger.info("QuadrupedVelocityTask disarmed", task=self._name)
        return True

    def _reset_policy_state(self) -> None:
        self._last_action[:] = 0.0
        self._tick_count = 0

    # Tick

    def _refresh_state_caches(self, state: CoordinatorState) -> bool:
        all_present = True
        for i, jname in enumerate(self._joint_names_list):
            pos = state.joints.get_position(jname)
            vel = state.joints.get_velocity(jname)
            if pos is None:
                all_present = False
            else:
                self._cached_q[i] = pos
            if vel is None:
                all_present = False
            else:
                self._cached_dq[i] = vel
        if all_present:
            self._state_seen = True
        return all_present

    def compute(self, state: CoordinatorState) -> JointCommandOutput | None:
        if not self._active:
            return None

        fresh = self._refresh_state_caches(state)
        if not self._state_seen and not fresh:
            return None

        # Unarmed: echo current joint positions (pure hold).
        if not self._armed:
            self._last_targets = self._cached_q.tolist()
            return JointCommandOutput(
                joint_names=self._joint_names_list,
                positions=self._last_targets,
                mode=ControlMode.SERVO_POSITION,
            )

        self._tick_count += 1
        if self._tick_count % self._config.decimation != 0:
            if self._last_targets is None:
                return None
            return JointCommandOutput(
                joint_names=self._joint_names_list,
                positions=self._last_targets,
                mode=ControlMode.SERVO_POSITION,
            )

        q = self._cached_q.copy()
        dq = self._cached_dq.copy()

        if state.imu:
            imu = next(iter(state.imu.values()))
        else:
            imu = self._adapter.read_imu()
        gyro = np.asarray(imu.gyroscope, dtype=np.float32)
        gravity = self._projected_gravity(imu.quaternion)
        lin_vel = np.asarray(self._adapter.read_base_lin_vel(), dtype=np.float32)

        scan = self._adapter.read_height_scan(self._config.height_scan_rays)
        if len(scan) != self._config.height_scan_rays:
            heights = np.full(
                self._config.height_scan_rays, self._config.height_scan_miss, dtype=np.float32
            )
        else:
            heights = np.asarray(scan, dtype=np.float32)
        heights = np.minimum(heights, self._config.height_scan_clip)

        with self._cmd_lock:
            if (
                self._config.timeout > 0.0
                and self._last_cmd_time > 0.0
                and (state.t_now - self._last_cmd_time) > self._config.timeout
            ):
                cmd = np.zeros(3, dtype=np.float32)
            else:
                cmd = self._cmd.copy()

        obs = self._obs[0]
        obs[0:3] = lin_vel
        obs[3:6] = gyro
        obs[6:9] = gravity
        obs[9:21] = q - self._default_q
        obs[21:33] = dq
        obs[33:45] = self._last_action
        obs[45:48] = cmd * self._cmd_scale
        obs[48:] = heights * self._config.height_scan_scale

        action = self._session.run(None, {self._input_name: self._obs})[0][0].astype(np.float32)
        self._last_action[:] = action

        target_q = action * self._action_scale + self._default_q
        self._last_targets = target_q.tolist()

        return JointCommandOutput(
            joint_names=self._joint_names_list,
            positions=self._last_targets,
            mode=ControlMode.SERVO_POSITION,
        )

    def on_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        if joints & self._joint_names_set:
            logger.warning(
                "QuadrupedVelocityTask preempted", task=self._name, by_task=by_task, joints=joints
            )

    # Velocity command input

    def set_velocity_command(self, vx: float, vy: float, yaw_rate: float, t_now: float) -> None:
        """Called by the coordinator's twist_command dispatcher. Thread-safe."""
        with self._cmd_lock:
            self._cmd[:] = [vx, vy, yaw_rate]
            self._last_cmd_time = t_now

    @staticmethod
    def _projected_gravity(quaternion: tuple[float, ...]) -> NDArray[np.float32]:
        """World gravity (0,0,-1) rotated into the body frame. Quat is wxyz."""
        w, x, y, z = quaternion
        return np.array(
            [
                -2.0 * (x * z - w * y),
                -2.0 * (y * z + w * x),
                -(1.0 - 2.0 * (x * x + y * y)),
            ],
            dtype=np.float32,
        )


class QuadrupedVelocityTaskParams(BaseConfig):
    policy_path: str
    hardware_id: str
    auto_arm: bool = False
    decimation: int = 1


def create_task(cfg: Any, hardware: Any) -> QuadrupedVelocityTask:
    params = QuadrupedVelocityTaskParams.model_validate(cfg.params)
    hw = hardware.get(params.hardware_id) if hardware else None
    if hw is None:
        raise ValueError(
            f"QuadrupedVelocityTask {cfg.name!r} references unknown hardware "
            f"{params.hardware_id!r}. Declare the hardware before the task."
        )
    if not isinstance(hw, ConnectedWholeBody):
        raise TypeError(
            f"QuadrupedVelocityTask {cfg.name!r} requires a WHOLE_BODY hardware "
            f"component for {params.hardware_id!r}, got {type(hw).__name__}."
        )
    return QuadrupedVelocityTask(
        cfg.name,
        QuadrupedVelocityTaskConfig(
            policy_onnx=params.policy_path,
            joint_names=tuple(cfg.joint_names),
            priority=cfg.priority,
            auto_arm=params.auto_arm,
            decimation=params.decimation,
        ),
        adapter=hw.adapter,
    )
