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

from __future__ import annotations

import importlib
from pathlib import Path
import time
from typing import Any, cast

import numpy as np

from dimos.hardware.manipulators.damiao.specs import DamiaoArmSpec
from dimos.hardware.manipulators.spec import ControlMode, JointLimits, ManipulatorInfo
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Shared adapter defaults. Subclasses reference these so the base and its
# subclasses can't drift apart.
_DEFAULT_TICK_DEADLINE_US = 1_000
_DEFAULT_STATE_CACHE_TTL_S = 0.002
# Conventional first SocketCAN interface; used when no address is configured.
_DEFAULT_ADDRESS = "can0"
# Position of each control mode in declaration order, reported by read_state().
_CONTROL_MODE_INDEX = {mode: index for index, mode in enumerate(ControlMode)}

_can_motor_control: Any | None
_damiao: Any | None

try:
    _can_motor_control = importlib.import_module("can_motor_control")
    _damiao = importlib.import_module("can_motor_control.damiao")
except ImportError as exc:
    _can_motor_control = None
    _damiao = None
    _can_motor_control_import_error: ImportError | None = exc
else:
    _can_motor_control_import_error = None


class DamiaoBindingUnavailableError(RuntimeError):
    pass


def _ensure_can_motor_control(
    *,
    adapter_type: str,
    error_type: type[RuntimeError] = DamiaoBindingUnavailableError,
) -> None:
    if _can_motor_control_import_error is not None:
        raise error_type(
            f"The selected '{adapter_type}' adapter requires the Rust-backed "
            "can-motor-control Python binding in the active environment. Install "
            f"dimos[manipulation] before selecting adapter_type='{adapter_type}'."
        ) from _can_motor_control_import_error


def _dynamic_attr(value: object, name: str) -> Any:
    return getattr(value, name)


def _resolve_motor_type(motor_type: object) -> object:
    assert _damiao is not None
    if isinstance(motor_type, str):
        try:
            return getattr(_damiao.MotorType, motor_type)
        except AttributeError as exc:
            raise ValueError(f"Unknown Damiao motor type {motor_type!r}") from exc
    if not isinstance(motor_type, int):
        return motor_type
    for name in dir(_damiao.MotorType):
        if name.startswith("_"):
            continue
        candidate = getattr(_damiao.MotorType, name)
        try:
            candidate_value = int(candidate)
        except (TypeError, ValueError):
            continue
        if candidate_value == motor_type:
            return candidate
    raise ValueError(f"Unknown Damiao motor type value {motor_type!r}")


class DamiaoArmAdapterBase:
    """Shared DimOS adapter behavior for Damiao-based manipulators."""

    _adapter_type: str = "damiao"
    _binding_error_type: type[RuntimeError] = DamiaoBindingUnavailableError
    _supported_control_modes: tuple[ControlMode, ...] = (
        ControlMode.POSITION,
        ControlMode.SERVO_POSITION,
        ControlMode.TORQUE,
    )

    def __init__(
        self,
        *,
        arm_spec: DamiaoArmSpec,
        dof: int | None = None,
        hardware_id: str = "arm",
        kp: list[float] | None = None,
        kd: list[float] | None = None,
        gravity_comp: bool = True,
        gravity_model_path: str | Path | None = None,
        gravity_torque_limits: list[float] | tuple[float, ...] | None = None,
        supported_control_modes: tuple[ControlMode, ...] | None = None,
        address: str | Path | None = _DEFAULT_ADDRESS,
        config_path: str | Path | None = None,
        use_mock_bus: bool = False,
        tick_deadline_us: int = _DEFAULT_TICK_DEADLINE_US,
        state_cache_ttl_s: float = _DEFAULT_STATE_CACHE_TTL_S,
    ) -> None:
        arm_spec.validate()
        if dof is not None and dof != arm_spec.dof:
            raise ValueError(f"{type(self).__name__} only supports {arm_spec.dof} DOF (got {dof})")
        self._arm_spec = arm_spec
        self._hardware_id = hardware_id
        self._dof = arm_spec.dof
        self._motor_specs = list(arm_spec.motors)
        self._position_lower = list(arm_spec.position_lower)
        self._position_upper = list(arm_spec.position_upper)
        self._velocity_max = list(arm_spec.velocity_max)
        self._kp = list(kp) if kp is not None else list(arm_spec.kp)
        self._kd = list(kd) if kd is not None else list(arm_spec.kd)
        self._validate_length("kp", self._kp)
        self._validate_length("kd", self._kd)
        self._gravity_comp = gravity_comp
        resolved_gravity_model = (
            gravity_model_path if gravity_model_path is not None else arm_spec.gravity_model_path
        )
        self._gravity_model_path = (
            str(resolved_gravity_model) if resolved_gravity_model is not None else None
        )
        resolved_torque_limits = (
            gravity_torque_limits
            if gravity_torque_limits is not None
            else arm_spec.gravity_torque_limits
        )
        self._gravity_torque_limits = (
            list(resolved_torque_limits) if resolved_torque_limits is not None else None
        )
        if self._gravity_torque_limits is not None:
            self._validate_length("gravity_torque_limits", self._gravity_torque_limits)
        self._supported_control_modes = (
            supported_control_modes
            if supported_control_modes is not None
            else type(self)._supported_control_modes
        )
        self._control_mode = ControlMode.POSITION
        self._enabled = False
        self._last_positions: list[float] | None = None
        self._pin_model = None
        self._pin_data: object | None = None
        self._address = str(address) if address is not None else _DEFAULT_ADDRESS
        self._config_path = str(config_path) if config_path is not None else None
        self._arm_name = arm_spec.arm_name
        self._bus_name = arm_spec.bus_name
        self._fd = arm_spec.fd
        self._use_mock_bus = use_mock_bus
        self._tick_deadline_us = tick_deadline_us
        self._state_cache_ttl_s = state_cache_ttl_s
        self._robot: Any | None = None
        self._arm: Any | None = None
        self._connected = False
        self._state_cache: tuple[list[float], list[float], list[float]] | None = None
        self._state_cache_time = 0.0

    def _validate_length(self, name: str, values: list[float]) -> None:
        if len(values) != self._dof:
            raise ValueError(f"{name} length {len(values)} does not match dof {self._dof}")

    def _validate_command_lengths(self, **commands: list[float]) -> None:
        for name, values in commands.items():
            self._validate_length(name, values)

    def _zero_vector(self) -> list[float]:
        return [0.0] * self._dof

    def get_info(self) -> ManipulatorInfo:
        return ManipulatorInfo(
            vendor=self._arm_spec.vendor,
            model=self._arm_spec.model,
            dof=self._dof,
            firmware_version=None,
            serial_number=None,
        )

    def get_dof(self) -> int:
        return self._dof

    def get_limits(self) -> JointLimits:
        return JointLimits(
            position_lower=list(self._position_lower),
            position_upper=list(self._position_upper),
            velocity_max=list(self._velocity_max),
        )

    def set_control_mode(self, mode: ControlMode) -> bool:
        if mode not in self._supported_control_modes:
            return False
        self._control_mode = mode
        return True

    def get_control_mode(self) -> ControlMode:
        return self._control_mode

    def read_enabled(self) -> bool:
        return self._enabled

    def read_cartesian_position(self) -> dict[str, float] | None:
        return None

    def write_cartesian_position(self, pose: dict[str, float], velocity: float = 1.0) -> bool:
        return False

    def read_gripper_position(self) -> float | None:
        return None

    def write_gripper_position(self, position: float) -> bool:
        return False

    def read_force_torque(self) -> list[float] | None:
        return None

    def connect(self) -> bool:
        try:
            _ensure_can_motor_control(
                adapter_type=self._adapter_type,
                error_type=self._binding_error_type,
            )
            robot = self._build_robot()
            robot.connect()
            arm = robot[self._arm_name]
            if len(arm) != self._dof:
                raise RuntimeError(
                    f"can_motor_control arm group {self._arm_name!r} has {len(arm)} joints, "
                    f"expected {self._dof}"
                )
            self._robot = robot
            self._arm = arm
            self._load_gravity_model()
            self._connected = True
            self.refresh_state(force=True)
        except self._binding_error_type:
            raise
        except Exception as exc:
            logger.error(
                f"{type(self).__name__} {self._hardware_id}@{self._address} connect failed: {exc}"
            )
            self._robot = None
            self._arm = None
            self._connected = False
            return False
        return True

    def _build_robot(self) -> Any:
        assert _can_motor_control is not None
        assert _damiao is not None
        if self._config_path is not None:
            return _can_motor_control.Robot.from_config(self._config_path)
        transport = (
            _can_motor_control.MockCanBus.new_fd(self._address)
            if self._use_mock_bus and self._fd
            else _can_motor_control.MockCanBus(self._address)
            if self._use_mock_bus
            else _can_motor_control.SocketCanBus(self._address, fd=self._fd)
        )
        codec = _damiao.DamiaoCodec()
        binding_specs = [
            _can_motor_control.MotorSpec(
                spec.name,
                cast("int", _resolve_motor_type(spec.type)),
                spec.send_id,
                spec.effective_recv_id,
            )
            for spec in self._motor_specs
        ]
        return (
            _can_motor_control.Robot.builder()
            .add_bus(self._bus_name, transport, codec)
            .add_arm(self._arm_name, bus=self._bus_name, motors=binding_specs)
            .build()
        )

    def disconnect(self) -> None:
        if self._robot is not None:
            try:
                self._robot.disable()
            except Exception as exc:
                logger.warning(
                    f"{type(self).__name__} {self._hardware_id} disable on disconnect failed: {exc}"
                )
        self._enabled = False
        self._connected = False
        self._robot = None
        self._arm = None
        self._state_cache = None

    def is_connected(self) -> bool:
        return self._connected

    def refresh_state(self, *, force: bool = False) -> tuple[list[float], list[float], list[float]]:
        if self._robot is None or self._arm is None:
            raise RuntimeError(f"{type(self).__name__} is not connected")
        now = time.monotonic()
        if (
            not force
            and self._state_cache is not None
            and now - self._state_cache_time <= self._state_cache_ttl_s
        ):
            return self._state_cache
        self._arm.refresh()
        self._robot.tick(self._tick_deadline_us)
        state = (
            self._arm.positions().astype(np.float64).tolist(),
            self._arm.velocities().astype(np.float64).tolist(),
            self._arm.torques().astype(np.float64).tolist(),
        )
        if any(len(values) != self._dof for values in state):
            raise RuntimeError("can_motor_control state length does not match configured DOF")
        self._state_cache = state
        self._state_cache_time = time.monotonic()
        self._last_positions = list(state[0])
        return state

    def read_joint_positions(self) -> list[float]:
        return list(self.refresh_state()[0])

    def read_joint_velocities(self) -> list[float]:
        return list(self.refresh_state()[1])

    def read_joint_efforts(self) -> list[float]:
        return list(self.refresh_state()[2])

    def read_state(self) -> dict[str, int]:
        return {
            "state": 1 if self._enabled else 0,
            "mode": _CONTROL_MODE_INDEX[self._control_mode],
        }

    def read_error(self) -> tuple[int, str]:
        if self._arm is None:
            return 0, ""
        faults = []
        for spec in self._motor_specs:
            fault = getattr(self._arm[spec.name], "fault", None)
            if fault is not None:
                faults.append(f"{spec.name}: {fault}")
        return (0, "") if not faults else (1, "; ".join(faults))

    def write_joint_positions(self, positions: list[float], velocity: float = 1.0) -> bool:
        if (
            self._arm is None
            or self._robot is None
            or not self._enabled
            or len(positions) != self._dof
        ):
            return False
        velocity = max(0.0, min(1.0, velocity))
        if self._gravity_comp:
            try:
                tau = self.compute_gravity_torques(self.read_joint_positions())
            except RuntimeError:
                tau = self._zero_vector()
        else:
            tau = self._zero_vector()
        return self.write_mit_commands(
            q=list(positions),
            dq=self._zero_vector(),
            kp=[kp * velocity for kp in self._kp],
            kd=list(self._kd),
            tau=tau,
        )

    def write_joint_velocities(self, velocities: list[float]) -> bool:
        return False

    def write_joint_torques(self, efforts: list[float]) -> bool:
        if (
            self._arm is None
            or self._robot is None
            or not self._enabled
            or len(efforts) != self._dof
        ):
            return False
        q = (
            self._last_positions
            if self._last_positions is not None
            else self.read_joint_positions()
        )
        return self.write_mit_commands(
            q=q, dq=self._zero_vector(), kp=self._zero_vector(), kd=self._zero_vector(), tau=efforts
        )

    def write_gravity_compensation(self, damping: float | list[float] = 0.0) -> bool:
        try:
            q, dq, _ = self.refresh_state(force=True)
            tau = self.compute_gravity_torques(q)
        except Exception as exc:
            logger.warning(
                f"Skipping {type(self).__name__} gravity compensation due to invalid state: {exc}"
            )
            return False
        kd = [float(damping)] * self._dof if isinstance(damping, int | float) else list(damping)
        return self.write_mit_commands(q=q, dq=dq, kp=self._zero_vector(), kd=kd, tau=tau)

    def write_mit_commands(
        self, *, q: list[float], dq: list[float], kp: list[float], kd: list[float], tau: list[float]
    ) -> bool:
        if self._arm is None or self._robot is None or not self._enabled:
            return False
        self._validate_command_lengths(q=q, dq=dq, kp=kp, kd=kd, tau=tau)
        # MIT command columns are (kp, kd, q, dq, tau) per joint.
        self._arm.mit_control(np.column_stack([kp, kd, q, dq, tau]).astype(np.float64))
        self._robot.tick(self._tick_deadline_us)
        self._state_cache = None
        self._last_positions = list(q)
        self._control_mode = (
            ControlMode.TORQUE if all(k == 0.0 for k in kp) else ControlMode.POSITION
        )
        return True

    def write_stop(self) -> bool:
        if self._arm is None or self._robot is None:
            return False
        if self._gravity_comp and self._enabled:
            try:
                q_now = self.read_joint_positions()
            except RuntimeError:
                return False
            return self.write_mit_commands(
                q=q_now,
                dq=self._zero_vector(),
                kp=list(self._kp),
                kd=list(self._kd),
                tau=self.compute_gravity_torques(q_now),
            )
        try:
            self._robot.disable()
        except Exception as exc:
            logger.warning(f"{type(self).__name__} {self._hardware_id} stop disable failed: {exc}")
            return False
        self._enabled = False
        return True

    def write_enable(self, enable: bool) -> bool:
        if self._robot is None:
            return False
        try:
            self._robot.enable() if enable else self._robot.disable()
        except Exception as exc:
            logger.error(f"{type(self).__name__} {self._hardware_id} enable={enable} failed: {exc}")
            return False
        self._enabled = enable
        if enable:
            positions = self.read_joint_positions()
            if not self.write_joint_positions(positions):
                logger.error(f"{type(self).__name__} {self._hardware_id} startup hold failed")
                return False
        return True

    def write_clear_errors(self) -> bool:
        if self._robot is None:
            return False
        try:
            self._robot.disable()
            self._robot.enable()
        except Exception as exc:
            logger.error(f"{type(self).__name__} {self._hardware_id} clear errors failed: {exc}")
            return False
        self._enabled = True
        positions = self.read_joint_positions()
        if not self.write_joint_positions(positions):
            logger.error(f"{type(self).__name__} {self._hardware_id} clear-error hold failed")
            return False
        return True

    def _load_gravity_model(self) -> None:
        if not self._gravity_comp or self._gravity_model_path is None:
            return
        # Lazy import: pinocchio is a heavy optional dep only needed when
        # gravity compensation is enabled, so keep it out of module import.
        import pinocchio

        build_model_from_urdf = _dynamic_attr(pinocchio, "buildModelFromUrdf")
        self._pin_model = build_model_from_urdf(self._gravity_model_path)
        self._pin_data = _dynamic_attr(self._pin_model, "createData")()

    def compute_gravity_torques(self, q: list[float]) -> list[float]:
        self._validate_length("q", q)
        if self._pin_model is None or self._pin_data is None:
            return [0.0] * self._dof
        # Lazy import: pinocchio is a heavy optional dep only needed when
        # gravity compensation is enabled, so keep it out of module import.
        import pinocchio

        compute_generalized_gravity = _dynamic_attr(pinocchio, "computeGeneralizedGravity")
        tau = compute_generalized_gravity(
            self._pin_model,
            self._pin_data,
            np.array(q, dtype=np.float64),
        )
        values = [float(tau[i]) for i in range(self._dof)]
        if self._gravity_torque_limits is None:
            return values
        return [
            float(np.clip(value, -limit, limit))
            for value, limit in zip(values, self._gravity_torque_limits, strict=False)
        ]


__all__ = ["DamiaoArmAdapterBase", "DamiaoBindingUnavailableError"]
