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

from dataclasses import dataclass
import importlib
from pathlib import Path
import time
from types import ModuleType
from typing import TYPE_CHECKING, Any

import numpy as np

from dimos.hardware.manipulators.spec import ControlMode, JointLimits, ManipulatorInfo
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.hardware.manipulators.registry import AdapterRegistry

logger = setup_logger()


class DMMotorBindingUnavailableError(RuntimeError):
    pass


@dataclass(frozen=True)
class DMMotorSpecConfig:
    name: str
    type: str | int
    send_id: int
    recv_id: int


_DEFAULT_OPENARM_MOTORS: tuple[DMMotorSpecConfig, ...] = (
    DMMotorSpecConfig("joint1", "DM8006", 0x01, 0x11),
    DMMotorSpecConfig("joint2", "DM8006", 0x02, 0x12),
    DMMotorSpecConfig("joint3", "DM4340", 0x03, 0x13),
    DMMotorSpecConfig("joint4", "DM4340", 0x04, 0x14),
    DMMotorSpecConfig("joint5", "DM4310", 0x05, 0x15),
    DMMotorSpecConfig("joint6", "DM4310", 0x06, 0x16),
    DMMotorSpecConfig("joint7", "DM4310", 0x07, 0x17),
)

_DEFAULT_POSITION_LOWER = [-3.45, -3.30, -1.50, -0.01, -1.50, -0.75, -1.50]
_DEFAULT_POSITION_UPPER = [1.35, 0.15, 1.50, 2.40, 1.50, 0.75, 1.50]
_DEFAULT_VELOCITY_MAX = [45.0, 45.0, 8.0, 8.0, 30.0, 30.0, 30.0]
# _DEFAULT_KP = [70.0, 70.0, 70.0, 60.0, 10.0, 10.0, 10.0]
_DEFAULT_KP = [0.0] * 7
_DEFAULT_KD = [2.75, 2.5, 2.0, 2.0, 0.7, 0.6, 0.5]
_STATE_CACHE_TTL_S = 0.002


def _load_dm_control() -> tuple[ModuleType, ModuleType]:
    try:
        dm_control = importlib.import_module("dm_control")
        damiao = importlib.import_module("dm_control.damiao")
    except ImportError as exc:
        raise DMMotorBindingUnavailableError(
            "The selected 'dm_motor_arm' adapter requires the Rust-backed dm_control "
            "Python binding in the active environment. Install/provide that binding "
            "before selecting adapter_type='dm_motor_arm'; DimOS will not install it "
            "automatically."
        ) from exc
    return dm_control, damiao


def _coerce_motor_specs(
    motor_specs: list[dict[str, Any] | DMMotorSpecConfig] | None,
    dof: int,
) -> list[DMMotorSpecConfig]:
    if motor_specs is None:
        if dof != len(_DEFAULT_OPENARM_MOTORS):
            raise ValueError(
                "motor_specs is required when constructing a dm_motor_arm with "
                f"{dof} DOF; the built-in default only describes OpenArm-style 7 DOF"
            )
        return list(_DEFAULT_OPENARM_MOTORS)
    specs: list[DMMotorSpecConfig] = []
    for spec in motor_specs:
        if isinstance(spec, DMMotorSpecConfig):
            specs.append(spec)
        else:
            specs.append(DMMotorSpecConfig(**spec))
    if len(specs) != dof:
        raise ValueError(f"motor_specs length {len(specs)} does not match dof {dof}")
    return specs


def _resolve_motor_type(damiao: ModuleType, motor_type: str | int) -> Any:
    if isinstance(motor_type, str):
        try:
            return getattr(damiao.MotorType, motor_type)
        except AttributeError as exc:
            raise ValueError(f"Unknown Damiao motor type {motor_type!r}") from exc

    for name in dir(damiao.MotorType):
        if name.startswith("_"):
            continue
        candidate = getattr(damiao.MotorType, name)
        try:
            candidate_value = int(candidate)
        except (TypeError, ValueError):
            continue
        if candidate_value == motor_type:
            return candidate
    raise ValueError(f"Unknown Damiao motor type value {motor_type!r}")


class DMMotorArm:
    def __init__(
        self,
        address: str | Path | None = "can0",
        dof: int = 7,
        *,
        hardware_id: str = "arm",
        config_path: str | Path | None = None,
        arm_name: str = "arm",
        bus_name: str = "can",
        fd: bool | None = None,
        canfd: bool = True,
        use_mock_bus: bool = False,
        motor_specs: list[dict[str, Any] | DMMotorSpecConfig] | None = None,
        position_lower: list[float] | None = None,
        position_upper: list[float] | None = None,
        velocity_max: list[float] | None = None,
        kp: list[float] | None = None,
        kd: list[float] | None = None,
        gravity_comp: bool = True,
        tick_deadline_us: int = 1_000,
        state_cache_ttl_s: float = _STATE_CACHE_TTL_S,
        gravity_model_path: str | Path | None = None,
        gravity_torque_limits: list[float] | None = None,
        **_: Any,
    ) -> None:
        self._address = str(address) if address is not None else "can0"
        self._dof = dof
        self._hardware_id = hardware_id
        self._config_path = str(config_path) if config_path is not None else None
        self._arm_name = arm_name
        self._bus_name = bus_name
        self._fd = canfd if fd is None else fd
        self._use_mock_bus = use_mock_bus
        self._motor_specs = _coerce_motor_specs(motor_specs, dof)
        self._position_lower = (
            list(position_lower) if position_lower is not None else _DEFAULT_POSITION_LOWER[:dof]
        )
        self._position_upper = (
            list(position_upper) if position_upper is not None else _DEFAULT_POSITION_UPPER[:dof]
        )
        self._velocity_max = (
            list(velocity_max) if velocity_max is not None else _DEFAULT_VELOCITY_MAX[:dof]
        )
        self._kp = list(kp) if kp is not None else _DEFAULT_KP[:dof]
        self._kd = list(kd) if kd is not None else _DEFAULT_KD[:dof]
        self._gravity_comp = gravity_comp
        self._tick_deadline_us = tick_deadline_us
        self._state_cache_ttl_s = state_cache_ttl_s
        self._gravity_model_path = (
            str(gravity_model_path) if gravity_model_path is not None else None
        )
        self._gravity_torque_limits = (
            list(gravity_torque_limits) if gravity_torque_limits is not None else None
        )

        for name, values in {
            "position_lower": self._position_lower,
            "position_upper": self._position_upper,
            "velocity_max": self._velocity_max,
            "kp": self._kp,
            "kd": self._kd,
        }.items():
            if len(values) != dof:
                raise ValueError(f"{name} length {len(values)} does not match dof {dof}")

        self._dm_control: ModuleType | None = None
        self._damiao: ModuleType | None = None
        self._robot: Any = None
        self._arm: Any = None
        self._connected = False
        self._enabled = False
        self._control_mode = ControlMode.POSITION
        self._last_positions: list[float] | None = None
        self._state_cache: tuple[list[float], list[float], list[float]] | None = None
        self._state_cache_time = 0.0
        self._pin_model: Any = None
        self._pin_data: Any = None

    def connect(self) -> bool:
        try:
            self._dm_control, self._damiao = _load_dm_control()
            self._robot = self._build_robot()
            self._robot.connect()
            self._arm = self._robot[self._arm_name]
            if len(self._arm) != self._dof:
                raise RuntimeError(
                    f"dm_control arm group {self._arm_name!r} has {len(self._arm)} joints, "
                    f"expected {self._dof}"
                )
            self._load_gravity_model()
            self._connected = True
            self.refresh_state(force=True)
        except DMMotorBindingUnavailableError:
            raise
        except Exception as exc:
            logger.error(f"DMMotorArm {self._hardware_id}@{self._address} connect failed: {exc}")
            self._robot = None
            self._arm = None
            self._connected = False
            return False
        return True

    def _build_robot(self) -> Any:
        assert self._dm_control is not None
        assert self._damiao is not None
        if self._config_path is not None:
            return self._dm_control.Robot.from_config(self._config_path)

        transport = (
            self._dm_control.MockCanBus.new_fd(self._address)
            if self._use_mock_bus and self._fd
            else self._dm_control.MockCanBus(self._address)
            if self._use_mock_bus
            else self._dm_control.SocketCanBus(self._address, fd=self._fd)
        )
        codec = self._damiao.DamiaoCodec()
        binding_specs = [
            self._dm_control.MotorSpec(
                spec.name,
                _resolve_motor_type(self._damiao, spec.type),
                spec.send_id,
                spec.recv_id,
            )
            for spec in self._motor_specs
        ]
        return (
            self._dm_control.Robot.builder()
            .add_bus(self._bus_name, transport, codec)
            .add_arm(self._arm_name, bus=self._bus_name, motors=binding_specs)
            .build()
        )

    def disconnect(self) -> None:
        if self._robot is not None:
            try:
                self._robot.disable()
            except Exception as exc:
                logger.warning(f"DMMotorArm {self._hardware_id} disable on disconnect failed: {exc}")
        self._enabled = False
        self._connected = False
        self._robot = None
        self._arm = None
        self._state_cache = None

    def is_connected(self) -> bool:
        return self._connected

    def get_info(self) -> ManipulatorInfo:
        return ManipulatorInfo(
            vendor="Damiao",
            model="DMMotorArm",
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
        if mode not in (
            ControlMode.POSITION,
            ControlMode.SERVO_POSITION,
            ControlMode.TORQUE,
        ):
            return False
        self._control_mode = mode
        return True

    def get_control_mode(self) -> ControlMode:
        return self._control_mode

    def refresh_state(
        self, *, force: bool = False
    ) -> tuple[list[float], list[float], list[float]]:
        if self._robot is None or self._arm is None:
            raise RuntimeError("DMMotorArm is not connected")
        now = time.monotonic()
        if (
            not force
            and self._state_cache is not None
            and now - self._state_cache_time <= self._state_cache_ttl_s
        ):
            return self._state_cache
        self._robot.tick(self._tick_deadline_us)
        state = (
            self._arm.positions().astype(float).tolist(),
            self._arm.velocities().astype(float).tolist(),
            self._arm.torques().astype(float).tolist(),
        )
        if any(len(values) != self._dof for values in state):
            raise RuntimeError("dm_control state length does not match configured DOF")
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
            "mode": list(ControlMode).index(self._control_mode),
        }

    def read_error(self) -> tuple[int, str]:
        if self._arm is None:
            return 0, ""
        faults = []
        for spec in self._motor_specs:
            motor = self._arm[spec.name]
            fault = getattr(motor, "fault", None)
            if fault is not None:
                faults.append(f"{spec.name}: {fault}")
        if not faults:
            return 0, ""
        return 1, "; ".join(faults)

    def write_joint_positions(self, positions: list[float], velocity: float = 1.0) -> bool:
        if self._arm is None or self._robot is None or not self._enabled:
            return False
        if len(positions) != self._dof:
            return False
        velocity = max(0.0, min(1.0, velocity))
        if self._gravity_comp:
            try:
                q_current = self.read_joint_positions()
                tau = self.compute_gravity_torques(q_current)
            except RuntimeError:
                tau = [0.0] * self._dof
        else:
            tau = [0.0] * self._dof
        # print(f"write_joint_positions: positions={positions}, velocity={velocity}, tau={tau}")
        return self.write_mit_commands(
            q=list(positions),
            dq=[0.0] * self._dof,
            kp=[kp * velocity for kp in self._kp],
            kd=list(self._kd),
            tau=tau,
        )

    def write_joint_velocities(self, velocities: list[float]) -> bool:
        return False

    def write_joint_torques(self, efforts: list[float]) -> bool:
        if self._arm is None or self._robot is None or not self._enabled:
            return False
        if len(efforts) != self._dof:
            return False
        q = self._last_positions if self._last_positions is not None else self.read_joint_positions()
        return self.write_mit_commands(
            q=q,
            dq=[0.0] * self._dof,
            kp=[0.0] * self._dof,
            kd=[0.0] * self._dof,
            tau=efforts,
        )

    def write_gravity_compensation(self, damping: float | list[float] = 0.0) -> bool:
        try:
            q, dq, _ = self.refresh_state(force=True)
            tau = self.compute_gravity_torques(q)
        except Exception as exc:
            logger.warning(f"Skipping DMMotor gravity compensation due to invalid state: {exc}")
            return False
        kd = (
            [float(damping)] * self._dof
            if isinstance(damping, int | float)
            else list(damping)
        )
        if len(kd) != self._dof:
            raise ValueError(f"damping length {len(kd)} does not match dof {self._dof}")
        return self.write_mit_commands(
            q=q, dq=dq, kp=[0.0] * self._dof, kd=kd, tau=tau
        )

    def write_mit_commands(
        self,
        *,
        q: list[float],
        dq: list[float],
        kp: list[float],
        kd: list[float],
        tau: list[float],
    ) -> bool:
        if self._arm is None or self._robot is None or not self._enabled:
            return False
        for name, values in {"q": q, "dq": dq, "kp": kp, "kd": kd, "tau": tau}.items():
            if len(values) != self._dof:
                raise ValueError(f"{name} length {len(values)} does not match dof {self._dof}")
        cmds = np.array(list(zip(kp, kd, q, dq, tau, strict=False)), dtype=np.float64)
        self._arm.mit_control(cmds)
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
            tau = self.compute_gravity_torques(q_now)
            return self.write_mit_commands(
                q=q_now,
                dq=[0.0] * self._dof,
                kp=list(self._kp),
                kd=list(self._kd),
                tau=tau,
            )
        try:
            self._robot.disable()
        except Exception as exc:
            logger.warning(f"DMMotorArm {self._hardware_id} stop disable failed: {exc}")
            return False
        self._enabled = False
        return True

    def write_enable(self, enable: bool) -> bool:
        if self._robot is None:
            return False
        try:
            if enable:
                self._robot.enable()
            else:
                self._robot.disable()
        except Exception as exc:
            logger.error(f"DMMotorArm {self._hardware_id} enable={enable} failed: {exc}")
            return False
        self._enabled = enable
        return True

    def read_enabled(self) -> bool:
        return self._enabled

    def write_clear_errors(self) -> bool:
        if self._robot is None:
            return False
        try:
            self._robot.disable()
            self._robot.enable()
        except Exception as exc:
            logger.error(f"DMMotorArm {self._hardware_id} clear errors failed: {exc}")
            return False
        self._enabled = True
        return True

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

    def _load_gravity_model(self) -> None:
        if not self._gravity_comp or self._gravity_model_path is None:
            return
        import pinocchio

        self._pin_model = pinocchio.buildModelFromUrdf(self._gravity_model_path)
        self._pin_data = self._pin_model.createData()

    def compute_gravity_torques(self, q: list[float]) -> list[float]:
        if len(q) != self._dof:
            raise ValueError(f"q length {len(q)} does not match dof {self._dof}")
        if self._pin_model is None or self._pin_data is None:
            return [0.0] * self._dof
        import pinocchio

        tau = pinocchio.computeGeneralizedGravity(
            self._pin_model,
            self._pin_data,
            np.array(q, dtype=np.float64),
        )
        values = [float(tau[i]) for i in range(self._dof)]
        if self._gravity_torque_limits is None:
            return values
        if len(self._gravity_torque_limits) != self._dof:
            raise ValueError("gravity_torque_limits length does not match dof")
        return [
            float(np.clip(value, -limit, limit))
            for value, limit in zip(values, self._gravity_torque_limits, strict=False)
        ]


def register(registry: AdapterRegistry) -> None:
    registry.register("dm_motor_arm", DMMotorArm)


__all__ = ["DMMotorArm", "DMMotorBindingUnavailableError", "DMMotorSpecConfig", "register"]
