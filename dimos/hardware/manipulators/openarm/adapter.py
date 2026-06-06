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

"""OpenArm ManipulatorAdapter — wraps the Damiao MIT-mode driver. SI units."""

from __future__ import annotations

from pathlib import Path
import time
from typing import TYPE_CHECKING, Any, cast

from dimos.hardware.manipulators.damiao.base_adapter import DamiaoArmAdapterBase
from dimos.hardware.manipulators.damiao.specs import DamiaoArmSpec, DamiaoMotorSpec
from dimos.hardware.manipulators.openarm.driver import (
    CTRL_MODE_MIT,
    DamiaoMotor,
    MotorType,
    OpenArmBus,
)
from dimos.hardware.manipulators.spec import (
    ControlMode,
)
from dimos.utils.data import LfsPath

if TYPE_CHECKING:
    from dimos.hardware.manipulators.registry import AdapterRegistry


def _socketcan_iface_up(name: str) -> bool:
    try:
        flags_path = Path("/sys/class/net") / name / "flags"
        if not flags_path.exists():
            return False
        return (int(flags_path.read_text().strip(), 16) & 0x1) == 0x1
    except OSError:
        return False


class OpenArmAdapter(DamiaoArmAdapterBase):
    _URDF_LEFT = LfsPath("openarm_description/urdf/robot/openarm_v10_left.urdf")
    _URDF_RIGHT = LfsPath("openarm_description/urdf/robot/openarm_v10_right.urdf")
    _V10_ARM_MOTORS: tuple[DamiaoMotorSpec, ...] = (
        DamiaoMotorSpec("joint1", MotorType.DM8006, 0x01, 0x11),
        DamiaoMotorSpec("joint2", MotorType.DM8006, 0x02, 0x12),
        DamiaoMotorSpec("joint3", MotorType.DM4340, 0x03, 0x13),
        DamiaoMotorSpec("joint4", MotorType.DM4340, 0x04, 0x14),
        DamiaoMotorSpec("joint5", MotorType.DM4310, 0x05, 0x15),
        DamiaoMotorSpec("joint6", MotorType.DM4310, 0x06, 0x16),
        DamiaoMotorSpec("joint7", MotorType.DM4310, 0x07, 0x17),
    )
    _V10_POS_LOWER_LEFT: tuple[float, ...] = (-3.45, -3.30, -1.50, -0.01, -1.50, -0.75, -1.50)
    _V10_POS_UPPER_LEFT: tuple[float, ...] = (1.35, 0.15, 1.50, 2.40, 1.50, 0.75, 1.50)
    _V10_POS_LOWER_RIGHT: tuple[float, ...] = (-1.35, -0.15, -1.50, -0.01, -1.50, -0.75, -1.50)
    _V10_POS_UPPER_RIGHT: tuple[float, ...] = (3.45, 3.30, 1.50, 2.40, 1.50, 0.75, 1.50)
    _V10_VEL_MAX: tuple[float, ...] = (
        16.754666,
        16.754666,
        5.445426,
        5.445426,
        20.943946,
        20.943946,
        20.943946,
    )
    _DEFAULT_KP: tuple[float, ...] = (100.0, 100.0, 80.0, 80.0, 60.0, 60.0, 60.0)
    _DEFAULT_KD: tuple[float, ...] = (1.5, 1.5, 1.0, 1.0, 0.8, 0.8, 0.8)
    _STATE_MAX_AGE_S = 0.1

    def __init__(
        self,
        address: str = "can0",
        dof: int = 7,
        *,
        side: str = "left",
        fd: bool = False,
        interface: str = "socketcan",
        kp: list[float] | None = None,
        kd: list[float] | None = None,
        gravity_comp: bool = True,
        auto_set_mit_mode: bool = True,
        **_: Any,
    ) -> None:
        if side not in ("left", "right"):
            raise ValueError(f"side must be 'left' or 'right', got {side!r}")
        position_lower = self._V10_POS_LOWER_LEFT if side == "left" else self._V10_POS_LOWER_RIGHT
        position_upper = self._V10_POS_UPPER_LEFT if side == "left" else self._V10_POS_UPPER_RIGHT
        urdf = self._URDF_LEFT if side == "left" else self._URDF_RIGHT
        arm_spec = DamiaoArmSpec(
            name=f"openarm_{side}",
            vendor="Enactic",
            model=f"OpenArm v10 ({side})",
            motors=self._V10_ARM_MOTORS,
            position_lower=position_lower,
            position_upper=position_upper,
            velocity_max=self._V10_VEL_MAX,
            kp=self._DEFAULT_KP,
            kd=self._DEFAULT_KD,
            gravity_model_path=urdf,
            gravity_torque_limits=tuple(
                DamiaoMotor(motor.send_id, cast("MotorType", motor.type), motor.recv_id).limits[2]
                for motor in self._V10_ARM_MOTORS
            ),
            fd=fd,
            supports_velocity=True,
        )
        super().__init__(
            arm_spec=arm_spec,
            dof=dof,
            kp=kp,
            kd=kd,
            gravity_comp=gravity_comp,
            supported_control_modes=(
                ControlMode.POSITION,
                ControlMode.SERVO_POSITION,
                ControlMode.VELOCITY,
                ControlMode.TORQUE,
            ),
        )
        self._address = address
        self._side = side
        self._fd = fd
        self._interface = interface
        self._auto_set_mit_mode = auto_set_mit_mode
        self._motors = [
            DamiaoMotor(spec.send_id, cast("MotorType", spec.type), spec.recv_id)
            for spec in self._motor_specs
        ]
        self._bus: OpenArmBus | None = None
        self._last_cmd_q: list[float] | None = None

    def connect(self) -> bool:
        # Preflight: verify the SocketCAN interface is up before opening the bus.
        # Bringing the interface up requires root privileges, so we don't do it
        # here — just fail early with a helpful message.
        if self._interface == "socketcan" and not _socketcan_iface_up(self._address):
            print(
                f"ERROR: SocketCAN interface '{self._address}' is not UP.\n"
                f"  Run: sudo ip link set {self._address} up type can bitrate 1000000\n"
                f"  (or: sudo ./dimos/robot/manipulators/openarm/scripts/openarm_can_up.sh {self._address})"
            )
            return False

        try:
            self._bus = OpenArmBus(
                channel=self._address,
                motors=self._motors,
                fd=self._fd,
                interface=self._interface,
            )
            self._bus.open()
        except Exception as e:
            print(f"ERROR: OpenArm {self._side}@{self._address} connect failed: {e}")
            self._bus = None
            return False

        # Ensure every motor is in MIT control mode. The write is idempotent
        # (setting CTRL_MODE=MIT when it's already MIT is a no-op), so we
        # write unconditionally rather than query-then-write.
        if self._auto_set_mit_mode:
            try:
                for m in self._motors:
                    self._bus.write_ctrl_mode(m.send_id, CTRL_MODE_MIT)
            except Exception as e:
                print(f"ERROR: failed to set MIT mode on {self._address}: {e}")
                self._bus.close()
                self._bus = None
                return False
        else:
            print(
                f"OpenArm {self._side}@{self._address}: "
                "auto_set_mit_mode disabled — relying on persisted register"
            )

        if self._gravity_comp:
            try:
                self._load_gravity_model()
                if self._pin_model is not None:
                    print(
                        f"OpenArm {self._side}: gravity compensation enabled (nq={self._pin_model.nq})"
                    )
            except Exception as e:
                print(f"WARNING: gravity comp disabled — {e}")
                self._pin_model = None
                self._pin_data = None

        return True

    def disconnect(self) -> None:
        if self._bus is None:
            return
        try:
            self._bus.disable_all()
        except Exception:
            pass
        self._enabled = False
        self._bus.close()
        self._bus = None

    def is_connected(self) -> bool:
        return self._bus is not None

    def _states_or_raise(self) -> list[Any]:
        # Raises on missing or stale data so hardware_interface.py can retry
        # (init) or skip the tick (steady-state).
        if self._bus is None:
            raise RuntimeError("OpenArmAdapter not connected")
        now = time.monotonic()
        states = self._bus.get_states()
        for i, s in enumerate(states):
            if s is None:
                raise RuntimeError(f"motor {i + 1} has no state yet")
            if now - s.timestamp > self._STATE_MAX_AGE_S:
                age_ms = (now - s.timestamp) * 1000
                raise RuntimeError(f"motor {i + 1} state stale ({age_ms:.0f} ms)")
        return states

    def read_joint_positions(self) -> list[float]:
        return [s.q for s in self._states_or_raise()]

    def read_joint_velocities(self) -> list[float]:
        return [s.dq for s in self._states_or_raise()]

    def read_joint_efforts(self) -> list[float]:
        return [s.tau for s in self._states_or_raise()]

    def read_state(self) -> dict[str, int]:
        if self._bus is None:
            return {"state": 0, "mode": 0}
        states = self._bus.get_states()
        # report the hottest rotor temperature so callers can monitor thermal
        # stress with a single scalar
        t_rotor = max((s.t_rotor for s in states if s is not None), default=0)
        return {
            "state": 1 if self._enabled else 0,
            "mode": 1,  # MIT
            "t_rotor_max": int(t_rotor),
        }

    def read_error(self) -> tuple[int, str]:
        # The Damiao motors don't report a structured error code in the state
        # frame; over-temperature / over-torque are detected by the host from
        # the normal state fields. Surface a soft thermal warning here.
        if self._bus is None:
            return 0, ""
        states = self._bus.get_states()
        t_rotor = max((s.t_rotor for s in states if s is not None), default=0)
        if t_rotor >= 85:
            return 1, f"rotor over-temperature ({t_rotor}°C)"
        return 0, ""

    def write_joint_positions(
        self,
        positions: list[float],
        velocity: float = 1.0,
    ) -> bool:
        if self._bus is None or not self._enabled:
            return False
        if len(positions) != self._dof:
            return False
        velocity = max(0.0, min(1.0, velocity))
        # Gravity feedforward: compute tau needed to hold the arm at the
        # current configuration. The PD gains handle the rest. Tolerate
        # transient state-cache misses (e.g. startup, brief CAN gap) — fall
        # back to commanded q with no feedforward instead of crashing.
        try:
            q_current = self.read_joint_positions()
            tau_ff = self.compute_gravity_torques(q_current)
        except RuntimeError:
            tau_ff = [0.0] * self._dof
        commands = [
            (q, 0.0, kp * velocity, kd, tau)
            for q, kp, kd, tau in zip(positions, self._kp, self._kd, tau_ff, strict=False)
        ]
        self._bus.send_mit_many(commands)
        self._last_cmd_q = list(positions)
        return True

    def write_joint_velocities(self, velocities: list[float]) -> bool:
        # MIT velocity tracking: kp=0, send dq directly, anchor q at the
        # last-commanded position so the motor doesn't drift. Gravity
        # feedforward is still needed — with kp=0 the only restoring force
        # is damping, so without tau_ff the arm droops under its own weight.
        if self._bus is None or not self._enabled:
            return False
        if len(velocities) != self._dof:
            return False
        # Seed anchor from current pose if we don't have a last-commanded one.
        # If state isn't ready yet, can't safely anchor velocity tracking → bail.
        if self._last_cmd_q is None:
            try:
                self._last_cmd_q = self.read_joint_positions()
            except RuntimeError:
                return False
        anchor = self._last_cmd_q
        try:
            q_current = self.read_joint_positions()
            tau_ff = self.compute_gravity_torques(q_current)
        except RuntimeError:
            tau_ff = [0.0] * self._dof
        commands = [
            (q_anchor, dq, 0.0, kd, tau)
            for q_anchor, dq, kd, tau in zip(anchor, velocities, self._kd, tau_ff, strict=False)
        ]
        self._bus.send_mit_many(commands)
        return True

    def write_stop(self) -> bool:
        if self._bus is None:
            return False
        # Without current positions we can't safely command "hold here" — sending
        # any guessed q would torque the arm toward that pose. Bail out instead.
        try:
            q_now = self.read_joint_positions()
        except RuntimeError:
            return False
        tau_ff = self.compute_gravity_torques(q_now)
        commands = [
            (q, 0.0, kp, kd, tau)
            for q, kp, kd, tau in zip(q_now, self._kp, self._kd, tau_ff, strict=False)
        ]
        self._bus.send_mit_many(commands)
        self._last_cmd_q = q_now
        return True

    def write_enable(self, enable: bool) -> bool:
        if self._bus is None:
            return False
        self._enabled = False
        try:
            if enable:
                self._bus.enable_all()
            else:
                self._bus.disable_all()
        except Exception:
            return False
        self._enabled = enable
        return True

    def write_clear_errors(self) -> bool:
        # Damiao motors have no separate clear-error command; re-enabling
        # after a fault is the recovery path.
        if self._bus is None:
            return False
        self._enabled = False
        try:
            self._bus.disable_all()
            self._bus.enable_all()
        except Exception:
            return False
        self._enabled = True
        return True


# ── Registry hook (required for auto-discovery) ───────────────────
def register(registry: AdapterRegistry) -> None:
    registry.register("openarm", OpenArmAdapter)


__all__ = ["OpenArmAdapter", "register"]
