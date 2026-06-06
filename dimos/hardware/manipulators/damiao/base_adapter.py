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
from typing import Any, Protocol, cast

import numpy as np

from dimos.hardware.manipulators.damiao.specs import DamiaoArmSpec
from dimos.hardware.manipulators.spec import ControlMode, JointLimits, ManipulatorInfo


class _PinocchioModule(Protocol):
    def buildModelFromUrdf(self, filename: str) -> Any: ...

    def computeGeneralizedGravity(self, model: Any, data: Any, q: np.ndarray) -> Any: ...


class DamiaoArmAdapterBase:
    """Shared DimOS adapter behavior for Damiao-based manipulators."""

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
        self._pin_model: Any = None
        self._pin_data: Any = None

    def _validate_length(self, name: str, values: list[float]) -> None:
        if len(values) != self._dof:
            raise ValueError(f"{name} length {len(values)} does not match dof {self._dof}")

    def _validate_command_lengths(self, **commands: list[float]) -> None:
        for name, values in commands.items():
            self._validate_length(name, values)

    def _zero_vector(self) -> list[float]:
        return [0.0] * self._dof

    def _mit_command_rows(
        self,
        *,
        q: list[float],
        dq: list[float],
        kp: list[float],
        kd: list[float],
        tau: list[float],
    ) -> list[tuple[float, float, float, float, float]]:
        self._validate_command_lengths(q=q, dq=dq, kp=kp, kd=kd, tau=tau)
        return list(zip(q, dq, kp, kd, tau, strict=True))

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

    def _load_gravity_model(self) -> None:
        if not self._gravity_comp or self._gravity_model_path is None:
            return
        pinocchio = cast("_PinocchioModule", cast("object", importlib.import_module("pinocchio")))

        self._pin_model = pinocchio.buildModelFromUrdf(self._gravity_model_path)
        self._pin_data = self._pin_model.createData()

    def compute_gravity_torques(self, q: list[float]) -> list[float]:
        self._validate_length("q", q)
        if self._pin_model is None or self._pin_data is None:
            return [0.0] * self._dof
        pinocchio = cast("_PinocchioModule", cast("object", importlib.import_module("pinocchio")))

        tau = pinocchio.computeGeneralizedGravity(
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


__all__ = ["DamiaoArmAdapterBase"]
