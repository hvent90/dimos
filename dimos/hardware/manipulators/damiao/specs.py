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

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class DamiaoMotorSpec:
    """Typed metadata for one Damiao motor in adapter joint order."""

    name: str
    type: object
    send_id: int
    recv_id: int | None = None

    @property
    def effective_recv_id(self) -> int:
        return self.recv_id if self.recv_id is not None else (self.send_id | 0x10)


@dataclass(frozen=True)
class DamiaoArmSpec:
    """Typed metadata for a Damiao-based arm adapter."""

    name: str
    vendor: str
    model: str
    motors: tuple[DamiaoMotorSpec, ...]
    position_lower: tuple[float, ...]
    position_upper: tuple[float, ...]
    velocity_max: tuple[float, ...]
    kp: tuple[float, ...]
    kd: tuple[float, ...]
    gravity_model_path: str | Path | None = None
    gravity_torque_limits: tuple[float, ...] | None = None
    requires_binding: bool = False
    bus_name: str = "can"
    arm_name: str = "arm"
    fd: bool = False
    supports_velocity: bool = False

    @property
    def dof(self) -> int:
        return len(self.motors)

    @property
    def joint_names(self) -> tuple[str, ...]:
        return tuple(motor.name for motor in self.motors)

    @classmethod
    def from_values(
        cls,
        *,
        name: str,
        vendor: str,
        model: str,
        motors: Sequence[Mapping[str, object] | DamiaoMotorSpec],
        position_lower: list[float] | tuple[float, ...],
        position_upper: list[float] | tuple[float, ...],
        velocity_max: list[float] | tuple[float, ...],
        kp: list[float] | tuple[float, ...],
        kd: list[float] | tuple[float, ...],
        gravity_model_path: str | Path | None = None,
        gravity_torque_limits: list[float] | tuple[float, ...] | None = None,
        requires_binding: bool = False,
        bus_name: str = "can",
        arm_name: str = "arm",
        fd: bool = False,
        supports_velocity: bool = False,
    ) -> DamiaoArmSpec:
        return cls(
            name=name,
            vendor=vendor,
            model=model,
            motors=coerce_motor_specs(motors, len(motors)),
            position_lower=tuple(float(value) for value in position_lower),
            position_upper=tuple(float(value) for value in position_upper),
            velocity_max=tuple(float(value) for value in velocity_max),
            kp=tuple(float(value) for value in kp),
            kd=tuple(float(value) for value in kd),
            gravity_model_path=gravity_model_path,
            gravity_torque_limits=(
                tuple(float(value) for value in gravity_torque_limits)
                if gravity_torque_limits is not None
                else None
            ),
            requires_binding=requires_binding,
            bus_name=bus_name,
            arm_name=arm_name,
            fd=fd,
            supports_velocity=supports_velocity,
        )

    def validate(self) -> None:
        if not self.motors:
            raise ValueError("DamiaoArmSpec requires at least one motor")
        send_ids = [motor.send_id for motor in self.motors]
        if len(set(send_ids)) != len(send_ids):
            raise ValueError(f"duplicate send_id in {send_ids}")
        recv_ids = [motor.effective_recv_id for motor in self.motors]
        if len(set(recv_ids)) != len(recv_ids):
            raise ValueError(f"duplicate recv_id in {recv_ids}")
        for name, values in {
            "position_lower": self.position_lower,
            "position_upper": self.position_upper,
            "velocity_max": self.velocity_max,
            "kp": self.kp,
            "kd": self.kd,
        }.items():
            if len(values) != self.dof:
                raise ValueError(f"{name} length {len(values)} does not match dof {self.dof}")
        if self.gravity_torque_limits is not None and len(self.gravity_torque_limits) != self.dof:
            raise ValueError("gravity_torque_limits length does not match dof")


def coerce_motor_specs(
    motor_specs: Sequence[Mapping[str, object] | DamiaoMotorSpec],
    dof: int,
) -> tuple[DamiaoMotorSpec, ...]:
    specs: list[DamiaoMotorSpec] = []
    for spec in motor_specs:
        if isinstance(spec, DamiaoMotorSpec):
            specs.append(spec)
        else:
            name = spec.get("name")
            send_id = spec.get("send_id")
            recv_id = spec.get("recv_id")
            if not isinstance(name, str):
                raise TypeError("motor spec name must be a string")
            if not isinstance(send_id, int):
                raise TypeError("motor spec send_id must be an integer")
            if recv_id is not None and not isinstance(recv_id, int):
                raise TypeError("motor spec recv_id must be an integer")
            specs.append(
                DamiaoMotorSpec(
                    name=name,
                    type=spec.get("type"),
                    send_id=send_id,
                    recv_id=recv_id,
                )
            )
    if len(specs) != dof:
        raise ValueError(f"motor_specs length {len(specs)} does not match dof {dof}")
    return tuple(specs)


__all__ = ["DamiaoArmSpec", "DamiaoMotorSpec", "coerce_motor_specs"]
