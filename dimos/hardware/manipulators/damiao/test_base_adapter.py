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

import pytest

from dimos.hardware.manipulators.damiao.base_adapter import DamiaoArmAdapterBase
from dimos.hardware.manipulators.damiao.specs import DamiaoArmSpec, DamiaoMotorSpec
from dimos.hardware.manipulators.spec import ControlMode


def _arm_spec() -> DamiaoArmSpec:
    return DamiaoArmSpec(
        name="test_damiao",
        vendor="Damiao",
        model="TestArm",
        motors=(
            DamiaoMotorSpec("j1", "DM4310", 0x01, 0x11),
            DamiaoMotorSpec("j2", "DM4310", 0x02, 0x12),
        ),
        position_lower=(-1.0, -2.0),
        position_upper=(1.0, 2.0),
        velocity_max=(3.0, 4.0),
        kp=(5.0, 6.0),
        kd=(0.1, 0.2),
        gravity_torque_limits=(7.0, 8.0),
    )


def test_arm_spec_preserves_joint_order_and_metadata() -> None:
    spec = _arm_spec()
    assert spec.dof == 2
    assert spec.joint_names == ("j1", "j2")
    assert [motor.send_id for motor in spec.motors] == [0x01, 0x02]
    assert [motor.effective_recv_id for motor in spec.motors] == [0x11, 0x12]


def test_arm_spec_rejects_duplicate_ids() -> None:
    with pytest.raises(ValueError, match="duplicate send_id"):
        DamiaoArmSpec(
            name="bad",
            vendor="Damiao",
            model="BadArm",
            motors=(
                DamiaoMotorSpec("j1", "DM4310", 0x01, 0x11),
                DamiaoMotorSpec("j2", "DM4310", 0x01, 0x12),
            ),
            position_lower=(-1.0, -2.0),
            position_upper=(1.0, 2.0),
            velocity_max=(3.0, 4.0),
            kp=(5.0, 6.0),
            kd=(0.1, 0.2),
        ).validate()


def test_arm_spec_rejects_length_mismatch() -> None:
    with pytest.raises(ValueError, match="kp length 1 does not match dof 2"):
        DamiaoArmSpec(
            name="bad",
            vendor="Damiao",
            model="BadArm",
            motors=(
                DamiaoMotorSpec("j1", "DM4310", 0x01, 0x11),
                DamiaoMotorSpec("j2", "DM4310", 0x02, 0x12),
            ),
            position_lower=(-1.0, -2.0),
            position_upper=(1.0, 2.0),
            velocity_max=(3.0, 4.0),
            kp=(5.0,),
            kd=(0.1, 0.2),
        ).validate()


def test_base_adapter_info_limits_and_modes() -> None:
    adapter = DamiaoArmAdapterBase(arm_spec=_arm_spec())
    info = adapter.get_info()
    assert info.vendor == "Damiao"
    assert info.model == "TestArm"
    assert adapter.get_dof() == 2
    limits = adapter.get_limits()
    assert limits.position_lower == [-1.0, -2.0]
    assert limits.position_upper == [1.0, 2.0]
    assert limits.velocity_max == [3.0, 4.0]
    assert adapter.set_control_mode(ControlMode.TORQUE) is True
    assert adapter.get_control_mode() == ControlMode.TORQUE
    assert adapter.set_control_mode(ControlMode.VELOCITY) is False


def test_base_adapter_validates_command_lengths() -> None:
    adapter = DamiaoArmAdapterBase(arm_spec=_arm_spec())
    with pytest.raises(ValueError, match="q length 1 does not match dof 2"):
        adapter._validate_command_lengths(
            q=[0.0],
            dq=[0.0, 0.0],
            kp=[0.0, 0.0],
            kd=[0.0, 0.0],
            tau=[0.0, 0.0],
        )
