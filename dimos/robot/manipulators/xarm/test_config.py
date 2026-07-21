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

from dimos.core.global_config import global_config
from dimos.robot.manipulators.xarm.config import xarm6_hardware, xarm7_hardware


def test_xarm_mock_factories_configure_gripper_endpoints(monkeypatch) -> None:
    monkeypatch.setattr(global_config, "simulation", "")
    monkeypatch.setattr(global_config, "xarm6_ip", "")
    monkeypatch.setattr(global_config, "xarm7_ip", "")

    for hardware in (
        xarm6_hardware(
            gripper=True,
            gripper_open_position=0.85,
            gripper_closed_position=0.0,
            mock_without_address=True,
        ),
        xarm7_hardware(
            gripper=True,
            gripper_open_position=0.85,
            gripper_closed_position=0.0,
            mock_without_address=True,
        ),
    ):
        assert (hardware.gripper_closed_position, hardware.gripper_open_position) == (0.0, 0.85)
