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

"""Basic Galaxea A1Z coordinator blueprints."""

from __future__ import annotations

from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.manipulators.galaxea_a1z.config import galaxea_a1z_hardware

# Arm-only stable configuration: the a1z SDK 'gripper' branch ships a G1Z
# gravity model that mismatches this unit's mounting (pushes the arm during
# zero-force startup; soft e-stop cannot catch the arm with comp disabled).
# Until the model/mount is calibrated, run the SDK *main* branch with
# gripper=False. Gripper code support remains in the adapter.
_a1z_hw = galaxea_a1z_hardware("arm", gripper=False)

coordinator_galaxea_a1z = autoconnect(
    ControlCoordinator.blueprint(
        hardware=[_a1z_hw],
        tasks=[
            TaskConfig(
                name="traj_arm",
                type="trajectory",
                joint_names=_a1z_hw.joints,
                priority=10,
            )
        ],
    ),
)
