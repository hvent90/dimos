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

"""G1 GR00T WBC policy with a WASD teleop panel.

Usage:
    dimos run unitree-g1-groot-wbc-keyboard-teleop
"""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.unitree.g1.blueprints.basic.unitree_g1_groot_wbc import (
    unitree_g1_groot_wbc,
)
from dimos.robot.unitree.g1.g1_groot_wbc_teleop import G1GrootWbcTeleop

unitree_g1_groot_wbc_keyboard_teleop = autoconnect(
    unitree_g1_groot_wbc,
    G1GrootWbcTeleop.blueprint(),
).remappings([(G1GrootWbcTeleop, "cmd_vel", "tele_cmd_vel")])
