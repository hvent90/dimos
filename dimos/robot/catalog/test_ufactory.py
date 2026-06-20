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

from collections.abc import Callable
import math

import pytest

from dimos.robot.catalog.ufactory import xarm6, xarm7
from dimos.robot.config import RobotConfig


@pytest.mark.parametrize("factory", [xarm6, xarm7])
def test_xarm_instance_offsets_are_encoded_only_in_base_pose(
    factory: Callable[..., RobotConfig],
) -> None:
    config = factory(name="arm", y_offset=0.5, pitch=0.25)
    model_config = config.to_robot_model_config()

    assert model_config.xacro_args["attach_xyz"] == "0 0 0"
    assert model_config.xacro_args["attach_rpy"] == "0 0 0"
    assert model_config.base_pose.position.y == 0.5
    assert model_config.base_pose.orientation.x == 0.0
    assert model_config.base_pose.orientation.y == pytest.approx(math.sin(0.125))
    assert model_config.base_pose.orientation.z == 0.0
    assert model_config.base_pose.orientation.w == pytest.approx(math.cos(0.125))
    assert model_config.strip_model_world_joint is True
