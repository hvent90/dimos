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

"""Basic Galaxea A1Z coordinator and planner blueprints."""

from __future__ import annotations

from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.manipulators.a1z.config import (
    A1Z_G1Z_MODEL_PATH,
    make_a1z_model_config,
)
from dimos.robot.manipulators.common.blueprints import (
    coordinator,
    planner,
    trajectory_task,
)
from dimos.robot.manipulators.galaxea_a1z.config import galaxea_a1z_hardware

# The real arm has a G1Z gripper. Its mass must be present in the dynamics
# model, and the hardware component exposes its measured/commanded opening as
# arm/gripper alongside the six arm joints.
_A1Z_DYNAMICS_URDF = str(A1Z_G1Z_MODEL_PATH)
_a1z_hw = galaxea_a1z_hardware("arm", gripper=True, dynamics_urdf_path=_A1Z_DYNAMICS_URDF)

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

# Planner and Viser use the same physical G1Z model and tool-center frame.
_planner_hw = galaxea_a1z_hardware("arm", gripper=True, dynamics_urdf_path=_A1Z_DYNAMICS_URDF)

galaxea_a1z_planner_coordinator = autoconnect(
    planner(
        robots=[
            make_a1z_model_config(
                name="arm",
                has_gripper=True,
            )
        ]
    ),
    coordinator(
        hardware=[_planner_hw],
        tasks=[trajectory_task(_planner_hw)],
    ),
)
