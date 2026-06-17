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

"""Mock-control blueprint for Franka Panda planning.

Usage:
    uv run --extra manipulation --extra vamp dimos run panda-coordinator \
      -o manipulationmodule.world.backend=vamp \
      -o manipulationmodule.world.artifact.mode=official \
      -o manipulationmodule.world.artifact.robot=panda \
      -o manipulationmodule.planner.backend=vamp \
      -o manipulationmodule.planner.algorithm=rrtc
"""

from dimos.control.coordinator import ControlCoordinator
from dimos.core.coordination.blueprints import autoconnect
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.robot.catalog.franka import FRANKA_PANDA_FK_MODEL, franka_panda

_panda_cfg = franka_panda(name="panda")

panda_coordinator = autoconnect(
    ManipulationModule.blueprint(
        robots=[_panda_cfg.to_robot_model_config()],
        planning_timeout=10.0,
        enable_viz=False,
    ),
    ControlCoordinator.blueprint(
        tick_rate=100.0,
        publish_joint_state=True,
        joint_state_frame_id="coordinator",
        hardware=[_panda_cfg.to_hardware_component()],
        tasks=[
            _panda_cfg.to_task_config(
                task_type="cartesian_ik",
                task_name="cartesian_ik_panda",
                model_path=FRANKA_PANDA_FK_MODEL,
                ee_joint_id=_panda_cfg.dof,
            ),
        ],
    ),
)

# Alias matching existing xArm naming style.
panda_planner_coordinator = panda_coordinator

__all__ = ["panda_coordinator", "panda_planner_coordinator"]
