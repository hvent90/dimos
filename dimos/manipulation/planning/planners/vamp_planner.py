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

"""VAMP-native joint-space planner adapter."""

from __future__ import annotations

from dimos.manipulation.planning.planners.config import VampPlannerConfig
from dimos.manipulation.planning.spec.models import PlanningResult, WorldRobotID
from dimos.manipulation.planning.spec.protocols import WorldSpec
from dimos.manipulation.planning.world.vamp_world import VampWorld
from dimos.msgs.sensor_msgs.JointState import JointState


class VampPlanner:
    """Joint-space planner adapter for VAMP robot modules."""

    def __init__(self, config: VampPlannerConfig) -> None:
        self.config = config

    def plan_joint_path(
        self,
        world: WorldSpec,
        robot_id: WorldRobotID,
        start: JointState,
        goal: JointState,
        timeout: float = 10.0,
    ) -> PlanningResult:
        """Plan a VAMP-native joint-space path."""
        if not isinstance(world, VampWorld):
            raise ValueError("VampPlanner requires VampWorld")
        return world.plan_joint_path(self.config, robot_id, start, goal, timeout=timeout)

    def get_name(self) -> str:
        """Get planner name."""
        return f"VAMP/{self.config.algorithm}"
