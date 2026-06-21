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

from collections.abc import Sequence
from typing import TYPE_CHECKING, cast

from dimos.manipulation.visualization.types import (
    PlanningGroupInfo,
    RobotInfo,
    TargetEvaluation,
    TargetSetEvaluation,
)
from dimos.msgs.sensor_msgs.JointState import JointState

if TYPE_CHECKING:
    from dimos.manipulation.manipulation_module import ManipulationModule
    from dimos.manipulation.planning.groups.models import PlanningGroup
    from dimos.manipulation.planning.monitor.world_monitor import WorldMonitor
    from dimos.manipulation.planning.spec.config import RobotModelConfig
    from dimos.manipulation.planning.spec.models import PlanningGroupID, RobotName, WorldRobotID
    from dimos.msgs.geometry_msgs.Pose import Pose
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped


def copy_joint_state(joint_state: JointState | None) -> JointState | None:
    """Make a local copy of a JointState-like message for rendering."""
    return None if joint_state is None else JointState(joint_state)


class InProcessViserAdapter:
    """Small in-process boundary between Viser callbacks and manipulation internals."""

    def __init__(
        self,
        *,
        world_monitor: WorldMonitor,
        manipulation_module: ManipulationModule,
    ) -> None:
        self._world_monitor = world_monitor
        self._module = manipulation_module

    def list_robots(self) -> list[RobotName]:
        return list(self._module.list_robots())

    def robot_items(self) -> list[tuple[RobotName, WorldRobotID, RobotModelConfig]]:
        return self._module.robot_items()

    def robot_id_for_name(self, robot_name: RobotName) -> WorldRobotID | None:
        return self._module.robot_id_for_name(robot_name)

    def robot_name_for_id(self, robot_id: WorldRobotID) -> RobotName | None:
        return self._module.robot_name_for_id(robot_id)

    def get_robot_config(self, robot_name: RobotName) -> RobotModelConfig | None:
        return self._module.get_robot_config(robot_name)

    def get_robot_info(self, robot_name: RobotName) -> RobotInfo | None:
        info = self._module.get_robot_info(robot_name)
        if info is None:
            return None
        return {
            "name": str(info["name"]),
            "world_robot_id": str(info["world_robot_id"]),
            "joint_names": [str(name) for name in info["joint_names"]],
            "end_effector_link": str(info["end_effector_link"]),
            "base_link": str(info["base_link"]),
            "max_velocity": float(info["max_velocity"]),
            "max_acceleration": float(info["max_acceleration"]),
            "has_joint_name_mapping": bool(info.get("has_joint_name_mapping", False)),
            "coordinator_task_name": None
            if info["coordinator_task_name"] is None
            else str(info["coordinator_task_name"]),
            "home_joints": None
            if info["home_joints"] is None
            else [float(value) for value in info["home_joints"]],
            "pre_grasp_offset": float(info["pre_grasp_offset"]),
            "init_joints": None
            if info["init_joints"] is None
            else [float(value) for value in info["init_joints"]],
            "planning_groups": [
                {
                    "id": str(group["id"]),
                    "name": str(group["name"]),
                    "robot_name": str(info["name"]),
                    "joint_names": [str(name) for name in group["joint_names"]],
                    "local_joint_names": [str(name) for name in group["local_joint_names"]],
                    "base_link": str(group["base_link"]),
                    "tip_link": None if group["tip_link"] is None else str(group["tip_link"]),
                    "has_pose_target": bool(group["has_pose_target"]),
                    "source": str(group["source"]),
                }
                for group in info.get("planning_groups", [])
            ],
        }

    def list_planning_groups(self) -> list[PlanningGroupInfo]:
        groups: list[PlanningGroupInfo] = []
        for robot_name in self.list_robots():
            info = self.get_robot_info(robot_name)
            if info is None:
                continue
            groups.extend(info.get("planning_groups", []))
        return groups

    def get_init_joints(self, robot_name: RobotName) -> JointState | None:
        return copy_joint_state(self._module.get_init_joints(robot_name))

    def get_current_joint_state(self, robot_name: RobotName) -> JointState | None:
        robot_id = self.robot_id_for_name(robot_name)
        if robot_id is None:
            return None
        return copy_joint_state(self._world_monitor.get_current_joint_state(robot_id))

    def is_state_stale(self, robot_name: RobotName, max_age: float = 1.0) -> bool:
        robot_id = self.robot_id_for_name(robot_name)
        return True if robot_id is None else self._world_monitor.is_state_stale(robot_id, max_age)

    def get_ee_pose(
        self, robot_name: RobotName, joint_state: JointState | None = None
    ) -> PoseStamped | None:
        robot_id = self.robot_id_for_name(robot_name)
        if robot_id is None:
            return None
        return self._world_monitor.get_ee_pose(robot_id, copy_joint_state(joint_state))

    def evaluate_joint_target(self, joints: JointState, robot_name: RobotName) -> TargetEvaluation:
        """Evaluate a joint target through WorldMonitor helpers, not raw WorldSpec access."""
        result: TargetEvaluation = {
            **self._module.evaluate_joint_target(copy_joint_state(joints), robot_name)
        }
        joint_state = result.get("joint_state")
        result["joint_state"] = copy_joint_state(
            joint_state if isinstance(joint_state, JointState) else None
        )
        return result

    def evaluate_pose_target(
        self, pose: Pose, robot_name: RobotName, *, check_collision: bool = True
    ) -> TargetEvaluation:
        """Evaluate a Cartesian target through module/WorldMonitor helper boundaries."""
        result: TargetEvaluation = {
            **self._module.evaluate_pose_target(pose, robot_name, check_collision=check_collision)
        }
        joint_state = result.get("joint_state")
        result["joint_state"] = copy_joint_state(
            joint_state if isinstance(joint_state, JointState) else None
        )
        return result

    def evaluate_joint_target_set(
        self, joint_targets: dict[PlanningGroupID, JointState]
    ) -> TargetSetEvaluation:
        result: TargetSetEvaluation = {
            **self._module.evaluate_joint_target_set(
                cast(
                    "dict[PlanningGroupID | PlanningGroup, JointState]",
                    {
                        group_id: copy_joint_state(target) or target
                        for group_id, target in joint_targets.items()
                    },
                )
            )
        }
        target_joints = result.get("target_joints")
        result["target_joints"] = copy_joint_state(
            target_joints if isinstance(target_joints, JointState) else None
        )
        return result

    def evaluate_pose_target_set(
        self,
        pose_targets: dict[PlanningGroupID, Pose | PoseStamped],
        auxiliary_groups: Sequence[PlanningGroupID] = (),
        seed: JointState | None = None,
        check_collision: bool = True,
    ) -> TargetSetEvaluation:
        result: TargetSetEvaluation = {
            **self._module.evaluate_pose_target_set(
                cast("dict[PlanningGroupID | PlanningGroup, Pose | PoseStamped]", pose_targets),
                auxiliary_groups=auxiliary_groups,
                seed=copy_joint_state(seed),
                check_collision=check_collision,
            )
        }
        target_joints = result.get("target_joints")
        result["target_joints"] = copy_joint_state(
            target_joints if isinstance(target_joints, JointState) else None
        )
        return result

    def get_module_state(self) -> str:
        return str(self._module.get_state())

    def get_error(self) -> str:
        return self._module.get_error()

    def reset(self) -> bool:
        return self._module.reset().is_success()

    def plan_to_pose(self, pose: Pose, robot_name: RobotName | None = None) -> bool:
        return self._module.plan_to_pose(pose, robot_name)

    def plan_to_joints(self, joints: JointState, robot_name: RobotName | None = None) -> bool:
        return self._module.plan_to_joints(joints, robot_name)

    def plan_target_set(self, joint_targets: dict[PlanningGroupID, JointState]) -> bool:
        return self._module.plan_to_joint_targets(
            cast("dict[PlanningGroupID | PlanningGroup, JointState]", joint_targets)
        )

    def preview_plan(self, robot_name: RobotName | None = None) -> bool:
        return self._module.preview_plan(robot_name=robot_name)

    def execute(self) -> bool:
        return self._module.execute()

    def cancel(self) -> bool:
        return self._module.cancel()

    def clear_planned_path(self) -> bool:
        return self._module.clear_planned_path()

    @staticmethod
    def joints_from_values(joint_names: Sequence[str], values: Sequence[float]) -> JointState:
        return JointState(
            {
                "name": list(joint_names),
                "position": [float(value) for value in values],
            }
        )
