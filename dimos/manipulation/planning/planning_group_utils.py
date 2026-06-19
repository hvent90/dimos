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

"""Shared helpers for planning-group selector and joint-state projection."""

from collections.abc import Mapping, Sequence

from dimos.manipulation.planning.planning_identifiers import assert_local_joint_names
from dimos.manipulation.planning.spec.models import (
    GlobalJointName,
    LocalModelJointName,
    PlanningGroupDescriptor,
    PlanningGroupID,
    ResolvedPlanningGroup,
    RobotName,
)
from dimos.msgs.sensor_msgs.JointState import JointState


def planning_group_id_from_selector(
    selector: PlanningGroupID | PlanningGroupDescriptor,
) -> PlanningGroupID:
    """Return the planning-group ID represented by a selector."""
    if isinstance(selector, PlanningGroupDescriptor):
        return selector.id
    return selector


def single_planning_group_id_for_robot(
    groups: Sequence[PlanningGroupDescriptor],
    robot_name: RobotName,
) -> PlanningGroupID:
    """Return a robot's only planning group ID, or raise if ambiguous."""
    group_ids = [group.id for group in groups if group.robot_name == robot_name]
    if len(group_ids) != 1:
        raise ValueError(
            f"Robot '{robot_name}' has {len(group_ids)} planning groups; "
            "select a planning group explicitly"
        )
    return group_ids[0]


def primary_pose_planning_group_id_for_robot(
    groups: Sequence[PlanningGroupDescriptor],
    robot_name: RobotName,
) -> PlanningGroupID | None:
    """Return the first pose-targetable group ID for compatibility paths."""
    for group in groups:
        if group.robot_name == robot_name and group.has_pose_target:
            return group.id
    return None


def matching_global_joint_name(
    positions_by_name: Mapping[str, float], local_joint_name: LocalModelJointName
) -> GlobalJointName | None:
    """Find the unique global joint name ending with a local joint name."""
    suffix = f"/{local_joint_name}"
    matches = [name for name in positions_by_name if name.endswith(suffix)]
    if len(matches) == 1:
        return matches[0]
    return None


def filter_joint_state_to_selected_joints(
    joint_state: JointState,
    global_joint_names: Sequence[GlobalJointName],
    local_joint_names: Sequence[LocalModelJointName] = (),
) -> JointState:
    """Project a joint state to selected global joints.

    Values are looked up by global name first. When ``local_joint_names`` is
    provided, each corresponding local name is used as a fallback.
    """
    if local_joint_names and len(global_joint_names) != len(local_joint_names):
        raise ValueError("Global and local selected joint lists must have the same length")

    positions_by_name = dict(zip(joint_state.name, joint_state.position, strict=True))
    selected_positions: list[float] = []
    missing: list[str] = []
    for index, global_name in enumerate(global_joint_names):
        if global_name in positions_by_name:
            selected_positions.append(float(positions_by_name[global_name]))
            continue
        if local_joint_names:
            local_name = local_joint_names[index]
            if local_name in positions_by_name:
                selected_positions.append(float(positions_by_name[local_name]))
                continue
        missing.append(global_name)

    if missing:
        raise ValueError(f"IK result is missing selected joints: {missing}")

    return JointState({"name": list(global_joint_names), "position": selected_positions})


def normalize_joint_target_for_group(
    group: ResolvedPlanningGroup,
    target: JointState,
) -> JointState:
    """Normalize a robot-scoped group target to global joint names in group order."""
    if not target.name:
        if len(target.position) != len(group.joint_names):
            raise ValueError(
                f"Target for '{group.id}' has {len(target.position)} positions, "
                f"expected {len(group.joint_names)}"
            )
        return JointState(name=list(group.joint_names), position=list(target.position))

    assert_local_joint_names(target.name)
    positions_by_name = dict(zip(target.name, target.position, strict=False))
    global_positions: list[float] = []
    missing: list[str] = []
    for local_name in group.local_joint_names:
        if local_name in positions_by_name:
            global_positions.append(positions_by_name[local_name])
        else:
            missing.append(local_name)
    if missing:
        raise ValueError(f"Target for '{group.id}' is missing joints: {missing}")

    extra = set(target.name) - set(group.local_joint_names)
    if extra:
        raise ValueError(f"Target for '{group.id}' has extra joints: {sorted(extra)}")
    return JointState(name=list(group.joint_names), position=global_positions)
