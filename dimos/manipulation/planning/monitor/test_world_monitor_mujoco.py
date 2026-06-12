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

"""WorldMonitor wiring tests for the mujoco backend: G1 catalog configs,
entity-state-batch sync, and graceful no-op on backends without entities."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("mujoco")

from dimos.manipulation.planning.monitor.world_monitor import WorldMonitor
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState

_REPO_ROOT = Path(__file__).parents[4]
_G1_ASSETS = _REPO_ROOT / "data" / "mujoco_sim" / "g1_gear_wbc.xml"


def _entity_batch(entity_id: str, position: list[float]):
    from dimos.experimental.pimsim.entity import EntityDescriptor, EntityStateBatch

    descriptor = EntityDescriptor(entity_id=entity_id, kind="dynamic", mass=1.0)
    pose = PoseStamped(position=position, orientation=[0.0, 0.0, 0.0, 1.0])
    return EntityStateBatch(entries=[(descriptor, pose)])


@pytest.mark.skipif(not _G1_ASSETS.exists(), reason="G1 MJCF assets not present")
def test_g1_catalog_mujoco_backend_plans() -> None:
    """The catalog's backend="mujoco" configs drive a full WorldMonitor +
    RRT flow: dual arms on one shared MJCF, FK, and a collision-checked plan."""
    from dimos.manipulation.planning.factory import create_planner
    from dimos.robot.catalog.g1 import g1_left_arm, g1_right_arm

    left_cfg = g1_left_arm(backend="mujoco").robot_model_config
    right_cfg = g1_right_arm(backend="mujoco").robot_model_config

    monitor = WorldMonitor(backend="mujoco")
    left = monitor.add_robot(left_cfg)
    right = monitor.add_robot(right_cfg, share_model_with=left)
    monitor.finalize()

    zeros = JointState(name=left_cfg.joint_names, position=[0.0] * 7)
    assert monitor.is_state_valid(left, zeros)
    assert monitor.is_state_valid(right, JointState(name=right_cfg.joint_names, position=[0.0] * 7))

    # Grasp offset applied: EE pose is the palm grasp center, not the wrist.
    pose = monitor.get_ee_pose(left, joint_state=zeros)
    assert pose.frame_id == "world"

    from dimos.manipulation.planning.spec.enums import PlanningStatus

    goal = JointState(name=left_cfg.joint_names, position=[0.3, 0.4, 0.1, 0.8, 0.0, 0.3, 0.2])
    result = create_planner().plan_joint_path(monitor.world, left, zeros, goal, timeout=20.0)
    assert result.status == PlanningStatus.SUCCESS, result.message


def test_entity_state_batch_updates_collision_world(tmp_path: Path) -> None:
    """Entity poses streamed through on_entity_state_batch move collision
    bodies in the planning world."""
    from dimos.manipulation.planning.world.test_mujoco_world import _JOINTS, _arm_config

    crate = {
        "id": "crate",
        "initial_pose": {"x": 0.4, "y": 0.0, "z": 0.1, "qw": 1.0, "qx": 0.0, "qy": 0.0, "qz": 0.0},
        "aabb": {"min": [0.35, -0.2, 0.0], "max": [0.45, 0.2, 0.2]},
        "descriptor": {
            "entity_id": "crate",
            "kind": "dynamic",
            "shape_hint": "box",
            "extents": [0.1, 0.4, 0.4],
            "mass": 1.0,
        },
        "physics": {"shape": "box"},
    }
    monitor = WorldMonitor(backend="mujoco", scene_entities=[crate])
    robot_id = monitor.add_robot(_arm_config(tmp_path))
    monitor.finalize()

    stretched = JointState(name=_JOINTS, position=[0.0, 0.0, 0.0])
    assert not monitor.is_state_valid(robot_id, stretched)

    monitor.on_entity_state_batch(_entity_batch("crate", [0.4, 2.0, 0.1]))
    assert monitor.is_state_valid(robot_id, stretched)

    monitor.on_entity_state_batch(_entity_batch("crate", [0.4, 0.0, 0.1]))
    assert not monitor.is_state_valid(robot_id, stretched)


def test_entity_state_batch_is_noop_on_drake(tmp_path: Path) -> None:
    pytest.importorskip("pydrake")
    from dimos.manipulation.planning.world.test_mujoco_world import _arm_config

    monitor = WorldMonitor(backend="drake")
    monitor.add_robot(_arm_config(tmp_path))
    monitor.finalize()
    # Must not raise — Drake has no entity support.
    monitor.on_entity_state_batch(_entity_batch("crate", [0.4, 0.0, 0.1]))


def test_unknown_entities_are_ignored(tmp_path: Path) -> None:
    from dimos.manipulation.planning.world.test_mujoco_world import _arm_config

    monitor = WorldMonitor(backend="mujoco")
    monitor.add_robot(_arm_config(tmp_path))
    monitor.finalize()
    monitor.on_entity_state_batch(_entity_batch("not_in_scene", [0.0, 0.0, 0.0]))


def test_g1_catalog_targets_grasp_center_not_wrist(tmp_path: Path) -> None:
    """Sanity: the G1 catalog config carries a non-zero grasp offset — the
    EE pose must not silently regress to the wrist link origin. The drake
    backend is rejected until DrakeWorld grows grasp-offset support."""
    from dimos.robot.catalog.g1 import g1_left_arm, g1_right_arm

    mjc_cfg = g1_left_arm(backend="mujoco").robot_model_config
    assert np.linalg.norm(mjc_cfg.grasp_offset_xyz) > 0.05
    assert mjc_cfg.end_effector_link == "left_wrist_yaw_link"
    assert str(mjc_cfg.model_path).endswith(".xml")
    assert mjc_cfg.model_meshdir is not None
    assert not mjc_cfg.weld_base  # pelvis stays a floating base

    # Mirrored offsets: same reach forward, opposite lateral component.
    right_cfg = g1_right_arm().robot_model_config
    assert right_cfg.grasp_offset_xyz[0] == mjc_cfg.grasp_offset_xyz[0]
    assert right_cfg.grasp_offset_xyz[1] == -mjc_cfg.grasp_offset_xyz[1]

    with pytest.raises(ValueError, match="mujoco"):
        g1_left_arm(backend="drake")
