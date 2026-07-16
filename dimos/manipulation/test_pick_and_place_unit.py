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

"""Unit tests for PickAndPlaceModule pure logic (no Drake required)."""

from __future__ import annotations

from types import SimpleNamespace
from unittest.mock import patch

import open3d as o3d
import pytest

from dimos.agents.skill_result import SkillResult
from dimos.core.module import ModuleBase
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.perception.detection.type.detection3d.object import Object as DetObject


def _make_det_object(
    name: str = "cup",
    object_id: str = "abc12345",
    center: tuple[float, float, float] = (0.5, 0.0, 0.3),
    size: tuple[float, float, float] = (0.05, 0.05, 0.10),
) -> DetObject:
    """Create a DetObject with the given attributes and sensible defaults."""
    return DetObject(
        name=name,
        object_id=object_id,
        center=Vector3(x=center[0], y=center[1], z=center[2]),
        size=Vector3(x=size[0], y=size[1], z=size[2]),
        pose=PoseStamped(),
        pointcloud=PointCloud2(o3d.geometry.PointCloud()),
        bbox=(0.0, 0.0, 1.0, 1.0),
        track_id=0,
        class_id=0,
        confidence=1.0,
        ts=0.0,
        image=Image(),
    )


class _FakeWorldMonitor:
    """Small perception monitor double for scan tests."""

    def __init__(self, objects: list[DetObject]) -> None:
        self.objects = objects
        self.refresh_durations: list[float] = []
        self.removed_object_ids: list[str] = []
        self.resync_count = 0
        self.active_object_ids = {obj.object_id for obj in objects}

    def on_objects(self, objects: list[DetObject]) -> None:
        self.objects = objects

    def refresh_obstacles(self, min_duration: float) -> list[dict[str, object]]:
        self.refresh_durations.append(min_duration)
        self.resync_count += 1
        self.active_object_ids = {obj.object_id for obj in self.objects}
        return [{"object_id": obj.object_id} for obj in self.objects]

    def get_cached_objects(self) -> list[DetObject]:
        return self.objects

    def remove_object_obstacle(self, object_id: str) -> bool:
        self.removed_object_ids.append(object_id)
        self.active_object_ids.discard(object_id)
        return True


class _FakeSceneRegistration:
    """Object registration RPC double."""

    def __init__(self, objects: list[DetObject]) -> None:
        self.objects = objects
        self.queried = False

    def get_registered_objects(self) -> list[DetObject]:
        self.queried = True
        return self.objects


@pytest.fixture
def module() -> PickAndPlaceModule:
    """Create a PickAndPlaceModule with heavy base init (RPC, config) patched out."""
    with patch.object(ModuleBase, "__init__", lambda self, config_args: None):
        return PickAndPlaceModule()


class TestFindObjectInDetections:
    """Test object lookup logic in detection snapshot."""

    def test_find_by_exact_name(self, module):
        det = _make_det_object(name="cup")
        module._detection_snapshot = [det]

        result = module._find_object_in_detections("cup")
        assert result is det

    def test_find_by_partial_name(self, module):
        det = _make_det_object(name="red cup")
        module._detection_snapshot = [det]

        result = module._find_object_in_detections("cup")
        assert result is det

    def test_find_by_object_id(self, module):
        det = _make_det_object(object_id="abc12345")
        module._detection_snapshot = [det]

        # Truncated prefix match
        result = module._find_object_in_detections("anything", object_id="abc1")
        assert result is det

    def test_find_by_object_id_ambiguous_returns_none(self, module):
        det1 = _make_det_object(object_id="abc12345")
        det2 = _make_det_object(object_id="abc19999")
        module._detection_snapshot = [det1, det2]

        result = module._find_object_in_detections("anything", object_id="abc1")
        assert result is None

    def test_find_missing_returns_none(self, module):
        module._detection_snapshot = [_make_det_object(name="bottle")]

        result = module._find_object_in_detections("keyboard")
        assert result is None

    def test_empty_snapshot_returns_none(self, module):
        module._detection_snapshot = []

        result = module._find_object_in_detections("cup")
        assert result is None


class TestGraspHeuristics:
    """Test grasp orientation and occlusion offset static methods."""

    def test_occlusion_offset_toward_robot(self):
        center = Vector3(x=0.5, y=0.0, z=0.3)
        size = Vector3(x=0.1, y=0.1, z=0.1)

        ox, oy = PickAndPlaceModule._occlusion_offset(center, size)
        # Offset should shift x closer to robot origin (smaller x)
        assert ox < center.x
        assert abs(oy - center.y) < 1e-6  # y should stay ~0

    def test_occlusion_offset_at_origin(self):
        center = Vector3(x=0.0, y=0.0, z=0.3)
        size = Vector3(x=0.1, y=0.1, z=0.1)

        ox, oy = PickAndPlaceModule._occlusion_offset(center, size)
        # At origin, no shift should occur (division-by-zero guard)
        assert abs(ox) < 1e-3
        assert abs(oy) < 1e-3

    def test_grasp_orientation_near_is_top_down(self):
        q = PickAndPlaceModule._grasp_orientation(gx=0.3, gy=0.0, xy_dist=0.3)
        # Near object: pitch = 180° (top-down), tilt = 0, yaw = 0
        # RPY(0, π, 0) → quaternion (x=0, y=1, z=0, w=0)
        assert abs(q.x) < 0.01
        assert abs(q.y - 1.0) < 0.01
        assert abs(q.z) < 0.01
        assert abs(q.w) < 0.01

    def test_grasp_orientation_far_differs_from_near(self):
        q_near = PickAndPlaceModule._grasp_orientation(gx=0.3, gy=0.0, xy_dist=0.3)
        q_far = PickAndPlaceModule._grasp_orientation(gx=1.0, gy=0.0, xy_dist=1.0)
        # Far object should have different orientation (tilted)
        assert not (
            abs(q_near.x - q_far.x) < 0.01
            and abs(q_near.y - q_far.y) < 0.01
            and abs(q_near.z - q_far.z) < 0.01
            and abs(q_near.w - q_far.w) < 0.01
        )


class TestPlaceBack:
    """Test place_back guard logic."""

    def test_place_back_no_pick_pose_errors(self, module):
        module._last_pick_pose = None

        result = module.place_back()
        assert not result.is_success()
        assert result.error_code == "NO_PRIOR_POSE"
        assert "pick" in result.message.lower()


class TestPickTargetObstacleRemoval:
    """Test targeted obstacle removal during the pick approach."""

    def test_pick_exposes_first_pregrasp_planning_failure(self, module):
        selected = _make_det_object(object_id="selected-id")
        module._world_monitor = _FakeWorldMonitor([selected])
        module._detection_snapshot = [selected]
        module._get_robot = lambda robot_name: (
            "arm",
            None,
            SimpleNamespace(pre_grasp_offset=0.1),
            None,
        )
        module._generate_grasps_for_pick = lambda object_name, object_id: [Pose(0.0, 0.0, 0.0)]
        module._lift_if_low = lambda robot_name: SkillResult.ok()
        module.plan_to_pose = lambda pose, robot_name: False
        module.get_error = lambda: "IK failed: NO_SOLUTION: target is unreachable"

        with patch("dimos.manipulation.pick_and_place_module.time.sleep"):
            result = module.pick("cup")

        assert result.error_code == "PLANNING_FAILED"
        assert "IK failed: NO_SOLUTION: target is unreachable" in result.message

    def test_removes_selected_object_after_pregrasp_before_final_plan(self, module):
        selected = _make_det_object(name="cup", object_id="selected-id")
        other = _make_det_object(name="bottle", object_id="other-id")
        monitor = _FakeWorldMonitor([selected, other])
        module._world_monitor = monitor
        module._detection_snapshot = [selected, other]

        grasp_pose = Pose(0.0, 0.0, 0.0)
        pre_grasp_pose = Pose(0.0, 0.0, 0.0)
        events = []
        module._get_robot = lambda robot_name: (
            "arm",
            None,
            SimpleNamespace(pre_grasp_offset=0.1),
            None,
        )
        module._generate_grasps_for_pick = lambda object_name, object_id: [grasp_pose]
        module._compute_pre_grasp_pose = lambda pose, offset: pre_grasp_pose
        module._lift_if_low = lambda robot_name: SkillResult.ok()
        module.plan_to_pose = (
            lambda pose, robot_name: events.append(
                "pregrasp-plan" if pose is pre_grasp_pose else "grasp-plan"
            )
            or True
        )
        module._preview_execute_wait = (
            lambda robot_name: events.append("pregrasp-execute") or SkillResult.ok()
        )
        module._set_gripper_position = lambda position, robot_name: True

        original_remove = monitor.remove_object_obstacle

        def remove_object_obstacle(object_id):
            events.append("remove")
            return original_remove(object_id)

        monitor.remove_object_obstacle = remove_object_obstacle

        with patch("dimos.manipulation.pick_and_place_module.time.sleep"):
            result = module.pick("cup")

        assert result.is_success()
        assert events[:4] == ["pregrasp-plan", "pregrasp-execute", "remove", "grasp-plan"]
        assert monitor.removed_object_ids == ["selected-id"]

    @pytest.mark.parametrize("failure", ["planning", "execution"])
    def test_resynchronizes_target_after_final_grasp_failure(self, module, failure):
        selected = _make_det_object(name="cup", object_id="selected-id")
        monitor = _FakeWorldMonitor([selected])
        module._world_monitor = monitor
        module._detection_snapshot = [selected]

        grasp_pose = Pose(0.0, 0.0, 0.0)
        pre_grasp_pose = Pose(0.0, 0.0, 0.0)
        events = []
        module._get_robot = lambda robot_name: (
            "arm",
            None,
            SimpleNamespace(pre_grasp_offset=0.1),
            None,
        )
        module._generate_grasps_for_pick = lambda object_name, object_id: [grasp_pose]
        module._compute_pre_grasp_pose = lambda pose, offset: pre_grasp_pose
        module._lift_if_low = lambda robot_name: SkillResult.ok()
        module.get_error = lambda: "final grasp IK failed"

        def plan_to_pose(pose, robot_name):
            events.append("pregrasp-plan" if pose is pre_grasp_pose else "grasp-plan")
            return failure != "planning" or pose is pre_grasp_pose

        module.plan_to_pose = plan_to_pose

        execution_count = 0

        def execute(robot_name):
            nonlocal execution_count
            execution_count += 1
            events.append("pregrasp-execute" if execution_count == 1 else "grasp-execute")
            if execution_count == 1:
                return SkillResult.ok()
            return SkillResult.fail("EXECUTION_FAILED", "final grasp execution failed")

        module._preview_execute_wait = execute
        module._set_gripper_position = lambda position, robot_name: True

        original_remove = monitor.remove_object_obstacle

        def remove_object_obstacle(object_id):
            events.append("remove")
            return original_remove(object_id)

        monitor.remove_object_obstacle = remove_object_obstacle

        original_refresh = monitor.refresh_obstacles

        def refresh_obstacles(min_duration=0.0):
            events.append("resync")
            return original_refresh(min_duration)

        monitor.refresh_obstacles = refresh_obstacles

        with patch("dimos.manipulation.pick_and_place_module.time.sleep"):
            result = module.pick("cup")

        assert not result.is_success()
        assert events[-1] == "resync"
        assert events.index("remove") < events.index("grasp-plan")
        if failure == "execution":
            assert events == [
                "pregrasp-plan",
                "pregrasp-execute",
                "remove",
                "grasp-plan",
                "grasp-execute",
                "resync",
            ]
        assert monitor.resync_count == 1
        assert monitor.active_object_ids == {"selected-id"}
        if failure == "planning":
            assert result.message == "Grasp pose planning failed: final grasp IK failed"
        else:
            assert result.message == "final grasp execution failed"

    def test_restores_target_when_gripper_close_fails(self, module):
        selected = _make_det_object(name="cup", object_id="selected-id")
        monitor = _FakeWorldMonitor([selected])
        module._world_monitor = monitor
        module._detection_snapshot = [selected]
        module._get_robot = lambda robot_name: (
            "arm",
            None,
            SimpleNamespace(pre_grasp_offset=0.1),
            None,
        )
        module._generate_grasps_for_pick = lambda object_name, object_id: [Pose(0.0, 0.0, 0.0)]
        module._lift_if_low = lambda robot_name: SkillResult.ok()
        module.plan_to_pose = lambda pose, robot_name: True
        module._preview_execute_wait = lambda robot_name: SkillResult.ok()
        module._set_gripper_position = lambda position, robot_name: position != 0.0

        with patch("dimos.manipulation.pick_and_place_module.time.sleep"):
            result = module.pick("cup")

        assert not result.is_success()
        assert result.error_code == "GRIPPER_FAILED"
        assert monitor.resync_count == 1
        assert monitor.active_object_ids == {"selected-id"}

    def test_reports_target_restoration_failure_after_close_failure(self, module):
        selected = _make_det_object(name="cup", object_id="selected-id")
        monitor = _FakeWorldMonitor([selected])
        module._world_monitor = monitor
        module._detection_snapshot = [selected]
        module._get_robot = lambda robot_name: (
            "arm",
            None,
            SimpleNamespace(pre_grasp_offset=0.1),
            None,
        )
        module._generate_grasps_for_pick = lambda object_name, object_id: [Pose(0.0, 0.0, 0.0)]
        module._lift_if_low = lambda robot_name: SkillResult.ok()
        module.plan_to_pose = lambda pose, robot_name: True
        module._preview_execute_wait = lambda robot_name: SkillResult.ok()
        module._set_gripper_position = lambda position, robot_name: False
        monitor.refresh_obstacles = lambda min_duration=0.0: (_ for _ in ()).throw(
            RuntimeError("planning world unavailable")
        )

        with patch("dimos.manipulation.pick_and_place_module.time.sleep"):
            result = module.pick("cup")

        assert not result.is_success()
        assert result.error_code == "GRIPPER_FAILED"
        assert "planning world unavailable" in result.message

    def test_failed_restore_blocks_subsequent_planning(self, module):
        module._scene_reconciliation_error = "planning world unavailable"
        module.plan_to_pose = lambda pose, robot_name: pytest.fail("planning was attempted")

        result = module.pick("cup")

        assert not result.is_success()
        assert result.error_code == "GRIPPER_FAILED"
        assert "Scene reconciliation required" in result.message

    def test_reset_failure_preserves_scene_reconciliation_block(self, module):
        module._scene_reconciliation_error = "planning world unavailable"
        monitor = _FakeWorldMonitor([])
        module._world_monitor = monitor
        monitor.refresh_obstacles = lambda min_duration=0.0: (_ for _ in ()).throw(
            RuntimeError("planning world still unavailable")
        )

        result = module.reset()

        assert not result.is_success()
        assert "planning world still unavailable" in result.message
        assert module._scene_reconciliation_error == "planning world still unavailable"

    def test_successful_reset_restores_scene_and_unblocks_planning(self, module):
        module._scene_reconciliation_error = "planning world unavailable"
        monitor = _FakeWorldMonitor([])
        module._world_monitor = monitor

        result = module.reset()

        assert result.is_success()
        assert module._scene_reconciliation_error is None
        assert monitor.resync_count == 1
        assert not module._planning_is_blocked()

    def test_empty_restore_keeps_reset_blocked(self, module):
        selected = _make_det_object(object_id="selected-id")
        module._world_monitor = _FakeWorldMonitor([selected])
        module._excluded_target_object_id = selected.object_id
        module._scene_reconciliation_error = "initial restore failure"
        module._world_monitor.objects = []

        result = module.reset()

        assert not result.is_success()
        assert "selected-id" in result.message
        assert module._scene_reconciliation_error == "target object 'selected-id' was not re-added"

    def test_reconciliation_blocks_scene_dependent_motion_apis(self, module):
        module._scene_reconciliation_error = "target was not restored"
        pose = Pose(0.0, 0.0, 0.0)

        assert not module.plan_to_pose(pose)
        assert not module.plan_to_joints(SimpleNamespace(position=[]))
        assert module.solve_ik(pose).status.name == "NO_SOLUTION"
        assert not module.preview_path()
        assert not module.execute()

    @pytest.mark.parametrize("failure", ["planning", "execution"])
    def test_does_not_remove_target_when_pregrasp_fails(self, module, failure):
        selected = _make_det_object(object_id="selected-id")
        module._world_monitor = _FakeWorldMonitor([selected])
        module._detection_snapshot = [selected]
        module._get_robot = lambda robot_name: (
            "arm",
            None,
            SimpleNamespace(pre_grasp_offset=0.1),
            None,
        )
        module._generate_grasps_for_pick = lambda object_name, object_id: [Pose(0.0, 0.0, 0.0)]
        module._lift_if_low = lambda robot_name: SkillResult.ok()
        module.plan_to_pose = lambda pose, robot_name: failure != "planning"
        module._set_gripper_position = lambda position, robot_name: None
        module._preview_execute_wait = lambda robot_name: SkillResult.fail(
            "EXECUTION_FAILED", "pregrasp failed"
        )

        with patch("dimos.manipulation.pick_and_place_module.time.sleep"):
            module.pick("cup")

        assert module._world_monitor.removed_object_ids == []


class TestScanObjects:
    """Test scans against the producer's permanent registered snapshot."""

    def test_scan_queries_after_init_without_stream_callback(self, module):
        objects = [_make_det_object(name="old permanent cup")]
        monitor = _FakeWorldMonitor([])
        registration = _FakeSceneRegistration(objects)
        module._world_monitor = monitor
        module._scene_registration = registration
        calls = []
        module.go_init = lambda robot_name=None: calls.append("init") or SkillResult.ok()

        def get_registered_objects():
            calls.append("query")
            return registration.objects

        registration.get_registered_objects = get_registered_objects

        def refresh_obstacles(min_duration=0.0):
            calls.append("refresh")
            result = monitor.refresh_obstacles(min_duration)
            module._detection_snapshot = monitor.get_cached_objects()
            return result

        module.refresh_obstacles = refresh_obstacles

        result = module.scan_objects()

        assert result.is_success()
        assert "old permanent cup" in result.message
        assert calls == ["init", "query", "refresh"]

    def test_scan_empty_query_clears_stale_state(self, module):
        stale = _make_det_object(name="stale cup")
        monitor = _FakeWorldMonitor([stale])
        module._world_monitor = monitor
        module._scene_registration = _FakeSceneRegistration([])
        module._detection_snapshot = [stale]
        module.go_init = lambda robot_name=None: SkillResult.ok()

        result = module.scan_objects()

        assert result.is_success()
        assert result.message == "No objects detected in scene"
        assert monitor.objects == []
        assert module._detection_snapshot == []

    def test_scan_failed_init_skips_query_and_refresh(self, module):
        registration = _FakeSceneRegistration([_make_det_object()])
        monitor = _FakeWorldMonitor([])
        module._scene_registration = registration
        module._world_monitor = monitor
        module.go_init = lambda robot_name=None: SkillResult.fail("EXECUTION_FAILED", "init failed")
        module.refresh_obstacles = lambda min_duration=0.0: pytest.fail("refresh called")

        result = module.scan_objects()

        assert not result.is_success()
        assert result.message == "init failed"
        assert not registration.queried

    def test_scan_forwards_min_duration(self, module):
        objects = [_make_det_object(name="cup")]
        monitor = _FakeWorldMonitor([])
        registration = _FakeSceneRegistration(objects)
        module._world_monitor = monitor
        module._scene_registration = registration
        module.go_init = lambda robot_name=None: SkillResult.ok()

        result = module.scan_objects(min_duration=2.5)

        assert result.is_success()
        assert monitor.refresh_durations == [2.5]

    def test_scan_reports_perception_rpc_failure(self, module):
        monitor = _FakeWorldMonitor([])
        registration = _FakeSceneRegistration([])
        module._world_monitor = monitor
        module._scene_registration = registration
        module.go_init = lambda robot_name=None: SkillResult.ok()

        def raise_rpc_error():
            raise RuntimeError("registration unavailable")

        registration.get_registered_objects = raise_rpc_error
        module.refresh_obstacles = lambda min_duration=0.0: pytest.fail("refresh called")

        result = module.scan_objects()

        assert not result.is_success()
        assert result.error_code == "PERCEPTION_UNAVAILABLE"
        assert "registration unavailable" in result.message
