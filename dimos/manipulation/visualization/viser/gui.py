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

from typing import TypeAlias

from dimos.manipulation.planning.spec.models import PlanningGroupID
from dimos.manipulation.visualization.types import (
    PlanningGroupInfo,
    RobotInfo,
    TargetEvaluation,
    TargetSetEvaluation,
)
from dimos.manipulation.visualization.viser.adapter import InProcessViserAdapter
from dimos.manipulation.visualization.viser.config import ViserVisualizationConfig
from dimos.manipulation.visualization.viser.runtime import VISER_INSTALL_HINT
from dimos.manipulation.visualization.viser.scene import ViserManipulationScene
from dimos.manipulation.visualization.viser.state import (
    ActionStatus,
    BackendConnectionStatus,
    FeasibilityStatus,
    OperationWorker,
    PanelPlanState,
    PanelRuntime,
    PanelState,
    PlanStatus,
    TargetEvaluationRequest,
    TargetEvaluationWorker,
    TargetStatus,
)
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

try:
    from viser import (
        GuiApi,
        GuiButtonHandle,
        GuiCheckboxHandle,
        GuiDropdownHandle,
        GuiFolderHandle,
        GuiMarkdownHandle,
        GuiSliderHandle,
        TransformControlsHandle,
        ViserServer,
    )
except ModuleNotFoundError as e:
    if e.name != "viser":
        raise
    raise ModuleNotFoundError(VISER_INSTALL_HINT) from e

PanelHandle: TypeAlias = (
    GuiFolderHandle
    | GuiMarkdownHandle
    | GuiDropdownHandle[str]
    | GuiButtonHandle
    | GuiCheckboxHandle
    | TransformControlsHandle
)

# Fallback joint-slider range (radians) when a robot config omits joint limits.
DEFAULT_JOINT_LIMITS = (-3.14, 3.14)


class ViserPanelGui:
    """Optional operator panel with parity for the original cc/viser-vis panel."""

    def __init__(
        self,
        server: ViserServer,
        adapter: InProcessViserAdapter,
        config: ViserVisualizationConfig,
        scene: ViserManipulationScene | None = None,
    ) -> None:
        self.server = server
        self.adapter = adapter
        self.config = config
        self.scene = scene
        self.state = PanelState(runtime=PanelRuntime.STARTING)
        self._closed = False
        self._operation_sequence_id = 0
        self._suppress_target_callbacks = False
        self._default_group_initialized = False
        self._handles: dict[str, PanelHandle] = {}
        self._joint_sliders: dict[str, GuiSliderHandle[float]] = {}
        self._worker = TargetEvaluationWorker(
            self._handle_target_evaluation_request,
            self._apply_target_evaluation_result,
        )
        self._operation_worker = OperationWorker(self._set_error)

    def start(self) -> None:
        if self._closed:
            raise RuntimeError("Cannot restart a closed ViserPanelGui")
        if self.state.runtime == PanelRuntime.RUNNING:
            return
        try:
            self._worker.start()
            self._operation_worker.start()
            self.state.runtime = PanelRuntime.RUNNING
            self._build()
            self.refresh()
        except Exception:
            self.close()
            self.state.runtime = PanelRuntime.FAILED
            raise

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self.state.runtime = PanelRuntime.STOPPING
        self._worker.stop()
        self._operation_worker.stop(timeout=2.0)
        self._clear_joint_sliders()
        self._remove_panel_handles()
        self._handles.clear()
        self.state.runtime = PanelRuntime.STOPPED

    def refresh(self) -> None:
        if self._closed:
            return
        robots = self.adapter.list_robots()
        groups = self.adapter.list_planning_groups()
        self.state.backend_status = (
            BackendConnectionStatus.READY if robots else BackendConnectionStatus.WAITING_FOR_ROBOT
        )
        if not self.state.selected_group_ids and groups and not self._default_group_initialized:
            self.state.selected_group_ids = (str(groups[0]["id"]),)
            self.state.selected_robot = str(groups[0]["robot_name"])
            self.state.target_status = TargetStatus.EMPTY
            self._default_group_initialized = True
            self._initialize_selected_group_targets()
            self._build_joint_sliders()
        self._sync_robot_dropdown(robots)
        self._sync_group_checkboxes(groups)
        self._refresh_selected_robot_state()
        self._ensure_scene_controls()
        self._sync_preset_dropdown()
        self._update_status_text()
        self._update_control_state()

    def _build(self) -> None:
        gui = self.server.gui
        folder = gui.add_folder("Manipulation Panel", expand_by_default=True)
        self._handles["panel_folder"] = folder
        with folder:
            self._build_panel_controls(gui)

    def _build_panel_controls(self, gui: GuiApi) -> None:
        self._handles["status"] = gui.add_markdown("Starting manipulation panel...")
        robots = self.adapter.list_robots()
        self._build_scene_controls(gui)
        robot_dropdown = gui.add_dropdown(
            "Robot",
            options=robots or [""],
            initial_value=robots[0] if robots else "",
        )
        robot_dropdown.on_update(lambda event: self._select_robot(event.target.value))
        self._handles["robot"] = robot_dropdown
        select_all_button = gui.add_button("Select all manipulators")
        select_all_button.on_click(lambda _: self._select_all_manipulators())
        self._handles["select_all_manipulators"] = select_all_button
        preset_dropdown = gui.add_dropdown(
            "Target Preset",
            options=["Select preset...", "Current"],
            initial_value="Select preset...",
        )
        preset_dropdown.on_update(lambda event: self._apply_preset(event.target.value))
        self._handles["preset"] = preset_dropdown
        plan_button = gui.add_button("Plan", disabled=True)
        plan_button.on_click(lambda _: self._submit_plan())
        self._handles["plan"] = plan_button
        preview_button = gui.add_button("Preview", disabled=True)
        preview_button.on_click(lambda _: self._submit_preview())
        self._handles["preview"] = preview_button
        execute_button = gui.add_button("Execute", disabled=True)
        execute_button.on_click(lambda _: self._submit_execute())
        self._handles["execute"] = execute_button
        cancel_button = gui.add_button("Cancel")
        cancel_button.on_click(lambda _: self._submit_cancel())
        self._handles["cancel"] = cancel_button
        clear_button = gui.add_button("Clear plan")
        clear_button.on_click(lambda _: self._submit_clear())
        self._handles["clear"] = clear_button
        self._build_joint_sliders()

    def _build_scene_controls(self, gui: GuiApi) -> None:
        if self.scene is None:
            return
        if not self.scene.has_reference_grid():
            return
        handle = gui.add_checkbox("Scene grid", initial_value=True)
        self._handles["scene_grid"] = handle
        handle.on_update(lambda event: self._set_scene_grid_visible(event.target.value))

    def _set_scene_grid_visible(self, visible: bool) -> None:
        if self._closed:
            return
        if self.scene is None:
            return
        self.scene.set_reference_grid_visible(bool(visible))

    def _refresh_selected_robot_state(self) -> None:
        robot_name = self.state.selected_robot
        if robot_name is None:
            self.state.robot_info = None
            self.state.current_joints = None
            self.state.current_ee_pose = None
            self.state.manipulation_state = self.adapter.get_module_state()
            return
        self.state.robot_info = self.adapter.get_robot_info(robot_name)
        current = self.adapter.get_current_joint_state(robot_name)
        self.state.current_joints = list(current.position) if current is not None else None
        self.state.current_ee_pose = self.adapter.get_ee_pose(robot_name)
        self.state.manipulation_state = self.adapter.get_module_state()
        adapter_error = self.adapter.get_error()
        if adapter_error:
            self.state.error = adapter_error

    def _ensure_scene_controls(self) -> None:
        if self.scene is None:
            return
        groups = self._group_info_by_id()
        selected = set(self.state.selected_group_ids)
        active_pose_groups = {
            group_id
            for group_id in selected
            if (group := groups.get(group_id)) is not None
            and bool(group["has_pose_target"])
            and group_id not in self.state.auxiliary_group_ids
        }
        for key in [key for key in self._handles if key.startswith("ee_control:")]:
            group_id = key.split(":", 1)[1]
            if group_id in active_pose_groups:
                continue
            handle = self._handles.pop(key)
            remove_target_controls = getattr(self.scene, "remove_target_controls", None)
            if callable(remove_target_controls):
                remove_target_controls(group_id)
            else:
                remove = getattr(handle, "remove", None)
                if callable(remove):
                    remove()
        for group_id in selected:
            group = groups.get(group_id)
            if group is None or not bool(group["has_pose_target"]):
                continue
            if group_id in self.state.auxiliary_group_ids:
                continue
            handle_key = f"ee_control:{group_id}"
            handle_exists = handle_key in self._handles
            ee_control = self.scene.ensure_target_controls(
                group_id,
                lambda target, gid=group_id: self._on_transform_update(gid, target),
            )
            if ee_control is not None:
                self._handles[handle_key] = ee_control
            if handle_exists:
                continue
            pose = self.state.pose_targets.get(group_id)
            if pose is not None:
                self._suppress_target_callbacks = True
                try:
                    self.scene.set_target_pose(group_id, pose)
                finally:
                    self._suppress_target_callbacks = False

    def _build_joint_sliders(self) -> None:
        if not self.state.selected_group_ids:
            return
        gui = self.server.gui
        self._clear_joint_sliders()
        groups = self._group_info_by_id()
        target_by_name: dict[str, float] = {}
        if self.state.target_joints is not None:
            target_by_name.update(
                zip(self.state.target_joints.name, self.state.target_joints.position, strict=False)
            )
        for group_id in self.state.selected_group_ids:
            group = groups.get(group_id)
            if group is None:
                continue
            config = self.adapter.get_robot_config(str(group["robot_name"]))
            current = self.adapter.get_current_joint_state(str(group["robot_name"]))
            current_by_name = (
                dict(zip(current.name, current.position, strict=False))
                if current is not None
                else {}
            )
            joint_limits_lower = config.joint_limits_lower if config is not None else None
            joint_limits_upper = config.joint_limits_upper if config is not None else None
            for index, (global_name, local_name) in enumerate(
                zip(group["joint_names"], group["local_joint_names"], strict=False)
            ):
                joint_name = str(global_name)
                local = str(local_name)
                value = float(target_by_name.get(joint_name, current_by_name.get(local, 0.0)))
                lower, upper = DEFAULT_JOINT_LIMITS
                if joint_limits_lower is not None and index < len(joint_limits_lower):
                    lower = joint_limits_lower[index]
                if joint_limits_upper is not None and index < len(joint_limits_upper):
                    upper = joint_limits_upper[index]
                handle = gui.add_slider(
                    f"{group_id}/{local}",
                    min=float(lower),
                    max=float(upper),
                    step=0.001,
                    initial_value=value,
                )

                def on_update(_event: object, name: str = joint_name) -> None:
                    self._on_joint_slider_update(name)

                handle.on_update(on_update)
                self._joint_sliders[joint_name] = handle

    def _target_set_from_sliders(self) -> dict[PlanningGroupID, JointState] | None:
        groups = self._group_info_by_id()
        targets: dict[PlanningGroupID, JointState] = {}
        for group_id in self.state.selected_group_ids:
            group = groups.get(group_id)
            if group is None:
                self._set_error(f"Unknown planning group: {group_id}")
                return None
            names = [str(name) for name in group["joint_names"]]
            positions: list[float] = []
            for name in names:
                handle = self._joint_sliders.get(name)
                if handle is None:
                    self._set_error(f"Missing target slider for {name}")
                    return None
                positions.append(float(handle.value))
            targets[group_id] = JointState({"name": names, "position": positions})
        return targets

    def _split_target_joints_by_group(self, target_joints: JointState) -> None:
        groups = self._group_info_by_id()
        positions_by_name = dict(zip(target_joints.name, target_joints.position, strict=False))
        self.state.group_joint_targets.clear()
        for group_id in self.state.selected_group_ids:
            group = groups.get(group_id)
            if group is None:
                continue
            names = [str(name) for name in group["joint_names"]]
            if not all(name in positions_by_name for name in names):
                continue
            self.state.group_joint_targets[group_id] = JointState(
                {"name": names, "position": [float(positions_by_name[name]) for name in names]}
            )

    def _clear_joint_sliders(self) -> None:
        for handle in self._joint_sliders.values():
            try:
                handle.remove()
            except AttributeError:
                pass
        self._joint_sliders.clear()

    def _remove_panel_handles(self) -> None:
        for key, handle in list(self._handles.items()):
            remove = getattr(handle, "remove", None)
            if callable(remove):
                remove()
            self._handles.pop(key, None)

    def _select_robot(self, robot_name: str) -> None:
        if self._closed:
            return
        if (robot_name or None) == self.state.selected_robot:
            self.refresh()
            return
        self.state.selected_robot = robot_name or None
        groups = [
            group
            for group in self.adapter.list_planning_groups()
            if str(group["robot_name"]) == self.state.selected_robot
        ]
        self.state.selected_group_ids = (str(groups[0]["id"]),) if groups else ()
        self.state.auxiliary_group_ids = ()
        self.state.group_joint_targets.clear()
        self.state.pose_targets.clear()
        self.state.target_status = TargetStatus.EMPTY
        self.state.feasibility.status = FeasibilityStatus.UNKNOWN
        self.state.plan_state = PanelPlanState()
        self._initialize_selected_group_targets()
        self._build_joint_sliders()
        self._sync_preset_dropdown()
        self.refresh()

    def _sync_robot_dropdown(self, robots: list[str]) -> None:
        handle = self._handles.get("robot")
        if handle is None:
            return
        options = robots or [""]
        for attr in ("options", "values"):
            if hasattr(handle, attr):
                try:
                    self._set_optional_handle_attr(handle, attr, options)
                except Exception:
                    logger.warning("Could not set robot dropdown %s", attr, exc_info=True)
        if hasattr(handle, "value") and self.state.selected_robot in robots:
            try:
                self._set_optional_handle_attr(handle, "value", self.state.selected_robot)
            except Exception:
                logger.warning("Could not set robot dropdown value", exc_info=True)

    def _sync_group_checkboxes(self, groups: list[PlanningGroupInfo]) -> None:
        seen_keys: set[str] = set()
        selected = set(self.state.selected_group_ids)
        for group in groups:
            group_id = str(group["id"])
            key = f"group:{group_id}"
            seen_keys.add(key)
            handle = self._handles.get(key)
            if handle is None:
                handle = self.server.gui.add_checkbox(
                    f"Group {group_id}", initial_value=group_id in selected
                )
                handle.on_update(
                    lambda event, gid=group_id: self._set_group_selected(
                        gid, bool(event.target.value)
                    )
                )
                self._handles[key] = handle
            elif hasattr(handle, "value"):
                self._set_optional_handle_attr(handle, "value", group_id in selected)

        for key in [key for key in self._handles if key.startswith("group:")]:
            if key not in seen_keys:
                handle = self._handles.pop(key)
                remove = getattr(handle, "remove", None)
                if callable(remove):
                    remove()

    def _set_group_selected(self, group_id: PlanningGroupID, selected: bool) -> None:
        current = list(self.state.selected_group_ids)
        if selected and group_id not in current:
            current.append(group_id)
        elif not selected and group_id in current:
            current.remove(group_id)
        self.state.selected_group_ids = tuple(current)
        self.state.auxiliary_group_ids = tuple(
            group_id for group_id in self.state.auxiliary_group_ids if group_id in current
        )
        self._sync_selected_robot_from_groups()
        self._initialize_selected_group_targets()
        self.state.mark_plan_stale()
        self._build_joint_sliders()
        self.refresh()

    def _select_all_manipulators(self) -> None:
        groups = self.adapter.list_planning_groups()
        manipulator_groups = [
            str(group["id"]) for group in groups if str(group["name"]) == "manipulator"
        ]
        self.state.selected_group_ids = tuple(
            manipulator_groups or [str(group["id"]) for group in groups]
        )
        self._sync_selected_robot_from_groups()
        self._initialize_selected_group_targets()
        self._build_joint_sliders()
        self.refresh()

    def _group_info_by_id(self) -> dict[PlanningGroupID, PlanningGroupInfo]:
        return {str(group["id"]): group for group in self.adapter.list_planning_groups()}

    def _sync_selected_robot_from_groups(self) -> None:
        groups = self._group_info_by_id()
        first_group = (
            groups.get(self.state.selected_group_ids[0]) if self.state.selected_group_ids else None
        )
        self.state.selected_robot = None if first_group is None else str(first_group["robot_name"])

    def _initialize_selected_group_targets(self) -> None:
        groups = self._group_info_by_id()
        for group_id in self.state.selected_group_ids:
            if group_id in self.state.group_joint_targets:
                continue
            group = groups.get(group_id)
            if group is None:
                continue
            current = self.adapter.get_current_joint_state(str(group["robot_name"]))
            if current is None:
                continue
            current_by_name = dict(zip(current.name, current.position, strict=False))
            names = [str(name) for name in group["joint_names"]]
            local_names = [str(name) for name in group["local_joint_names"]]
            positions = [float(current_by_name.get(local, 0.0)) for local in local_names]
            self.state.group_joint_targets[group_id] = JointState(
                {"name": names, "position": positions}
            )
            if bool(group["has_pose_target"]) and group_id not in self.state.pose_targets:
                pose = self.adapter.get_ee_pose(str(group["robot_name"]))
                if pose is not None:
                    self.state.pose_targets[group_id] = pose
                    self.state.group_poses[group_id] = pose
                    if self.state.cartesian_target is None:
                        self.state.cartesian_target = pose
        self._refresh_target_joints_from_groups()

    def _refresh_target_joints_from_groups(self) -> None:
        names: list[str] = []
        positions: list[float] = []
        for group_id in self.state.selected_group_ids:
            target = self.state.group_joint_targets.get(group_id)
            if target is None:
                continue
            names.extend(target.name)
            positions.extend(target.position)
        self.state.target_joints = (
            JointState({"name": names, "position": positions}) if names else None
        )

    def _current_snapshot_by_group(self) -> dict[PlanningGroupID, list[float]]:
        groups = self._group_info_by_id()
        snapshot: dict[PlanningGroupID, list[float]] = {}
        for group_id in self.state.selected_group_ids:
            group = groups.get(group_id)
            if group is None:
                continue
            current = self.adapter.get_current_joint_state(str(group["robot_name"]))
            if current is None:
                continue
            current_by_name = dict(zip(current.name, current.position, strict=False))
            snapshot[group_id] = [
                float(current_by_name.get(str(local_name), 0.0))
                for local_name in group["local_joint_names"]
            ]
        return snapshot

    def _sync_preset_dropdown(self) -> None:
        handle = self._handles.get("preset")
        if handle is None or self.state.selected_robot is None:
            return
        info: RobotInfo | None = self.adapter.get_robot_info(self.state.selected_robot)
        config = self.adapter.get_robot_config(self.state.selected_robot)
        options = ["Select preset..."]
        if (info is not None and info["init_joints"] is not None) or self.adapter.get_init_joints(
            self.state.selected_robot
        ) is not None:
            options.append("Init")
        options.append("Current")
        home_joints = config.home_joints if config is not None else None
        if (info is not None and info["home_joints"] is not None) or home_joints is not None:
            options.append("Home")
        for attr in ("options", "values"):
            if hasattr(handle, attr):
                try:
                    self._set_optional_handle_attr(handle, attr, options)
                except Exception:
                    logger.warning("Could not set preset dropdown %s", attr, exc_info=True)

    def _apply_preset(self, preset: str) -> None:
        if self._closed:
            return
        robot_name = self.state.selected_robot
        if robot_name is None:
            return
        config = self.adapter.get_robot_config(robot_name)
        if config is None:
            return
        if preset == "Current":
            current = self.adapter.get_current_joint_state(robot_name)
            values_by_name = (
                dict(zip(current.name, current.position, strict=False))
                if current is not None
                else {}
            )
        elif preset == "Init":
            init = self.adapter.get_init_joints(robot_name)
            values_by_name = (
                dict(zip(init.name, init.position, strict=False)) if init is not None else {}
            )
        elif preset == "Home":
            values_by_name = dict(zip(config.joint_names, config.home_joints or [], strict=False))
        else:
            return
        groups = [
            group
            for group in self.adapter.list_planning_groups()
            if group["id"] in self.state.selected_group_ids
            and str(group["robot_name"]) == robot_name
        ]
        for group in groups:
            global_names = [str(name) for name in group["joint_names"]]
            local_names = [str(name) for name in group["local_joint_names"]]
            values = [
                float(values_by_name.get(local_name, values_by_name.get(global_name, 0.0)))
                for local_name, global_name in zip(local_names, global_names, strict=False)
            ]
            self._set_slider_values(global_names, values)
        self.state.joint_target = [float(handle.value) for handle in self._joint_sliders.values()]
        self._submit_joint_target_evaluation()
        self.refresh()

    def _set_slider_values(self, joint_names: list[str], values: list[float]) -> None:
        self._suppress_target_callbacks = True
        try:
            for joint_name, value in zip(joint_names, values, strict=False):
                handle = self._joint_sliders.get(joint_name)
                if handle is not None:
                    handle.value = float(value)
        finally:
            self._suppress_target_callbacks = False

    def _target_from_sliders(self, robot_name: str) -> JointState | None:
        config = self.adapter.get_robot_config(robot_name)
        if config is None:
            self._set_error("No robot config")
            return None
        values = [
            float(self._joint_sliders[name].value)
            for name in config.joint_names
            if name in self._joint_sliders
        ]
        return self.adapter.joints_from_values(config.joint_names, values)

    def _on_joint_slider_update(self, _joint_name: str) -> None:
        if self._closed:
            return
        if self._suppress_target_callbacks:
            return
        self._submit_joint_target_evaluation()

    def _on_transform_update(
        self, group_id: PlanningGroupID, target: TransformControlsHandle
    ) -> None:
        if self._closed:
            return
        if self._suppress_target_callbacks or group_id not in self.state.selected_group_ids:
            return
        pose = self._pose_from_transform_target(target)
        if pose is None:
            return
        self.state.cartesian_target = pose
        self.state.pose_targets[group_id] = pose
        sequence_id = self.state.next_sequence_id()
        self._worker.submit(
            TargetEvaluationRequest(
                sequence_id=sequence_id,
                source="cartesian",
                group_ids=self.state.selected_group_ids,
                auxiliary_group_ids=self.state.auxiliary_group_ids,
                pose_targets=dict(self.state.pose_targets),
                check_collision=self.config.target_evaluation_check_collision,
            )
        )
        self.refresh()

    def _submit_joint_target_evaluation(self) -> None:
        targets = self._target_set_from_sliders()
        if targets is None:
            return
        self.state.group_joint_targets = targets
        self._refresh_target_joints_from_groups()
        self._move_joint_target_visuals()
        sequence_id = self.state.next_sequence_id()
        self._worker.submit(
            TargetEvaluationRequest(
                sequence_id=sequence_id,
                source="joints",
                group_ids=self.state.selected_group_ids,
                joint_targets=dict(targets),
            )
        )
        self.refresh()

    def _move_joint_target_visuals(self) -> None:
        """Optimistically move target visuals before collision/feasibility returns."""
        if self.scene is None:
            return
        groups = self._group_info_by_id()
        for group_id, target in self.state.group_joint_targets.items():
            group = groups.get(group_id)
            if group is None:
                continue
            robot_name = str(group["robot_name"])
            robot_id = self.adapter.robot_id_for_name(robot_name)
            config = self.adapter.get_robot_config(robot_name)
            if robot_id is None or config is None:
                continue
            local_positions = dict(zip(target.name, target.position, strict=False))
            joints = [
                float(local_positions.get(str(global_name), 0.0))
                for global_name in group["joint_names"]
            ]
            self.scene.set_target_joints(str(robot_id), group["local_joint_names"], joints)

    def _handle_target_evaluation_request(
        self, request: TargetEvaluationRequest
    ) -> TargetEvaluation | TargetSetEvaluation:
        if request.source == "cartesian":
            if not request.pose_targets:
                return {"success": False, "status": "INVALID", "message": "No pose target"}
            return self.adapter.evaluate_pose_target_set(
                request.pose_targets,
                auxiliary_groups=request.auxiliary_group_ids,
                seed=self.state.last_valid_target_joints,
                check_collision=request.check_collision,
            )
        if not request.joint_targets:
            return {"success": False, "status": "INVALID", "message": "No joint target"}
        return self.adapter.evaluate_joint_target_set(request.joint_targets)

    def _apply_target_evaluation_result(
        self, request: TargetEvaluationRequest, result: TargetEvaluation | TargetSetEvaluation
    ) -> None:
        if self._closed:
            return
        if request.sequence_id != self.state.latest_sequence_id:
            return
        collision_free = bool(result.get("collision_free", False))
        success = bool(result.get("success", False))
        self.state.feasibility.status = self._feasibility_status(result, success, collision_free)
        self.state.feasibility.message = str(result.get("message", ""))
        self.state.target_status = (
            TargetStatus.FEASIBLE if success and collision_free else TargetStatus.INFEASIBLE
        )
        self.state.error = "" if success and collision_free else self.state.feasibility.message
        target_joints = result.get("target_joints") or result.get("joint_state")
        if isinstance(target_joints, JointState) and success and collision_free:
            self.state.target_joints = JointState(target_joints)
            self.state.last_valid_target_joints = JointState(target_joints)
            self._split_target_joints_by_group(target_joints)
        group_poses = result.get("group_poses", {})
        if isinstance(group_poses, dict):
            self.state.group_poses = {
                str(group_id): pose
                for group_id, pose in group_poses.items()
                if isinstance(pose, Pose)
            }
        if request.source == "joints" and success and collision_free:
            self._sync_pose_targets_from_group_poses()
        group_diagnostics = result.get("group_diagnostics", {})
        if isinstance(group_diagnostics, dict):
            self.state.group_diagnostics = {
                str(group_id): str(message) for group_id, message in group_diagnostics.items()
            }
        if request.source == "cartesian" and success and collision_free:
            self._sync_controls_from_targets()
        self._update_target_visual_state()
        self.refresh()

    def _sync_controls_from_targets(self) -> None:
        if self.state.target_joints is not None:
            positions_by_name = dict(
                zip(self.state.target_joints.name, self.state.target_joints.position, strict=False)
            )
            self._set_slider_values(list(positions_by_name), list(positions_by_name.values()))
            self._move_joint_target_visuals()
        # Do not write the Cartesian target back into the active transform
        # control here. The gizmo is the source of truth for Cartesian edits;
        # programmatic pose writes from delayed IK results can fight fast user
        # dragging and make the gizmo jump back.

    def _sync_pose_targets_from_group_poses(self) -> None:
        groups = self._group_info_by_id()
        updated_group_ids: list[PlanningGroupID] = []
        for group_id, pose in self.state.group_poses.items():
            group = groups.get(group_id)
            if group is None or not bool(group["has_pose_target"]):
                continue
            if group_id in self.state.auxiliary_group_ids:
                continue
            self.state.pose_targets[group_id] = pose
            updated_group_ids.append(group_id)
        first_group_id = next(iter(self.state.selected_group_ids), None)
        if first_group_id is not None:
            self.state.cartesian_target = self.state.pose_targets.get(first_group_id)
        self._sync_scene_target_pose_controls(updated_group_ids)

    def _sync_scene_target_pose_controls(self, group_ids: list[PlanningGroupID]) -> None:
        if self.scene is None:
            return
        self._suppress_target_callbacks = True
        try:
            for group_id in group_ids:
                pose = self.state.pose_targets.get(group_id)
                if pose is not None:
                    self.scene.set_target_pose(group_id, pose)
        finally:
            self._suppress_target_callbacks = False

    def _update_status_text(self) -> None:
        current = self.state.current_joints
        status = [
            "### Manipulation Panel",
            f"Robot: `{self.state.selected_robot or 'none'}`",
            f"Module: `{self.state.module_state}`",
            f"Backend: `{self.state.backend_status.value}`",
            f"Target: `{self.state.target_status.value}`",
            f"Feasibility: `{self.state.feasibility.status.value}`",
            f"Plan: `{self.state.plan_state.status.value}`",
            f"Action: `{self.state.action_status.value}`",
        ]
        if self.state.selected_robot is not None:
            status.append(
                f"State stale: `{self.adapter.is_state_stale(self.state.selected_robot)}`"
            )
        if current is not None:
            status.append(f"Current joints: `{[round(v, 3) for v in current]}`")
        if self.state.last_result:
            status.append(f"Last result: `{self.state.last_result}`")
        if self.state.error:
            status.append(f"Error: `{self.state.error}`")
        self._set_handle_value("status", "\n\n".join(status))

    def _update_control_state(self) -> None:
        self._set_disabled("plan", not self.state.can_plan())
        self._set_disabled("preview", not self.state.can_preview())
        self._set_disabled(
            "execute",
            not (
                self.config.allow_plan_execute
                and self.state.can_execute(self.config.current_match_tolerance)
            ),
        )
        self._set_disabled("cancel", not self.state.can_cancel())
        self._update_target_visual_state()

    def _update_target_visual_state(self) -> None:
        if self.scene is None:
            return
        for group_id in self.state.selected_group_ids:
            self.scene.set_target_visual_state(
                group_id, self.state.feasibility.status == FeasibilityStatus.FEASIBLE
            )

    def _submit_plan(self) -> None:
        if self._closed:
            return
        if not self.state.selected_group_ids:
            self._set_recoverable_error(
                "Cannot plan until target is feasible and manipulation is idle"
            )
            return
        if not self.state.can_plan():
            self._set_recoverable_error(
                "Cannot plan until target is feasible and manipulation is idle"
            )
            return
        operation_id = self._next_operation_id()

        def operation() -> None:
            if not self._operation_is_current(operation_id):
                return
            self.state.action_status = ActionStatus.RUNNING
            self.state.plan_state.status = PlanStatus.PLANNING
            if self.state.manipulation_state == "FAULT" and not self.adapter.reset():
                self.state.plan_state.status = PlanStatus.FAILED
                self._finish_operation("reset=False", clear_error=False, operation_id=operation_id)
                return
            targets = self._target_set_from_sliders()
            if targets is None:
                self.state.plan_state.status = PlanStatus.FAILED
                self._finish_operation(
                    "plan_to_joints=False", clear_error=False, operation_id=operation_id
                )
                return
            ok = self.adapter.plan_target_set(targets)
            if not self._operation_is_current(operation_id):
                return
            if ok:
                self.state.plan_state.status = PlanStatus.FRESH
                self.state.plan_state.group_ids = self.state.selected_group_ids
                self.state.plan_state.robot = self.state.selected_robot
                self.state.plan_state.target_joints = list(
                    self.state.target_joints.position if self.state.target_joints else []
                )
                self.state.plan_state.target_pose = self.state.cartesian_target
                self.state.plan_state.start_joints_snapshot = self._current_snapshot_by_group()
                self.state.plan_state.planned_path = None
            else:
                self.state.plan_state.status = PlanStatus.FAILED
            self._finish_operation(f"plan_to_joints={ok}", operation_id=operation_id)

        self._operation_worker.submit(
            operation, on_error=lambda message: self._set_operation_error(message, operation_id)
        )

    def _submit_preview(self) -> None:
        if self._closed:
            return
        if not self.state.can_preview():
            self._set_recoverable_error("No fresh plan to preview")
            return
        operation_id = self._next_operation_id()

        def operation() -> None:
            if not self._operation_is_current(operation_id):
                return
            self.state.action_status = ActionStatus.PREVIEWING
            ok = self.adapter.preview_target_set_plan()
            self._finish_operation(f"preview={ok}", operation_id=operation_id)

        self._operation_worker.submit(
            operation,
            timeout_seconds=self.config.preview_request_timeout,
            on_error=lambda message: self._set_operation_error(message, operation_id),
        )

    def _submit_execute(self) -> None:
        if self._closed:
            return
        if not self.config.allow_plan_execute:
            self._set_recoverable_error(
                "Panel execution disabled; set allow_plan_execute=True to enable"
            )
            return
        if not self.state.can_execute(self.config.current_match_tolerance):
            self._set_recoverable_error(
                "Cannot execute: require feasible fresh plan and matching current joints"
            )
            return
        operation_id = self._next_operation_id()

        def operation() -> None:
            if not self._operation_is_current(operation_id):
                return
            self.state.action_status = ActionStatus.EXECUTING
            self.state.plan_state.status = PlanStatus.EXECUTING
            ok = self.adapter.execute_target_set_plan()
            if not self._operation_is_current(operation_id):
                return
            if not ok:
                self.state.plan_state.status = PlanStatus.FAILED
            self._finish_operation(f"execute={ok}", operation_id=operation_id)

        self._operation_worker.submit(
            operation, on_error=lambda message: self._set_operation_error(message, operation_id)
        )

    def _submit_cancel(self) -> None:
        if self._closed:
            return
        cancelled_action = self.state.action_status
        operation_id = self._next_operation_id()
        if not self._operation_is_current(operation_id):
            return
        self.state.action_status = ActionStatus.CANCELLING
        self._mark_cancelled_plan_state(cancelled_action)
        self._restart_operation_worker()
        try:
            ok = self.adapter.cancel()
        except Exception as e:
            self._set_operation_error(str(e), operation_id)
            return
        self._finish_operation(f"cancel={ok}", operation_id=operation_id)

    def _mark_cancelled_plan_state(self, cancelled_action: ActionStatus) -> None:
        if self.state.plan_state.status == PlanStatus.PLANNING:
            self.state.plan_state.status = PlanStatus.FAILED
        elif (
            cancelled_action == ActionStatus.EXECUTING
            or self.state.plan_state.status == PlanStatus.EXECUTING
        ):
            self.state.plan_state.status = PlanStatus.STALE

    def _restart_operation_worker(self) -> None:
        self._operation_worker.stop(timeout=0.0)
        self._operation_worker = OperationWorker(self._set_error)
        self._operation_worker.start()

    def _submit_clear(self) -> None:
        if self._closed:
            return
        operation_id = self._next_operation_id()

        def operation() -> None:
            if not self._operation_is_current(operation_id):
                return
            self.state.action_status = ActionStatus.CLEARING_PLAN
            ok = self.adapter.clear_planned_path()
            if not self._operation_is_current(operation_id):
                return
            self.state.plan_state = PanelPlanState()
            self._finish_operation(f"clear={ok}", operation_id=operation_id)

        self._operation_worker.submit(
            operation, on_error=lambda message: self._set_operation_error(message, operation_id)
        )

    def _next_operation_id(self) -> int:
        self._operation_sequence_id += 1
        return self._operation_sequence_id

    def _operation_is_current(self, operation_id: int) -> bool:
        return not self._closed and operation_id == self._operation_sequence_id

    def _finish_operation(
        self, result: str, *, clear_error: bool = True, operation_id: int | None = None
    ) -> None:
        if self._closed or (
            operation_id is not None and not self._operation_is_current(operation_id)
        ):
            return
        self.state.action_status = ActionStatus.IDLE
        if clear_error:
            self.state.error = ""
        self.state.last_result = result
        self.refresh()

    def _set_operation_error(self, message: str, operation_id: int) -> None:
        if self._operation_is_current(operation_id):
            self._operation_sequence_id += 1
            self._set_error(message)

    def _set_recoverable_error(self, message: str) -> None:
        if self._closed:
            return
        self.state.error = message
        self.refresh()

    def _set_error(self, message: str) -> None:
        if self._closed:
            return
        self.state.action_status = ActionStatus.FAILED
        self.state.error = message
        self.refresh()

    def _set_handle_value(self, key: str, value: str) -> None:
        handle = self._handles.get(key)
        if isinstance(handle, GuiMarkdownHandle):
            self._set_optional_handle_attr(handle, "value", value)

    def _set_disabled(self, key: str, disabled: bool) -> None:
        handle = self._handles.get(key)
        if isinstance(handle, GuiButtonHandle):
            self._set_optional_handle_attr(handle, "disabled", disabled)

    @staticmethod
    def _set_optional_handle_attr(handle: object, attr: str, value: object) -> None:
        setattr(handle, attr, value)

    def _pose_from_transform_target(self, target: TransformControlsHandle) -> Pose | None:
        px, py, pz = (float(value) for value in target.position)
        qw, qx, qy, qz = (float(value) for value in target.wxyz)
        return Pose({"position": [px, py, pz], "orientation": [qx, qy, qz, qw]})

    def _feasibility_status(
        self,
        result: TargetEvaluation | TargetSetEvaluation,
        success: bool,
        collision_free: bool,
    ) -> FeasibilityStatus:
        status = str(result.get("status", "")).upper()
        if success and collision_free:
            return FeasibilityStatus.FEASIBLE
        if status in {"COLLISION", "COLLISION_AT_START", "COLLISION_AT_GOAL"}:
            return FeasibilityStatus.COLLISION
        if status in {"NO_SOLUTION", "SINGULARITY", "JOINT_LIMITS", "TIMEOUT"}:
            return FeasibilityStatus.IK_FAILED
        return FeasibilityStatus.INVALID
