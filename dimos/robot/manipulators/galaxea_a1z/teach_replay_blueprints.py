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

"""A1Z hand-teaching and measured-trajectory replay blueprint factories."""

from __future__ import annotations

from pathlib import Path

from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.learning.collection.episode_monitor import EpisodeMonitorModule
from dimos.learning.collection.recorder import CollectionRecorder
from dimos.memory2.module import OnExisting
from dimos.robot.manipulators.a1z.config import A1Z_G1Z_MODEL_PATH
from dimos.robot.manipulators.galaxea_a1z.config import galaxea_a1z_hardware

A1Z_REPLAY_TASK_NAME = "teach_replay_arm"


def make_a1z_teach_blueprint(
    db_path: Path,
    *,
    task_label: str | None = None,
) -> Blueprint:
    """Record measured arm/gripper state while both are hand-drivable."""
    hardware = galaxea_a1z_hardware(
        "arm",
        gripper=True,
        dynamics_urdf_path=str(A1Z_G1Z_MODEL_PATH),
        adapter_kwargs={
            "zero_gravity": True,
            "gripper_free_drive": True,
        },
    )
    return autoconnect(
        ControlCoordinator.blueprint(hardware=[hardware], tasks=[]),
        EpisodeMonitorModule.blueprint(default_task_label=task_label),
        CollectionRecorder.blueprint(
            db_path=db_path,
            on_existing=OnExisting.ERROR,
            root_frame="coordinator",
            default_frame_id="coordinator",
            record_tf=False,
        ),
    )


def make_a1z_replay_blueprint() -> Blueprint:
    """Run a validated seven-joint arm/gripper trajectory through the coordinator."""
    hardware = galaxea_a1z_hardware(
        "arm",
        gripper=True,
        dynamics_urdf_path=str(A1Z_G1Z_MODEL_PATH),
    )
    return autoconnect(
        ControlCoordinator.blueprint(
            hardware=[hardware],
            tasks=[
                TaskConfig(
                    name=A1Z_REPLAY_TASK_NAME,
                    type="trajectory",
                    joint_names=hardware.all_joints,
                    priority=10,
                )
            ],
        )
    )
