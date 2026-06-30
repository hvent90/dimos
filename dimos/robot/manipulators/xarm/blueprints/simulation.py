# Copyright 2026 Dimensional Inc.
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

"""Simulation xArm perception manipulation blueprints."""

from __future__ import annotations

import math
from typing import Any

import rerun.blueprint as rrb

from dimos.core.coordination.blueprints import autoconnect
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.perception.object_scene_registration import ObjectSceneRegistrationModule
from dimos.robot.manipulators.common.blueprints import coordinator, trajectory_task
from dimos.robot.manipulators.xarm.config import (
    XARM7_SIM_PATH,
    make_xarm7_model_config,
    make_xarm_hardware,
)
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule
from dimos.visualization.rerun.bridge import RerunBridgeModule

XARM7_SIM_HOME = [0.0, 0.0, 0.0, 0.0, 0.0, -0.7, 0.0]


# Rerun data viz. The point cloud, color/depth images, annotated YOLO image and
# 3D detections all carry their own to_rerun(), so the bridge logs them with no
# help. The only override links the camera pinhole to the image entities so the
# frames render inside the 3D frustum (entity_prefix defaults to "world").
def _color_camera_info_to_rerun(msg: Any) -> list[tuple[str, Any]]:
    optical_frame = getattr(msg, "frame_id", None)
    return [
        *msg.to_rerun(image_topic="world/color_image", optical_frame=optical_frame),
        *msg.to_rerun(image_topic="world/annotated_image", optical_frame=optical_frame),
    ]


def _depth_camera_info_to_rerun(msg: Any) -> list[tuple[str, Any]]:
    optical_frame = getattr(msg, "frame_id", None)
    return msg.to_rerun(image_topic="world/depth_image", optical_frame=optical_frame)


def _rerun_layout() -> rrb.Blueprint:
    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial2DView(origin="world/annotated_image", name="Camera"),
            rrb.Spatial3DView(origin="world", name="3D"),
        )
    )


_xarm7_sim_hw = make_xarm_hardware(
    "arm",
    7,
    adapter_type="sim_mujoco",
    address=str(XARM7_SIM_PATH),
    gripper=True,
    home_joints=XARM7_SIM_HOME,
)

xarm_perception_sim = autoconnect(
    PickAndPlaceModule.blueprint(
        robots=[
            make_xarm7_model_config(
                name="arm",
                add_gripper=True,
                pitch=math.radians(45),
                tf_extra_links=["link7"],
                home_joints=XARM7_SIM_HOME,
                pre_grasp_offset=0.05,
            )
        ],
        planning_timeout=10.0,
        visualization={"backend": "meshcat"},
    ),
    MujocoSimModule.blueprint(
        address=str(XARM7_SIM_PATH),
        headless=False,
        dof=7,
        camera_name="wrist_camera",
        base_frame_id="link7",
    ),
    ObjectSceneRegistrationModule.blueprint(target_frame="world"),
    coordinator(
        hardware=[_xarm7_sim_hw],
        tasks=[trajectory_task(_xarm7_sim_hw)],
    ),
    # Data viz in Rerun, alongside the Meshcat arm view above (not instead of it).
    RerunBridgeModule.blueprint(
        blueprint=_rerun_layout,
        visual_override={
            "world/camera_info": _color_camera_info_to_rerun,
            "world/depth_camera_info": _depth_camera_info_to_rerun,
        },
        max_hz={
            "world/color_image": 10.0,
            "world/annotated_image": 10.0,
            "world/depth_image": 5.0,
            "world/pointcloud": 5.0,
        },
    ),
)
