#!/usr/bin/env python3

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

"""AgiBot X2 Ultra basic stack with the Babylon viewer in place of Rerun."""

from pathlib import Path

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.transport import JpegShmTransport
from dimos.msgs.sensor_msgs.Image import Image
from dimos.robot.agibot.x2_ultra.connection import X2Connection
from dimos.visualization.babylon_scene_viewer import BabylonSceneViewerModule
from dimos.web.websocket_vis.websocket_vis_module import WebsocketVisModule

_X2_MJCF_PATH = Path(__file__).resolve().parent.parent.parent / "x2_ultra.xml"

agibot_x2_basic_babylon = (
    autoconnect(
        BabylonSceneViewerModule.blueprint(
            mjcf_path=str(_X2_MJCF_PATH),
            camera_name="rgbd_head_front",
        ),
        WebsocketVisModule.blueprint(),
        X2Connection.blueprint(
            clear_rmw_env=True,
            enable_lidar=True,
            force_cyclonedds=False,
        ),
    )
    .remappings(
        [
            # X2Connection publishes the head RGB on `color_image`; route it to
            # the viewer's generic `camera_image` input.
            (BabylonSceneViewerModule, "camera_image", "color_image"),
            (WebsocketVisModule, "tele_cmd_vel", "cmd_vel"),
        ]
    )
    .transports(
        {
            # JpegShmTransport: JPEG-compress on publish, shared-memory IPC
            # between workers (~150 KB per frame instead of 2.7 MB raw).
            ("color_image", Image): JpegShmTransport("/agibot_x2/color_image"),
        }
    )
    .global_config(n_workers=4, robot_model="agibot_x2_ultra")
)

__all__ = ["agibot_x2_basic_babylon"]
