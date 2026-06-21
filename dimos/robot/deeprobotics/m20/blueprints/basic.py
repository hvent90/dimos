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

"""Basic Lynx M20 blueprint: front-camera video + a Rerun viewer.

``MovementManager`` muxes movement sources (``nav_cmd_vel`` / ``tele_cmd_vel`` /
``clicked_point``) into the single ``cmd_vel`` the connection consumes; wire a
teleop or nav source into the manager's inputs to drive.
"""

from typing import Any

from dimos.core.coordination.blueprints import autoconnect
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.robot.deeprobotics.m20.connection import M20Connection
from dimos.robot.deeprobotics.m20.tf import M20TF
from dimos.visualization.rerun.bridge import RerunBridgeModule
from dimos.visualization.rerun.websocket_server import RerunWebSocketServer
from dimos.web.websocket_vis.websocket_vis_module import WebsocketVisModule


def m20_rerun_blueprint() -> Any:
    import rerun as rr
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Vertical(
                rrb.Spatial2DView(origin="world/color_image", name="M20 Front"),
                rrb.Spatial2DView(origin="world/color_image_rear", name="M20 Rear"),
            ),
            rrb.Spatial3DView(
                origin="world",
                name="3D",
                background=rrb.Background(kind="SolidColor", color=[0, 0, 0]),
                line_grid=rrb.LineGrid3D(
                    plane=rr.components.Plane3D.XY.with_distance(0.5),
                ),
            ),
            column_shares=[1, 2],
        ),
        rrb.TimePanel(state="hidden"),
        rrb.SelectionPanel(state="hidden"),
    )


rerun = autoconnect(
    RerunBridgeModule.blueprint(
        blueprint=m20_rerun_blueprint,
        max_hz={
            "world/color_image": 0,
            "world/color_image_rear": 0,
        },
    ),
    RerunWebSocketServer.blueprint(),
    WebsocketVisModule.blueprint(),
)


m20 = autoconnect(
    rerun,
    M20TF.blueprint(),
).global_config(n_workers=3)

m20_api = autoconnect(
    m20,
    M20Connection.blueprint(ip="m20"),
    MovementManager.blueprint(),
).global_config(n_workers=3)
