#!/usr/bin/env python3
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

"""Go2 + DimSim blueprint — browser-based sim with nav stack.

Drop-in replacement for unitree_go2 that uses DimSim instead of
hardware or MuJoCo. Works on macOS and ARM Linux.
"""

from typing import Any

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.mapping.costmapper import CostMapper
from dimos.mapping.voxels import VoxelGridMapper
from dimos.navigation.replanning_a_star.module import ReplanningAStarPlanner
from dimos.robot.sim.bridge import DimSimBridge
from dimos.robot.sim.jpeg_lcm import SimJpegLCM
from dimos.visualization.vis_module import vis_module


def _go2_sim_rerun_blueprint() -> Any:
    import rerun as rr
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial2DView(origin="camera/color_image", name="Camera"),
            rrb.Spatial3DView(
                origin="world",
                name="3D",
                background=rrb.Background(kind="SolidColor", color=[0, 0, 0]),
                line_grid=rrb.LineGrid3D(
                    plane=rr.components.Plane3D.XY.with_distance(0.2),
                ),
            ),
            column_shares=[1, 2],
        ),
    )


def _convert_color_image(image: Any) -> Any:
    rerun_data = image.to_rerun()
    return [
        ("camera/color_image", rerun_data),
        ("world/tf/camera_optical/image", rerun_data),
    ]


def _convert_navigation_costmap(grid: Any) -> Any:
    return grid.to_rerun(
        colormap="Accent",
        z_offset=0.015,
        opacity=0.2,
        background="#484981",
    )


def _static_base_link(rr: Any) -> list[Any]:
    return [
        rr.Boxes3D(
            half_sizes=[0.35, 0.155, 0.2],
            colors=[(0, 255, 127)],
        ),
        rr.Transform3D(parent_frame="tf#/base_link"),
    ]


rerun_config = {
    "blueprint": _go2_sim_rerun_blueprint,
    "pubsubs": [SimJpegLCM()],
    "visual_override": {
        "world/camera_info": DimSimBridge.rerun_suppress_camera_info,
        "world/color_image": _convert_color_image,
        "world/navigation_costmap": _convert_navigation_costmap,
    },
    "static": {
        "world/tf/base_link": _static_base_link,
        "world/tf/camera_optical": DimSimBridge.rerun_static_pinhole,
    },
}

unitree_go2_dimsim = (
    autoconnect(
        vis_module(
            viewer_backend=global_config.viewer,
            rerun_config=rerun_config,
        ),
        DimSimBridge.blueprint(
            scene="apt",
            vehicle_height=0.3,
        ),
        VoxelGridMapper.blueprint(voxel_size=0.1),
        CostMapper.blueprint(),
        ReplanningAStarPlanner.blueprint(),
    )
    .global_config(n_workers=6, robot_model="unitree_go2", simulation=True)
)

__all__ = ["unitree_go2_dimsim"]
