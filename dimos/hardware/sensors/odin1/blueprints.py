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

"""Demo: visualize the Odin1's onboard point cloud, image, and odometry."""

from typing import Any

from dimos.core.coordination.blueprints import autoconnect
from dimos.hardware.sensors.odin1.module import Odin1
from dimos.visualization.vis_module import vis_module


def _cloud_1cm(pc: Any) -> Any:
    # voxel_size/2 is the sphere radius, so 0.01 -> ~1 cm spheres.
    return pc.to_rerun(voxel_size=0.01)


demo_odin1 = autoconnect(
    Odin1.blueprint(),
    vis_module(
        "rerun",
        rerun_config={
            "visual_override": {
                "world/lidar": _cloud_1cm,
                "world/slam_cloud": _cloud_1cm,
            }
        },
    ),
).global_config(n_workers=2, robot_model="odin1")
