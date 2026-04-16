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

"""PCTPlanner NativeModule: C++ 3D point-cloud-tomography route planner.

Ported from PCT_planner (point cloud tomography + A* + GPMP). Slices an
explored-area point cloud into traversability layers, plans across floors
with A*, smooths the result with GPMP, and publishes lookahead waypoints
for the local planner.

Upstream requirement: PCTPlanner consumes the accumulated explored-areas
point cloud published by ``PreloadedMapTracker``. A blueprint that sets
``use_pct_planner=True`` on ``smart_nav(...)`` picks up PreloadedMapTracker
automatically.
"""

from __future__ import annotations

from pathlib import Path

from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.nav_msgs.Path import Path as NavPath
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


class PCTPlannerConfig(NativeModuleConfig):
    """Config for the PCT planner native module.

    Defaults mirror the upstream reference
    ``~/repos/ros-navigation-autonomy-stack/src/route_planner/PCT_planner/config/pct_planner_params.yaml``
    verbatim. Scene-specific tuning (e.g., wider safe_margin / inflation
    for the G1 in Unity sim) should be passed via the ``pct_planner``
    kwarg dict on ``smart_nav(...)``, not baked into the module defaults.
    """

    cwd: str | None = str(Path(__file__).resolve().parent)
    executable: str = "result/bin/pct_planner"
    build_command: str | None = (
        "nix build github:dimensionalOS/dimos-module-pct-planner/v0.1.2 --no-write-lock-file"
    )

    # Loop / frame
    update_rate: float = 5.0
    frame_id: str = "map"

    # Tomogram grid (upstream yaml values)
    resolution: float = 0.075
    slice_dh: float = 0.4
    slope_max: float = 0.45
    step_max: float = 0.5
    cost_barrier: float = 100.0
    kernel_size: int = 11
    safe_margin: float = 0.025
    inflation: float = 0.05
    interval_min: float = 0.3
    interval_free: float = 0.5
    standable_ratio: float = 0.02

    # Waypoint follower
    lookahead_distance: float = 1.25

    # Minimum seconds between tomogram rebuilds. PreloadedMapTracker
    # publishes at ~10 Hz; rebuilding the grid is ~1 s, so the default
    # of 1.0 caps the rebuild cost while still tracking exploration.
    # Set to 0.0 for strict upstream behavior (rebuild every iteration,
    # appropriate when the cloud is a small preloaded offline map).
    tomogram_rebuild_period_sec: float = 1.0

    # Superset of upstream: expand the tomogram grid to include at
    # least a square of this half-extent around the robot, even if the
    # observed cloud is smaller. 0.0 = strict upstream behavior (grid
    # sized to cloud bbox + 4 cells, appropriate for a preloaded
    # offline scene map that is always large enough to contain the
    # goal). Required for our live-growing cloud so A* has room to
    # route toward goals just outside the current scan.
    min_plan_half_extent_m: float = 15.0


class PCTPlanner(NativeModule):
    """PCT (Point Cloud Tomography) planner: 3D multi-floor global route planner.

    Rebuilds a tomogram every time it receives a new explored-areas point cloud
    and plans across floors with A* + GPMP. Publishes lookahead waypoints at
    ``update_rate`` for the local planner to follow.

    Requires ``PreloadedMapTracker`` upstream to publish the ``explored_areas``
    PointCloud2 stream that this module consumes.

    Ports:
        explored_areas (In[PointCloud2]): Accumulated mapped point cloud.
        odometry (In[Odometry]): Vehicle state.
        goal (In[PointStamped]): Navigation goal.
        way_point (Out[PointStamped]): Lookahead waypoint.
        goal_path (Out[NavPath]): Full planned path.
        tomogram (Out[PointCloud2]): Tomogram visualization.
    """

    config: PCTPlannerConfig

    explored_areas: In[PointCloud2]
    odometry: In[Odometry]
    goal: In[PointStamped]
    way_point: Out[PointStamped]
    goal_path: Out[NavPath]
    tomogram: Out[PointCloud2]
