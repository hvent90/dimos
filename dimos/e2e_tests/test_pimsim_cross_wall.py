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

"""Cross-wall routing on the pimsim (Babylon) simulator.

The pimsim analogue of ``test_cross_wall_planning_simple`` / the dimsim
``test_dimsim_path_replaning``: wire the nav stack onto the browser-physics box
sim, drop a wall (with a doorway) between the robot and a goal, publish the
goal, and assert the planner routes through the gap and the box reaches it.

Wiring (everything else is autoconnected by stream name):
- ``BabylonSceneViewerModule`` integrates ``/nav_cmd_vel`` and publishes
  ``/odom`` (PoseStamped); the rust ``SceneLidarModule`` publishes ``/lidar``.
- ``PoseStampedToOdometry`` republishes ``/odom`` -> ``/odometry`` (Odometry),
  which ``TerrainAnalysis`` + the planners consume.
- ``/lidar`` is bridged to the nav stack's ``registered_scan`` input.
- the goal is a ``PointStamped`` on ``/clicked_point``.

Platform: the nav-stack planners (``TerrainAnalysis``, ``LocalPlanner``) are
Nix-built native binaries, so like the other cross-wall tests this is
``skipif_macos`` + ``self_hosted`` — it runs on a Linux runner, not macOS.
"""

from __future__ import annotations

import time

import lcm as lcmlib
import pytest

pytest.importorskip("gtsam")

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.transport import LCMTransport
from dimos.experimental.pimsim.blueprints.babylon_smoketest import babylon_smoketest
from dimos.experimental.pimsim.client import PimSimClient
from dimos.experimental.pimsim.headless import HeadlessBrowser
from dimos.experimental.pimsim.odometry_adapter import PoseStampedToOdometry
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.nav_stack.main import create_nav_stack
from dimos.protocol.service.lcmservice import _DEFAULT_LCM_URL

pytestmark = [pytest.mark.self_hosted, pytest.mark.skipif_in_ci, pytest.mark.skipif_macos]

ODOM_TOPIC = "/odom#geometry_msgs.PoseStamped"
GOAL_TOPIC = "/clicked_point#geometry_msgs.PointStamped"
WORLD_FRAME = "map"

# Robot starts at the origin; the wall sits ahead in +y with a doorway gap, and
# the goal is on the far side, so the only route is through the gap.
ROBOT_START = (0.0, 0.0)
WALL_Y = 2.0
WALL_SPAN = 3.0
DOOR_HALF_WIDTH = 0.6
GOAL = (0.0, 4.0)
GOAL_THRESHOLD_M = 1.0
WARMUP_SEC = 15.0
GOAL_TIMEOUT_SEC = 120.0


def _babylon_nav_blueprint():
    """pimsim sim + odom adapter + nav stack, wired for cross-wall routing."""
    odom_adapter = PoseStampedToOdometry.blueprint(world_frame=WORLD_FRAME).transports(
        {
            ("pose", PoseStamped): LCMTransport("/odom", PoseStamped),
            ("odometry", Odometry): LCMTransport("/odometry", Odometry),
        }
    )
    nav_stack = create_nav_stack(
        planner="simple",
        vehicle_height=0.40,
        max_speed=0.8,
        waypoint_threshold=GOAL_THRESHOLD_M,
    ).transports(
        {
            # The pimsim lidar already publishes a world-frame cloud, so it is
            # the nav stack's registered scan.
            ("registered_scan", PointCloud2): LCMTransport("/lidar", PointCloud2),
        }
    )
    return autoconnect(babylon_smoketest, odom_adapter, nav_stack).global_config(simulation=True)


def _spawn_wall_with_doorway(client: PimSimClient) -> None:
    left_end = -WALL_SPAN
    right_end = WALL_SPAN
    client.add_wall(left_end, WALL_Y, -DOOR_HALF_WIDTH, WALL_Y)
    client.add_wall(DOOR_HALF_WIDTH, WALL_Y, right_end, WALL_Y)


def test_pimsim_cross_wall() -> None:
    coordinator = ModuleCoordinator.build(_babylon_nav_blueprint())

    robot_x = robot_y = 0.0
    odom_seen = 0

    def _on_odom(_channel: str, data: bytes) -> None:
        nonlocal robot_x, robot_y, odom_seen
        msg = PoseStamped.lcm_decode(data)
        robot_x, robot_y, odom_seen = msg.x, msg.y, odom_seen + 1

    lcm = lcmlib.LCM(_DEFAULT_LCM_URL)
    lcm.subscribe(ODOM_TOPIC, _on_odom)

    browser = HeadlessBrowser()
    client = PimSimClient()
    try:
        browser.start()
        client.start()
        client.set_agent_position(*ROBOT_START)

        deadline = time.monotonic() + 60.0
        while time.monotonic() < deadline and odom_seen == 0:
            lcm.handle_timeout(200)
        assert odom_seen > 0, "no /odom from pimsim — sim not running"

        _spawn_wall_with_doorway(client)
        warmup_end = time.monotonic() + WARMUP_SEC
        while time.monotonic() < warmup_end:
            lcm.handle_timeout(200)

        goal = PointStamped(x=GOAL[0], y=GOAL[1], z=0.0, ts=time.time(), frame_id=WORLD_FRAME)
        lcm.publish(GOAL_TOPIC, goal.lcm_encode())

        reached = False
        goal_end = time.monotonic() + GOAL_TIMEOUT_SEC
        while time.monotonic() < goal_end:
            lcm.handle_timeout(200)
            if ((robot_x - GOAL[0]) ** 2 + (robot_y - GOAL[1]) ** 2) ** 0.5 < GOAL_THRESHOLD_M:
                reached = True
                break

        assert reached, (
            f"box did not reach the far side of the wall: "
            f"final ({robot_x:.2f}, {robot_y:.2f}), goal {GOAL}"
        )
    finally:
        browser.stop()
        client.stop()
        coordinator.stop()
