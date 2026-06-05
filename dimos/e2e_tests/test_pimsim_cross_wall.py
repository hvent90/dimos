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

from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.experimental.pimsim.blueprints.babylon_nav import (
    build_babylon_nav,
    ensure_flat_floor_scene,
)
from dimos.experimental.pimsim.client import PimSimClient
from dimos.experimental.pimsim.headless import HeadlessBrowser
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.protocol.service.lcmservice import _DEFAULT_LCM_URL

pytestmark = [pytest.mark.self_hosted, pytest.mark.skipif_in_ci]

ODOM_TOPIC = "/odom#geometry_msgs.PoseStamped"
GOAL_TOPIC = "/clicked_point#geometry_msgs.PointStamped"
WORLD_FRAME = "map"

# Robot starts at the origin; a wall spans +y with the doorway pushed off to the
# right, so the only way to the goal at (0, 4) is to detour around through it.
ROBOT_START = (0.0, 0.0)
WALL_Y = 2.0
WALL_X_MIN = -4.0
WALL_X_MAX = 5.0
DOOR_X_MIN = 0.9
DOOR_X_MAX = 3.1
GOAL = (0.0, 4.0)
# The planner stops within ~0.6 m of the goal; the test asserts the box ended
# within the looser REACH_THRESHOLD_M so a clean stop counts as reached.
REACH_THRESHOLD_M = 1.2
WARMUP_SEC = 15.0
GOAL_TIMEOUT_SEC = 150.0


def _babylon_nav_blueprint():
    """pimsim sim (open floor) + odom adapter + nav stack, wired for routing."""
    return build_babylon_nav(ensure_flat_floor_scene())


def _spawn_wall_with_doorway(client: PimSimClient) -> None:
    client.add_wall(WALL_X_MIN, WALL_Y, DOOR_X_MIN, WALL_Y)
    client.add_wall(DOOR_X_MAX, WALL_Y, WALL_X_MAX, WALL_Y)


def test_pimsim_cross_wall() -> None:
    coordinator = ModuleCoordinator.build(_babylon_nav_blueprint())

    robot_x = robot_y = 0.0
    odom_seen = 0
    max_x = 0.0
    crossed_wall = False

    def _on_odom(_channel: str, data: bytes) -> None:
        nonlocal robot_x, robot_y, odom_seen, max_x, crossed_wall
        msg = PoseStamped.lcm_decode(data)
        robot_x, robot_y, odom_seen = msg.x, msg.y, odom_seen + 1
        max_x = max(max_x, msg.x)
        if msg.y >= WALL_Y:
            crossed_wall = True

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
            if ((robot_x - GOAL[0]) ** 2 + (robot_y - GOAL[1]) ** 2) ** 0.5 < REACH_THRESHOLD_M:
                reached = True
                break

        print(
            f"[cross_wall] final=({robot_x:.2f},{robot_y:.2f}) goal={GOAL} "
            f"max_x={max_x:.2f} crossed_wall={crossed_wall} reached={reached}"
        )
        assert crossed_wall, "box never crossed the wall plane (y >= WALL_Y)"
        assert reached, (
            f"box did not reach the goal past the wall: "
            f"final ({robot_x:.2f}, {robot_y:.2f}), goal {GOAL}"
        )
    finally:
        browser.stop()
        client.stop()
        coordinator.stop()
