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

"""Drive the cross-wall route against an already-running ``babylon-nav`` daemon.

Assumes ``dimos --rerun-web run babylon-nav --daemon`` is up. Connects a
headless browser (so physics ticks), spawns the wall + goal, waits for the box
to route to the goal, then holds the sim open so the rerun web viewer can be
screenshotted.
"""

from __future__ import annotations

import math
import os
import sys
import time

import lcm as lcmlib

from dimos.e2e_tests.test_pimsim_cross_wall import (
    GOAL,
    GOAL_TOPIC,
    ROBOT_START,
    WORLD_FRAME,
    _spawn_wall_with_doorway,
)
from dimos.experimental.pimsim.client import PimSimClient
from dimos.experimental.pimsim.headless import HeadlessBrowser
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.protocol.service.lcmservice import _DEFAULT_LCM_URL

HOLD_SECONDS = 240.0


def main() -> int:
    robot = {"x": 0.0, "y": 0.0}

    def on_odom(_channel: str, data: bytes) -> None:
        msg = PoseStamped.lcm_decode(data)
        robot["x"], robot["y"] = msg.x, msg.y

    lcm = lcmlib.LCM(_DEFAULT_LCM_URL)
    lcm.subscribe("/odom#geometry_msgs.PoseStamped", on_odom)

    browser = HeadlessBrowser()
    client = PimSimClient()
    browser.start()
    client.start()
    client.set_agent_position(*ROBOT_START)
    time.sleep(2.0)
    _spawn_wall_with_doorway(client)
    time.sleep(2.0)

    goal = PointStamped(x=GOAL[0], y=GOAL[1], z=0.0, ts=time.time(), frame_id=WORLD_FRAME)
    lcm.publish(GOAL_TOPIC, goal.lcm_encode())
    print(f"[drive] goal {GOAL} published; routing...", flush=True)

    end = time.time() + 90
    while time.time() < end:
        lcm.handle_timeout(200)
        dist = math.hypot(robot["x"] - GOAL[0], robot["y"] - GOAL[1])
        if dist < 1.2:
            print(f"[drive] REACHED goal at ({robot['x']:.2f},{robot['y']:.2f})", flush=True)
            break

    print(f"[drive] holding sim open {HOLD_SECONDS:.0f}s for the rerun viewer", flush=True)
    hold_end = time.time() + HOLD_SECONDS
    while time.time() < hold_end:
        lcm.handle_timeout(200)
    browser.stop()
    client.stop()
    return 0


if __name__ == "__main__":
    code = main()
    sys.stdout.flush()
    os._exit(code)
