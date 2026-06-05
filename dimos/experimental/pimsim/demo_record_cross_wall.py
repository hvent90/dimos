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

"""Record the cross-wall nav route in the pimsim Babylon viewer.

Brings up the nav stack on the open-floor scene, drives the box around the
spawned wall to the goal, and records the browser (which is also the physics
client) to a video + periodic screenshots so a human can watch the route.

Run: ``DIMOS_PIMSIM_VISUAL=0 python -m dimos.experimental.pimsim.demo_record_cross_wall``
"""

from __future__ import annotations

import math
import os
from pathlib import Path
import sys
import time

import lcm as lcmlib
from playwright.sync_api import sync_playwright

from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.e2e_tests.test_pimsim_cross_wall import (
    DOOR_X_MAX,
    DOOR_X_MIN,
    GOAL,
    GOAL_TOPIC,
    ROBOT_START,
    WALL_X_MAX,
    WALL_X_MIN,
    WALL_Y,
    WORLD_FRAME,
    _babylon_nav_blueprint,
    _spawn_wall_with_doorway,
)
from dimos.experimental.pimsim.client import PimSimClient
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.protocol.service.lcmservice import _DEFAULT_LCM_URL

OUT_DIR = Path("/tmp/pimsim_cross_wall_recording")
VIEWER_URL = "http://localhost:8091/"
RECORD_SECONDS = 70.0


def _plot_route(track: list[tuple[float, float]]) -> None:
    """Plot the robot's odometry path over the wall, doorway, and goal."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(6, 7))
    ax.plot([WALL_X_MIN, DOOR_X_MIN], [WALL_Y, WALL_Y], color="firebrick", lw=8)
    ax.plot([DOOR_X_MAX, WALL_X_MAX], [WALL_Y, WALL_Y], color="firebrick", lw=8, label="wall")
    ax.plot(
        [DOOR_X_MIN, DOOR_X_MAX], [WALL_Y, WALL_Y], color="seagreen", lw=2, ls=":", label="doorway"
    )
    if track:
        xs, ys = zip(*track, strict=False)
        ax.plot(xs, ys, color="steelblue", lw=2, label="robot path")
    ax.scatter([ROBOT_START[0]], [ROBOT_START[1]], color="black", s=80, label="start", zorder=5)
    ax.scatter([GOAL[0]], [GOAL[1]], color="goldenrod", marker="*", s=260, label="goal", zorder=5)
    ax.set_aspect("equal")
    ax.set_xlim(WALL_X_MIN - 1, WALL_X_MAX + 1)
    ax.set_ylim(-1, GOAL[1] + 1.5)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("pimsim cross-wall route (nav stack, macOS)")
    ax.legend(loc="upper right")
    ax.grid(alpha=0.3)
    fig.savefig(str(OUT_DIR / "route_plot.png"), dpi=110, bbox_inches="tight")
    plt.close(fig)
    print(f"[record] route plot: {OUT_DIR / 'route_plot.png'}")


def main() -> int:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    robot = {"x": 0.0, "y": 0.0, "n": 0}
    track: list[tuple[float, float]] = []

    def on_odom(_channel: str, data: bytes) -> None:
        msg = PoseStamped.lcm_decode(data)
        robot["x"], robot["y"], robot["n"] = msg.x, msg.y, robot["n"] + 1
        track.append((msg.x, msg.y))

    lcm = lcmlib.LCM(_DEFAULT_LCM_URL)
    lcm.subscribe("/odom#geometry_msgs.PoseStamped", on_odom)

    coordinator = ModuleCoordinator.build(_babylon_nav_blueprint())
    client = PimSimClient()
    playwright = sync_playwright().start()
    browser = playwright.chromium.launch(
        headless=False,
        args=["--headless=new", "--no-sandbox", "--use-gl=angle", "--use-angle=swiftshader"],
    )
    context = browser.new_context(
        viewport={"width": 1280, "height": 720},
        record_video_dir=str(OUT_DIR),
        record_video_size={"width": 1280, "height": 720},
    )
    page = context.new_page()
    try:
        page.goto(VIEWER_URL, wait_until="load")
        page.wait_for_function("window.__pimsimReady === true", timeout=60000)
        client.start()
        client.set_agent_position(*ROBOT_START)
        time.sleep(2.0)
        _spawn_wall_with_doorway(client)
        time.sleep(2.0)

        # Frame a fixed, mostly top-down view of the whole route (start at y=0,
        # wall at y=2, goal at y=4) so the box's path is clearly visible.
        page.evaluate(
            "() => { const c = window.__pimsimCamera;"
            " c.setTarget(new BABYLON.Vector3(0, 2, 0));"
            " c.alpha = -Math.PI / 2; c.beta = 0.42; c.radius = 9.5; }"
        )

        goal = PointStamped(x=GOAL[0], y=GOAL[1], z=0.0, ts=time.time(), frame_id=WORLD_FRAME)
        lcm.publish(GOAL_TOPIC, goal.lcm_encode())
        print(f"[record] goal published at {GOAL}; recording ~{RECORD_SECONDS:.0f}s")

        deadline = time.time() + RECORD_SECONDS
        shot = 0
        while time.time() < deadline:
            lcm.handle_timeout(200)
            if shot % 10 == 0:
                page.screenshot(path=str(OUT_DIR / f"frame_{shot // 10:03d}.png"))
                dist = math.hypot(robot["x"] - GOAL[0], robot["y"] - GOAL[1])
                print(
                    f"[record] t={shot // 5}s odom=({robot['x']:.2f},{robot['y']:.2f}) dist={dist:.2f}"
                )
                if dist < 1.2:
                    page.screenshot(path=str(OUT_DIR / "frame_final.png"))
                    break
            shot += 1
            time.sleep(0.2)

        video_path = page.video.path() if page.video else None
        page.close()
        context.close()
        if video_path:
            print(f"[record] video: {video_path}")
        _plot_route(track)
        print(f"[record] artifacts in: {OUT_DIR}")
    finally:
        browser.close()
        playwright.stop()
        client.stop()
        coordinator.stop()
    return 0


if __name__ == "__main__":
    code = main()
    sys.stdout.flush()
    os._exit(code)
