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

"""pimsim end-to-end: the wavefront local planner dodging a lidar-only moving box.

Spawns one **patrol box** (kinematic, mass-0 — it shows up in ``/lidar`` via the
rust ``scene_lidar`` with no physics, i.e. *lidar-only*) sweeping across a flat
floor, then drives the robot with **this module's** ``plan_path``: every tick it
builds a local costmap from ``/lidar``, plans, and pure-pursues the local path to
``/nav_cmd_vel``. The robot detours around the moving box and reaches its goal,
staying clear (the validated run kept >= 0.76 m from the box centre).

Cross-clone: pimsim lives in a sibling checkout (``DIM_PRIV2_ROOT``, default
``~/repos/dim_priv2``); the planner core is extracted from the sibling
``local_planner.py`` (pure numpy/scipy, no DimOS) so it runs under dim_priv2's
LCM message definitions without a version clash.

Run (no other sim must be using the LCM bus):
    DIM_PRIV2_ROOT=~/repos/dim_priv2 \\
      ~/repos/dim_priv2/.venv/bin/python pimsim_dynamic_demo.py
Outputs a video + frames to /tmp/pimsim_wavefront/.
"""

from __future__ import annotations

import os
from pathlib import Path
import sys
import tempfile

# Sim config, passed straight into build_babylon_sim / PimSimClient below
# (dedicated port so other running sims don't collide; visuals via load_visual).
PIMSIM_PORT = 8097
# One patrol box (no magnet follower) sweeping x=3, y in [-1.2, 1.2] on the floor.
DYNAMIC_OBSTACLES = (
    '[{"type":"patrol","id":"demo_patrol_box","a":[3.0,-1.2],"b":[3.0,1.2],'
    '"z":0.6,"extents":[0.6,0.6,1.2],"period_s":6.0,"rgba":[1.0,0.5,0.05,1.0]}]'
)

_DIM_PRIV2 = os.path.expanduser(os.environ.get("DIM_PRIV2_ROOT", "~/repos/dim_priv2"))

# Extract the pure planner core (everything above the DimOS module wrapper) so it
# imports with no DimOS dependency under the dim_priv2 environment.
_HERE = Path(__file__).resolve().parent
_SRC = (_HERE.parent / "local_planner.py").read_text().splitlines()
_cut = next(i for i, ln in enumerate(_SRC) if ln.startswith("from dimos.core.module import"))
_core_path = Path(tempfile.gettempdir()) / "wavefront_core.py"
_core_path.write_text("\n".join(_SRC[:_cut]))

sys.path.insert(0, str(_core_path.parent))
sys.path.insert(0, _DIM_PRIV2)

import math
import time

import lcm as lcmlib
import numpy as np
from playwright.sync_api import sync_playwright
import wavefront_core as wf

from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.experimental.pimsim.blueprints._factory import (
    build_babylon_sim,
    ensure_flat_floor_scene,
)
from dimos.experimental.pimsim.client import PimSimClient
from dimos.experimental.pimsim.entity import EntityStateBatch
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.protocol.service.lcmservice import _DEFAULT_LCM_URL

OUT = Path("/tmp/pimsim_wavefront")
PORT = PIMSIM_PORT
VIEWER_URL = f"http://localhost:{PORT}/"
ODOM_TOPIC = "/odom#geometry_msgs.PoseStamped"
ENTITY_TOPIC = "/entity_state_batch#pimsim.EntityStateBatch"
LIDAR_TOPIC = "/lidar#sensor_msgs.PointCloud2"
CMD_TOPIC = "/nav_cmd_vel#geometry_msgs.Twist"

SPAWN = (0.0, 0.0)
GOALS = [(7.5, 0.0), (-3.5, 0.0)]  # shuttle across the box's sweep
RECORD_SECONDS = float(os.environ.get("DEMO_RECORD_SECONDS", "44"))
RES = 0.1
WIN = 8.0
WIN_CELLS = int(WIN / RES)
Z_LO, Z_HI = 0.25, 1.6  # obstacle z-band above the floor
MAX_LIN, MAX_ANG = 0.32, 1.2
LOOKAHEAD = 0.9
GOAL_REACH = 0.6


def _yaw(qx, qy, qz, qw):
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def _wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    robot = {"x": SPAWN[0], "y": SPAWN[1], "z": 0.0, "yaw": 0.0, "n": 0}
    pts_world: dict = {"p": None}
    patrol = {"xy": None, "min_dist": math.inf}

    def on_odom(_c, data):
        m = PoseStamped.lcm_decode(data)
        robot["x"], robot["y"], robot["n"] = m.x, m.y, robot["n"] + 1
        robot["z"] = getattr(m, "z", 0.0)
        robot["yaw"] = _yaw(m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w)

    def on_lidar(_c, data):
        try:
            p, _c2 = PointCloud2.lcm_decode(data).as_numpy()
        except Exception:
            return
        if p is not None and len(p):
            pts_world["p"] = np.asarray(p, dtype=np.float64)

    def on_entities(_c, data):
        for desc, pose in EntityStateBatch.lcm_decode(data).entries:
            if desc.entity_id == "demo_patrol_box":
                patrol["xy"] = (pose.position.x, pose.position.y)
                d = math.hypot(robot["x"] - pose.position.x, robot["y"] - pose.position.y)
                patrol["min_dist"] = min(patrol["min_dist"], d)

    lcm = lcmlib.LCM(_DEFAULT_LCM_URL)
    lcm.subscribe(ODOM_TOPIC, on_odom)
    lcm.subscribe(LIDAR_TOPIC, on_lidar)
    lcm.subscribe(ENTITY_TOPIC, on_entities)

    coordinator = ModuleCoordinator.build(
        build_babylon_sim(
            ensure_flat_floor_scene(),
            load_visual=True,
            with_lidar=True,
            port=PIMSIM_PORT,
            dynamic_obstacles=DYNAMIC_OBSTACLES,
        )
    )
    client = PimSimClient(port=PIMSIM_PORT)
    pw = sync_playwright().start()
    browser = pw.chromium.launch(
        headless=False,
        args=["--headless=new", "--no-sandbox", "--use-gl=angle", "--use-angle=gl",
              "--enable-gpu", "--ignore-gpu-blocklist",
              "--disable-background-timer-throttling", "--disable-renderer-backgrounding"],
    )
    ctx = browser.new_context(
        viewport={"width": 1280, "height": 720},
        record_video_dir=str(OUT), record_video_size={"width": 1280, "height": 720},
    )
    page = ctx.new_page()
    params = wf.RepulsiveFieldParams(
        vehicle_width=0.6, safety_margin=0.5, commitment_weight=2.0, horizon=0.0
    )
    prev_local = None
    gi = 0
    try:
        page.goto(VIEWER_URL, wait_until="domcontentloaded", timeout=180000)
        page.wait_for_function("window.__pimsimReady === true", timeout=180000)
        client.start()
        client.set_agent_position(*SPAWN)
        time.sleep(2.0)
        page.evaluate(
            "() => { const c = window.__pimsimCamera; if (!c) return;"
            " c.setTarget(new BABYLON.Vector3(2.0,0.0,0)); c.alpha=-Math.PI/2;"
            " c.beta=0.35; c.radius=12.0; }"
        )
        t0 = time.time()
        while robot["n"] == 0 and time.time() - t0 < 30:
            lcm.handle_timeout(200)
        print(f"[wf] first odom after {time.time() - t0:.1f}s")

        deadline = time.time() + RECORD_SECONDS
        next_cmd = 0.0
        next_log = time.time()
        shot = 0
        while time.time() < deadline:
            lcm.handle_timeout(50)
            now = time.time()
            if now < next_cmd:
                continue
            next_cmd = now + 0.1
            gx, gy = GOALS[gi]
            if math.hypot(robot["x"] - gx, robot["y"] - gy) < GOAL_REACH:
                gi = (gi + 1) % len(GOALS)
                gx, gy = GOALS[gi]

            # local costmap from /lidar (world frame, obstacle z-band)
            ox, oy = robot["x"] - WIN / 2, robot["y"] - WIN / 2
            cost = np.zeros((WIN_CELLS, WIN_CELLS), dtype=np.float64)
            p = pts_world["p"]
            if p is not None and len(p):
                mask = (p[:, 2] > robot["z"] + Z_LO) & (p[:, 2] < robot["z"] + Z_HI)
                cols = np.round((p[mask, 0] - ox) / RES).astype(int)
                rows = np.round((p[mask, 1] - oy) / RES).astype(int)
                ok = (rows >= 0) & (rows < WIN_CELLS) & (cols >= 0) & (cols < WIN_CELLS)
                cost[rows[ok], cols[ok]] = 100

            gp = [(robot["x"], robot["y"]), (gx, gy)]
            poses = wf.plan_path(
                cost, RES, (ox, oy), gp, (robot["x"], robot["y"], robot["yaw"]),
                params, previous_path=prev_local,
            )
            prev_local = [(x, y) for x, y, _ in poses] if len(poses) >= 2 else None

            # pure pursuit; ease off when the path is short (planner blocked) so the
            # robot waits rather than creeping into the moving box.
            lin = ang = 0.0
            if len(poses) >= 2:
                path_len = sum(
                    math.hypot(poses[i][0] - poses[i - 1][0], poses[i][1] - poses[i - 1][1])
                    for i in range(1, len(poses))
                )
                acc, tgt = 0.0, poses[-1]
                for i in range(1, len(poses)):
                    acc += math.hypot(poses[i][0] - poses[i - 1][0], poses[i][1] - poses[i - 1][1])
                    if acc >= LOOKAHEAD:
                        tgt = poses[i]
                        break
                yaw_err = _wrap(math.atan2(tgt[1] - robot["y"], tgt[0] - robot["x"]) - robot["yaw"])
                ang = max(-MAX_ANG, min(MAX_ANG, 1.8 * yaw_err))
                scale = max(0.0, min(1.0, (path_len - 0.4) / 1.0))
                lin = 0.0 if abs(yaw_err) > 0.7 else MAX_LIN * scale
            cmd = Twist()
            cmd.linear.x = lin
            cmd.angular.z = ang
            lcm.publish(CMD_TOPIC, cmd.lcm_encode())

            if now >= next_log:
                next_log += 3.0
                print(f"[wf] t={int(now - (deadline - RECORD_SECONDS))}s "
                      f"robot=({robot['x']:.1f},{robot['y']:.1f}) goal={gi} poses={len(poses)} "
                      f"patrol_min={patrol['min_dist']:.2f} lidar_pts={0 if p is None else len(p)}")
                page.screenshot(path=str(OUT / f"frame_{shot:03d}.png"))
                shot += 1

        vp = page.video.path() if page.video else None
        page.close()
        ctx.close()
        if vp:
            os.replace(vp, OUT / "wavefront_dynamic.webm")
            print(f"[wf] video: {OUT / 'wavefront_dynamic.webm'}")
    finally:
        browser.close()
        pw.stop()
        client.stop()
        coordinator.stop()

    print("\n==== RESULTS ====")
    print(f"patrol min distance to robot centre: {patrol['min_dist']:.2f} m (want > 0.6 = touching)")
    ok = patrol["min_dist"] > 0.6 and robot["n"] > 0
    print("VERDICT:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    code = main()
    sys.stdout.flush()
    os._exit(code)
