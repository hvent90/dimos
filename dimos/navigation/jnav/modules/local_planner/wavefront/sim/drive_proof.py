# Copyright 2026 Dimensional Inc. Licensed under the Apache License, Version 2.0.
"""Headless browser validation of the wavefront sim (playwright).

Drives the real sim UI: paints an obstacle wall, sets a goal, toggles facing
modes, and loads a URL-defined costmap — capturing screenshots + asserting the
planner actually produced a detour. Screenshots land in PROOF_DIR.
"""

from __future__ import annotations

import json
from pathlib import Path
import sys
import time
from urllib.parse import quote

from playwright.sync_api import sync_playwright

BASE = "http://localhost:8000/"
PROOF_DIR = Path("/home/dimos/.local/share/cbg/long-tasks/BigFish/proof")
PROOF_DIR.mkdir(parents=True, exist_ok=True)


def canvas_rect(page):
    return page.evaluate(
        """() => { const r = document.getElementById('map').getBoundingClientRect();
                   return {x:r.x, y:r.y, w:r.width, h:r.height}; }"""
    )


def wait_for_path(page, timeout=8.0):
    """Wait until the planner has returned a non-trivial path."""
    end = time.time() + timeout
    while time.time() < end:
        n = page.evaluate("() => state.poses.length")
        if n and n > 3:
            return n
        time.sleep(0.1)
    return page.evaluate("() => state.poses.length")


def main() -> int:
    results = {}
    with sync_playwright() as p:
        browser = p.chromium.launch(headless=True, args=["--no-sandbox"])
        page = browser.new_page(viewport={"width": 1180, "height": 700})
        page.goto(BASE, wait_until="load")
        page.wait_for_function("() => typeof state !== 'undefined'")
        # Wait for websocket connect + initial plan (clear map, straight path).
        wait_for_path(page)
        page.screenshot(path=str(PROOF_DIR / "01_empty.png"))
        results["clear_path_poses"] = page.evaluate("() => state.poses.length")

        r = canvas_rect(page)
        # Paint a vertical wall across the straight path: from the top edge down
        # to ~60% height, leaving a clear gap below for the robot to round it.
        wall_x = r["x"] + r["w"] * 0.48
        page.mouse.move(wall_x, r["y"] + r["h"] * 0.04)
        page.mouse.down()
        steps = 24
        y0, y1 = r["y"] + r["h"] * 0.04, r["y"] + r["h"] * 0.60
        for i in range(steps + 1):
            page.mouse.move(wall_x, y0 + (y1 - y0) * i / steps)
            time.sleep(0.01)
        page.mouse.up()
        n = wait_for_path(page)
        time.sleep(0.4)
        page.screenshot(path=str(PROOF_DIR / "02_wall_detour.png"))
        detour = page.evaluate(
            """() => { const ys = state.poses.map(p => p[1]);
                       const c = state.start[1];
                       return Math.max(...ys.map(y => Math.abs(y - c))); }"""
        )
        # Confirm no pose sits on a lethal cell.
        on_obstacle = page.evaluate(
            """() => { let worst = 0;
                 for (const [x,y] of state.poses) {
                     const col = Math.round((x - state.origin[0]) / state.resolution);
                     const row = Math.round((y - state.origin[1]) / state.resolution);
                     if (col>=0 && row>=0 && col<state.cols && row<state.rows)
                         worst = Math.max(worst, state.grid[row*state.cols+col]);
                 } return worst; }"""
        )
        results["wall"] = {
            "poses": n,
            "max_lateral_detour_m": round(detour, 3),
            "worst_cost_on_path": on_obstacle,
        }

        # Right-click to move the goal to the lower-right (clear of the wall).
        page.mouse.click(r["x"] + r["w"] * 0.88, r["y"] + r["h"] * 0.78, button="right")
        wait_for_path(page)
        time.sleep(0.3)
        page.screenshot(path=str(PROOF_DIR / "03_goal_moved.png"))
        results["goal_after_rightclick"] = page.evaluate("() => state.goal")

        # Toggle omnidirectional — arrows should now face the goal, not travel.
        page.check("#omni")
        wait_for_path(page)
        time.sleep(0.3)
        page.screenshot(path=str(PROOF_DIR / "04_omnidirectional.png"))
        results["omni_on"] = page.evaluate("() => state.omnidirectional")
        page.uncheck("#omni")

        # URL-defined costmap scenario (JSON in the query string).
        spec = {
            "cols": 80,
            "rows": 56,
            "resolution": 0.1,
            "origin": [0, 0],
            "start": [0.6, 2.8],
            "goal": [7.4, 2.8],
            # Slalom: first wall hangs from the top (go under), second rises from
            # the bottom (go over) -> the robot weaves down then up to the goal.
            "rects": [[30, 22, 4, 34], [50, 0, 4, 34]],
        }
        page.goto(BASE + "?map=" + quote(json.dumps(spec)), wait_until="load")
        page.wait_for_function("() => typeof state !== 'undefined'")
        n2 = wait_for_path(page)
        time.sleep(0.5)
        page.screenshot(path=str(PROOF_DIR / "05_url_costmap_slalom.png"))
        results["url_costmap"] = {
            "poses": n2,
            "cols": page.evaluate("() => state.cols"),
            "reached": page.evaluate(
                """() => { const p = state.poses[state.poses.length-1];
                     return Math.hypot(p[0]-state.goal[0], p[1]-state.goal[1]) < 0.5; }"""
            ),
        }
        browser.close()

    (PROOF_DIR / "results.json").write_text(json.dumps(results, indent=2))
    print(json.dumps(results, indent=2))
    # Basic pass/fail gate for the driver itself.
    ok = (
        results["wall"]["max_lateral_detour_m"] > 0.5
        and results["wall"]["worst_cost_on_path"] < 50
        and results["url_costmap"]["reached"]
    )
    print("VALIDATION", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
