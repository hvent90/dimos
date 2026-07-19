# Copyright 2026 Dimensional Inc. Licensed under the Apache License, Version 2.0.
"""Scenario battery for refining the wavefront planner via the sim.

Loads a set of URL-defined costmaps at a chosen vehicle width, screenshots each,
and reports metrics (reached? min clearance? self-intersections? stalls short?)
so misbehaviours are easy to spot. Output -> PROOF_DIR/refine/.
"""

from __future__ import annotations

import json
from pathlib import Path
import time
from urllib.parse import quote

from playwright.sync_api import sync_playwright

BASE = "http://localhost:8000/"
OUT = Path("/home/dimos/.local/share/cbg/long-tasks/BigFish/proof/refine")
OUT.mkdir(parents=True, exist_ok=True)

# Each scenario: (name, vehicle_width, spec). rects = [col,row,wCells,hCells] lethal.
G = {"cols": 90, "rows": 60, "resolution": 0.1, "origin": [0, 0]}
SCENARIOS = [
    ("corridor_fits", 0.5, {**G, "start": [0.3, 2.7], "goal": [8.7, 2.7],
        "rects": [[0, 0, 90, 24], [0, 30, 90, 30]]}),          # 0.6 m channel
    ("corridor_too_tight", 0.5, {**G, "start": [0.3, 2.8], "goal": [8.7, 2.8],
        "rects": [[0, 0, 90, 26], [0, 30, 90, 30]]}),          # 0.4 m channel
    ("enclosed_goal", 0.5, {**G, "start": [0.5, 3.0], "goal": [4.5, 3.0],
        "rects": [[38, 22, 2, 18], [52, 22, 2, 18], [38, 22, 16, 2], [38, 38, 16, 2]]}),
    ("deep_u_trap", 0.5, {**G, "start": [1.0, 3.0], "goal": [7.5, 3.0],
        "rects": [[44, 12, 3, 36], [20, 12, 26, 3], [20, 45, 26, 3]]}),
    ("clutter", 0.5, {**G, "start": [0.4, 3.0], "goal": [8.6, 3.0],
        "rects": [[25, 10, 5, 12], [25, 38, 5, 12], [45, 24, 5, 14], [62, 8, 5, 16],
                  [62, 40, 5, 14]]}),
    ("goal_on_obstacle", 0.5, {**G, "start": [0.5, 3.0], "goal": [5.0, 3.0],
        "rects": [[46, 24, 8, 12]]}),                          # goal sits in the block
    ("double_pinch", 0.5, {**G, "start": [0.4, 3.0], "goal": [8.6, 3.0],
        "rects": [[28, 32, 5, 28], [44, 0, 5, 30], [60, 32, 5, 28]]}),
    ("start_near_wall", 0.5, {**G, "start": [0.5, 2.45], "goal": [8.5, 3.5],
        "rects": [[0, 0, 90, 22]]}),                           # start ~0.25 m above floor
]


def metrics(page):
    return page.evaluate(
        """() => {
        const p = state.poses; if (!p.length) return {poses:0, reached:false};
        const last = p[p.length-1];
        const reached = Math.hypot(last[0]-state.goal[0], last[1]-state.goal[1]) < 0.5;
        const shortBy = Math.hypot(last[0]-state.goal[0], last[1]-state.goal[1]);
        // min clearance to a lethal cell
        let lethal = [];
        for (let r=0;r<state.rows;r++) for (let c=0;c<state.cols;c++)
            if (state.grid[r*state.cols+c] >= 50) lethal.push([c*state.resolution, r*state.resolution]);
        let minc = Infinity;
        for (const [x,y,_] of p) for (const [ox,oy] of lethal)
            minc = Math.min(minc, Math.hypot(x-ox, y-oy));
        // self-intersections
        function ccw(a,b,c){return (c[1]-a[1])*(b[0]-a[0])>(b[1]-a[1])*(c[0]-a[0])}
        let xings=0;
        for (let i=0;i<p.length-1;i++) for (let j=i+2;j<p.length-1;j++){
            const a=p[i],b=p[i+1],c=p[j],d=p[j+1];
            if (ccw(a,c,d)!=ccw(b,c,d) && ccw(a,b,c)!=ccw(a,b,d)) xings++;
        }
        return {poses:p.length, reached, shortBy:+shortBy.toFixed(2),
                minClear:+minc.toFixed(3), xings};
    }"""
    )


def main():
    results = {}
    with sync_playwright() as pw:
        b = pw.chromium.launch(headless=True, args=["--no-sandbox"])
        pg = b.new_page(viewport={"width": 1180, "height": 720})
        for i, (name, vw, spec) in enumerate(SCENARIOS, 1):
            pg.goto(BASE + "?map=" + quote(json.dumps(spec)), wait_until="load")
            pg.wait_for_function("() => typeof state !== 'undefined'")
            pg.eval_on_selector(
                "#vehicleWidth",
                f"el=>{{el.value='{vw}';el.dispatchEvent(new Event('input'))}}",
            )
            time.sleep(0.9)
            m = metrics(pg)
            m["vehicle_width"] = vw
            results[name] = m
            pg.screenshot(path=str(OUT / f"{i:02d}_{name}.png"))
            print(f"{name:20} {m}")
        b.close()
    (OUT / "results.json").write_text(json.dumps(results, indent=2))


if __name__ == "__main__":
    main()
