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

"""Web-sim backend for the repulsive-field local planner.

Serves the static frontend over HTTP and runs the **real** planner core
(``plan_path`` from the sibling ``local_planner`` module — the exact code the
DimOS module uses) over a websocket. The browser sends the painted costmap +
goal + facing params; the backend builds a straight-line global path from a
fixed start to the goal, plans, and returns the oriented local path.

Run:  ``python -m dimos.navigation.nav_3d.repulsive_local_planner.sim.server``
or    ``python server.py``  (from this directory; it fixes up sys.path).

Then open http://localhost:8000/ . Right-click sets the goal, left-click paints
obstacles. A canned scenario can be passed in the URL, e.g. ``/?map=<json>``.
"""

from __future__ import annotations

import asyncio
from functools import partial
import http.server
import json
from pathlib import Path
import sys
import threading

import numpy as np
from websockets.asyncio.server import serve

# Allow ``python server.py`` (no package context) to find the dimos package.
_REPO_ROOT = Path(__file__).resolve().parents[7]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from dimos.navigation.nav_3d.repulsive_local_planner.local_planner import (
    RepulsiveFieldParams,
    plan_path,
)

HTTP_PORT = 8000
WS_PORT = 8765
_HERE = Path(__file__).resolve().parent


def _straight_path(
    start: tuple[float, float], goal: tuple[float, float], resolution: float
) -> list[tuple[float, float]]:
    """Sample a straight global path from start to goal at ~one point per cell."""
    dist = float(np.hypot(goal[0] - start[0], goal[1] - start[1]))
    n = max(2, int(dist / max(resolution, 1e-3)) + 1)
    return [
        (
            start[0] + (goal[0] - start[0]) * t,
            start[1] + (goal[1] - start[1]) * t,
        )
        for t in np.linspace(0.0, 1.0, n)
    ]


def plan_from_request(req: dict) -> dict:
    """Run the planner for one frontend request; returns a JSON-able response."""
    width = int(req["width"])
    height = int(req["height"])
    resolution = float(req.get("resolution", 0.1))
    origin = tuple(req.get("origin", [0.0, 0.0]))
    grid = np.asarray(req["grid"], dtype=np.float64).reshape(height, width)

    start = tuple(req["start"][:2])
    start_yaw = float(req["start"][2]) if len(req["start"]) > 2 else 0.0
    goal = tuple(req["goal"][:2])

    p = req.get("params", {})
    params = RepulsiveFieldParams(
        vehicle_width=float(p.get("vehicle_width", 0.5)),
        safety_margin=float(p.get("safety_margin", 0.0)),
        influence_radius=float(p.get("influence_radius", 0.8)),
        commitment_weight=float(p.get("commitment_weight", 0.0)),
        face_forward_weight=float(p.get("face_forward_weight", 0.8)),
        omnidirectional=bool(p.get("omnidirectional", False)),
        horizon=float(p.get("horizon", 0.0)),  # 0 = roll all the way to the goal
        lethal_threshold=int(p.get("lethal_threshold", 50)),
    )
    previous_path = req.get("previous_path") or None

    global_path = _straight_path(start, goal, resolution)
    poses = plan_path(
        grid,
        resolution,
        origin,
        global_path,
        (start[0], start[1], start_yaw),
        params,
        previous_path=previous_path,
    )
    return {
        "type": "path",
        "poses": [[round(x, 4), round(y, 4), round(yaw, 4)] for x, y, yaw in poses],
        "global_path": [[round(x, 4), round(y, 4)] for x, y in global_path],
    }


async def _handler(websocket) -> None:  # type: ignore[no-untyped-def]
    async for message in websocket:
        try:
            req = json.loads(message)
            resp = plan_from_request(req)
        except Exception as exc:  # surface errors to the browser console
            resp = {"type": "error", "message": f"{type(exc).__name__}: {exc}"}
        await websocket.send(json.dumps(resp))


def _serve_static() -> None:
    handler = partial(http.server.SimpleHTTPRequestHandler, directory=str(_HERE))
    httpd = http.server.ThreadingHTTPServer(("0.0.0.0", HTTP_PORT), handler)
    httpd.serve_forever()


async def _main() -> None:
    threading.Thread(target=_serve_static, daemon=True).start()
    print(f"  static : http://localhost:{HTTP_PORT}/")
    print(f"  ws     : ws://localhost:{WS_PORT}/")
    async with serve(_handler, "0.0.0.0", WS_PORT):
        await asyncio.Future()  # run forever


if __name__ == "__main__":
    try:
        asyncio.run(_main())
    except KeyboardInterrupt:
        pass
