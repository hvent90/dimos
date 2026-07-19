# Wavefront Local Planner

A **wavefront** local planner that consumes a 2D **costmap**
(`nav_msgs/OccupancyGrid`) and a **global path**, and produces a **local path of
oriented poses** that follows the route and rounds obstacles with clearance.

## Layout

| File | What |
|------|------|
| `local_planner.py` | The planner. A pure, DimOS-free core (`plan_path`, `clearance_cost`, `WavefrontParams`) plus the DimOS `WavefrontLocalPlanner` `Module` that wraps it. |
| `test_local_planner.py` | Hermetic pytest (no hardware/sim/network). |
| `sim/` | Standalone web sim — Python websocket backend + plain-JS frontend. |

## Algorithm — a wavefront navigation function over a clearance-shaped cost field

The classic Khatib APF sums an attractive and a repulsive *force* and integrates
them step by step. That is greedy: it gets stuck in local minima and ties the
path into knots/loops when an obstacle sits head-on. This planner keeps the
wavefront but uses it the robust way — as a **cost layer**, solved
**globally**:

0. **Vehicle width** — obstacles are first inflated by the robot radius
   (`vehicle_width / 2`), so a cell is traversable only if the whole footprint
   fits. Gaps narrower than the vehicle are blocked, and the path centreline
   keeps at least a body-radius of clearance from every wall.
1. **Clearance cost** — an EDT-inflated obstacle-proximity penalty measured from
   the body (0 beyond the influence radius, ramping to 1 at the inflated face).
   The same Khatib repulsion intuition, expressed as a *cost* rather than a
   force. Plus a distance-to-the-global-path term so the local path stays in the
   corridor.
2. **Wavefront (Dijkstra)** rooted at the **robot** builds the shortest-path tree
   over the free cells (8-connected, no diagonal corner-cutting): the optimal
   cost and predecessor to every reachable cell.
3. The local path is the tree's optimal path to a **target** = the *furthest
   point along the global route* the robot can reach (the carrot); a smoothing
   pass rounds the grid steps into a clean curve.

Because the path is an acyclic optimal path in a shortest-path tree, this is
**loop- and local-minimum-free by construction** — not patched after the fact:

- **No local minima / no loops/knots** — the path can't circle, double back, or
  stall in a pocket.
- **Clearance + avoidance** — the clearance cost makes the optimal path bow away
  from obstacles.
- **Follows the global route** — the target is the furthest point along the
  global path the robot can reach (the carrot), so when a goal's region is a
  disconnected island in the local costmap (its door a long loop away, outside
  the window) the planner makes progress *along the route toward the door*
  instead of beelining at the goal and stalling at the near wall (the wp4
  failure). A path-distance cost term keeps the local path in the corridor.
- **Best effort** — when the goal itself is reachable the carrot is the goal;
  when it is walled off or inside an obstacle the path runs as far along the
  route as it safely can rather than freezing or clipping. Empty only if the
  robot itself is boxed in.

### Orientation (output poses carry yaw)

- `face_forward_weight` ∈ [0, 1] — blends each pose's yaw between the travel
  tangent (1.0) and the goal direction (0.0). Robots are assumed able to turn in
  place, so this is a preference.
- `omnidirectional` — holonomic mode: facing is decoupled from motion; poses
  face the goal.

## Dynamic (moving) obstacles

The planner replans every costmap/odometry tick, so a moving obstacle painted
into the costmap is avoided reactively. Two knobs make that robust (verified by
`test_dynamic_obstacles.py`, a closed-loop sim that moves an obstacle while the
robot follows the replanned path):

- `safety_margin` (m) — extra hard clearance kept beyond the body, a buffer for
  moving obstacles. Trades tight-gap access for margin. Default 0.
- `commitment_weight` — penalises straying from last tick's path (pass
  `previous_path`), i.e. temporal hysteresis. Without it, two obstacles wiggling
  either side of the path make the route flip-flop tick-to-tick; with it the
  route stays committed (no oscillation, no clip). The DimOS module feeds last
  tick's path automatically. Default 0.

## Running the web sim

```bash
source .venv/bin/activate            # the dimos venv (numpy, scipy, websockets)
python sim/server.py                 # serves http://localhost:8000 + ws://localhost:8765
```

Open <http://localhost:8000/>. **Left-drag** paints obstacles, **right-click**
sets the goal, **Shift+left** erases, **C** clears. The "prefer face forward"
slider, "omnidirectional" checkbox, and "vehicle width" slider drive the planner
live (a translucent band the width of the vehicle is swept along the path so you
can see the clearance). The backend runs the *real* `plan_path` core, so the sim
and the DimOS module plan identically.

A canned scenario can be loaded straight from the URL — the `?map=` query param
takes URL-encoded JSON:

```json
{ "cols": 80, "rows": 56, "resolution": 0.1, "origin": [0, 0],
  "start": [0.6, 2.8], "goal": [7.4, 2.8],
  "rects": [[30, 22, 4, 34], [50, 0, 4, 34]] }
```

(`rects` are filled lethal rectangles given as `[col, row, widthCells, heightCells]`.)

`sim/drive_proof.py` is the headless Playwright validation that paints a wall,
sets a goal, toggles omni mode, and loads a URL costmap, capturing screenshots.

## Tests

```bash
python -m pytest dimos/navigation/jnav/modules/local_planner/wavefront/test_local_planner.py -q
```

## Notes

- It is a *local* planner: it plans over the costmap it is given each tick (a
  rolling local window in normal use), with `horizon` capping the emitted path
  length. The wavefront is a Dijkstra over that grid — a few ms for typical local
  costmaps; on very large grids prefer a windowed costmap.
- `clearance_weight` trades clearance vs. path length; `path_weight` trades
  corridor-adherence vs. taking the locally shortest detour.
