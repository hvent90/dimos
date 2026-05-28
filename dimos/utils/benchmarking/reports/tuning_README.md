# Twist-base controller tuning — operator guide

Two blueprints, run in order:

1. **`unitree-go2-characterization`** — measures the robot's FOPDT plant, writes a config JSON.
2. **`unitree-go2-benchmark`** — runs the baseline controller across a speed ladder, scores CTE, writes the operating-point map.

Both bundle GO2Connection + ControlCoordinator + pygame teleop + the relevant module + a per-session SQLite recorder. One terminal each.

## Prerequisites

1. From an X11 desktop terminal:
   ```
   cd ~/dimos && source .venv/bin/activate
   ```
2. Robot powered on, network reachable. Clear ~3m × 3m of floor space.

## Pygame controls (both blueprints)

Window must have focus. **WASD/QE** = reposition. **Enter** = advance. **K** = skip. **Backspace** = quit.

## Step 1 — characterization

```
dimos run unitree-go2-characterization
```

Per axis (vx, wz on Go2 default gait) × a few amplitudes, the SI loop prompts you to reposition, then issues a velocity step and records the response. Settling pre-roll (~1.5 s) happens *after* you press Enter — wait for it before assuming nothing's happening.

**Duration**: ~5–8 minutes for the default amplitude set.

**Output** (lands together with the same timestamp suffix):

```
data/characterization/go2/
├── go2_config_hw_concrete_<date>_<sha>.json   ← the tuning artifact
├── go2_config_hw_concrete_<date>_<sha>.png    ← fit-quality plot
└── go2_recording_<date>_<sha>.db              ← raw streams (cmd_vel, joint_state, odom, gate)
```

The PNG is what you READ to judge the fit. K/τ/L per channel + r² are annotated; raw (dotted) overlays the savgol-filtered fit (solid) when Hampel replaced points.

Override defaults with `-o characterizer.<field>=<value>`, e.g.:
- `-o characterizer.surface=grass`
- `-o characterizer.step_s=10`
- `-o characterizer.savgol_window=15` (more aggressive gait smoothing)

## Step 2 — benchmark

```
dimos run unitree-go2-benchmark \
  -o benchmarker.config=/path/to/go2_config_hw_*.json     # bare baseline

dimos run unitree-go2-benchmark-rg \
  -o benchmarker.config=/path/to/go2_config_hw_*.json     # RG arm baked in (rg=true)
```

Each comparison arm is its own blueprint variant; the `-o` overrides
are reserved for runtime knobs only (the artifact path, e_max sweeps,
etc.).

Per (path, speed) × fixed path set (`straight_line`, `single_corner`, `square`, `circle`), the loop prompts you to aim the robot, then drives the baseline follower over the anchored path. CTE is scored from the real trajectory.

**Comparison arms** (each measures *against* the bare physical limit):

- `-o benchmarker.ff=true` — apply derived feedforward.
- `-o benchmarker.profile=true` — apply derived curvature velocity profile.
- `-o benchmarker.rg=true` `-o benchmarker.e_max=0.20` — apply reference governor's per-waypoint cap (precision = `e_max / max(τ+L)`).

For the RG arm, **the Go2 stalls below ~0.2 m/s commanded velocity** even when the math says go slower. `benchmarker.min_speed` (default `0.2`) is the floor; set to `None` to defer to the artifact's value. Tight corners against tight `e_max` will track badly because saturation-binding cells get floored above ω_max — that's a physical limit, not a bug.

**Duration**: ~15–25 min depending on speeds × arms.

**Output**:

```
data/benchmark/go2/
├── go2_benchmark_<date>_<sha>.png  ← XY trajectory overlay + CTE plot
├── …_<arm>_<date>_<sha>.json       ← per-arm operating-point map (bare run also appends section 5 to the input artifact)
└── go2_benchmark_<date>_<sha>.db   ← raw streams
```

## Step 3 — precision-controlled nav (end-to-end)

```
dimos run unitree-go2-precision-nav
```

Bundles the Go2 coord + rerun viewer + click-to-goal planning chain
(VoxelGridMapper + CostMapper + ReplanningAStarPlanner) + KeyboardTeleop
(0-9 e_max slider, WASD/QE disabled) + per-session recorder. Stock Go2
hardware (L1 lidar, GO2Connection odom) is sufficient — **no Mid360 /
FastLio2 required**. One terminal.

Composition mirrors the working smart-tier Go2 nav blueprint and swaps
only the actuation seam: instead of `MovementManager` mixing
`nav_cmd_vel` into `cmd_vel` and driving GO2Connection directly,
`ReplanningAStarPlanner.path` flows into `ControlCoordinator.path`, the
coord broadcasts `set_path(path, odom)` to the `precision_follower`
task, and the precision controller drives the robot via the coord's
tick loop.

Operator flow:

1. Open rerun (auto-launches with the blueprint).
2. Click a point on the map. `RerunWebSocketServer` emits a
   `clicked_point: PointStamped`.
3. `ReplanningAStarPlanner` consumes the click + the live costmap (from
   `VoxelGridMapper → CostMapper`) + GO2Connection's `odom: PoseStamped`
   and emits `path: Path`.
4. `ControlCoordinator.path` receives the planned path; `_on_path`
   snapshots the latest odom and broadcasts `set_path(path, odom)` to
   any task with the method.
5. `PrecisionPathFollowerTask.set_path` delegates to its existing
   `start_path` (which lazy-loads the artifact, solves
   `solve_profile()`, and starts the state machine).
6. The coord's tick loop drives the precision controller's velocity
   commands to GO2Connection.

Live-tune precision: keys **0-9** in the pygame window set corridor
half-width 0.0-0.9 m. `PrecisionPathFollowerTask` re-solves the profile
on each keypress and atomically swaps the per-waypoint cap mid-path.

**Output**:

```
data/precision_nav/go2/
└── go2_precision_nav_<date>_<sha>.db    ← cmd_vel, joint_state, odom, gate
```

**Hardware notes**:

- L1 lidar has narrow FOV — the costmap is sparser than what Mid360
  would provide. Quality of click-to-plan paths is limited accordingly.
  Mid360 + FastLio2 is the upgrade path if you want the proper
  nav-stack pipeline (see `unitree-g1-nav-onboard` for that variant).
- `ReplanningAStarPlanner.nav_cmd_vel: Out[Twist]` is intentionally
  unwired — the precision controller, not the planner, drives the
  robot. The planner is only used as a path source.

## Reading recordings

```python
from dimos.memory2.store.sqlite import SqliteStore
from dimos.msgs.sensor_msgs.JointState import JointState
store = SqliteStore(path="<.db file>"); store.start()
for obs in store.stream("joint_state", JointState):
    ts, msg = obs.ts, obs.data   # re-fit, plot, etc.
```

Streams: `cmd_vel` (Twist), `joint_state` (JointState, x/y/yaw), `odom` (PoseStamped, raw), `gate` (Int8, operator events). The Step 3 (precision-nav) recording adds nothing new — same schema, different `tag`.

## Troubleshooting

- **Pygame window doesn't open**: X11 not reachable. `xeyes` test, then `export DISPLAY=:1; export XAUTHORITY=/run/user/$(id -u)/.Xauthority`.
- **Enter does nothing**: pygame window isn't focused. Click it before pressing.
- **Terminal flooded with TF warnings**: pipe through `grep --line-buffered -E 'Benchmarker|Characterizer|reject|aborted|arrived|timeout|configure|start_path|required|reposition'`.
- **Robot won't move on RG arm**: see `min_speed` note above. Try `-o benchmarker.e_max=0.2` first; if still stuck, raise `-o benchmarker.min_speed=0.25`.
- **Self-test (no robot)**: keep using the CLI: `uv run python -m dimos.utils.benchmarking.characterization --mode self-test`.
