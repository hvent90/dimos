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

Per axis (vx, vy, wz on Go2), the SI loop runs **three phases** with one
ENTER per step:

1. **Floor probe** (~4 tiny amps, e.g. vx 0.02→0.15 m/s) — the AND-test
   (D3) detects the smallest commanded value that actually moves the
   robot (`|v_body| > 0.02 m/s` **AND** `> 5% of |amp|`, sustained for
   ≥5 samples).
2. **Dense sweep** (5 amps, unified `[0.2, 0.5, 1.0, 1.5, 2.0]` ladder
   in m/s for vx/vy and rad/s for wz) — FOPDT fit per amplitude; the
   canonical fit used by DERIVE is the lowest-amplitude one with
   r² > 0.9 (the **linear regime**, methodology v2 — D1).
3. **Ceiling probe** (~2 supra-sweep amps, e.g. vx 2.5, 3.0) — FOPDT
   fit per amplitude. **The operational ceiling DERIVE uses is
   `min(max(|K(amp)·amp|), profile.{vx,wz}_max)` across the FULL sweep
   + probe table** — i.e. the max steady-state OUTPUT magnitude the
   plant can deliver, clamped to the platform cap. This is robust to
   noisy FOPDT fits: a misfit K with the right amp still gives a
   sensible K·amp because the output magnitude is what matters for RG,
   not K alone. The K-sag amplitude (where K drops 15% below the
   linear-regime K) is still recorded as `saturating_at_amp` in the
   artifact for forensics, but is **not** used as the operational cap
   — with low-r² fits the K-sag detector misfires.

Settling pre-roll (~1.5 s) happens *after* you press Enter — wait for
it. Each phase prints a banner so you know which sub-test is running.

**Duration**: ~30–45 min hands-on for the full dense flow on Go2
(~32 ENTERs: 4 floor + 5 sweep + 2 ceiling per channel × 3 channels,
minus one floor amp on vy). Per-step gating is preserved — drift is
bounded to one step. The cost is "do it once, properly."

**Output** (same timestamp suffix):

```
data/characterization/go2/
├── go2_config_hw_concrete_<date>_<sha>.json   ← the tuning artifact
├── go2_config_hw_concrete_<date>_<sha>.png    ← fit-quality plot + K(amp) panel
└── go2_recording_<date>_<sha>.db              ← raw streams (cmd_vel, joint_state, odom, gate)
```

**PNG layout** (four rows × one column per channel; single y-axis per
subplot — easier to read than the old twin-axis version):

- Step response — measured (solid) + raw (dotted, when Hampel touched
  it) + fitted FOPDT (dashed). Skipped on re-derive (no raw traces in
  the artifact).
- **K(amp)** — gain per amplitude. Sweep (○-line, blue), ceiling probe
  (■-line, orange). Floor/ceiling marked as vertical dotted lines
  (green/red).
- **Output K·amp(amp)** — **the steady-state output magnitude the
  plant delivers.** This is the most diagnostic panel: it plateaus
  visually where the plant saturates. The operational ceiling is the
  max of this curve (clamped to the platform cap).
- **τ(amp)** — first-order time constant per amplitude.

**Artifact sections** (additive — older v1 readers ignore the new
ones):

- 1–4 + 6 (existing): plant, feedforward, velocity_profile,
  recommended_controller, valid_for_tuning.
- 5 **`velocity_envelope`** (new) — per-channel `{floor, ceiling,
  floor_not_found, ceiling_not_found}`.
- 7 **`dynamics_by_amplitude`** (new) — full K/τ/L table per channel
  across sweep + ceiling-probe amps (forensics + future lookup-based
  RG without re-collecting data).
- `floor_probe_results` (new sibling) — per-amp D3 pass/fail.
- `provenance.methodology_version` = 2 (defaults to 1 on legacy
  artifacts); `schema_version` is unchanged at 1.

**DERIVE** consumes the measured envelope:

- RG `min_speed` = `max(envelope.vx.floor, profile.min_speed_floor)`
- RG `v_max` = `envelope.vx.ceiling` (already clamped to `profile.vx_max`)
- RG `ω_max` = `envelope.wz.ceiling · 0.85` (15% headroom for cross-track)
- FF gain = `1/K` from the linear-regime fit (unchanged).
- `floor_not_found` / `ceiling_not_found` only fire when there is no
  data at all (empty probe / no converged fits). Clamping to the
  platform cap is silent (it's an envelope decision, not a measurement
  failure).

### Re-derive a stale artifact

If the methodology changed (e.g. ceiling logic was bug-fixed) and you
don't want to re-run on the robot, re-apply the current logic to an
existing v2 artifact:

```
python -m dimos.utils.benchmarking.characterization \
  --mode re-derive --robot go2 \
  --artifact data/characterization/go2/go2_config_hw_concrete_<date>_<sha>.json
```

The tool reads the artifact's stored `dynamics_by_amplitude` +
`floor_probe_results`, recomputes envelope + velocity_profile +
caveats, and writes `<stem>__rederived.json` + matching PNG alongside
the source. Plant + feedforward pass through unchanged (those were
already measured). No robot needed.

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
