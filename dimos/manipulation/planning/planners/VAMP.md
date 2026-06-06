# Testing the VAMP Planner

VAMP (Vector-Accelerated Motion Planning) is a SIMD-accelerated motion planner.
It uses its own internal collision engine — obstacles are pulled from `WorldSpec`
and mirrored into VAMP's environment.

**Supported robots:** Panda (7-DOF), UR5, Fetch, Baxter. VAMP ships pre-compiled
robot models for these — other robots require contributing to VAMP upstream.

## Install (NixOS / Debian + Nix)

VAMP needs Eigen3 + BMI2/AVX2 intrinsics, and must be built without uv's build
isolation so the dev shell's compiler env reaches the build:

```bash
# 1. Enter the dev shell
nix develop

# 2. Recreate venv against the nix-provided Python (avoids system header leakage)
deactivate 2>/dev/null
rm -rf .venv
uv venv --python python3.12
source .venv/bin/activate
uv sync --all-extras

# 3. Build-time deps + flags
uv pip install scikit-build-core nanobind cmake ninja
export Eigen3_DIR=/nix/store/rzbrwrvqvx10wpad0z9qf1hrxa0mmfyr-eigen-3.4.0-unstable-2022-05-19/share/eigen3/cmake
export NIX_ENFORCE_NO_NATIVE=0
export CXXFLAGS="-march=native -mbmi2 -mavx2"
export CFLAGS="-march=native -mbmi2 -mavx2"

# 4. Install VAMP
uv pip install vamp-planner --no-build-isolation
```

The resulting wheel is built for your CPU's exact instruction set — not portable.

## Launch

```bash
# Terminal 1 — Panda arm, mock hardware, meshcat viz
dimos run panda-vamp-planner

# Terminal 2 — interactive client
python -i -m dimos.manipulation.planning.examples.manipulation_client
```

Expected startup line in terminal 1:
```
Init joints captured: [0.000, -0.785, 0.000, -2.356, 0.000, 1.571, 0.785]
```

The arm should start at Franka's "ready" pose — if it starts at all zeros, the
`adapter_kwargs={"initial_positions": ...}` wiring in the blueprint is broken
and joint4 will be outside its valid range `[-3.07, -0.07]`.

## Test commands (Python REPL)

```python
# Introspection
commands()              # list all client functions
robots()                # ['panda']
url()                   # meshcat URL — open in browser
state()                 # IDLE / PLANNING / EXECUTING
joints()                # current joint angles
info()                  # robot config incl. home_joints

# Basic moves
home()                                                          # plan+execute to home
plan([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]); preview(3.0); execute()

# Cartesian targets (orientation preserved if rpy omitted)
plan_pose(0.4, 0.0, 0.5); preview(); execute()
plan_pose(0.4, 0.2, 0.3, roll=0, pitch=1.57, yaw=0); preview(); execute()

# Obstacles
add_box("cube", 0.3, 0.0, 0.3, 0.1, 0.1, 0.1)
add_sphere("ball", 0.3, 0.2, 0.4, 0.05)
add_cylinder("can", 0.3, -0.2, 0.3, 0.04, 0.15)
plan_pose(0.4, 0.0, 0.5)                                        # planner avoids them
remove("cube")

# Collision check (no planning)
collision_free([0.1] * 7)
```

Watch terminal 1 for planner messages — e.g.
`VAMP resolved robot module: panda` confirms VAMP is in use.

## Switching planners

To compare VAMP vs RRT-Connect on the same blueprint, edit
[../../blueprints.py](../../blueprints.py) (`panda_vamp_planner`) and change:

```python
planner_name="vamp"        # or "rrt_connect"
```

Or pass `planner_name` directly when constructing `ManipulationModule`.

VAMP also accepts an algorithm choice via planner kwargs:
`rrtc` (default), `prm`, `fcit`, `aorrtc`.

## Tests

```bash
# Unit tests for the VAMP adapter (no Drake world)
pytest dimos/manipulation/planning/planners/test_vamp_planner.py -v

# End-to-end planning tests
pytest dimos/manipulation/planning/tests/test_vamp_planner.py -v
```

## Known gotchas

- **Panda URDF is primitives, not meshes.** `data/panda_description/urdf/panda.urdf`
  is a hand-written cylinder placeholder. VAMP uses its own pre-compiled
  collision model, so planning is accurate — but the viz looks like a stack of
  cylinders, not a real Franka. Not a bug.
- **Mock adapter starts at zero by default.** Panda's joint4 range is
  `[-3.07, -0.07]`, so zero pose is invalid (self-collision +
  `COLLISION_AT_START`). The Panda blueprint passes
  `initial_positions=_PANDA_HOME_JOINTS` through `adapter_kwargs` to avoid this.
- **`from dimos.msgs.sensor_msgs import JointState` imports the module, not the
  class.** Must be `from dimos.msgs.sensor_msgs.JointState import JointState`.
- **Build isolation hides the shell env.** `uv pip install` without
  `--no-build-isolation` runs CMake in a sandbox that drops `Eigen3_DIR` and
  `CMAKE_PREFIX_PATH` — build fails to find Eigen even though it's in the dev
  shell.
- **`NIX_ENFORCE_NO_NATIVE` strips `-march=native`,** which removes BMI2
  enablement, which breaks VAMP's `_pdep_u32` intrinsics. Must export
  `NIX_ENFORCE_NO_NATIVE=0` and the CXXFLAGS before building.
