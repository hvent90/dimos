# Manipulation planning backends

DimOS manipulation planning is configured as a stack:

- `world`: robot/environment representation and collision or validity checks
- `planner`: joint-space path planning
- `kinematics`: pose-to-joint solving for pose goals

The default stack remains Drake world + RRT-Connect planner + Jacobian IK.

## VAMP backend

VAMP is an optional joint-space planning backend. Install it only when needed:

```bash
uv sync --extra vamp
```

VAMP owns its robot artifact and environment representation, so select the VAMP
world and VAMP planner together:

```python
from dimos.manipulation.manipulation_module import ManipulationModule

module = ManipulationModule.blueprint(
    world={"backend": "vamp", "artifact": {"mode": "official", "robot": "panda"}},
    planner={"backend": "vamp", "algorithm": "rrtc", "simplify": True},
)
```

Supported VAMP algorithms are `rrtc`, `prm`, `fcit`, and `aorrtc`.

For a user-prepared custom robot artifact, point the world config at a local
Python module/package produced outside DimOS:

```python
module = ManipulationModule.blueprint(
    world={"backend": "vamp", "artifact": {"mode": "custom", "path": "/path/to/vamp_robot"}},
    planner={"backend": "vamp", "algorithm": "prm"},
)
```

CLI/config overrides follow the same nested field shape, for example:

```bash
uv run --extra manipulation --extra vamp dimos run panda-coordinator \
  -o manipulationmodule.world.backend=vamp \
  -o manipulationmodule.world.artifact.mode=official \
  -o manipulationmodule.world.artifact.robot=panda \
  -o manipulationmodule.planner.backend=vamp \
  -o manipulationmodule.planner.algorithm=rrtc
```

## Artifact scope

DimOS does not generate VAMP artifacts at runtime. Official artifacts are loaded
from the installed `vamp-planner` package. Custom robot artifacts must be
prepared by the user/upstream VAMP tooling before DimOS starts.

## Pose planning with VAMP

The VAMP planner is joint-space only. It does not run IK, convert poses, or
probe Jacobians. Pose planning is available only if the configured kinematics
backend can solve against the selected world. A VAMP world currently raises a
clear unsupported-capability error for Jacobian requests.

## Franka Panda mock-control support

`dimos.robot.catalog.franka.franka_panda()` provides a mock-control Franka Panda
configuration for VAMP tests and planner benchmarks. It uses:

- mock manipulator control by default (`adapter_type="mock"`)
- Panda arm joint order `panda_joint1` through `panda_joint7`
- LFS-backed model resources under `franka_description`
- `RobotConfig.to_hardware_component()` for `ControlCoordinator`
- `RobotConfig.to_robot_model_config()` for `ManipulationModule`

The Panda model constants are:

- `FRANKA_PANDA_MODEL`: `franka_description/urdf/panda.urdf.xacro`
- `FRANKA_PANDA_FK_MODEL`: `franka_description/urdf/panda.urdf`
- `FRANKA_PANDA_SRDF`: `franka_description/srdf/panda.srdf`

The description package should be stored using the repository LFS data pattern:
`data/.lfs/franka_description.tar.gz` extracts to `data/franka_description/`.
Do not download or generate the Panda URDF/SRDF at import time.

Start the registered mock coordinator/planner blueprint with:

```bash
uv run --extra manipulation --extra vamp dimos run panda-coordinator \
  -o manipulationmodule.world.backend=vamp \
  -o manipulationmodule.world.artifact.mode=official \
  -o manipulationmodule.world.artifact.robot=panda \
  -o manipulationmodule.planner.backend=vamp \
  -o manipulationmodule.planner.algorithm=rrtc
```

Then use the manipulation client in another terminal and plan joint-space Panda
motions with `plan([...], "panda")`.

## Failure modes

- Missing VAMP dependency: selecting a VAMP world raises an install hint for
  `vamp-planner` / `dimos[vamp]`.
- Invalid pairing: VAMP world and VAMP planner must be selected together.
- Incompatible kinematics: Drake optimization IK requires a Drake world; VAMP
  pose planning requires a kinematics backend that naturally supports the VAMP
  world capabilities it needs.
- Unsupported capability: VAMP does not expose Jacobians or minimum-distance
  queries through the current Python API.
- Model/artifact mismatch: Panda benchmarks should verify URDF joint order and
  limits against the official VAMP Panda artifact.

## Contributor notes

- Keep optional planner imports lazy and backend-scoped.
- Add new backends through typed `world`, `planner`, or `kinematics` config
  variants with discriminator fields.
- Use explicit `DeprecationWarning` shims for migrated config fields.
- Raise clear unsupported-capability errors instead of synthesizing planner
  features the backend does not expose.
- Prefer mock-control catalog fixtures for backend tests and benchmarks before
  adding real hardware adapters.
