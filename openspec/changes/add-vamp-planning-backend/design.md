## Context

DimOS manipulation planning is currently centered on the existing planning contracts in `dimos/manipulation/planning/spec/protocols.py`:

- `WorldSpec` owns robot/world state, collision checks, FK/link poses, and currently includes Jacobian access.
- `PlannerSpec` owns joint-space path planning from start to goal joint states.
- `KinematicsSpec` owns pose-to-joint-state solving.
- `VisualizationSpec` is optional and currently implemented by `DrakeWorld` through Meshcat.

The current module wiring in `dimos/manipulation/manipulation_module.py` initializes a `WorldMonitor`, adds configured `RobotModelConfig` objects, finalizes the world, then creates planner and kinematics backends. `WorldMonitor` already has a backend seam (`WorldMonitor(backend="drake", enable_viz=False, **kwargs)`) that delegates to `create_world(...)` in `dimos/manipulation/planning/factory.py`.

VAMP has a different model than the current Drake-backed world. Its Python package exposes robot-specific planning modules and utilities such as joint-space planning algorithms, validation/debug helpers, `fk`, and `eefk`. It does not expose a general runtime URDF-to-planner pipeline through the runtime binding, and it does not expose a public Jacobian API. Custom robot support is an offline/user-owned artifact-generation concern upstream of runtime planning.

PR #2481 for Pink IK establishes the configuration pattern this change should follow: typed discriminated backend config objects with `backend` as the discriminator, backend-specific settings in the nested config, and deprecated flat compatibility shims that emit warnings when used.

## Goals / Non-Goals

**Goals:**

- Add VAMP as an optional manipulation planning backend for joint-space planning.
- Load VAMP robot artifacts from either official VAMP robot modules/artifacts or a user-provided custom artifact path.
- Keep VAMP dependency loading lazy and backend-scoped.
- Add typed nested world and planner config objects instead of new flat `ManipulationModuleConfig.vamp_*` fields.
- Validate VAMP world/planner pairing eagerly during planning-stack initialization.
- Keep pose planning conditional on an explicitly compatible kinematics backend.
- Add a Franka Panda catalog/configuration target for mock-control planning tests and future planner benchmarks.
- Fail clearly when a selected backend or kinematics solver calls an unsupported world capability.
- Preserve existing Drake/default manipulation behavior unless users opt into VAMP or migrated config fields.

**Non-Goals:**

- DimOS will not generate VAMP artifacts from URDF/SRDF/meshes.
- DimOS will not own a foam/cricket/fkcc_gen artifact-preparation pipeline in this change.
- VAMP will not expose synthetic Jacobian support.
- VAMP will not pretend to satisfy unsupported `WorldSpec` behavior just to match the Drake surface.
- VAMP planner selection will not imply pose-planning support.
- Real Franka Panda hardware control is not required in this change.
- No new runnable robot blueprint is required unless implementation later chooses to add a VAMP/Panda demo or benchmark blueprint.

## DimOS Architecture

### Configuration

Introduce typed nested planning config objects analogous to the Pink IK pattern:

```python
world={
    "backend": "vamp",
    "artifact": {
        "mode": "official",
        "robot": "panda",
    },
}

planner={
    "backend": "vamp",
    "algorithm": "rrtc",
    "simplify": True,
    "validate_path": True,
}
```

Custom artifact mode uses an explicit user-prepared path:

```python
world={
    "backend": "vamp",
    "artifact": {
        "mode": "custom",
        "path": "/path/to/user/prepared/artifact",
    },
}
```

VAMP-specific settings must be nested-only. Do not add fields such as `vamp_robot` or `vamp_artifact_path` directly to `ManipulationModuleConfig`.

Existing flat fields that are migrated, such as `planner_name`, `kinematics_name`, or `enable_viz`, may remain temporarily as compatibility shims. Those shims must emit a visible `DeprecationWarning` when used. The `Deprecated` package is appropriate for deprecated callable APIs, but config-field migration should use explicit warnings in config normalization/validation because config fields are data, not callables.

### World backend

Add a VAMP world backend that is responsible for:

- Loading official VAMP artifacts/modules by configured robot name.
- Loading custom user-prepared artifacts by explicit configured path.
- Holding VAMP-native robot/environment representation.
- Exposing native VAMP validity/FK/EE pose behavior where supported.
- Converting supported DimOS obstacle/environment inputs into VAMP environment data.
- Raising a clear unsupported-capability error for operations that are not natively available.

The VAMP world must not dynamically generate artifacts or invoke the VAMP artifact-generation toolchain. If a robot is not covered by official VAMP artifacts, the user owns generating/building the artifact and provides the path/config needed to load it.

### Planner backend

Add a VAMP planner backend that is responsible for:

- Joint-space planning only.
- Configurable algorithm selection (`rrtc`, `prm`, `fcit`, `aorrtc`) with default `rrtc`.
- Native path simplification when enabled and available.
- Native path validation when enabled and available.
- Backend-scoped lazy imports and actionable dependency errors.

`VampPlanner` must not call `WorldSpec.get_jacobian()`, solve IK, or own pose-to-joint conversion. Jacobian and IK are kinematics concerns, not planner concerns.

### Kinematics and pose planning

Pose planning remains a DimOS-level flow:

```text
target pose -> KinematicsSpec -> goal joint state -> PlannerSpec -> joint path
```

For VAMP, this flow is enabled only when the selected `KinematicsSpec` declares or demonstrates compatibility with the VAMP world surface. The initial VAMP scope is joint-space planning. A future FK-only or derivative-free IK backend could enable VAMP pose planning without requiring VAMP to manufacture Jacobian behavior.

If `kinematics={"backend": "jacobian"}` is paired with `world={"backend": "vamp"}` and the VAMP world does not support Jacobian, initialization or the first compatibility check should fail with an explicit incompatibility message rather than a generic attribute error.

### Factory and validation

Update planning factory wiring to dispatch from typed config objects rather than only string names. The factory should validate cross-backend combinations before planning:

- `VampWorldConfig` requires `VampPlannerConfig`.
- `VampPlannerConfig` requires `VampWorldConfig`.
- VAMP world/planner pairs are invalid with incompatible kinematics backends.
- Drake/default behavior remains valid without VAMP installed.

### Streams, blueprints, skills, and CLI

No new streams are required. Existing manipulation RPCs and skills continue to use the existing planning module surface. VAMP affects the planning implementation selected under the module, not the LCM/SHM/ROS/DDS transport topology.

Blueprints can opt into VAMP through nested config. CLI examples should use nested override paths, such as:

```bash
-o manipulationmodule.world.backend=vamp
-o manipulationmodule.world.artifact.mode=official
-o manipulationmodule.world.artifact.robot=panda
-o manipulationmodule.planner.backend=vamp
-o manipulationmodule.planner.algorithm=rrtc
```

If no new blueprints are added, `dimos/robot/all_blueprints.py` should not need regeneration. If implementation adds or renames blueprint variables, run `pytest dimos/robot/test_all_blueprints_generation.py`.

### Franka Panda mock-control support

Add Franka Panda support through the existing robot catalog and coordinator patterns, not as a VAMP-only special case. The shape should mirror `dimos.robot.catalog.ufactory` and `dimos.robot.catalog.piper`:

- Provide a catalog function such as `franka_panda(...) -> RobotConfig` in a Franka/Panda catalog module.
- Default `adapter_type` to `"mock"` so the `ControlCoordinator` uses the existing mock manipulator adapter unless a future real adapter is explicitly selected.
- Provide model/FK constants for the Panda robot model used by manipulation planning, cartesian IK tasks, tests, and benchmarks.
- Store Panda URDF/SRDF/model resources in a repository LFS-backed robot description package, following the existing `data/.lfs/<robot>_description.tar.gz` pattern used by catalog assets and referencing the extracted package through `LfsPath`.
- Configure the Panda `RobotConfig` with explicit arm joint names, base link, end-effector link, home joints, package paths, and collision exclusions if needed.
- Let `RobotConfig.to_hardware_component()` feed the coordinator hardware list and `RobotConfig.to_robot_model_config()` feed the `ManipulationModule`, matching the xArm/Piper pattern.

The initial Panda scope is mock control plus planning metadata. A real Panda hardware adapter may be introduced later behind the same `adapter_type` seam, but VAMP testing and benchmarking should not depend on real hardware availability.

## Decisions

### Q&A decision record

1. **VAMP is a full optional backend, not a generic planner-only plugin.** It owns a VAMP-native robot/environment representation and planner behavior when selected.
2. **DimOS will not dynamically generate VAMP artifacts.** Artifact generation is too large for this integration. Users either use official VAMP artifacts or provide a custom user-prepared artifact path.
3. **VAMP artifacts are runtime-loaded, not maintained as checked-in generated source by DimOS.** Official artifacts come from the VAMP distribution; custom artifacts come from the user.
4. **VAMP remains an optional dependency.** Adapter imports should be lazy and raise actionable installation errors when VAMP is selected but unavailable.
5. **`planner.backend="vamp"` selects the DimOS VAMP planner adapter.** The specific VAMP algorithm is a planner config option, not separate global planner names like `vamp_rrtc`.
6. **Config uses typed nested objects.** This follows PR #2481's Pink IK pattern and avoids new flat VAMP fields on `ManipulationModuleConfig`.
7. **VAMP world and planner must be strictly paired.** Mixed VAMP/Drake world-planner combinations are invalid unless a future bridge explicitly supports them.
8. **Pose planning is not VAMP planner behavior.** It remains a `KinematicsSpec` concern that converts pose goals to joint goals before planning.
9. **Do not expose synthetic Jacobian support.** VAMP has `fk`/`eefk` utilities but no public Jacobian API; the adapter should not invent one.
10. **Initial VAMP capability set is minimal and native.** Joint-space planning, algorithm selection, native validation/simplification, official/custom artifact loading, environment conversion, joint-configuration validity, joint limits when available, and native FK/EE pose where available.
11. **Pose planning with VAMP is conditional.** It is available only if an explicitly compatible kinematics backend is configured.
12. **Unsupported world capabilities fail clearly.** Backends may raise a dedicated unsupported-capability error rather than faking interface completeness.
13. **Legacy config compatibility must be noisy.** Existing flat migrated fields may be accepted temporarily but must emit deprecation warnings.
14. **Franka Panda support is mock-control first.** The Panda catalog target exists to test and benchmark planning flows without requiring physical Panda hardware.
15. **Panda robot descriptions follow the repo LFS pattern.** Panda URDF/SRDF/model assets should be checked in as LFS-backed data package contents and referenced via `LfsPath`, not fetched or generated at runtime.

### Alternatives considered

- **Generate VAMP artifacts in DimOS:** rejected as too complex for this change because it would require spherization, tracing/codegen, native compilation, and local cache semantics.
- **Expose VAMP as planner-only over arbitrary `WorldSpec`:** rejected because VAMP planning depends on VAMP-native robot/environment artifacts.
- **Add a numerical Jacobian wrapper over `eefk`:** rejected because it manufactures a capability not inherently supported by the planner/backend.
- **Add flat VAMP module fields:** rejected in favor of typed nested backend config, matching the Pink IK direction.
- **Require real Panda hardware support before benchmarking:** rejected because planning tests and benchmarks need a repeatable mock-control target first.
- **Fetch Panda URDF/SRDF dynamically during tests:** rejected because DimOS catalog robots should follow the existing LFS-backed description package pattern for reproducible offline tests.

## Safety / Simulation / Replay

VAMP changes planning, not control execution. Generated trajectories still flow through existing manipulation execution and coordinator surfaces. Safety depends on correct VAMP artifact loading, obstacle conversion, collision validity, and trajectory execution checks.

Hardware safety constraints:

- VAMP must fail closed on invalid artifact config, missing dependency, invalid backend pairing, or unsupported kinematics combination.
- VAMP should not silently skip collision/path validation when validation is configured.
- User-prepared artifacts must be treated as trusted local planning inputs; DimOS should validate presence/loadability but cannot prove the artifact accurately models the robot.

Simulation/replay:

- Non-VAMP replay and simulation stacks must not import or initialize VAMP.
- VAMP behavior should be testable with fake or official lightweight artifacts before hardware use.
- Franka Panda mock-control support should allow planner testing and benchmarking without commanding physical hardware.
- Manual QA should include a joint-space plan on an official VAMP robot artifact and verification that existing Drake planning still works without VAMP installed.

## Risks / Trade-offs

- **Artifact mismatch risk:** A user-prepared artifact may not match the physical robot or DimOS `RobotModelConfig`. Mitigate with explicit config, load-time validation, and documentation.
- **Protocol mismatch risk:** Existing `WorldSpec` includes methods VAMP should not implement. Mitigate with explicit unsupported-capability errors and compatibility validation.
- **Dependency risk:** VAMP is native/compiled. Mitigate with lazy imports and optional extras.
- **Config migration risk:** Moving toward nested config may disturb existing blueprints. Mitigate with deprecation warnings and tests covering legacy shims.
- **Environment conversion risk:** VAMP supports specific environment representations. Start with documented/supported obstacle conversions and fail clearly for unsupported obstacle types.
- **Panda model mismatch risk:** The DimOS Panda catalog model, mock coordinator joint names, and VAMP official Panda artifact may diverge. Mitigate with explicit joint-name/model validation in tests and documentation.
- **LFS asset drift risk:** The Panda URDF/SRDF package may diverge from catalog constants or benchmark expectations. Mitigate by resolving assets through `LfsPath` in tests and validating expected files/joints during catalog tests.

## Migration / Rollout

1. Add typed world and planner config objects with default Drake-compatible behavior.
2. Add temporary compatibility shims for existing flat fields that are being migrated, each with `DeprecationWarning`.
3. Add VAMP world/planner adapters behind lazy imports.
4. Add cross-config validation before planning begins.
5. Update docs with official/custom artifact loading examples and CLI override examples.
6. Add the LFS-backed Franka Panda robot description package with URDF/SRDF/model assets and catalog references through `LfsPath`.
7. Add Franka Panda mock-control catalog support and tests for coordinator/manipulation config generation.
8. Add tests before enabling any blueprint-level VAMP examples.
9. If new blueprints are introduced, regenerate and verify `dimos/robot/all_blueprints.py` with the registry test.

Rollback is straightforward if VAMP remains optional and default config remains Drake-compatible: remove the VAMP config/adapters and keep or revert the generic typed-config migration separately.

## Open Questions

- Which official VAMP robots should be documented as supported at first depends on the installed `vamp-planner` distribution used during implementation.
- The exact custom artifact loading shape depends on how user-prepared VAMP artifacts are importable from the Python binding or local path.
- The initial supported obstacle/environment conversion set should be confirmed against VAMP's current Python API during implementation.
- The exact Franka Panda LFS package name, internal URDF/SRDF paths, and package source should be selected during implementation, then validated against the official VAMP Panda artifact joint order.
