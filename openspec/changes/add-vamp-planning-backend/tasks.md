## 1. Configuration and validation

- [x] 1.1 Add typed nested manipulation world config with default Drake-compatible behavior and a `backend` discriminator.
- [x] 1.2 Add typed nested manipulation planner config with default RRT-compatible behavior and a `backend` discriminator.
- [x] 1.3 Add VAMP world config variants for official artifact loading and custom user-prepared artifact path loading.
- [x] 1.4 Add VAMP planner config for algorithm selection, path simplification, and path validation behavior.
- [x] 1.5 Update manipulation planning factory wiring to create world and planner backends from typed config objects.
- [x] 1.6 Add planning-stack compatibility validation for VAMP world/planner pairing and incompatible kinematics combinations.
- [x] 1.7 Add warning-backed compatibility shims for migrated pre-existing flat config fields, using `DeprecationWarning` with replacement guidance.
- [x] 1.8 Add tests that nested world/planner/kinematics config parses from dict/CLI override shapes and preserves current defaults.
- [x] 1.9 Add tests that legacy flat config shims preserve behavior and emit deprecation warnings.

## 2. VAMP backend implementation

- [x] 2.1 Add optional VAMP dependency wiring in packaging, keeping VAMP imports lazy and backend-scoped.
- [x] 2.2 Add VAMP-specific dependency error type or error path with actionable install guidance.
- [x] 2.3 Implement VAMP artifact loading for official robot artifacts exposed by the installed VAMP package.
- [x] 2.4 Implement VAMP artifact loading for user-prepared custom artifact paths.
- [x] 2.5 Implement the VAMP world adapter surface for native validity, FK/end-effector pose, joint limits when available, and supported environment conversion.
- [x] 2.6 Add a clear unsupported-world-capability error for operations the VAMP world does not natively support, including Jacobian requests.
- [x] 2.7 Implement the VAMP planner adapter for joint-space planning with configured algorithm selection.
- [x] 2.8 Implement configured VAMP path simplification and path validation only through VAMP-native capabilities.
- [x] 2.9 Ensure `VampPlanner` does not perform IK, pose conversion, or Jacobian probing.

## 3. Manipulation module integration

- [x] 3.1 Update `WorldMonitor`/planning initialization to pass typed world config and backend-specific options into world creation.
- [x] 3.2 Update `ManipulationModule` planning initialization to use typed world, planner, and kinematics config consistently.
- [x] 3.3 Ensure non-VAMP manipulation stacks do not import VAMP, load VAMP artifacts, or require VAMP dependencies.
- [x] 3.4 Ensure pose-planning entry points fail clearly when VAMP is selected with an incompatible kinematics backend.
- [x] 3.5 Preserve existing Drake/default planning behavior and existing blueprint behavior unless a blueprint explicitly opts into VAMP.

## 4. Franka Panda mock support

- [x] 4.1 Add a Franka/Panda robot catalog module with a `franka_panda(...) -> RobotConfig` helper and exported Panda model/FK constants.
- [x] 4.2 Add Panda URDF/SRDF/model resources through the existing repository LFS data package pattern, e.g. a `data/.lfs/<panda_description>.tar.gz` package resolved by `LfsPath`.
- [x] 4.3 Configure the Panda catalog helper with explicit arm joint names, base link, end-effector link, home joints, package paths, LFS-backed model/SRDF paths, and collision exclusions if required by the selected model.
- [x] 4.4 Keep Panda control mock by default through `adapter_type="mock"`; reserve any real Panda adapter selection for explicit future configuration.
- [x] 4.5 Add a mock-control Panda coordinator path for tests/benchmarks, using `RobotConfig.to_hardware_component()` and `RobotConfig.to_task_config()` in the same style as xArm/Piper.
- [x] 4.6 Ensure Panda manipulation planning setup uses `RobotConfig.to_robot_model_config()` and can pair with the official VAMP Panda artifact for joint-space tests.
- [x] 4.7 If a runnable Panda blueprint is added, add it through normal blueprint registration and regenerate `dimos/robot/all_blueprints.py` with the registry test.

## 5. Tests

- [x] 5.1 Add factory/config tests for VAMP world/planner creation, invalid pairings, invalid algorithms, and invalid artifact configs.
- [x] 5.2 Add lazy-import tests proving non-VAMP stacks do not require VAMP to be installed.
- [x] 5.3 Add dependency-error tests for selecting VAMP without the optional dependency installed.
- [x] 5.4 Add VAMP world adapter tests using fakes/mocks for official artifact loading, custom artifact loading, supported queries, and unsupported capability errors.
- [x] 5.5 Add VAMP planner adapter tests using fakes/mocks for algorithm dispatch, planning result conversion, simplification, validation, and planning failure reporting.
- [x] 5.6 Add manipulation module wiring tests for nested config propagation and VAMP kinematics compatibility validation.
- [x] 5.7 Add Franka Panda catalog tests covering default mock adapter selection, coordinator joint names, hardware component conversion, manipulation robot model conversion, and LFS-backed URDF/SRDF resolution.
- [x] 5.8 Add a Panda-backed VAMP joint-space planning test or benchmark harness using fakes/mocks where the optional VAMP dependency or official artifact is unavailable.
- [x] 5.9 Add regression tests showing existing Drake/default manipulation tests still pass without VAMP installed.

## 6. Documentation

- [x] 6.1 Update user-facing manipulation planning docs with VAMP backend overview, initial joint-space-only scope, official artifact config, and custom artifact path config.
- [x] 6.2 Document that DimOS does not generate VAMP artifacts and that users must prepare custom artifacts outside DimOS.
- [x] 6.3 Document nested CLI override examples for VAMP world artifact settings and VAMP planner algorithm settings.
- [x] 6.4 Document Franka Panda mock-control catalog usage for planning tests and planner benchmarks, including the LFS-backed URDF/SRDF/model asset location.
- [x] 6.5 Document failure modes for missing VAMP dependency, invalid world/planner pairing, invalid artifact config, unsupported world capabilities, incompatible kinematics, and Panda model/artifact mismatch.
- [x] 6.6 Update contributor planning-backend guidance with lazy-import, typed-config, deprecation-warning, unsupported-capability, and mock robot catalog expectations.

## 7. Verification

- [x] 7.1 Run `openspec validate add-vamp-planning-backend`.
- [x] 7.2 Run focused manipulation planning config/factory tests.
- [x] 7.3 Run focused VAMP world/planner adapter tests.
- [x] 7.4 Run focused manipulation module wiring tests.
- [x] 7.5 Run focused Franka Panda catalog/coordinator conversion tests.
- [x] 7.6 Run existing default/Drake manipulation tests to verify compatibility.
- [x] 7.7 Run docs validation commands for changed docs, including documented link/snippet validation where applicable.
- [x] 7.8 Manually QA a VAMP joint-space planning flow through the manipulation module user surface using an official Panda VAMP artifact or a fake artifact-backed Panda test harness.
- [x] 7.9 Manually QA that selecting an incompatible VAMP pose-planning stack fails clearly before executing robot motion.
- [x] 7.10 If new blueprints are added or renamed, run `pytest dimos/robot/test_all_blueprints_generation.py` and commit generated registry changes.
