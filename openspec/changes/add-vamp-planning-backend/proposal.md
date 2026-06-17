## Why

DimOS manipulation currently has a Drake-centered planning stack with generic world, planner, and kinematics protocols. VAMP offers a fast optional motion-planning backend for supported robot artifacts, but it has a different capability model: it plans over VAMP-native robot/environment representations and exposes native validation/FK utilities rather than arbitrary runtime URDF ingestion or a full kinematics interface.

This change adds VAMP as an optional manipulation planning backend while preserving DimOS' backend boundaries. It should let users run joint-space planning through official VAMP robot artifacts or their own user-prepared artifacts, without making non-VAMP stacks pay dependency, import, or configuration costs.

The change also adds Franka Panda mock-control support so VAMP can be tested and benchmarked against an official VAMP robot without requiring real hardware.

## What Changes

- Add a VAMP world backend for loading official or user-prepared VAMP artifacts and representing VAMP-native planning state.
- Add a VAMP planner backend for joint-space planning with configurable VAMP algorithms such as `rrtc`, `prm`, `fcit`, and `aorrtc`.
- Add typed nested world/planner configuration for backend selection and VAMP-specific options, following the Pink IK configuration pattern.
- Require strict VAMP world/planner pairing during planning-stack initialization.
- Keep VAMP pose planning conditional on an explicitly compatible kinematics backend; joint-space VAMP planning must not imply IK or Jacobian support.
- Provide clear unsupported-capability failures when a backend-specific world operation is not natively supported.
- Keep VAMP optional and lazily imported with actionable dependency errors.
- Do not generate VAMP artifacts in DimOS; users must rely on official VAMP artifacts or provide a custom user-prepared artifact path.
- Add a Franka Panda robot catalog entry and mock-control coordinator path for planning tests and later planner benchmarking, with Panda URDF/SRDF assets stored through the existing LFS-backed robot description pattern.
- Add noisy deprecation warnings for migrated pre-existing flat configuration fields where compatibility shims remain.

## Affected DimOS Surfaces

- Modules/streams:
  - `ManipulationModule` planning-stack initialization and validation.
  - `WorldMonitor` world backend creation path.
  - Manipulation planning factory functions for world, planner, and config dispatch.
  - Planning world/planner/kinematics protocols where unsupported optional capabilities need clear failure behavior.
- Blueprints/CLI:
  - Manipulation blueprints may configure nested world/planner backend objects.
  - CLI override examples should use nested config paths such as `manipulationmodule.world.backend=vamp` and `manipulationmodule.planner.backend=vamp`.
- Skills/MCP:
  - No direct skill or MCP tool behavior changes are expected; existing manipulation RPCs/skills should fail clearly for unsupported pose-planning combinations.
- Hardware/simulation/replay:
  - VAMP is an optional planning backend for manipulation stacks.
  - Franka Panda support should default to the existing mock manipulator adapter pattern for control.
  - Franka Panda URDF/SRDF assets should live in an LFS-backed robot description package and be referenced through `LfsPath`, matching existing catalog assets such as xArm, Piper, A-750, and OpenArm descriptions.
  - A future real Panda hardware adapter may be added behind the same catalog/config seam, but real hardware control is not required for this change.
  - Runtime planning behavior changes only when VAMP is selected.
- Docs/generated registries:
  - Planning backend documentation and manipulation capability docs need updates.
  - No blueprint registry generation change is expected unless new runnable blueprints are added.

## Capabilities

### New Capabilities

- `vamp-planning-backend`: Behavior for optional VAMP world/planner configuration, artifact loading, joint-space planning, validation, and unsupported-capability handling.
- `manipulation-stack`: Behavior for typed manipulation planning backend configuration, compatibility validation, and legacy config migration.

### Modified Capabilities

- None.

## Impact

Users gain an optional VAMP planning path for fast joint-space manipulation planning when official or custom user-prepared VAMP artifacts are available. Developers gain a typed backend configuration pattern for world and planner options rather than adding VAMP-specific flat module fields. Test and benchmarking workflows gain a mock-control Franka Panda catalog target aligned with VAMP's official Panda artifact.

Compatibility risk is concentrated around migrating existing flat fields such as planner and visualization selection into nested config objects. Any retained compatibility path should emit a visible `DeprecationWarning` so maintainers know it is temporary. VAMP adds optional runtime dependencies only when selected; missing dependencies or invalid backend combinations should fail at planning-stack initialization with actionable messages.

Testing should cover config parsing, factory dispatch, lazy import errors, VAMP world/planner pairing validation, artifact-mode validation, joint-space planning adapter behavior with fakes/mocks, Franka Panda mock catalog/coordinator wiring, LFS-backed Panda URDF/SRDF asset resolution, unsupported kinematics combinations, and preservation of existing Drake/default behavior.
