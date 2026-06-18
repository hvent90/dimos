## Context

DimOS currently has two robot-description loading patterns. Unitree Go2 and G1 URDFs already live under `dimos/robot/unitree/...` and are included as package data. Manipulator catalog entries for xArm, Piper, A-750, and OpenArm use `LfsPath("*_description")` to lazily hydrate Git LFS archives under top-level `data/.lfs/` before returning paths for `RobotConfig.model_path`, `package_paths`, and FK/teleop model constants.

The generic data loader is appropriate for large replay/model/dataset assets, but it is a poor fit for small built-in robot descriptions because PyPI users may need git, Git LFS, and a cloned DimOS repository before they can instantiate supported robots. Current custom-arm documentation also directs users toward LFS for URDF/xacro assets, which conflicts with the desired path-only external model.

Constraints from the exploration:
- "Robot description" means URDF files, meshes referenced by those URDFs, and SRDF files for now.
- Built-in robot descriptions must work from both development checkouts/editable installs and pip-installed wheels.
- External robot descriptions must be normal filesystem paths, not DimOS-managed external repositories.
- Existing stored descriptions are expected to already contain normalized path references; runtime loading should not rewrite URDF contents.
- The repo already has a global non-LFS large-file guard (`bin/hooks/largefiles_check`) with a 50KB default threshold.

## Goals / Non-Goals

**Goals:**
- Ship supported built-in robot descriptions directly in the DimOS Python package.
- Provide a small helper that returns normal `Path` objects for built-in robot-description roots in repo and wheel installs.
- Migrate built-in catalogs away from `LfsPath` and `get_data()` for robot descriptions.
- Preserve plain `Path`-based extensibility for external robot descriptions.
- Remove robot-description archives from LFS after package-owned descriptions are present.
- Update docs and tests so robot descriptions are no longer described as LFS-backed data.

**Non-Goals:**
- No external repository loader, clone workflow, or package manager for third-party robot descriptions.
- No runtime URDF rewriting, mesh URI rewriting, or automatic patching of vendor descriptions.
- No migration of replay data, maps, model weights, recordings, or datasets out of LFS.
- No simulator-scene migration unless a scene file is required as part of robot-description parsing.
- No new robot-description-specific size guard beyond the existing large-file hook.
- No compatibility promise that `get_data("*_description")` or `LfsPath("*_description")` remains the supported way to access built-in robot descriptions.

## DimOS Architecture

This change does not introduce new modules, streams, transports, RPC contracts, DimOS `Spec` Protocols, adapter Protocols, skills, MCP tools, or CLI commands. It changes static runtime asset resolution for robot catalog entries that feed existing robot configuration and manipulation planning paths.

The package layout should become:

```text
dimos/robot/
├─ description_assets.py
└─ descriptions/
   ├─ xarm_description/
   ├─ piper_description/
   ├─ a750_description/
   └─ openarm_description/
```

`dimos.robot.description_assets` should expose a built-in resolver:

```python
from pathlib import Path

def robot_description_path(name: str) -> Path:
    ...
```

The helper returns the root directory for a built-in robot description. It may use Python package-resource mechanisms internally so the same call works when DimOS is imported from a checkout, editable install, or wheel. Callers should only receive and pass around normal `Path` values.

Built-in catalog entries should use the helper:

```text
description_root = robot_description_path("openarm_description")
model_path = description_root / "urdf/robot/openarm_v10_left.urdf"
package_paths = {"openarm_description": description_root}
```

External robot descriptions remain caller-provided paths:

```text
model_path = Path("/path/to/foo_description/urdf/foo.urdf")
package_paths = {"foo_description": Path("/path/to/foo_description")}
```

Package-data configuration must include the full `dimos/robot/descriptions/**` subtree. The existing curated global extension allowlist can remain for other `dimos/**` data, but robot descriptions should be included as a scoped subtree because referenced mesh/material metadata can use varied extensions.

Generated blueprint registries are not expected to change unless moving constants changes import behavior. If blueprint names or locations are touched during implementation, regenerate with `pytest dimos/robot/test_all_blueprints_generation.py`.

## Decisions

1. **Package built-in robot descriptions under `dimos/robot/descriptions/`.**
   - Rationale: package-owned files ship naturally in wheels and are available in repo installs.
   - Alternative rejected: keep top-level `data/` plus LFS archives, because that preserves git/Git LFS runtime dependency for PyPI users.

2. **Expose `dimos.robot.description_assets.robot_description_path(name)` as the catalog-facing helper.**
   - Rationale: `dimos/robot/` currently uses namespace-style subdirectories, so a concrete module avoids relying on `__init__.py` exports while keeping the API robot-specific.
   - Alternative rejected: place this in `dimos.utils.data`, because that would blur the boundary between robot descriptions and large LFS-backed data.

3. **Return normal `Path` objects at all boundaries.**
   - Rationale: existing `RobotConfig`, parser, and package path APIs already work with paths, and external extensibility needs no new abstraction.
   - Alternative rejected: a custom path/protocol object for built-in descriptions, because it would leak package-resource concerns into robot catalogs and user configuration.

4. **Do not perform runtime rewriting.**
   - Rationale: stored robot descriptions should be canonical and runtime-ready; tests should catch broken references.
   - Alternative rejected: rewrite URDF or mesh paths at load time, because that adds hidden mutation and makes external descriptions harder to reason about.

5. **Remove robot-description LFS archives.**
   - Rationale: keeping duplicate LFS copies invites drift and makes old docs/patterns look supported.
   - Alternative rejected: leave archives for fallback compatibility, because the agreed boundary is to remove `LfsPath`/`get_data()` usage for descriptions.

6. **Use the existing large-file guard.**
   - Rationale: the repo already blocks non-LFS files above 50KB unless explicitly ignored. Necessary mesh-file exceptions can be reviewed in `[tool.largefiles].ignore`.
   - Alternative rejected: a separate robot-description size guard, because it is extra scope and overlaps existing policy.

## Safety / Simulation / Replay

Robot descriptions influence planning, kinematics, collision geometry, and teleop/IK model loading, so broken paths can prevent robots from starting or planning. This change should not alter robot kinematics, joint limits, collision semantics, stream data, skill behavior, or hardware commands.

Safety validation should focus on loading/parsing equivalence and path completeness:
- every built-in catalog `model_path` exists,
- every `package_paths` root exists,
- robot model parsing succeeds without Git LFS,
- known collision exclusions and per-side model choices remain unchanged.

Simulation and replay data remain on their current paths unless a file is part of a robot description as defined by URDF, referenced meshes, or SRDF. Simulator scene XML files such as `xarm6/scene.xml`, `xarm7/scene.xml`, or `piper/scene.xml` should not move in this change unless an implementation investigation proves they are required for robot-description runtime behavior.

Manual QA should instantiate representative built-in catalog configs for xArm, Piper, A-750, OpenArm, and Unitree where applicable. Hardware motion is not required for this packaging change, but model parsing/planning smoke tests should run before relying on hardware blueprints.

## Risks / Trade-offs

- **Wheel size increases.** Mitigation: scope included files to robot descriptions and rely on the existing large-file hook with explicit reviewed exceptions for required meshes.
- **Missing mesh/material files in wheels.** Mitigation: include the entire `dimos/robot/descriptions/**` subtree and add tests that build or inspect package contents.
- **Path resolution in installed wheels.** Mitigation: implement the helper with package-resource semantics internally while exposing only normal `Path`s.
- **Duplicate or stale assets during migration.** Mitigation: remove `*_description.tar.gz` archives from `data/.lfs` and add a regression check that no robot-description archives remain there.
- **Direct downstream use of `get_data("*_description")`.** Mitigation: document the new built-in helper and the external path-only boundary; do not preserve the old pattern as a supported API for descriptions.
- **Large mesh files blocked by pre-commit.** Mitigation: add narrow `[tool.largefiles].ignore` entries only for necessary robot-description files with review context.

## Migration / Rollout

1. Add `dimos/robot/descriptions/**` with built-in URDF, referenced mesh, and SRDF assets for supported robots.
2. Add `dimos/robot/description_assets.py` with `robot_description_path(name)`.
3. Update package-data and MANIFEST configuration to include the full robot-description subtree.
4. Update built-in robot catalog constants and `RobotConfig` defaults to use `robot_description_path(...)` instead of `LfsPath` or `get_data()` for descriptions.
5. Remove robot-description archives such as `xarm_description.tar.gz`, `piper_description.tar.gz`, `a750_description.tar.gz`, and `openarm_description.tar.gz` from `data/.lfs` once their contents are package-owned.
6. Update documentation that currently tells users to put URDF/xacro files under LFS or describes robot descriptions as LFS data.
7. Add tests for resolver behavior, catalog path existence, parser loading, package inclusion, and absence of description archives in LFS.
8. Run relevant packaging, robot catalog, parser, and doc validation commands.

Rollback is straightforward before release: revert catalog paths to `LfsPath` and restore LFS archives. After release, downstream users should migrate to built-in helper paths for supported robot descriptions or explicit external paths for custom descriptions.

## Open Questions

- None. The exploration resolved scope, terminology, loading boundaries, package location, compatibility expectations, and size-guard policy.
