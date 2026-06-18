## 1. Implementation

- [x] 1.1 Inventory supported built-in robot descriptions required by catalog/blueprint surfaces: Unitree Go2, Unitree G1, xArm, Piper, A-750, and OpenArm.
- [x] 1.2 Create `dimos/robot/descriptions/` and move or add runtime-ready URDF, referenced mesh, and SRDF assets for supported built-in robot descriptions.
- [x] 1.3 Add `dimos/robot/description_assets.py` with `robot_description_path(name: str) -> Path`, returning a normal existing `Path` for package-provided built-in descriptions in source, editable, and wheel installs.
- [x] 1.4 Update `pyproject.toml` package-data configuration and `MANIFEST.in` so the full `dimos/robot/descriptions/**` subtree is included in wheels and sdists.
- [x] 1.5 Add narrow `[tool.largefiles].ignore` exceptions only for necessary robot-description files that exceed the existing large-file threshold.
- [x] 1.6 Update xArm catalog entries to resolve built-in description paths with `robot_description_path("xarm_description")` while leaving simulator scene assets on their current data-loading path unless they are required robot-description files.
- [x] 1.7 Update Piper catalog entries to resolve built-in description paths with `robot_description_path("piper_description")` while leaving simulator scene assets on their current data-loading path unless they are required robot-description files.
- [x] 1.8 Update A-750 catalog entries to resolve built-in description paths with `robot_description_path("a750_description")`.
- [x] 1.9 Update OpenArm catalog entries to resolve built-in description paths with `robot_description_path("openarm_description")`.
- [x] 1.10 Remove built-in robot-description archives from `data/.lfs` after their contents are package-owned, including `xarm_description.tar.gz`, `piper_description.tar.gz`, `a750_description.tar.gz`, and `openarm_description.tar.gz`.
- [x] 1.11 Search for remaining `LfsPath` or `get_data()` usages tied only to built-in robot descriptions and replace them with package-owned paths or explicit external paths as appropriate.

## 2. Tests

- [x] 2.1 Add resolver tests proving `robot_description_path(name)` returns existing normal paths for each built-in description and raises a clear error for unknown names.
- [x] 2.2 Add catalog tests proving every supported built-in `RobotConfig.model_path` and every `package_paths` root exists without hydrating Git LFS.
- [x] 2.3 Add parser/model-loading tests for supported built-in robot descriptions, covering xArm, Piper, A-750, OpenArm, and existing Unitree descriptions where applicable.
- [x] 2.4 Add package-content tests or build-inspection tests proving `dimos/robot/descriptions/**` files are included in wheel/sdist artifacts.
- [x] 2.5 Add regression tests proving no built-in `*_description.tar.gz` archives remain under `data/.lfs` as robot-description runtime sources.
- [x] 2.6 Add tests or static checks proving built-in robot-description catalogs no longer use `LfsPath` or `get_data()` for URDF, referenced mesh, or SRDF assets.

## 3. Documentation

- [x] 3.1 Update `docs/capabilities/manipulation/adding_a_custom_arm.md` so custom robot descriptions use normal filesystem paths, not LFS-backed `LfsPath` values.
- [x] 3.2 Update `docs/capabilities/manipulation/openarm_integration.md` and related manipulation docs to state that supported built-in robot descriptions ship with DimOS.
- [x] 3.3 Update `docs/development/large_file_management.md` to reserve `get_data()`/`LfsPath` for non-description large assets and describe the `dimos/robot/descriptions/**` package-data boundary.
- [x] 3.4 Update `AGENTS.md` or `docs/coding-agents/` guidance if any instructions still point contributors at LFS for built-in URDF, mesh, or SRDF robot descriptions.
- [x] 3.5 Search user-facing and contributor docs for robot-description `LfsPath`/`get_data()` examples and update any remaining stale guidance.

## 4. Verification

- [x] 4.1 Run `openspec validate package-robot-descriptions`.
- [x] 4.2 Run focused pytest targets for the new resolver, catalog path checks, parser/model-loading checks, LFS archive regression checks, and package-content tests.
- [x] 4.3 Run `uv build --sdist --wheel` or the repo's focused package-build equivalent, then inspect the built artifacts for `dimos/robot/descriptions/**` contents.
- [x] 4.4 Run `python bin/hooks/largefiles_check` to verify only intended large-file exceptions are present.
- [x] 4.5 Run docs validation for changed docs, including `python -m dimos.utils.docs.doclinks docs/` if available in the current environment.
- [x] 4.6 Manually instantiate representative supported built-in robot catalog configs for xArm, Piper, A-750, OpenArm, and Unitree in a clean environment without Git LFS hydration.
- [x] 4.7 If blueprint names or generated registry inputs changed unexpectedly, run `pytest dimos/robot/test_all_blueprints_generation.py` and commit any generated registry update.
