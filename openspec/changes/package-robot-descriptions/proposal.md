## Why

DimOS currently loads several supported robot descriptions from Git LFS-backed archives through the generic data loader. That makes normal PyPI installs depend on git, Git LFS, and in some cases a cloned DimOS repository before built-in robots can be instantiated.

Robot descriptions for supported robots are small runtime assets and should ship directly with the DimOS Python package. External extensibility should remain simple: users provide filesystem paths to their own robot descriptions rather than asking DimOS to clone or manage external repositories.

## What Changes

- Built-in robot descriptions are packaged with DimOS and resolved from package-owned files in repo checkouts, editable installs, and PyPI wheel installs.
- Built-in robot-description catalogs stop using `LfsPath` and `get_data()` for URDF/mesh/SRDF assets.
- Robot-description LFS archives are removed once their runtime URDF/mesh/SRDF contents are package data.
- External robot descriptions remain path-only: callers provide normal `Path` values for model files and package roots.
- `get_data()` and `LfsPath` remain available for non-description large assets such as replay data, maps, ML model weights, recordings, and datasets.
- No runtime URDF rewriting or external repository loading is introduced.

## Affected DimOS Surfaces

- Modules/streams: manipulation and robot model parsing code that consumes `RobotConfig.model_path` and `package_paths`; no stream protocol changes.
- Blueprints/CLI: built-in robot/manipulator blueprints using catalog entries for xArm, Piper, A-750, OpenArm, and existing Unitree descriptions; no CLI command changes.
- Skills/MCP: no direct skill or MCP tool behavior changes.
- Hardware/simulation/replay: hardware robot-description loading changes for supported robots; replay data and simulator-only assets remain outside this migration unless required by robot-description parsing.
- Docs/generated registries: robot-description and custom-arm documentation must stop recommending LFS for descriptions; package-data configuration and MANIFEST rules must include built-in robot descriptions.

## Capabilities

### New Capabilities

- `robot-description-assets`: Covers built-in robot descriptions shipped with DimOS, path-only external robot descriptions, and the boundary between robot descriptions and LFS-backed large data.

### Modified Capabilities

- None.

## Impact

PyPI users can instantiate supported robot configurations without Git LFS or a cloned DimOS repository for robot descriptions. Developers get a clearer boundary: package resources for built-in robot descriptions, plain paths for external descriptions, and `get_data()`/`LfsPath` only for large data.

Compatibility risk is limited to code that directly expected `get_data("*_description")` or `LfsPath("*_description")` to be the blessed way to access built-in robot descriptions; internal code and docs will migrate away from that pattern without adding a robot-description fallback. Tests must verify package inclusion, parser loading, catalog path existence, and removal of description archives from LFS. Existing global large-file checks remain the size control mechanism, with reviewed exceptions only for necessary robot-description mesh files.
