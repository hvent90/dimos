## ADDED Requirements

### Requirement: Built-in robot descriptions are package-provided runtime assets

DimOS SHALL ship built-in robot descriptions for directly supported robots as Python package data, including the URDF files, meshes referenced by those URDFs, and SRDF files required to instantiate those supported robot models at runtime.

#### Scenario: Supported robot description resolves without repository data checkout
- **GIVEN** DimOS is installed from a Python package distribution
- **WHEN** a supported built-in robot configuration resolves its robot description
- **THEN** the returned model path and package paths MUST refer to files included with the installed DimOS package
- **AND** resolving the robot description MUST NOT require Git, Git LFS, or cloning the DimOS repository

#### Scenario: Supported robot description works from a development checkout
- **GIVEN** DimOS is used from a source checkout or editable install
- **WHEN** a supported built-in robot configuration resolves its robot description
- **THEN** the returned model path and package paths MUST exist as normal filesystem paths
- **AND** the same supported robot configuration MUST remain usable without hydrating LFS robot-description archives

#### Scenario: Robot description package contents include referenced assets
- **GIVEN** a built-in robot description included with DimOS references mesh or SRDF files needed at runtime
- **WHEN** DimOS package contents are built or inspected
- **THEN** those referenced files MUST be included with the package distribution
- **AND** parser or model-loading tests MUST fail if referenced robot-description files are missing

### Requirement: Built-in robot descriptions do not use LFS data loading

Built-in robot-description resolution SHALL NOT use DimOS LFS data-loading APIs for URDF, referenced mesh, or SRDF assets.

#### Scenario: Built-in catalog exposes normal paths
- **GIVEN** a supported built-in robot catalog entry
- **WHEN** code reads its model path and package paths
- **THEN** those values MUST be normal filesystem paths
- **AND** they MUST NOT depend on lazy `LfsPath` hydration for robot-description assets

#### Scenario: Robot-description archives are not retained as LFS runtime source
- **GIVEN** built-in robot-description files have moved into package data
- **WHEN** the repository's LFS archive area is inspected
- **THEN** robot-description archives for supported built-in descriptions MUST NOT remain there as the runtime source of truth
- **AND** unrelated large assets such as replay data, maps, recordings, model weights, and datasets MAY continue to use LFS data loading

### Requirement: External robot descriptions are path-only

DimOS SHALL support external robot descriptions by accepting caller-provided filesystem paths for model files and package roots, without introducing a DimOS-managed external repository loader.

#### Scenario: User supplies a custom robot description
- **GIVEN** a user has a custom robot description on the local filesystem
- **WHEN** the user configures DimOS with a model path and package paths for that description
- **THEN** DimOS MUST use those paths directly for parsing and model loading
- **AND** DimOS MUST NOT require the description to be registered as a Git repository or downloaded through a DimOS-managed repository mechanism

#### Scenario: Built-in and external descriptions use separate loading boundaries
- **GIVEN** a developer needs a supported built-in robot description and a user needs a custom external robot description
- **WHEN** each description is configured
- **THEN** the built-in description MUST resolve from DimOS package-provided files
- **AND** the external description MUST be represented by caller-provided paths
- **AND** neither case MUST require runtime mutation of URDF contents

### Requirement: Robot-description documentation matches the packaging boundary

DimOS documentation SHALL describe robot descriptions as package-provided for supported built-in robots and path-provided for external robots, while reserving LFS-backed data loading for non-description large assets.

#### Scenario: Custom robot documentation avoids LFS guidance
- **GIVEN** a user reads documentation for adding a custom robot or arm
- **WHEN** the documentation explains how to provide URDF, referenced mesh, or SRDF assets
- **THEN** it MUST instruct the user to provide normal filesystem paths
- **AND** it MUST NOT instruct the user to use `LfsPath` or `get_data()` for robot descriptions

#### Scenario: Large-data documentation preserves LFS scope
- **GIVEN** a developer reads documentation for large files and runtime data
- **WHEN** the documentation describes LFS-backed data loading
- **THEN** it MUST make clear that replay data, maps, recordings, model weights, datasets, and similar large assets remain valid LFS-backed data use cases
- **AND** built-in robot descriptions MUST be documented outside that LFS runtime-data workflow
