## ADDED Requirements

### Requirement: VAMP backend selection

DimOS MUST allow users to select VAMP as an optional manipulation planning backend through typed world and planner backend configuration.

#### Scenario: select VAMP stack
- **GIVEN** a manipulation module configuration with `world.backend` set to `vamp`
- **AND** `planner.backend` set to `vamp`
- **WHEN** the planning stack is initialized
- **THEN** DimOS SHALL initialize the VAMP planning path instead of the default non-VAMP planning path
- **AND** non-VAMP backends SHALL remain usable without VAMP being installed.

#### Scenario: reject mixed VAMP and non-VAMP stack
- **GIVEN** a manipulation module configuration with exactly one of the world or planner backends set to `vamp`
- **WHEN** the planning stack is initialized
- **THEN** DimOS MUST reject the configuration before planning begins
- **AND** the error message MUST identify the incompatible world/planner pairing.

### Requirement: VAMP artifact loading

DimOS MUST load VAMP robot artifacts from either official VAMP artifacts or a user-provided custom artifact path.

#### Scenario: load official artifact
- **GIVEN** `world.backend` is `vamp`
- **AND** the VAMP artifact mode is `official`
- **AND** an official robot artifact name is configured
- **WHEN** the planning stack is initialized
- **THEN** DimOS SHALL attempt to load the named official VAMP artifact
- **AND** initialization MUST fail clearly if the artifact is unavailable.

#### Scenario: load custom user-prepared artifact
- **GIVEN** `world.backend` is `vamp`
- **AND** the VAMP artifact mode is `custom`
- **AND** a custom artifact path is configured
- **WHEN** the planning stack is initialized
- **THEN** DimOS SHALL attempt to load the user-prepared artifact from that path
- **AND** initialization MUST fail clearly if the path is missing, invalid, or not loadable.

### Requirement: No artifact generation by DimOS

DimOS MUST treat VAMP artifact generation as outside the runtime planning backend.

#### Scenario: unsupported robot requires custom artifact
- **GIVEN** a robot that is not available as an official VAMP artifact
- **WHEN** a user selects VAMP planning for that robot
- **THEN** DimOS MUST require a loadable user-prepared custom artifact
- **AND** DimOS SHALL report that artifact generation must be performed outside DimOS when no loadable artifact is provided.

### Requirement: VAMP joint-space planning

The VAMP planner backend MUST support joint-space planning with a configured VAMP planning algorithm.

#### Scenario: plan between joint states
- **GIVEN** a valid VAMP world and planner configuration
- **AND** a start joint state and goal joint state for the configured robot
- **WHEN** joint-space planning is requested
- **THEN** DimOS SHALL invoke the configured VAMP planning algorithm
- **AND** the result MUST report either a collision-free joint path or a clear planning failure.

#### Scenario: configure planner algorithm
- **GIVEN** a valid VAMP planner configuration with an algorithm value such as `rrtc`, `prm`, `fcit`, or `aorrtc`
- **WHEN** the planning stack is initialized
- **THEN** DimOS SHALL use that algorithm for VAMP joint-space planning
- **AND** DimOS MUST reject unsupported algorithm values before planning begins.

### Requirement: VAMP native validation and simplification

DimOS MUST use VAMP-native validation and simplification behavior only when it is available and configured.

#### Scenario: validate planned path
- **GIVEN** VAMP path validation is enabled
- **AND** VAMP exposes validation for the planned path or sampled path states
- **WHEN** a VAMP plan is produced
- **THEN** DimOS SHALL validate the path before reporting success
- **AND** DimOS MUST report failure if validation detects an invalid path.

#### Scenario: unavailable validation capability
- **GIVEN** VAMP path validation is enabled
- **AND** the loaded VAMP artifact does not provide the needed validation capability
- **WHEN** the planning stack or plan result is validated
- **THEN** DimOS MUST fail clearly instead of silently skipping validation.

### Requirement: VAMP pose planning compatibility

VAMP pose planning MUST be available only when the configured kinematics backend is compatible with the VAMP world surface.

#### Scenario: incompatible kinematics backend
- **GIVEN** a VAMP world and planner configuration
- **AND** a configured kinematics backend requires a world capability that the VAMP world does not support
- **WHEN** the planning stack is initialized or pose planning compatibility is checked
- **THEN** DimOS MUST reject the pose-planning combination clearly
- **AND** joint-space VAMP planning SHALL remain the supported initial VAMP planning mode.

#### Scenario: compatible future kinematics backend
- **GIVEN** a VAMP world and planner configuration
- **AND** a kinematics backend that is compatible with the VAMP world surface
- **WHEN** pose planning is requested
- **THEN** DimOS SHALL convert the target pose to a goal joint state through the configured kinematics backend
- **AND** DimOS SHALL then plan the joint-space path with the configured VAMP planner.

### Requirement: Unsupported VAMP world capabilities

DimOS MUST provide a clear unsupported-capability failure when a VAMP world operation is not natively supported.

#### Scenario: unsupported Jacobian request
- **GIVEN** a VAMP world that does not provide Jacobian support
- **WHEN** a kinematics backend requests the end-effector Jacobian
- **THEN** DimOS MUST raise or surface a clear unsupported-capability error
- **AND** the error MUST identify the incompatible requested capability.

### Requirement: Optional VAMP dependency behavior

DimOS MUST keep VAMP dependency loading scoped to VAMP selection.

#### Scenario: default stack without VAMP installed
- **GIVEN** VAMP is not installed
- **AND** the user selects a non-VAMP planning stack
- **WHEN** the manipulation module starts
- **THEN** DimOS SHALL start without importing VAMP
- **AND** non-VAMP planning behavior SHALL remain available.

#### Scenario: VAMP selected without dependency
- **GIVEN** VAMP is not installed
- **AND** the user selects a VAMP planning stack
- **WHEN** the planning stack is initialized
- **THEN** DimOS MUST fail with an actionable dependency error
- **AND** the error SHOULD identify the optional dependency or installation path needed for VAMP planning.
