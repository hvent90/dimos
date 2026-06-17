## ADDED Requirements

### Requirement: Typed manipulation planning backend configuration

DimOS MUST support typed nested configuration for manipulation world, planner, and kinematics backend selection.

#### Scenario: nested backend config selects planning components
- **GIVEN** a manipulation module configuration with nested `world`, `planner`, and `kinematics` backend objects
- **WHEN** the manipulation planning stack is initialized
- **THEN** DimOS SHALL use the configured backend discriminators to create the selected planning components
- **AND** backend-specific options SHALL be read from the matching nested backend config object.

#### Scenario: backend-specific options stay local
- **GIVEN** a backend-specific option for world loading or planner tuning
- **WHEN** the option is configured
- **THEN** DimOS MUST read world-loading options from the world config
- **AND** DimOS MUST read planner behavior options from the planner config.

### Requirement: Planning stack compatibility validation

DimOS MUST validate incompatible world, planner, and kinematics backend combinations before unsafe or unsupported planning proceeds.

#### Scenario: invalid backend combination
- **GIVEN** a manipulation planning configuration with incompatible world and planner backends
- **WHEN** the planning stack is initialized
- **THEN** DimOS MUST reject the configuration before executing a plan
- **AND** the error message MUST identify the incompatible backends.

#### Scenario: invalid kinematics combination
- **GIVEN** a manipulation planning configuration with a kinematics backend that requires unsupported world capabilities
- **WHEN** pose-planning compatibility is checked
- **THEN** DimOS MUST reject the pose-planning configuration clearly
- **AND** joint-space-only planning modes SHALL not be advertised as pose-planning support.

### Requirement: Legacy flat config migration warnings

DimOS MUST emit visible deprecation warnings when pre-existing flat manipulation planning config fields are accepted as temporary compatibility shims.

#### Scenario: legacy planner field used
- **GIVEN** a user configures a pre-existing flat planner-selection field that has a nested replacement
- **WHEN** the config is normalized or the planning stack is initialized
- **THEN** DimOS SHALL preserve the legacy behavior for the compatibility period
- **AND** DimOS MUST emit a `DeprecationWarning` that identifies the nested replacement.

#### Scenario: new backend settings avoid flat fields
- **GIVEN** a new backend-specific setting is introduced for manipulation planning
- **WHEN** the setting is exposed to users
- **THEN** DimOS MUST expose it through the typed nested backend configuration
- **AND** DimOS SHALL avoid introducing new flat module-level backend fields.

### Requirement: Default manipulation planning compatibility

DimOS MUST preserve the default manipulation planning behavior for users who do not opt into new backend config.

#### Scenario: existing default config
- **GIVEN** a manipulation module configuration that relies on current defaults
- **WHEN** the manipulation module starts
- **THEN** DimOS SHALL initialize the default world, planner, and kinematics behavior
- **AND** optional backend dependencies SHALL not be required.

#### Scenario: existing non-VAMP blueprint
- **GIVEN** a manipulation blueprint that does not select VAMP
- **WHEN** the blueprint is run
- **THEN** DimOS SHALL not perform VAMP artifact loading
- **AND** DimOS SHALL not require VAMP-specific dependencies.

### Requirement: Franka Panda mock-control catalog support

DimOS MUST provide Franka Panda catalog support that can be used for manipulation planning tests and planner benchmarks without requiring physical Panda hardware.

#### Scenario: Panda catalog creates mock hardware config
- **GIVEN** a user or test constructs the Franka Panda catalog configuration with default control settings
- **WHEN** the configuration is converted to a `HardwareComponent`
- **THEN** DimOS SHALL configure a manipulator hardware component using the mock adapter
- **AND** the component SHALL expose the Panda arm joints expected by the coordinator.

#### Scenario: Panda catalog creates manipulation model config
- **GIVEN** a user or test constructs the Franka Panda catalog configuration
- **WHEN** the configuration is converted to a manipulation robot model config
- **THEN** DimOS SHALL provide the Panda model path, base link, end-effector link, joint names, package paths, and home joints needed by the manipulation planning stack.

#### Scenario: Panda model assets resolve from LFS-backed data
- **GIVEN** the Franka Panda catalog configuration references Panda URDF/SRDF resources
- **WHEN** tests or blueprints resolve those resources
- **THEN** DimOS SHALL resolve them from an LFS-backed robot description package through `LfsPath`
- **AND** DimOS SHALL avoid downloading or generating Panda robot descriptions at runtime.

#### Scenario: Panda target supports VAMP planning tests
- **GIVEN** VAMP is configured with an official Panda artifact and the DimOS Panda catalog robot model
- **WHEN** a joint-space planning test or benchmark initializes the manipulation stack
- **THEN** DimOS SHALL support running the flow through mock control surfaces
- **AND** DimOS SHALL not require a real Panda hardware adapter.
