## ADDED Requirements

### Requirement: Pink is the default control IK backend
The system SHALL use generic Pink control IK by default for Cartesian and EEF-twist tasks created by `ControlCoordinator`. The typed backend configuration SHALL retain `backend="pinocchio"` as an explicit legacy compatibility option. The system SHALL NOT silently select Pinocchio when Pink is the configured backend or when Pink initialization fails.

#### Scenario: Shipped Cartesian task uses the default backend
- **GIVEN** a shipped Cartesian task without an explicit backend override
- **WHEN** the task is constructed
- **THEN** it SHALL construct the Pink control IK backend

#### Scenario: Legacy backend is explicitly selected
- **GIVEN** a Cartesian or EEF-twist task configured with `backend="pinocchio"`
- **WHEN** the task is constructed
- **THEN** it SHALL use the legacy PinocchioIK backend
- **AND** no implicit backend migration SHALL occur for that task

### Requirement: Model and named frame validation
The system SHALL prepare the configured URDF/Xacro model and validate the named end-effector frame, controlled-joint mapping, and model/task joint correspondence before control starts. Invalid or missing model, frame, or mapping configuration SHALL fail initialization with a diagnostic error.

#### Scenario: Valid Xacro model and named frame
- **GIVEN** a task with resolvable Xacro package paths and arguments, a valid URDF result, a named EEF frame, and matching mapped joints
- **WHEN** the Pink backend initializes
- **THEN** it SHALL construct the frame task from that model and frame

#### Scenario: Frame or joint mapping is invalid
- **GIVEN** a task whose named EEF frame or mapped controlled joint is absent from the prepared model
- **WHEN** the backend initializes
- **THEN** initialization SHALL fail with a diagnostic error

### Requirement: Measured-state one-step control
The Pink backend SHALL re-anchor its configuration to the coordinator's current measured joint state on every control tick. It SHALL update a named frame task, solve one differential-IK step, clamp the tick duration to the configured safe bounds, integrate the finite velocity once, and return a bounded joint-position candidate.

#### Scenario: Robot state lags the previous command
- **GIVEN** measured joints differ from the previously emitted command
- **WHEN** the next Cartesian or EEF-twist tick runs
- **THEN** the backend SHALL start from the measured joints
- **AND** EEF-twist target preparation SHALL use FK from those measured joints

#### Scenario: Tick duration exceeds the safe bound
- **GIVEN** a control tick with an elapsed duration outside the configured safe range
- **WHEN** one-step integration runs
- **THEN** the backend SHALL use the bounded duration
- **AND** it SHALL emit a finite bounded position candidate

### Requirement: Control limits and runtime failure behavior
The Pink control solve SHALL enforce configured joint-position and velocity limits. The shared task pipeline SHALL reject non-finite or unbounded output and SHALL produce a bounded hold for expected runtime solve errors rather than emitting an invalid command.

#### Scenario: Solver returns a valid limited step
- **GIVEN** a valid target and measured state
- **WHEN** Pink solves one step
- **THEN** the emitted position command SHALL respect joint and velocity limits
- **AND** the command SHALL pass finite-value and joint-delta safety checks

#### Scenario: Expected runtime solve error
- **GIVEN** Pink raises an expected runtime solve error during a control tick
- **WHEN** the task handles the backend result
- **THEN** it SHALL emit a bounded hold or equivalent safe command
- **AND** it SHALL NOT emit non-finite or unvalidated joint positions

### Requirement: Shared task and blueprint migration
Common Cartesian and EEF-twist task helpers SHALL expose the same backend configuration, default to Pink, and preserve independent coordinator routing and lifecycle behavior. All shipped Cartesian and EEF-twist task blueprints SHALL use that default without a Piper-specific backend exception. Piper Cartesian and EEF-twist tasks SHALL use the matching existing Xacro/URDF model and named `gripper_base` frame instead of the MJCF/numeric EEF path.

#### Scenario: Piper task is constructed
- **GIVEN** a shipped Piper Cartesian or EEF-twist blueprint without an explicit legacy override
- **WHEN** its task configuration is built
- **THEN** it SHALL select Pink
- **AND** it SHALL use the matching Xacro/URDF model and `gripper_base` frame

### Requirement: Control and planning remain separate
The control Pink backend SHALL provide local Cartesian differential IK only. It SHALL NOT claim, load, or enforce planning-world or dynamic-obstacle avoidance. Manipulation planning SHALL remain responsible for its separate `WorldSpec` and planning backend behavior.

#### Scenario: Planning knows about a world obstacle
- **GIVEN** an obstacle represented only in the planning world
- **WHEN** a control IK command is generated
- **THEN** control SHALL apply its configured kinematic and command-safety behavior only
- **AND** world-obstacle handling SHALL remain the responsibility of planning
