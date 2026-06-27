## ADDED Requirements

### Requirement: First-class trajectory parametrization spec
The system SHALL provide a `TrajectoryParametrizerSpec` manipulation-planning role that converts a successful geometric `GeneratedPlan` into a time-parametrized `GeneratedTrajectory`.

#### Scenario: Parametrizer consumes geometric plan
- **WHEN** geometric planning succeeds and produces a `GeneratedPlan`
- **THEN** trajectory parametrization MUST consume the geometric path without overwriting the planning result

#### Scenario: Parametrizer remains independent of coordinator wiring
- **WHEN** a trajectory parametrization backend is implemented
- **THEN** it MUST NOT depend on `ControlCoordinator` task names, task instances, or task-specific dispatch wiring

### Requirement: Generated trajectory artifact
The system SHALL define `GeneratedTrajectory` as the canonical global time-parametrized manipulation artifact produced from a `GeneratedPlan`.

#### Scenario: Generated trajectory is global
- **WHEN** a `GeneratedTrajectory` is produced
- **THEN** it MUST represent the global selected joint trajectory rather than task-specific or robot-local command messages

#### Scenario: Generated trajectory has explicit status
- **WHEN** trajectory parametrization succeeds or fails
- **THEN** `GeneratedTrajectory` MUST report an explicit parametrization status and message without changing `GeneratedPlan.status`

### Requirement: Shared trajectory time domain
A `GeneratedTrajectory` produced from a composite or multi-robot `GeneratedPlan` SHALL preserve one shared time domain across all selected joints.

#### Scenario: Composite trajectory is parametrized
- **WHEN** a composite or multi-robot `GeneratedPlan` is parametrized
- **THEN** all selected joints in the resulting `GeneratedTrajectory` MUST use the same timing basis

#### Scenario: Dispatch derives local messages
- **WHEN** task-specific or robot-local command messages are derived from a composite `GeneratedTrajectory`
- **THEN** those messages MUST preserve the generated trajectory's shared timing basis

### Requirement: Trajectory dispatch boundary
The system SHALL define a `TrajectoryDispatch` execution-preparation artifact that derives control-task-specific `JointTrajectory` messages from a `GeneratedTrajectory`.

#### Scenario: Dispatch prepares coordinator messages
- **WHEN** execution is requested for a successful `GeneratedTrajectory`
- **THEN** manipulation orchestration MUST derive the required task-specific `JointTrajectory` messages through dispatch before invoking control tasks

#### Scenario: Dispatch is separate from parametrization
- **WHEN** a `GeneratedTrajectory` is produced
- **THEN** execution-specific projections MUST be produced by a separate dispatch/preparation step rather than stored as canonical generated trajectory data

### Requirement: Shared generated trajectory for preview, validation, benchmarking, and execution
Preview, validation, benchmarking, and execution dispatch SHALL consume the same `GeneratedTrajectory` artifact for a planned manipulation motion.

#### Scenario: Preview uses generated trajectory timing
- **WHEN** a planned motion is previewed
- **THEN** preview MUST use the timing from `GeneratedTrajectory` rather than assigning an independent preview duration to the geometric `GeneratedPlan`

#### Scenario: Execution dispatch uses generated trajectory timing
- **WHEN** a planned motion is executed
- **THEN** execution dispatch MUST consume the same `GeneratedTrajectory` artifact used by preview, validation, or benchmarking for that motion

### Requirement: Parametrization backend policy
The system SHALL support backend-configurable trajectory parametrization policy with a default `simple_trapezoid` backend and an explicit opt-in TOPPRA backend.

#### Scenario: Default backend is available
- **WHEN** no trajectory parametrization backend is explicitly configured
- **THEN** the system MUST use a `simple_trapezoid` backend that preserves current baseline timing behavior behind the new spec boundary

#### Scenario: TOPPRA backend is configured and installed
- **WHEN** `backend="toppra"` is configured and TOPPRA is available
- **THEN** the system MUST use TOPPRA for joint velocity and acceleration constrained trajectory parametrization

#### Scenario: TOPPRA backend is configured but missing
- **WHEN** `backend="toppra"` is configured and TOPPRA cannot be imported during planning or parametrization initialization
- **THEN** initialization MUST fail clearly with guidance to install TOPPRA support through `dimos[manipulation-toppra]`

#### Scenario: TOPPRA policy is reproducible
- **WHEN** TOPPRA parametrization is configured for tests or benchmarks
- **THEN** gridpoint or discretization policy MUST be explicit enough for reproducible comparisons

### Requirement: TOPPRA packaging and repository tests
The system SHALL expose TOPPRA support through a `manipulation-toppra` optional extra, include that extra in `all`, and run TOPPRA parametrization tests unconditionally in the repository test environment.

#### Scenario: Runtime user omits TOPPRA extra
- **WHEN** a runtime installation omits `manipulation-toppra` and does not configure `backend="toppra"`
- **THEN** baseline manipulation trajectory parametrization MUST remain available through `simple_trapezoid`

#### Scenario: Repository tests run
- **WHEN** the repository test environment runs trajectory parametrization tests
- **THEN** TOPPRA tests MUST run without import-based skipping

### Requirement: Separate planning, parametrization, dispatch, and execution statuses
The system SHALL distinguish geometric planning status, trajectory parametrization status, dispatch preparation status, and runtime execution status.

#### Scenario: Parametrization is infeasible after planning succeeds
- **WHEN** geometric planning succeeds but trajectory constraints are infeasible
- **THEN** `GeneratedPlan.status` MUST remain successful and `GeneratedTrajectory.status` MUST report parametrization infeasibility

#### Scenario: Dispatch fails after parametrization succeeds
- **WHEN** trajectory parametrization succeeds but coordinator task dispatch fails
- **THEN** `GeneratedTrajectory.status` MUST remain successful and dispatch or execution status MUST report the dispatch failure
