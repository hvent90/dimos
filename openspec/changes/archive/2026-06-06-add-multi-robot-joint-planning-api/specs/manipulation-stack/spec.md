## ADDED Requirements

### Requirement: Timed motion plans

The manipulation stack SHALL treat a successful joint planning result as a time-parameterized motion plan that can be previewed and explicitly executed.

#### Scenario: Single robot plan includes timing
- **GIVEN** a manipulation planner with one configured robot and current joint state available
- **WHEN** a caller plans to a joint target for that robot
- **THEN** the successful result SHALL be stored as the active motion plan
- **AND** the active motion plan SHALL include a trajectory with `time_from_start` values and a total duration
- **AND** no hardware motion SHALL begin until the caller explicitly invokes execution

#### Scenario: Planned motion can be previewed before execution
- **GIVEN** a successful active motion plan
- **WHEN** a caller previews the plan
- **THEN** the stack SHALL visualize or otherwise preview the planned motion without sending motion commands to hardware
- **AND** the plan SHALL remain available for later explicit execution

### Requirement: Multi-robot joint planning through existing planning calls

The manipulation stack SHALL support coordinated multi-robot joint-space planning through the existing joint planning surface by accepting ordered multi-robot inputs.

#### Scenario: Plan two robots together
- **GIVEN** a manipulation planner with two configured robots and current joint state available for both
- **WHEN** a caller provides two robot names and two target joint states in matching order
- **THEN** the stack SHALL plan one coordinated motion for both robots
- **AND** the caller-provided robot order SHALL define the composite joint-vector order
- **AND** the resulting active motion plan SHALL include one trajectory per requested robot

#### Scenario: Preserve existing single-robot joint planning
- **GIVEN** an existing caller that provides one joint target and one robot name, or omits the robot name when only one robot is configured
- **WHEN** the caller invokes the joint planning API
- **THEN** the stack SHALL preserve the existing single-robot planning behavior
- **AND** the caller SHALL NOT be required to use a new multi-robot-only method

#### Scenario: Reject malformed multi-robot requests
- **GIVEN** a caller provides ordered multi-robot planning inputs
- **WHEN** the number of robot names and target joint states differs, a robot name is repeated, a robot is unknown, or a target has the wrong number of joints
- **THEN** the stack SHALL reject the request before sampling a plan
- **AND** the previously active motion plan SHALL NOT be partially replaced

### Requirement: Multi-robot pose planning matches existing pose semantics

The manipulation stack SHALL support multi-robot pose planning through the existing pose planning surface, while preserving the existing meaning of pose planning as target IK followed by joint-space planning.

#### Scenario: Plan multiple target poses
- **GIVEN** a manipulation planner with two configured robots and current joint state available for both
- **WHEN** a caller provides two robot names and two target end-effector poses in matching order
- **THEN** the stack SHALL solve a joint target for each requested pose
- **AND** it SHALL plan one coordinated joint-space motion to those joint targets
- **AND** the resulting active motion plan SHALL include synchronized timed trajectories for the requested robots

#### Scenario: Pose planning is not Cartesian path planning
- **GIVEN** a caller requests a pose plan for one or more robots
- **WHEN** the plan succeeds
- **THEN** the stack SHALL only guarantee a collision-checked joint-space motion to the solved pose target
- **AND** it SHALL NOT imply that the end-effector followed a Cartesian path between start and goal

### Requirement: Synchronized multi-robot timing

The manipulation stack SHALL time-parameterize coordinated multi-robot plans as one combined motion before exposing per-robot trajectories.

#### Scenario: Split trajectories share one clock
- **GIVEN** a successful coordinated plan for multiple robots
- **WHEN** the stack exposes per-robot trajectories for preview or execution
- **THEN** every per-robot trajectory SHALL have the same total duration
- **AND** corresponding trajectory points SHALL use the same `time_from_start` values
- **AND** each per-robot trajectory SHALL contain only that robot's joints

#### Scenario: Avoid independently timed dual-arm plans
- **GIVEN** a coordinated multi-robot planning request
- **WHEN** the stack generates executable trajectories
- **THEN** it SHALL NOT independently time-parameterize each robot path in a way that can produce different trajectory clocks for the same coordinated plan

### Requirement: Shared-context collision validation

The manipulation stack SHALL validate coordinated multi-robot candidate states and edges with all participating robots set in the same planning-world context.

#### Scenario: Detect inter-robot collision
- **GIVEN** a multi-robot target where each robot's target is individually valid
- **AND** the combined robot configuration causes an inter-robot collision or world collision
- **WHEN** the caller requests a coordinated plan
- **THEN** the stack SHALL reject colliding starts, goals, or intermediate edges
- **AND** it SHALL NOT return a successful coordinated motion plan for the colliding motion

#### Scenario: Preserve non-participating robot state during validation
- **GIVEN** a planning world with robots that are not included in a coordinated planning request
- **WHEN** candidate states for the participating robots are validated
- **THEN** non-participating robots SHALL remain at their current world state for collision validation

### Requirement: Explicit multi-robot preview and execution

The manipulation stack SHALL allow a successful coordinated motion plan to be previewed and explicitly executed for the requested robots without changing existing single-robot preview and execution behavior.

#### Scenario: Preview requested robots from active plan
- **GIVEN** a successful active motion plan for multiple robots
- **WHEN** a caller previews the same ordered robot set, or a subset of that set
- **THEN** the stack SHALL preview the corresponding planned robot trajectories without executing hardware motion

#### Scenario: Execute requested robots from active plan
- **GIVEN** a successful active motion plan for multiple robots
- **WHEN** a caller explicitly executes the same ordered robot set, or a subset of that set
- **THEN** the stack SHALL submit the corresponding timed trajectories to the existing execution surface for those robots
- **AND** each submitted trajectory SHALL preserve the synchronized timing from the active motion plan

#### Scenario: Preserve ambiguous default behavior
- **GIVEN** a manipulation planner with multiple configured robots
- **WHEN** a caller invokes preview or execution without specifying which robot or robot set to use
- **THEN** the stack SHALL NOT silently choose a robot set unless the existing single-robot default is unambiguous
- **AND** the caller SHALL be able to specify the robot or robot set explicitly

### Requirement: Backwards compatibility for single-robot callers

The manipulation stack SHALL preserve existing single-robot plan, preview, execute, and skill-wrapper behavior unless the caller opts into ordered multi-robot inputs.

#### Scenario: Existing plan-preview-execute flow remains valid
- **GIVEN** an existing single-robot caller using joint planning, preview, and execution
- **WHEN** the caller uses the same scalar inputs as before this change
- **THEN** the flow SHALL continue to plan, preview, and execute that robot's trajectory
- **AND** the caller SHALL NOT need to update to multi-robot input shapes

#### Scenario: Existing skill wrappers remain single-robot
- **GIVEN** an existing manipulation skill that moves one selected robot by pose or joint target
- **WHEN** the skill is invoked with scalar inputs
- **THEN** the skill SHALL continue to operate on one robot using the existing explicit `robot_name` behavior
- **AND** this change SHALL NOT expose a new multi-robot skill unless a later change specifies one

### Requirement: Mock dual-arm manual verification surface

The manipulation stack SHALL provide a no-hardware manual verification surface for coordinated dual-arm planning.

#### Scenario: Run coordinated planning through a mock dual-arm blueprint
- **GIVEN** the implementation includes the `dual-xarm6-mock-planner-coordinator` blueprint
- **WHEN** an operator runs that blueprint and connects the dual-arm planning REPL example
- **THEN** the operator SHALL be able to plan, preview, and explicitly execute a coordinated `left_arm` plus `right_arm` joint-space motion through the public manipulation RPC surface
- **AND** the preview step SHALL allow visual inspection of both planned arm trajectories before execution

#### Scenario: Manual verification includes a malformed request
- **GIVEN** the mock dual-arm blueprint is running and an active coordinated plan exists
- **WHEN** the operator sends a malformed multi-robot planning request through the REPL example
- **THEN** the request SHALL fail without commanding motion
- **AND** the previously active coordinated plan SHALL remain available for preview or explicit execution
