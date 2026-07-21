## ADDED Requirements

### Requirement: Optionally visualize the accepted manipulation planning world
The manipulation planning stack SHALL support an optional, read-only visualization of obstacles in the planning world. `WorldMonitor` SHALL provide explicit coordinated add/remove helpers that call `WorldSpec` first and then the optional Viser visualizer only after a successful world mutation. `WorldObstacleMonitor` SHALL use those helpers. When enabled, the visualization SHALL mirror each successfully accepted obstacle addition and removal exactly once, and SHALL NOT forward a failed world mutation. No native-world hook is part of this capability.

#### Scenario: Accepted obstacle addition is visualized once
- **GIVEN** obstacle visualization is enabled
- **WHEN** the planning world successfully accepts an obstacle addition
- **THEN** one corresponding obstacle representation is added to the visualization
- **AND** the visualization does not alter the planning-world result

#### Scenario: Rejected obstacle addition is not visualized
- **GIVEN** obstacle visualization is enabled
- **WHEN** the planning world rejects an obstacle addition
- **THEN** no representation for that rejected addition is added to the visualization

#### Scenario: Accepted obstacle removal is visualized once
- **GIVEN** obstacle visualization is enabled and an obstacle is represented in the visualization
- **WHEN** the planning world successfully accepts removal of that obstacle
- **THEN** the corresponding representation is removed exactly once

#### Scenario: Rejected obstacle removal is not visualized
- **GIVEN** obstacle visualization is enabled and an obstacle remains in the planning world
- **WHEN** the planning world rejects its removal
- **THEN** its visualization representation remains unchanged

### Requirement: Disabled visualization is a no-op
Visualization SHALL be disabled by default unless explicitly enabled. When disabled, obstacle additions and removals SHALL have no visualization effect and SHALL retain the existing planning and actuation behavior.

#### Scenario: Existing behavior is preserved when disabled
- **GIVEN** obstacle visualization is disabled
- **WHEN** the planning world adds or removes obstacles
- **THEN** no visualization is initialized or updated
- **AND** the planner receives the same obstacle mutations and outcomes as without this capability
- **AND** robot actuation semantics are unchanged

### Requirement: Initialize enabled visualization before planning-world mutations
When visualization is enabled, the visualization SHALL be ready before the floor or any obstacle mutation is applied to the planning world.

#### Scenario: Floor is visible after startup initialization
- **GIVEN** obstacle visualization is enabled
- **WHEN** the manipulation planning stack starts
- **THEN** visualization is initialized before the floor is added to the planning world
- **AND** the floor is represented in the visualization when the initial world is ready

#### Scenario: First obstacle cannot race initialization
- **GIVEN** obstacle visualization is enabled and the planning world has not yet received an obstacle mutation
- **WHEN** the first obstacle addition is accepted
- **THEN** its representation is available without requiring a later retry or refresh

### Requirement: Preserve planner geometry and appearance semantics
The visualization SHALL provide planner-parity representations for box, sphere, cylinder, and mesh obstacles. Valid obstacle RGBA appearance supplied by the planning world SHALL be preserved; when RGBA is absent or unusable, the visualization SHALL use a consistent fallback appearance without rejecting or changing the planning-world mutation.

#### Scenario: Primitive obstacle parity
- **GIVEN** visualization is enabled and the planning world accepts a box, sphere, or cylinder obstacle
- **WHEN** the obstacle is visualized
- **THEN** its shape, pose, dimensions, and valid RGBA appearance match the accepted planning-world obstacle

#### Scenario: Mesh obstacle parity
- **GIVEN** visualization is enabled and the planning world accepts a mesh obstacle
- **WHEN** the mesh can be rendered
- **THEN** the visualization represents the mesh with the accepted mesh geometry, pose, and valid RGBA appearance

#### Scenario: Appearance fallback does not affect planning
- **GIVEN** an accepted obstacle has absent or unusable RGBA appearance data
- **WHEN** the obstacle is visualized
- **THEN** a consistent fallback appearance is used
- **AND** the obstacle remains accepted with its original planning semantics

### Requirement: Expose persistent local obstacle visibility control
The visualization SHALL expose exactly one local visibility toggle named `manipulation.obstacles`. The toggle SHALL be visible by default, and hiding or showing it SHALL change only visibility while preserving the obstacle representations and their render state.

#### Scenario: Obstacles are visible by default
- **GIVEN** visualization is enabled and the visualization is ready
- **WHEN** the operator first views the local controls
- **THEN** `manipulation.obstacles` exists and is enabled
- **AND** obstacle representations are visible

#### Scenario: Visibility state is preserved across updates
- **GIVEN** the operator has hidden `manipulation.obstacles`
- **WHEN** obstacles are added or removed through successful planning-world mutations
- **THEN** the toggle remains hidden
- **AND** newly added obstacle representations follow the hidden state
- **AND** hiding does not discard existing representations or their render state

### Requirement: Make mesh rendering failures observable
If an accepted mesh obstacle cannot be rendered, the visualization SHALL retain a local proxy representation and a user-visible failure label. A mesh rendering failure SHALL NOT remove or invalidate the accepted planning-world obstacle.

#### Scenario: Failed mesh keeps a labeled proxy
- **GIVEN** visualization is enabled and the planning world accepts a mesh obstacle
- **WHEN** rendering that mesh fails
- **THEN** a local proxy remains at the obstacle's accepted pose
- **AND** the proxy is labeled to indicate mesh rendering failure
- **AND** the accepted obstacle remains available to the planner

### Requirement: Keep the capability read-only and limited to add/remove
The visualization SHALL observe planning-world obstacle additions and removals only. Pose updates are explicitly out of scope. It SHALL NOT change planner decisions, obstacle acceptance, obstacle geometry, robot commands, actuator behavior, or introduce synchronization for mutation types outside add/remove.

#### Scenario: Visualization cannot change planner or robot outcomes
- **GIVEN** visualization is enabled
- **WHEN** an obstacle is added or removed
- **THEN** planner acceptance and planning behavior remain authoritative over the visualization
- **AND** no robot actuation is caused by, or changed by, visualization

#### Scenario: Out-of-scope mutation has no visualization contract
- **GIVEN** visualization is enabled
- **WHEN** a planning-world mutation other than obstacle add or obstacle remove occurs
- **THEN** this capability does not synchronize that mutation
