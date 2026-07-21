## ADDED Requirements

### Requirement: Visual mode is the default
The Viser manipulation view SHALL display the primary robot's visual geometry by default for every new visualization session.

#### Scenario: New visualization session
- **GIVEN** a new Viser manipulation visualization session is opened
- **WHEN** the primary robot is first displayed
- **THEN** its visual geometry is rendered
- **AND** its collision geometry is not rendered

### Requirement: Robot display mode is accessible
The Viser sidebar SHALL provide a keyboard-accessible, text-labelled three-mode control named `Robot display` with the options `Visual`, `Collision`, and `Both`.

#### Scenario: Select a display mode
- **GIVEN** the `Robot display` control is visible
- **WHEN** an operator focuses the control and chooses one of its labelled options using keyboard or pointer input
- **THEN** the chosen option becomes the active display mode

### Requirement: Display mode switches immediately for the whole robot
The Viser manipulation view MUST apply a changed display mode immediately and uniformly to all links of the primary robot, without stopping or altering ongoing joint-state updates.

#### Scenario: Change mode while joints update
- **GIVEN** joint states for the primary robot are being updated
- **WHEN** the operator changes the display mode
- **THEN** the primary robot's displayed geometry changes immediately to match the selected mode
- **AND** subsequent joint updates continue to update the robot in the selected mode

### Requirement: Visual and collision geometry render according to the selected mode
The Viser manipulation view SHALL render visual geometry in `Visual`, collision geometry in `Collision`, and both representations in `Both`. Collision geometry MUST use diagnostic magenta `#D228DC` at 35% opacity whenever it is rendered.

#### Scenario: Render each available representation
- **GIVEN** the primary robot has visual and collision geometry
- **WHEN** the active mode is `Visual`
- **THEN** only the visual geometry is rendered
- **WHEN** the active mode is `Collision`
- **THEN** only the collision geometry is rendered in `#D228DC` at 35% opacity
- **WHEN** the active mode is `Both`
- **THEN** the visual geometry and collision geometry are rendered together
- **AND** the collision geometry uses `#D228DC` at 35% opacity

### Requirement: Display mode persists for the current session
The Viser manipulation view SHALL retain the selected display mode when the primary robot representation is recreated during the current visualization session.

#### Scenario: Recreate the primary robot
- **GIVEN** the operator selected `Collision` or `Both`
- **WHEN** the primary robot representation is recreated
- **THEN** the recreated primary robot is rendered using the selected mode
- **AND** the selection remains active in the `Robot display` control

### Requirement: Display mode excludes target and preview representations
The Viser manipulation view MUST apply the `Robot display` mode only to the primary robot and SHALL NOT change target or preview-ghost rendering.

#### Scenario: Change the primary robot display
- **GIVEN** target or preview representations are present
- **WHEN** the operator changes the `Robot display` mode
- **THEN** only the primary robot's geometry visibility and collision appearance change
- **AND** target and preview-ghost representations retain their existing rendering

### Requirement: Missing collision geometry falls back to visual geometry
When primary collision geometry is unavailable, the Viser manipulation view SHALL retain the selected display mode and gracefully render the available visual geometry. In `Collision` and `Both`, it SHALL apply the diagnostic magenta `#D228DC` at 35% opacity to the substituted visual geometry so the selected mode remains obvious, and SHALL clearly tell the user that collision geometry is unavailable and visual meshes remain shown. It SHALL NOT show this notice in `Visual` or when primary collision geometry exists.

#### Scenario: Select collision mode without collision geometry
- **GIVEN** the primary robot has visual geometry but no collision geometry
- **WHEN** the operator selects `Collision` or `Both`
- **THEN** the available visual geometry remains rendered
- **AND** the available visual geometry uses diagnostic magenta `#D228DC` at
  35% opacity so the selected mode remains obvious
- **AND** the selected mode remains active
- **AND** the user is clearly told that collision geometry is unavailable and
  visual meshes remain shown

#### Scenario: Missing collision geometry in visual mode
- **GIVEN** the primary robot has visual geometry but no collision geometry
- **WHEN** the active mode is `Visual`
- **THEN** the visual geometry remains rendered
- **AND** no missing-collision notice is shown

#### Scenario: Missing collision geometry in both mode
- **GIVEN** the primary robot has visual geometry but no collision geometry
- **WHEN** the active mode is `Both`
- **THEN** the available visual geometry remains rendered with diagnostic
  magenta `#D228DC` at 35% opacity
- **AND** the user is clearly told that collision geometry is unavailable and
  visual meshes remain shown

#### Scenario: Collision geometry exists
- **GIVEN** the primary robot has collision geometry
- **WHEN** the active mode is `Collision` or `Both`
- **THEN** the selected geometry is rendered according to the active mode
- **AND** no missing-collision notice is shown

#### Scenario: Fallback notice scope
- **GIVEN** target or preview-ghost representations are present
- **WHEN** the primary robot has no collision geometry and the active mode is
  `Collision` or `Both`
- **THEN** the missing-collision notice describes only the primary robot
- **AND** target and preview-ghost rendering remains unchanged

### Requirement: Robot display is view-only
The `Robot display` control MUST NOT affect robot commands, planning inputs, collision-checking semantics, execution, or the live robot context.

#### Scenario: Change display while operating
- **GIVEN** a robot is connected or a simulation or replay is running
- **WHEN** the operator changes the `Robot display` mode
- **THEN** only Viser scene visibility and collision-geometry appearance change
- **AND** robot commands, planning, collision checking, and execution retain their existing behavior
