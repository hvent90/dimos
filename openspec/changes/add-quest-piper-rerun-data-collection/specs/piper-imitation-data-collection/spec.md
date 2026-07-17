## ADDED Requirements

### Requirement: Dedicated monitored Piper collection blueprint
DimOS SHALL expose a separate runnable Quest Piper collection blueprint that collects physical-robot demonstrations with an external RealSense RGB observation stream and Rerun monitoring. The blueprint SHALL not require the manipulation or Viser stacks.

#### Scenario: Start monitored hardware collection
- **GIVEN** a Piper, Quest teleoperation service, and RealSense camera are available
- **WHEN** an operator starts the dedicated collection blueprint
- **THEN** the system SHALL make the RealSense RGB stream available to both recording and the Rerun viewer
- **AND** the existing Quest-to-Piper teleoperation path SHALL remain available to the operator

#### Scenario: Existing collector compatibility
- **GIVEN** an operator uses the existing non-Rerun Piper collection blueprint
- **WHEN** this capability is added
- **THEN** its blueprint name, dependencies, and collection behavior SHALL remain unchanged

### Requirement: Controlled episode lifecycle
The collection workflow SHALL persist explicit episode lifecycle events and support one control that starts an idle episode and saves a recording episode, plus a distinct control that discards an active episode.

#### Scenario: Start and save a take
- **GIVEN** the collection workflow is idle
- **WHEN** the operator presses the configured toggle control
- **THEN** the system SHALL begin an episode and show recording state to the operator
- **WHEN** the operator presses the toggle control again while recording
- **THEN** the system SHALL save the episode and return to idle

#### Scenario: Discard an active take
- **GIVEN** an episode is recording
- **WHEN** the operator presses the configured discard control
- **THEN** the system SHALL end the active episode as discarded
- **AND** the raw recording SHALL remain available for audit while dataset conversion excludes that episode

#### Scenario: Discard outside an active take
- **GIVEN** collection is idle after a saved episode
- **WHEN** the operator presses the discard control
- **THEN** the system SHALL NOT retroactively discard the saved episode

### Requirement: Task-labeled LeRobot-ready demonstrations
The collection recorder SHALL accept a task-label configuration value and preserve it with saved episode metadata. Kept episodes SHALL be convertible to a 30 Hz LeRobot dataset containing RGB observations, continuous Piper arm-and-gripper joint observations, and next absolute-joint actions.

#### Scenario: Convert kept task-labeled episodes
- **GIVEN** a collection session has saved episodes and a configured task label
- **WHEN** the session is converted to LeRobot at 30 Hz
- **THEN** each emitted sample SHALL contain the RGB observation, Piper joint-state observation, and next absolute Piper joint-state action
- **AND** each emitted episode SHALL carry the configured task label

#### Scenario: Preserve raw timing for conversion
- **GIVEN** the collection streams arrive at their native rates
- **WHEN** the recorder persists the session
- **THEN** it SHALL preserve raw timestamps for offline synchronization
- **AND** the 30 Hz conversion SHALL omit samples that cannot satisfy its configured synchronization tolerance rather than fabricate an observation or action

### Requirement: Collection-state observability
The Rerun view SHALL provide live visibility of the RGB camera feed and collection lifecycle state without participating in teleoperation commands or recorder persistence.

#### Scenario: Observe collection without influencing it
- **GIVEN** monitored collection is running
- **WHEN** the camera publishes frames or an episode lifecycle event occurs
- **THEN** the Rerun view SHALL display the camera feed and current collection state or counts
- **AND** loss or closure of the viewer SHALL NOT stop teleoperation, alter commands, or prevent the recorder from persisting connected streams
