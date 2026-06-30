## ADDED Requirements

### Requirement: OpenArm Mini direct Feetech integration
The system SHALL implement OpenArm Mini teleoperation using the lower-level Feetech motor communication library rather than a LeRobot runtime dependency.

#### Scenario: OpenArm Mini adapter is imported without LeRobot
- **WHEN** the OpenArm Mini teleop adapter is imported in an environment without LeRobot installed
- **THEN** the import does not fail because of a missing LeRobot package

#### Scenario: Feetech dependency is missing
- **WHEN** OpenArm Mini teleop is started without the required Feetech dependency installed
- **THEN** the system fails with a clear message identifying the OpenArm Mini optional dependency needed to use the adapter

### Requirement: OpenArm Mini non-interactive runtime startup
The OpenArm Mini teleop adapter SHALL load calibration artifacts non-interactively during normal blueprint startup and fail fast when required calibration is missing or invalid.

#### Scenario: Calibration artifacts exist
- **WHEN** both side-specific OpenArm Mini calibration directories contain valid calibration artifacts
- **THEN** the OpenArm Mini teleop adapter loads them without prompting the user and proceeds with connection

#### Scenario: Calibration artifact is missing
- **WHEN** a required OpenArm Mini calibration artifact is missing during normal blueprint startup
- **THEN** the adapter fails fast with a clear error explaining how to run the manual calibration/demo script

### Requirement: OpenArm Mini side-specific calibration paths
The OpenArm Mini teleop adapter SHALL support side-specific calibration path configuration with DimOS state-directory defaults.

#### Scenario: Calibration paths are omitted
- **WHEN** `left_calibration_path` and `right_calibration_path` are not configured
- **THEN** the adapter resolves them to `STATE_DIR / "teleop" / "openarm_mini" / "left"` and `STATE_DIR / "teleop" / "openarm_mini" / "right"`

#### Scenario: Calibration paths are configured
- **WHEN** `left_calibration_path` or `right_calibration_path` is configured explicitly
- **THEN** the adapter uses the configured path for that side instead of the default DimOS state-directory path

### Requirement: OpenArm Mini to OpenArm joint command mapping
The OpenArm Mini teleop adapter SHALL convert OpenArm Mini leader readings into OpenArm follower `JointState` commands using OpenArm follower joint names and adapter-owned mapping rules.

#### Scenario: Leader joint readings are available
- **WHEN** calibrated OpenArm Mini leader joint readings are available and teleop authority is active
- **THEN** the adapter returns a `JointState` command envelope whose joint names match the target OpenArm follower joints

#### Scenario: OpenArm Mini transform rules are applied
- **WHEN** the adapter converts leader readings into follower commands
- **THEN** it applies the OpenArm Mini side-specific sign/order mapping, joint_6/joint_7 remap, gripper conversion, and OpenArm joint limits before returning the command

### Requirement: Manual OpenArm Mini calibration/demo script
The system SHALL provide a manual OpenArm Mini calibration/demo script for interactive leader setup outside normal teleop runtime.

#### Scenario: Calibration script is run
- **WHEN** the OpenArm Mini calibration/demo script is run by an operator
- **THEN** it connects only to the OpenArm Mini leader hardware, performs interactive setup/calibration, and writes side-specific calibration artifacts

#### Scenario: Calibration script is run near follower hardware
- **WHEN** the calibration/demo script is running
- **THEN** it does not connect to follower OpenArm hardware and does not start `ControlCoordinator`

### Requirement: OpenArm Mini teleop blueprint
The system SHALL provide an OpenArm Mini teleop blueprint that wires the generic teleop module to the existing OpenArm control coordinator through `joint_command`.

#### Scenario: Blueprint is listed
- **WHEN** DimOS blueprints are listed after the OpenArm Mini teleop blueprint is added and the generated registry is refreshed
- **THEN** the OpenArm Mini teleop blueprint appears with the expected blueprint name

#### Scenario: Blueprint is built
- **WHEN** the OpenArm Mini teleop blueprint is built
- **THEN** the teleop module's `joint_command` output is connected to the coordinator's `joint_command` input
