## ADDED Requirements

### Requirement: Gravity-compensation-only operation

DimOS SHALL provide adapter-level gravity-compensation behavior for supported manipulators that compensates gravity without requiring a separate gravity-compensation module.

#### Scenario: Starting gravity compensation
- **GIVEN** a supported DMMotor arm is configured with `gravity_comp=True`
- **WHEN** the adapter receives a supported position, effort, or gravity-only command
- **THEN** DimOS SHALL apply model-based gravity feed-forward torque for the current joint configuration
- **AND** DimOS SHALL keep gravity compensation in the adapter command path rather than a standalone module.

#### Scenario: Joints remain free to move
- **GIVEN** the adapter gravity-only helper is active
- **WHEN** a person or external force moves a joint within the safe operating range
- **THEN** the arm SHALL remain manually movable
- **AND** DimOS SHALL send zero position stiffness with configurable low/no damping and gravity feed-forward from the current measured joint state.

### Requirement: Gravity-compensation safety

Adapter gravity-compensation operation SHALL make hardware safety expectations explicit and fail safe on shutdown.

#### Scenario: Stopping gravity compensation
- **GIVEN** adapter gravity compensation is enabled
- **WHEN** the adapter is stopped, disconnected, or disabled
- **THEN** DimOS SHALL disable or otherwise stop commanding the arm through the hardware binding
- **AND** the operator-visible result SHALL be a safe shutdown rather than a continued background control loop.

#### Scenario: Invalid or stale state
- **GIVEN** gravity compensation requires current joint state
- **WHEN** DimOS cannot obtain valid or fresh joint state from the hardware binding
- **THEN** DimOS SHALL avoid sending a gravity-compensation command based on invalid state
- **AND** DimOS SHALL surface the condition as a fault, warning, or stopped state for operator action.

### Requirement: Gravity-compensation configuration

DimOS SHALL expose gravity compensation through adapter configuration rather than a separate runnable gravity-compensation blueprint.

#### Scenario: Disabling adapter gravity compensation
- **GIVEN** a supported DMMotor arm is configured with `gravity_comp=False`
- **WHEN** the adapter receives position commands
- **THEN** DimOS SHALL send MIT position commands using configured `kp/kd` gains
- **AND** DimOS SHALL not add model gravity feed-forward torque to those commands.
