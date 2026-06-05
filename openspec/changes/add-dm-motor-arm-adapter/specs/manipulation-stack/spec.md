## ADDED Requirements

### Requirement: Non-breaking DMMotor integration

DimOS SHALL add DMMotor adapter and gravity-compensation behavior without silently changing existing OpenArm adapter behavior.

#### Scenario: Existing OpenArm blueprints remain stable
- **GIVEN** an existing OpenArm blueprint selects the current OpenArm adapter
- **WHEN** this change is present
- **THEN** the blueprint SHALL continue to use the existing OpenArm adapter unless explicitly changed
- **AND** users SHALL be able to opt into the DMMotor adapter through a distinct adapter or blueprint selection.

### Requirement: Manipulation hardware bring-up path

DimOS SHALL support a staged bring-up path for DMMotor manipulators before normal trajectory execution.

#### Scenario: Staged validation before full arm use
- **GIVEN** a developer or operator is validating DMMotor hardware
- **WHEN** they follow the documented DimOS bring-up path
- **THEN** the path SHALL allow validation with mock or virtual CAN transport before real hardware
- **AND** the path SHALL separate state monitoring, gravity compensation, and trajectory execution into distinct operator-visible steps.

### Requirement: Blueprint registry visibility

DimOS SHALL make new DMMotor manipulation blueprints discoverable through the normal blueprint listing surface when runnable blueprints are added.

#### Scenario: Listing new blueprints
- **GIVEN** DMMotor runnable blueprints have been added
- **WHEN** a user runs the DimOS blueprint listing command
- **THEN** the new DMMotor blueprints SHALL appear with names that distinguish gravity compensation from trajectory-control operation
- **AND** generated blueprint registry files SHALL be kept current.
