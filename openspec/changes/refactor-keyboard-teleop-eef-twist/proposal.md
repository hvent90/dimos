## Why

The current keyboard arm teleop module owns robot-derived state: it reads coordinator joint state, requires robot model configuration, computes forward kinematics, tracks an absolute end-effector pose, and publishes `PoseStamped` targets. That makes a keyboard input device responsible for robot-specific control state and creates extra model/FK wiring in blueprints that should already live inside the control coordinator task layer.

This change moves end-effector twist control into a coordinator task. Keyboard teleop will publish operator intent as a routed spatial EEF twist command, while the task owns FK, target-pose integration, IK, safety checks, and timeout behavior using coordinator state.

## What Changes

- Add support for routed spatial EEF twist commands using `TwistStamped` with `frame_id` identifying the target control task.
- Add an `EEFTwistTask` that consumes spatial EEF twist commands, integrates them into an internal target pose, solves IK, and outputs servo-position joint commands through normal coordinator arbitration.
- Refactor keyboard arm teleop so it no longer reads robot joint state or requires `model_path`, `ee_joint_id`, or robot joint names.
- Keep existing planar/base `twist_command` behavior unchanged; the dedicated EEF twist stream is a temporary compatibility path until twist routing semantics are cleaned up globally.
- Preserve existing Cartesian IK task behavior for absolute pose command users.

## Affected DimOS Surfaces

- Modules/streams: `KeyboardTeleopModule`, `ControlCoordinator`, new routed spatial EEF twist stream, new `EEFTwistTask` control task.
- Blueprints/CLI: manipulator keyboard teleop blueprints that currently wire keyboard teleop directly to `CartesianIKTask` with FK model parameters, including xArm, Piper, OpenArm, and A-750.
- Skills/MCP: None expected.
- Hardware/simulation/replay: Manipulator teleop on real and simulated arm stacks; safety behavior depends on timeout, command integration, and IK rejection limits.
- Docs/generated registries: Control-task registry manifest for the new task type; teleop/control docs or glossary updates if user-facing docs mention keyboard EEF teleop.

## Capabilities

### New Capabilities

- `spatial-eef-twist-teleop`: Routed end-effector twist teleoperation for manipulators through coordinator-owned control tasks.

### Modified Capabilities

- None.

## Impact

Keyboard arm teleop becomes simpler and less robot-specific, reducing duplicated FK/model wiring outside the coordinator. Developers get a clearer distinction between planar base twist and spatial EEF twist, though the initial dedicated coordinator stream is known technical debt. Testing should cover task routing, twist integration, timeout/stop behavior, keyboard command publication, current manipulator keyboard blueprint wiring, and existing planar/base twist behavior remaining unchanged.
