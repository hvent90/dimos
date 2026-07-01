## ADDED Requirements

### Requirement: Keyboard arm teleop publishes routed spatial EEF twist intent

Keyboard arm teleop SHALL publish spatial EEF twist commands as routed `TwistStamped` messages for manipulator end-effector jogging.

#### Scenario: Movement keys produce spatial EEF twist commands

- GIVEN keyboard arm teleop is configured with a target EEF twist task name
- WHEN the operator holds a translation or rotation key
- THEN keyboard arm teleop publishes a `TwistStamped` command on the coordinator EEF twist command stream
- AND the command `frame_id` identifies the target EEF twist task
- AND the command contains nonzero spatial EEF twist components corresponding to the held key

#### Scenario: Keyboard teleop does not require robot-derived state

- GIVEN keyboard arm teleop is configured for manipulator EEF twist control
- WHEN the module starts
- THEN it does not require robot joint state before accepting keyboard input
- AND it does not require FK model path, EEF joint id, or robot joint names in its own configuration

#### Scenario: Stop input publishes a safe stop intent

- GIVEN keyboard arm teleop is actively publishing a nonzero spatial EEF twist command
- WHEN the operator releases all movement keys
- THEN keyboard arm teleop publishes a zero spatial EEF twist command for the same target task
- AND no stale nonzero keyboard command continues to be published for that target

### Requirement: Coordinator routes spatial EEF twist commands without changing planar base twist behavior

The control coordinator SHALL route spatial EEF twist commands by `TwistStamped.frame_id` while preserving existing planar/base twist command behavior.

#### Scenario: Routed spatial EEF twist reaches matching task

- GIVEN the coordinator has an EEF twist task named by a `TwistStamped.frame_id`
- WHEN a spatial EEF twist command arrives on the coordinator EEF twist command stream
- THEN the coordinator delivers the command to the matching EEF twist task
- AND other task routes are not invoked for that command

#### Scenario: Unmatched spatial EEF twist is safe

- GIVEN no coordinator task matches a spatial EEF twist command `frame_id`
- WHEN the command arrives
- THEN the coordinator does not command manipulator motion from that message
- AND existing active tasks continue according to their own safety and timeout behavior

#### Scenario: Existing planar base twist remains unchanged

- GIVEN an existing base or velocity task consumes planar base twist commands
- WHEN a `Twist` command arrives on the existing planar/base twist stream
- THEN the coordinator handles that command using the existing planar/base convention
- AND the command is not treated as a routed spatial EEF twist command

### Requirement: EEFTwistTask converts spatial EEF twist into safe servo-position joint commands

`EEFTwistTask` SHALL use coordinator-owned robot state to convert routed spatial EEF twist intent into manipulator joint commands through normal coordinator arbitration.

#### Scenario: First nonzero command seeds from current robot state

- GIVEN `EEFTwistTask` has access to current coordinator robot state
- AND no active integrated target pose exists
- WHEN the task receives a nonzero spatial EEF twist command
- THEN it seeds its target pose from the current end-effector pose
- AND it uses the received twist to update that target over coordinator time

#### Scenario: Valid spatial EEF twist produces servo-position output

- GIVEN `EEFTwistTask` has a seeded target pose and a valid nonzero spatial EEF twist command
- WHEN the coordinator computes the next control cycle
- THEN the task solves for a joint target using manipulator kinematics
- AND it outputs a servo-position joint command through the coordinator task output path

#### Scenario: Invalid IK result is rejected safely

- GIVEN `EEFTwistTask` receives a spatial EEF twist command that would produce an invalid or unsafe target
- WHEN the task evaluates the command
- THEN it rejects the unsafe output
- AND it does not emit a joint command that violates the task safety checks

### Requirement: EEFTwistTask stops and resets on timeout or stop commands

`EEFTwistTask` SHALL prevent stale spatial EEF twist commands from causing continued manipulator motion.

#### Scenario: Command timeout stops active EEF twist motion

- GIVEN `EEFTwistTask` has received a nonzero spatial EEF twist command
- WHEN no fresh spatial EEF twist command arrives before the configured timeout
- THEN the task stops producing motion from the stale command
- AND the active integrated target is cleared or made inactive

#### Scenario: Zero command clears active target

- GIVEN `EEFTwistTask` has an active integrated target pose
- WHEN it receives a zero spatial EEF twist command for its task route
- THEN it stops active EEF twist motion
- AND it clears the active target so the next nonzero command re-seeds from current robot state

### Requirement: Manipulator keyboard teleop uses EEFTwistTask for manipulator jogging

Manipulator keyboard teleop blueprints SHALL use the routed spatial EEF twist flow for keyboard manipulator jogging.

#### Scenario: Manipulator keyboard teleop blueprint wires keyboard to EEFTwistTask

- GIVEN a manipulator keyboard teleop blueprint is built
- WHEN the blueprint connects its modules and tasks
- THEN keyboard arm teleop publishes routed spatial EEF twist commands
- AND the coordinator contains a matching `EEFTwistTask`
- AND robot model and EEF joint configuration are owned by the task configuration rather than the keyboard module
