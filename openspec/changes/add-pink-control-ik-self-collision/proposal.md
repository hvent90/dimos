## Why

Cartesian and end-effector twist control currently use the legacy
`PinocchioIK` path. The control stack needs one generic, bounded differential-IK
backend so Cartesian pose and keyboard twist tasks share the same measured-state
and command-safety behavior.

## What Changes

- Add generic Pink control IK as the default `ControlCoordinator` backend for
  Cartesian and EEF-twist tasks.
- Retain `PinocchioIK` only through an explicit `backend="pinocchio"` option for
  compatibility.
- Validate named end-effector frames, model/joint mappings, and prepared
  URDF/Xacro models before control starts.
- Re-anchor every solve to measured joints, use bounded `dt`, apply Pink frame
  tasks with one-step integration, enforce joint/velocity limits, and hold on
  expected runtime solve errors or invalid output.
- Update common task helpers and all shipped task blueprints to use Pink by
  default without a Piper-specific backend exception.
- Migrate Piper Cartesian and EEF-twist control from its MJCF/numeric EEF path
  to the matching existing Xacro/URDF model and named `gripper_base` frame.

Self-collision and world-obstacle avoidance are out of scope for this change.

## Affected DimOS Surfaces

- `CartesianIKTask`, `EEFTwistTask`, `ControlCoordinator` task configuration,
  `PoseStamped`, `TwistStamped`, and `JointCommandOutput` behavior.
- Common Cartesian and EEF-twist task helpers and shipped manipulator
  blueprints, including Piper.
- Pink and legacy Pinocchio control-IK backend selection and model preparation.
- Simulation/replay and hardware control latency validation.

## Capabilities

### New Capabilities

- `pink-control-ik`: Generic Pink differential control IK for Cartesian and EEF
  twist tasks, with explicit legacy Pinocchio compatibility.

### Modified Capabilities

- None. No baseline OpenSpec capability specification exists for this control
  path.

## Impact

Pink becomes the default control behavior for all shipped Cartesian and EEF-twist
task blueprints. Existing users can select `backend="pinocchio"` explicitly
during migration. Piper uses its existing Xacro/URDF model and
`gripper_base` frame instead of its MJCF/numeric EEF path. The rollout requires
focused backend, task, blueprint, and model-preparation tests plus
simulation/replay latency validation; it does not add self-collision behavior.
