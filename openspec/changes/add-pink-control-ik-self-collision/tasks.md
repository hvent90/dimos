## 1. Pink control backend

- [x] 1.1 Define the typed control backend configuration with Pink as the
  default and `backend="pinocchio"` as the only explicit legacy option.
- [x] 1.2 Implement shared URDF/Xacro model preparation, including package and
  Xacro argument resolution, named EEF frame validation, controlled-joint
  mapping validation, and actionable startup diagnostics.
- [x] 1.3 Implement the generic Pink one-step backend: measured-state reset,
  named frame task update, bounded `dt`, joint/velocity limits, finite velocity
  integration, and normalized joint-position result.
- [x] 1.4 Preserve the existing PinocchioIK implementation behind explicit
  `backend="pinocchio"` selection; do not silently fall back from Pink.
- [x] 1.5 Add expected runtime solve-error handling that emits a bounded hold
  and rejects non-finite or otherwise invalid backend output.

## 2. Shared tasks and blueprint migration

- [x] 2.1 Refactor `CartesianIKTask` so target preparation, measured-state
  extraction, backend solve, output validation, timeout, and hold behavior form
  one reusable pipeline.
- [x] 2.2 Make `EEFTwistTask` a Cartesian task specialization that derives its
  short-horizon target from measured FK and the bounded twist increment.
- [x] 2.3 Preserve independent Cartesian and EEF-twist coordinator routing,
  lifecycle, timeout, zero-input, and clear semantics.
- [x] 2.4 Update common task helpers so Pink is the default backend and
  Pinocchio requires explicit `backend="pinocchio"`.
- [x] 2.5 Update every shipped Cartesian and EEF-twist task blueprint to use
  the Pink default without backend special cases.
- [x] 2.6 Migrate Piper Cartesian and EEF-twist tasks from the MJCF/numeric EEF
  path to the matching existing Xacro/URDF model and named `gripper_base`
  frame.

## 3. Tests

- [x] 3.1 Test Pink initialization, URDF/Xacro preparation, named EEF frame
  validation, model/joint mapping validation, and startup diagnostics.
- [x] 3.2 Test measured-state re-anchoring, bounded `dt`, frame-task one-step
  integration, joint/velocity limits, finite output, and bounded holds on
  expected runtime solve errors.
- [x] 3.3 Test explicit legacy `backend="pinocchio"` compatibility and prove
  that invalid Pink setup is not silently converted to Pinocchio.
- [x] 3.4 Test the shared Cartesian and EEF-twist pipeline, including measured FK
  target preparation, timeout, zero input, clear behavior, and output guards.
- [x] 3.5 Test common helper defaults and all shipped blueprint backend settings.
- [x] 3.6 Test Piper's Xacro/URDF model selection, named `gripper_base` frame,
  and removal of its MJCF/numeric EEF configuration.

## 4. Documentation

- [x] 4.1 Update manipulation capability documentation with the default Pink
  backend, explicit Pinocchio compatibility, model/frame validation, runtime
  holds, planning/control separation, and non-collision rollout guidance.
- [x] 4.2 Update the custom-arm integration guide with generic Pink task
  configuration, direct URDF/Xacro preparation, mapping validation, explicit
  legacy backend selection, and Piper's model/frame migration reference.

## 5. Verification and rollout

- [x] 5.1 Run `openspec validate add-pink-control-ik-self-collision`.
- [x] 5.2 Run focused tests for Pink control IK, Cartesian IK, EEF twist,
  common helpers, Piper blueprints, and model preparation.
- [x] 5.3 Run the blueprint registry generation test if blueprint discovery
  inputs change.
- [x] 5.4 Run the relevant documentation link checker and executable-example
  validation when available.
- [x] 5.5 Run type and lint checks for changed control/manipulation modules.
- [ ] 5.6 Validate Pink in simulation or replay at the coordinator rate,
  benchmark end-to-end control latency, exercise Cartesian and twist commands,
  and verify bounded holds and emergency-stop readiness.
- [ ] 5.7 Perform supervised low-speed hardware validation only after
  simulation/replay checks pass; record latency and runtime error behavior
  without claiming validation before it occurs.
