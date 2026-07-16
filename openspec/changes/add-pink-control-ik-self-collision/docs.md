## Documentation Updates

- Update `docs/capabilities/manipulation/index.md` to describe Pink as the
  default Cartesian and EEF-twist control IK backend, with explicit
  `backend="pinocchio"` compatibility selection.
- Update the same manipulation capability documentation to explain named EEF
  frames, URDF/Xacro preparation, model/joint mapping validation, measured-state
  anchoring, bounded one-step control, runtime holds, and the distinction from
  planning `WorldSpec`.
- Update the existing Piper-related sections in the manipulation capability
  documentation to state that Piper uses the matching Xacro/URDF model and
  named `gripper_base` frame. Do not describe collision protection or create a
  new Piper document.
- Update `docs/capabilities/manipulation/adding_a_custom_arm.md` with the
  generic Pink control configuration, explicit legacy Pinocchio selection,
  direct model preparation, frame/joint validation, and task-helper usage.
- Document simulation/replay latency benchmarking and supervised low-speed
  hardware rollout checks, without claiming that validation has occurred.

## Out of Scope

Do not document self-collision or planning-world obstacle avoidance as control
features. This change does not add collision behavior.

## Doc Validation

- Run the repository documentation link checker if the changed documentation
  participates in it.
- Run `md-babel-py run <changed-doc>` for changed executable examples when the
  tool is available.
