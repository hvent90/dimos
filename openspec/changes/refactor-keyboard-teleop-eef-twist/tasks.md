## 1. Implementation

- [x] 1.1 Add a coordinator-owned routed spatial EEF twist stream using `TwistStamped`, with routing by `frame_id` and no changes to existing planar/base `twist_command` behavior.
- [x] 1.2 Extend the control task interface or coordinator dispatch path so tasks can receive spatial EEF twist commands with coordinator time.
- [x] 1.3 Add the `EEFTwistTask` package, config/factory, registry entry, and manipulator blueprint helper following existing task package patterns.
- [x] 1.4 Implement `EEFTwistTask` FK seeding, coordinator/world-frame twist integration, IK solving, servo-position output, finite checks, bounded dt/step integration, explicit IK error policy, joint-delta/IK rejection safety, timeout, and zero-command reset behavior.
- [x] 1.5 Refactor `KeyboardTeleopModule` arm behavior to publish routed `TwistStamped` operator intent and remove keyboard-owned joint state, FK model, EEF joint id, joint names, absolute pose tracking, and FK sync behavior.
- [x] 1.6 Update current manipulator keyboard teleop blueprints to wire keyboard teleop to `EEFTwistTask` and move robot model/EEF configuration into task configuration.
- [x] 1.7 Update generated blueprint registry outputs if blueprint generation detects changes. No registry output update was needed because blueprint names were unchanged.

## 2. Tests

- [x] 2.1 Add coordinator routing and subscription lifecycle tests for matched and unmatched `TwistStamped.frame_id` spatial EEF twist commands.
- [x] 2.2 Add regression tests proving existing planar/base `twist_command` handling remains unchanged.
- [x] 2.3 Add `EEFTwistTask` tests for first-command FK seeding, pose integration over time, servo-position output mode, IK/joint-delta rejection, command timeout, and zero-command target reset.
- [x] 2.4 Add keyboard teleop tests for movement-key twist publication, stop/zero publication, task `frame_id`, and absence of joint-state/FK startup dependency.
- [x] 2.5 Add blueprint wiring tests or assertions for current manipulator keyboard teleop task/module configuration.

## 3. Documentation

- [x] 3.1 Update user-facing manipulator keyboard teleop docs if they describe absolute pose jogging, FK model parameters, or SPACE-as-FK-sync behavior.
- [x] 3.2 Update contributor control/teleop docs with the planar base twist vs spatial EEF twist distinction and the temporary `coordinator_ee_twist_command` compatibility stream.
- [x] 3.3 Update coding-agent guidance if existing docs describe keyboard teleop as owning robot state/FK/IK responsibilities.

## 4. Verification

- [x] 4.1 Run `openspec validate refactor-keyboard-teleop-eef-twist`.
- [x] 4.2 Run focused pytest targets for coordinator routing, `EEFTwistTask`, keyboard teleop, and manipulator teleop blueprint wiring.
- [x] 4.3 Run `pytest dimos/robot/test_all_blueprints_generation.py` if blueprint registry output changes. Not needed because no blueprint names or generated registry entries changed.
- [x] 4.4 Run relevant lint/type checks for changed Python modules.
- [x] 4.5 Run docs validation commands for any edited docs, if the repository provides a specific checker. No repo-specific docs checker applies to the edited Markdown files.
- [ ] 4.6 Manually QA through the relevant DimOS surface: run an xArm keyboard teleop blueprint in simulation/replay or safe hardware procedure, verify movement keys jog the EEF, stop input halts motion, and existing base twist users still behave normally.
