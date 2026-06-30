## 1. Teleop Adapter Runtime

- [ ] 1.1 Add teleop runtime package structure under `dimos/teleop/` without changing existing Quest modules.
- [ ] 1.2 Define `TeleopPrimaryOutput`, `TeleopCommandMetadata`, `TeleopCommand`, and `TeleopAdapter` types with `connect()`, `disconnect()`, and `get_current_command()`.
- [ ] 1.3 Implement a generic `TeleopModule` with stable coordinator-facing outputs for `joint_command`, `coordinator_cartesian_command`, and `twist_command`.
- [ ] 1.4 Implement structural safety in `TeleopModule`: no publish on `None`, one primary output, max publish rate, stale-command timeout, explicit stop handling, and adapter disconnect on stop.
- [ ] 1.5 Add unit tests for command envelope handling, output routing, conflicting primary output rejection, stale timeout behavior, rate limiting, and stop/disconnect behavior.

## 2. OpenArm Mini Feetech Integration

- [ ] 2.1 Verify the lower-level Feetech Python package name/import surface and add it to a narrow OpenArm Mini optional extra in `pyproject.toml`.
- [ ] 2.2 Add localized missing-dependency handling so OpenArm Mini teleop fails with a clear install hint when the Feetech extra is not installed.
- [ ] 2.3 Implement OpenArm Mini config with `port_left`, `port_right`, `left_calibration_path`, and `right_calibration_path` plus DimOS `STATE_DIR / "teleop" / "openarm_mini" / <side>` defaults.
- [ ] 2.4 Implement OpenArm Mini calibration artifact load/save models and validation for side-specific calibration directories.
- [ ] 2.5 Implement Feetech bus connection/configuration for both OpenArm Mini sides using non-interactive calibration loading during runtime startup.
- [ ] 2.6 Implement OpenArm Mini leader reading, side-specific sign/order mapping, joint_6/joint_7 remap, gripper conversion, OpenArm follower joint naming, joint-limit checks, and jump-threshold checks.
- [ ] 2.7 Implement `OpenArmMiniTeleopAdapter.get_current_command()` to return `JointState` command envelopes only when teleop authority is active and leader readings are valid.
- [ ] 2.8 Add unit tests for calibration path defaults, missing/invalid calibration errors, transform/remap/gripper conversion, follower joint names, joint limits, and jump-threshold rejection.

## 3. Manual Calibration Demo

- [ ] 3.1 Add `dimos/teleop/openarm_mini/demo_calibrate_openarm_mini.py` as a manual script excluded from pytest collection by name.
- [ ] 3.2 Implement interactive OpenArm Mini leader setup/calibration UX that writes side-specific calibration artifacts and never connects to follower OpenArm hardware.
- [ ] 3.3 Add an optional live-readout mode to the demo script for inspecting calibrated leader positions without starting `ControlCoordinator`.
- [ ] 3.4 Document the calibration script usage and default calibration storage paths near the OpenArm Mini teleop code.

## 4. Blueprint Wiring

- [ ] 4.1 Add an OpenArm Mini teleop blueprint that connects the generic teleop module to the existing OpenArm control coordinator through `joint_command`.
- [ ] 4.2 Keep existing Quest teleop blueprints unchanged and verify they still reference their existing Quest modules.
- [ ] 4.3 Regenerate `dimos/robot/all_blueprints.py` with the existing blueprint generation test after adding the new blueprint.
- [ ] 4.4 Add blueprint build/list tests or update existing blueprint generation coverage so the new blueprint appears and wires `joint_command` correctly.

## 5. Validation

- [ ] 5.1 Run focused unit tests for the teleop adapter runtime and OpenArm Mini adapter mapping/calibration logic.
- [ ] 5.2 Run the blueprint generation test for `dimos/robot/all_blueprints.py`.
- [ ] 5.3 Run the relevant fast pytest subset for teleop/OpenArm modules.
- [ ] 5.4 Perform hardware validation with calibrated OpenArm Mini and OpenArm follower when hardware is available, including missing-calibration and explicit-stop checks.
