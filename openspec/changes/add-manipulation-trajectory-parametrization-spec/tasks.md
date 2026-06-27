## 1. Spec, Config, and Models

- [ ] 1.1 Add trajectory parametrization config with backend selection, velocity/acceleration scales, optional velocity/acceleration limits, optional minimum segment duration, and explicit TOPPRA gridpoint/discretization policy.
- [ ] 1.2 Add `TrajectoryParametrizerSpec` as a DimOS Spec Protocol that converts a successful `GeneratedPlan` into a `GeneratedTrajectory`.
- [ ] 1.3 Add `GeneratedTrajectory` as the canonical global timed manipulation artifact with joint names, timed points, shared duration/time domain, status, message, and source planning metadata.
- [ ] 1.4 Add `TrajectoryDispatch` as the execution-preparation artifact that contains task-specific `JointTrajectory` messages and dispatch status/message without changing `GeneratedTrajectory` timing.
- [ ] 1.5 Add explicit parametrization and dispatch status values that do not overwrite `GeneratedPlan.status`.

## 2. Parametrization Backends

- [ ] 2.1 Wrap the current trapezoidal timing logic behind a `simple_trapezoid` trajectory parametrizer backend.
- [ ] 2.2 Preserve current default behavior by selecting `simple_trapezoid` unless another backend is configured.
- [ ] 2.3 Add a TOPPRA backend using the PyPI `toppra` Python API for joint velocity and acceleration constrained path parametrization.
- [ ] 2.4 Fail planning/parametrization initialization clearly when `backend="toppra"` is configured but TOPPRA cannot be imported, including guidance to install `dimos[manipulation-toppra]`.
- [ ] 2.5 Ensure TOPPRA policy exposes gridpoint/discretization behavior explicitly enough for reproducible tests and benchmark comparisons.

## 3. Orchestration and Dispatch

- [ ] 3.1 Update manipulation planning flow so geometric planning stores `GeneratedPlan` and a separate parametrization step stores `GeneratedTrajectory`.
- [ ] 3.2 Update preview, validation, and benchmark-facing paths to consume `GeneratedTrajectory` instead of independently timing `GeneratedPlan`.
- [ ] 3.3 Add dispatch orchestration in `ManipulationModule` that maps global `GeneratedTrajectory` into coordinator task-specific `JointTrajectory` messages.
- [ ] 3.4 Ensure `TrajectoryParametrizerSpec` has no dependency on `ControlCoordinator` task names or task wiring.
- [ ] 3.5 Preserve one shared trajectory time domain across composite and multi-robot generated trajectories and their dispatch projections.

## 4. Packaging

- [ ] 4.1 Add `manipulation-toppra = ["toppra>=0.6.8"]` or an equivalent current TOPPRA dependency constraint to optional extras.
- [ ] 4.2 Include `manipulation-toppra` in the `all` extra.
- [ ] 4.3 Ensure the repository test environment installs TOPPRA support so TOPPRA tests run unconditionally.

## 5. Tests and Validation

- [ ] 5.1 Add contract tests showing `GeneratedPlan` remains geometric and `GeneratedTrajectory` carries the time-parametrized global path.
- [ ] 5.2 Add tests that parametrization failure leaves `GeneratedPlan.status` unchanged and reports failure on `GeneratedTrajectory`.
- [ ] 5.3 Add `simple_trapezoid` tests for joint ordering, monotonic `time_from_start`, final waypoint preservation, and shared time domain behavior.
- [ ] 5.4 Add TOPPRA tests that run unconditionally in the repo test environment and cover velocity/acceleration constraints plus explicit gridpoint/discretization policy.
- [ ] 5.5 Add dispatch tests proving task-specific `JointTrajectory` messages preserve global generated trajectory timing and joint ordering.
- [ ] 5.6 Add orchestration tests proving preview and execution dispatch consume the same `GeneratedTrajectory` artifact.
- [ ] 5.7 Run focused manipulation trajectory parametrization tests.
- [ ] 5.8 Run OpenSpec validation for this change and fix proposal, spec, design, or task formatting issues.
