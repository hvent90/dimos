## Why

Manipulation planning currently produces a geometric `GeneratedPlan`, then `ManipulationModule.execute_plan()` assigns timing as an execution-time implementation detail using the current joint trajectory generator. Preview, execution, and future benchmarking can therefore use different temporal assumptions, and composite or multi-robot plans can be split into independently timed per-robot trajectories even when the geometric plan was coordinated.

DimOS needs trajectory parametrization as a first-class manipulation-planning capability: a planner produces a geometric path, a parametrizer assigns time under motion constraints, and orchestration dispatches the resulting trajectory to control tasks without conflating planning, parametrization, and execution status.

## What Changes

- Add a `TrajectoryParametrizerSpec` role to the manipulation multi-spec architecture.
- Add `GeneratedTrajectory` as the canonical global time-parametrized manipulation artifact produced from a `GeneratedPlan`.
- Add `TrajectoryDispatch` as the execution-preparation artifact that derives task-specific `JointTrajectory` messages from a `GeneratedTrajectory`.
- Preserve a shared trajectory time domain for composite and multi-robot motions.
- Make preview, validation, benchmarking, and execution dispatch consume the same `GeneratedTrajectory` artifact.
- Provide a default `simple_trapezoid` backend that wraps the current timing behavior behind the new spec boundary.
- Add optional TOPPRA support through a `manipulation-toppra` extra and include that extra in `all` and the repository test environment.
- Keep TOPPRA explicit opt-in initially; do not make it the default backend until validation and benchmarks justify the switch.

## Capabilities

### New Capabilities
- `manipulation-trajectory-parametrization`: First-class trajectory parametrization, generated trajectory artifacts, dispatch preparation, and backend policy for manipulation planning.

### Modified Capabilities

## Impact

- Affected code areas:
  - `dimos/manipulation/planning/spec/` for the new spec role, models, config, and status types.
  - `dimos/manipulation/planning/trajectory_generator/` or successor backend package for the `simple_trapezoid` and TOPPRA parametrizers.
  - `dimos/manipulation/manipulation_module.py` for orchestration, preview, parametrization, and dispatch flow.
  - `dimos/msgs/trajectory_msgs/` only as the low-level dispatch message boundary, not as the canonical manipulation artifact.
  - `pyproject.toml` for the `manipulation-toppra` optional extra and inclusion in `all`.
  - manipulation unit/integration tests for artifact contracts, backend behavior, preview/execution sharing, and TOPPRA execution in the repo test environment.
- Existing geometric planning APIs should remain conceptually geometric. Timing belongs to the parametrization step.
- `ControlCoordinator` task internals are out of scope; dispatch prepares existing task messages.
