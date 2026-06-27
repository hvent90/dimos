## Context

The current manipulation pipeline separates kinematics and geometric planning through the existing multi-spec architecture, but trajectory timing is still bolted on inside execution. `GeneratedPlan` stores the selected planning groups and geometric joint path. `ManipulationModule.execute_plan()` then projects that path into robot-local paths, calls the current joint trajectory generator, wraps each result in a low-level `JointTrajectory`, and invokes coordinator tasks.

This makes time parametrization hard to reason about. It is not a named capability, it has no explicit status, it is not available to preview or benchmarking as a stable artifact, and it can silently retime coordinated multi-robot paths independently per robot. The current generator also uses a simple per-segment trapezoidal profile that likely stops at intermediate planner waypoints and lacks explicit backend policy such as TOPPRA gridpoint selection.

## Goals / Non-Goals

**Goals:**

- Introduce trajectory parametrization as a first-class manipulation-planning spec role.
- Preserve `GeneratedPlan` as a geometric planning artifact.
- Add `GeneratedTrajectory` as a global time-parametrized artifact produced from a `GeneratedPlan`.
- Add `TrajectoryDispatch` as the separate execution-preparation boundary from generated trajectory to control-task messages.
- Preserve one shared time domain across all selected joints in composite and multi-robot motions.
- Ensure preview, validation, benchmarking, and execution dispatch all consume the same `GeneratedTrajectory`.
- Keep a required `simple_trapezoid` backend that wraps current behavior and a TOPPRA backend available through an optional install extra.
- Expose backend policy controls for velocity, acceleration, minimum segment duration, and TOPPRA gridpoints/discretization.
- Distinguish planning, parametrization, dispatch, and execution failure semantics.

**Non-Goals:**

- Making TOPPRA the default backend in this change.
- Changing geometric planner algorithms such as RRT or IK.
- Reworking `ControlCoordinator` task internals.
- Adding a jerk-limited or S-curve backend unless it falls out naturally from a selected backend.
- Overhauling the benchmark harness beyond consuming the new generated trajectory artifact where relevant.

## Decisions

### TrajectoryParametrizerSpec as a new multi-spec role

Add a `TrajectoryParametrizerSpec` Protocol that converts a successful geometric `GeneratedPlan` plus parametrization policy into a `GeneratedTrajectory`.

Rationale: kinematics, planning, and trajectory parametrization are different robotics capabilities. Treating parametrization as a spec role keeps `ManipulationModule` as orchestration and allows simple and serious backends to share one contract.

Alternative considered: keep time generation inside `ManipulationModule.execute_plan()`. This preserves the current implementation shape but leaves timing invisible to preview, benchmarking, validation, and failure handling.

### GeneratedTrajectory is global and canonical

`GeneratedTrajectory` should represent the canonical global timed joint trajectory for the selected planning groups. It should not own coordinator-task-specific projections as canonical data.

Rationale: a generated trajectory is still a manipulation-planning artifact. Keeping it global preserves the semantics of the source `GeneratedPlan`, especially for composite and multi-robot paths with a shared time domain. It also avoids coupling parametrization backends to current coordinator task wiring.

Alternative considered: store robot-local or task-local projections directly inside `GeneratedTrajectory`. This is convenient for execution but makes the artifact dependent on dispatch topology and invites independent per-task timing.

### TrajectoryDispatch as a separate execution-preparation boundary

Add `TrajectoryDispatch` as the artifact that maps a `GeneratedTrajectory` into the per-task or per-robot `JointTrajectory` messages required by `ControlCoordinator` tasks.

Rationale: dispatch is not trajectory parametrization. Dispatch knows about current task names, joint ownership, and coordinator invocation details. Separating it lets `TrajectoryParametrizerSpec` stay independent of control-task wiring while keeping execution preparation observable and testable.

Alternative considered: let each parametrizer emit coordinator task messages. This would make every backend depend on control task layout and would duplicate dispatch rules.

### Shared trajectory time domain for composite motion

A `GeneratedTrajectory` produced from a composite or multi-robot `GeneratedPlan` must preserve one shared time domain across all selected joints and robot-local projections derived during dispatch.

Rationale: coupled planning can be invalidated if each robot is retimed independently. A shared time basis allows preview, validation, benchmarking, and execution to refer to the same coordinated motion.

Alternative considered: independently retime each robot-local path and start all tasks together. This is simpler but breaks the semantics of coordinated composite motion and can produce relative timing changes the planner did not validate.

### Preview, validation, benchmarking, and execution share the same artifact

Preview, validation, benchmarking, and execution dispatch should all consume the same `GeneratedTrajectory`, not independently assign duration or timing to the geometric `GeneratedPlan`.

Rationale: the motion the user previews, the benchmark measures, and the robot executes should be the same time-parametrized artifact. This removes the current mismatch where preview can use a fixed display duration while execution uses generated timing.

Alternative considered: keep preview as display-only interpolation over the geometric path. This is acceptable for rough visualization but not for validating timing-sensitive behavior.

### simple_trapezoid default and TOPPRA opt-in

Keep `simple_trapezoid` as the initial default backend and support TOPPRA as an explicit opt-in backend.

Rationale: this separates architecture risk from backend integration risk. The new spec/artifact boundary can land with current behavior preserved, while TOPPRA becomes available for serious parametrization and future validation before flipping defaults.

Alternative considered: make TOPPRA the default immediately. This may be the right future direction, but it should follow validation and benchmark evidence.

### TOPPRA install and test policy

Add a `manipulation-toppra` optional extra containing the PyPI `toppra` package, include that extra in `all`, and require the repository test environment to run TOPPRA tests unconditionally.

Rationale: runtime users should not need TOPPRA unless they configure it, but DimOS developers and CI should continuously validate the supported TOPPRA backend. Optional runtime dependency should not mean skipped repository coverage.

Alternative considered: skip TOPPRA tests when import is unavailable. This hides regressions in a supported backend and contradicts the decision to include TOPPRA in the repo test surface.

### Missing configured TOPPRA dependency fails initialization

If configuration selects `backend="toppra"` and the dependency is unavailable, planning/parametrization initialization should fail clearly with guidance to install `dimos[manipulation-toppra]`.

Rationale: an explicitly configured missing backend is an environment/configuration error, not a motion-planning outcome. Failing early avoids discovering the problem only when a motion is attempted.

Alternative considered: return `GeneratedTrajectory.status = BACKEND_UNAVAILABLE` during parametrization. This is useful only for dynamic backend unavailability, not predictable missing imports from explicit configuration.

### Separate status layers

Keep planning, parametrization, dispatch, and execution statuses separate. A parametrization failure must not overwrite `GeneratedPlan.status`.

Rationale: users and tests need to distinguish “planner failed to find a path,” “planner found a path but constraints made timing infeasible,” and “trajectory succeeded but dispatch or execution failed.”

Alternative considered: use one combined status on the latest artifact. This loses diagnostic information and makes recovery decisions ambiguous.

## Risks / Trade-offs

- [Risk] TOPPRA's Python package exists on PyPI but upstream documentation signals possible future migration away from Python support. → Mitigation: keep the backend behind a spec boundary and optional extra so another implementation can replace it without changing orchestration.
- [Risk] Adding `GeneratedTrajectory` and `TrajectoryDispatch` increases artifact count. → Mitigation: the artifacts correspond to real pipeline boundaries: geometric planning, time parametrization, and execution preparation.
- [Risk] The `simple_trapezoid` backend may preserve current stop-at-waypoint behavior. → Mitigation: treat it as a compatibility fallback, not the final quality target.
- [Risk] Benchmark comparisons can be misleading if TOPPRA gridpoint/discretization policy varies. → Mitigation: make gridpoint/discretization policy explicit in config and tests.
- [Risk] Dispatch logic can drift from parametrization assumptions. → Mitigation: test that dispatch preserves the generated trajectory's global timing and joint ordering.

## Migration Plan

1. Add the new config, model, status, and Protocol definitions without changing planner algorithms.
2. Wrap current trapezoid timing behavior behind `simple_trapezoid` as the default parametrizer.
3. Change manipulation orchestration so planning produces `GeneratedPlan`, parametrization produces `GeneratedTrajectory`, and dispatch produces task-specific `JointTrajectory` messages.
4. Update preview paths to consume `GeneratedTrajectory` timing rather than an ad hoc duration.
5. Add TOPPRA backend support and dependency extra, then include it in `all` and the repo test environment.
6. Add tests for artifact contracts, backend status behavior, shared time domain, TOPPRA behavior, and dispatch preservation.
7. Rollback is restoring direct execution-time generation in `ManipulationModule` and removing the new spec/models/backends; geometric planner APIs can remain unchanged.

## Open Questions

- What validation or benchmark threshold should justify making TOPPRA the default backend later?
- Should future parametrizers support duration targeting or only constraint-satisfying minimum-time behavior?
- Which backend, if any, should introduce jerk-limited or S-curve profiles?
