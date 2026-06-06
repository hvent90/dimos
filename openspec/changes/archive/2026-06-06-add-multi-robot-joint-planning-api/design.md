# Design: timed multi-robot joint planning

## Context

DimOS manipulation planning currently has two low-level public planning RPCs on `ManipulationModule`:

- `plan_to_joints(joints: JointState, robot_name: str | None = None) -> bool`
- `plan_to_pose(pose: Pose, robot_name: str | None = None) -> bool`

`plan_to_pose` is not Cartesian path planning. It solves IK once for the target pose, then calls the same joint-space planner used by `plan_to_joints`. Planning currently produces a geometric `JointPath`, then `JointTrajectoryGenerator` converts that path into a time-parameterized `JointTrajectory`. The module stores per-robot paths and trajectories in `_planned_paths[robot_name]` and `_planned_trajectories[robot_name]`.

This split works for one robot, but it is conceptually incomplete for dual-arm planning. A coordinated dual-arm plan is not just two collision-free geometric paths. It is a synchronized timed motion: both arms must share the same trajectory clock, waypoint timing, and collision validation context.

The existing control system is already joint-centric. `ManipulationModule.execute(robot_name)` translates a stored trajectory into coordinator joint names and calls `ControlCoordinator.task_invoke(config.coordinator_task_name, "execute", {"trajectory": translated})`. The trajectory task samples `JointTrajectory` by `time_from_start` inside the coordinator tick loop. This means the execution layer already consumes timed trajectories; the planning layer should make timing part of the plan artifact.

## Goals / Non-Goals

**Goals:**

- Keep the public planning surface small by extending the existing two planning APIs rather than adding a family of `multiple` methods.
- Introduce a coherent internal timed plan artifact representing one single- or multi-robot motion.
- For multi-robot plans, generate timing from the combined joint vector first, then split synchronized per-robot trajectories.
- Preserve existing single-robot behavior for current callers.
- Preserve explicit preview/execute safety: planning must not move hardware.
- Keep SRDF/named planning groups as future wrappers over ordered robot sets, not part of this change.

**Non-Goals:**

- Do not add SRDF parsing.
- Do not add named persistent planning groups.
- Do not add true coupled multi-end-effector Cartesian IK.
- Do not redesign `ControlCoordinator`, task arbitration, or hardware adapters.
- Do not expose new MCP/LLM skills in the first implementation.

## DimOS Architecture

### Planning API surface

Extend the existing two planning RPCs to accept either scalar or list inputs:

```python
plan_to_joints(
    joints: JointState | list[JointState],
    robot_name: str | list[str] | None = None,
) -> bool

plan_to_pose(
    pose: Pose | list[Pose],
    robot_name: str | list[str] | None = None,
) -> bool
```

Behavior:

- `JointState` + `str | None`: existing single-robot behavior.
- `Pose` + `str | None`: existing single-robot IK-to-joint-plan behavior.
- `list[JointState]` + `list[str]`: coordinated multi-robot joint plan.
- `list[Pose]` + `list[str]`: independently solve IK per robot, then coordinated multi-robot joint plan to those IK goals.
- List lengths must match and robot names must be unique.
- Caller-provided robot-name order defines composite joint-vector order.

This avoids adding `plan_multiple`, `plan_to_joints_multiple`, or similar parallel APIs.

### Timed plan artifact

Add an internal plan object, tentatively named `MotionPlan` or `PlannedMotion`, owned by `ManipulationModule`:

```python
@dataclass
class MotionPlan:
    robot_names: list[RobotName]
    paths: dict[RobotName, JointPath]
    trajectories: dict[RobotName, JointTrajectory]
    duration: float
    created_from: Literal["joints", "pose"]
```

`MotionPlan` is the authoritative stored artifact. Existing `_planned_paths` and `_planned_trajectories` can either be retained as compatibility mirrors or replaced if all internal call sites are updated. The important invariant is that a multi-robot plan is one object with one duration, not N unrelated cached trajectories.

### Multi-robot planning flow

For multi-robot joint planning:

```text
resolve robot_names -> ordered robot_ids/configs
get current JointState for each robot
validate target JointState for each robot
concatenate starts/goals/limits in robot_names order
run RRT over the combined vector
for every sampled candidate:
  split candidate into per-robot JointState
  set all participating robot states in one scratch context
  check full-scene collision
generate one combined JointTrajectory from the combined path
split that timed trajectory into per-robot JointTrajectory objects
store one MotionPlan
```

Timing must be generated before splitting. This is the key coherence rule. If each robot path is time-parameterized independently, durations and waypoint times can diverge, and the result is no longer a meaningful dual-arm plan.

### Collision validation

Keep the `WorldSpec` surface mostly unchanged. Multi-robot composition can live in planner/module helper code by using existing methods:

- `world.get_joint_limits(robot_id)`
- `world.get_joint_state(ctx, robot_id)`
- `world.set_joint_state(ctx, robot_id, joint_state)`
- `world.is_collision_free(ctx, robot_id)`

In the Drake backend, `is_collision_free(ctx, robot_id)` currently validates the robot id but checks `query_object.HasCollisions()` for the whole scene. Therefore, after setting all participating robot states in one scratch context, one collision query is enough to detect inter-arm/world collisions.

Optionally add a small semantic cleanup:

```python
world.is_context_collision_free(ctx) -> bool
```

This would avoid pretending one robot owns a full-scene collision query. It is optional and should only be added if it keeps the implementation clearer than calling `is_collision_free(ctx, robot_ids[0])`.

### Preview and execution

Keep existing public methods:

```python
preview_path(duration: float = 3.0, robot_name: str | list[str] | None = None) -> bool
execute(robot_name: str | list[str] | None = None) -> bool
```

The `robot_name` argument is currently meaningful because stored paths/trajectories are keyed per robot and docs already use `preview(robot_name="left_arm")` / `execute(robot_name="left_arm")` for multi-arm setups.

For this change:

- Passing a string keeps existing behavior.
- Passing a list previews/executes the selected subset from the active `MotionPlan`.
- Passing `None` keeps existing single-robot default behavior. Avoid changing ambiguous multi-robot `None` semantics in the first implementation.

Execution should still submit per-robot trajectories to existing coordinator tasks. Exact atomic start across multiple tasks is not guaranteed by the current coordinator RPC path, but the stored trajectories will share identical `time_from_start` values and duration. If later hardware QA shows start skew is unacceptable, add a coordinator-level batch start or combined trajectory task in a separate change.

### Streams, transports, blueprints, skills/MCP, registries

- Streams/transports: no new streams or transport changes are expected. The existing aggregated `joint_state` stream remains the state source.
- Blueprints: add a mock dual-arm planner/coordinator blueprint named `dual-xarm6-mock-planner-coordinator` so manual QA can start the planner, coordinator, mock arms, and Meshcat with one command.
- Skills/MCP: do not expose multi-robot planning as a new skill in this change. Existing single-robot skills remain wrappers over the scalar API path.
- Generated registries: because this change adds a blueprint, regenerate the blueprint registry and include the generated update with the implementation.

### Manual verification surface

The implementation should include a runnable mock dual-arm example rather than relying only on unit tests. The intended flow is:

```bash
dimos run dual-xarm6-mock-planner-coordinator
python -i -m dimos.manipulation.planning.examples.demo_dual_arm_planning
```

The REPL example should drive the public RPC/library surface and expose helpers for:

- `robots()` to confirm `left_arm` and `right_arm` are configured.
- `dual_plan_joints()` to call `plan_to_joints([left_target, right_target], ["left_arm", "right_arm"])`.
- `dual_preview()` to call `preview_path(duration, ["left_arm", "right_arm"])` and let the user observe synchronized Meshcat motion without hardware execution.
- `dual_execute()` to call `execute(["left_arm", "right_arm"])` explicitly.
- `bad_request()` to send a malformed multi-robot request and confirm it fails without replacing the active plan.

The example should be safe for mock-first verification and reusable for hardware only after the operator explicitly chooses execution.

## Decisions

### Decision: make timing part of the plan

A plan in this change means a time-parameterized `MotionPlan`, not only a geometric path. The combined path is an intermediate planner product; the stored artifact must include synchronized trajectories.

Rationale: dual-arm planning without shared timing is misleading. Execution consumes timed `JointTrajectory` objects, so timing belongs in the plan artifact.

### Decision: overload the existing planning APIs

Extend `plan_to_joints` and `plan_to_pose` to accept list inputs rather than adding a new public `plan_multiple` family.

Rationale: the conceptual operation is still “plan to joints” or “plan to pose”; cardinality should be an input shape, not a new API namespace.

Alternative considered: add `plan_to_joint_targets(targets: dict[str, JointState])`. This is explicit and RPC-friendly, but adds a third public planning API. Keep it as a fallback if RPC/schema tooling handles union/list overloads poorly.

### Decision: do not implement coupled multi-robot Cartesian IK yet

Multi-robot `plan_to_pose([pose_left, pose_right], ["left", "right"])` should solve IK per robot independently, then run one combined joint-space plan.

Rationale: this matches current `plan_to_pose` semantics. True coupled Cartesian IK is a different capability and should not be hidden inside this change.

### Decision: keep execution split through existing coordinator tasks

Multi-robot `MotionPlan` stores synchronized per-robot trajectories and `execute(["left", "right"])` submits those trajectories to each robot's existing coordinator task.

Rationale: `ControlCoordinator` already arbitrates joint commands per joint and routes them to hardware. Changing execution atomicity belongs in a later coordinator-focused change if needed.

## Safety / Simulation / Replay

- Planning and preview must not initiate hardware motion.
- Execution remains explicit.
- Multi-robot collision checks must set all participating robots in the same scratch context before querying collisions.
- Initial manual QA should use mock or simulation dual-arm setups.
- Hardware QA should verify both arms start close enough for the intended use case; if not, stop and design coordinator-side synchronized start.

## Risks / Trade-offs

- **RPC union/list ergonomics**: RPC clients may not handle `JointState | list[JointState]` cleanly. Mitigation: if tooling resists overloads, add one explicit `plan_to_joint_targets(dict[str, JointState])` API instead of forcing awkward serialization.
- **Start skew during execution**: per-robot task invocations are not atomically started. Mitigation: this change guarantees shared trajectory timing but not atomic dispatch; defer batch coordinator start unless real QA requires it.
- **State cache ambiguity**: keeping both `_active_plan` and old per-robot caches can drift. Mitigation: make `MotionPlan` authoritative and update compatibility maps from it atomically.
- **Planner dimensionality**: combined RRT space is larger and may need timeout tuning. Mitigation: preserve configurable planning timeout and test validation/splitting separately from stochastic planner success.

## Migration / Rollout

- Existing scalar calls to `plan_to_joints`, `plan_to_pose`, `preview_path`, and `execute` remain compatible.
- Add tests for scalar behavior to prevent regressions.
- Add tests for list input validation, deterministic ordering, synchronized trajectory splitting, and atomic plan storage.
- Add `dual-xarm6-mock-planner-coordinator` as the primary manual QA blueprint for coordinated planning.
- Update manipulation planning docs only after the API shape is confirmed in implementation.
- Regenerate `dimos/robot/all_blueprints.py` after adding the manual QA blueprint.

## Open Questions

- Can the RPC layer represent overloaded `JointState | list[JointState]` and `str | list[str] | None` cleanly enough, or should we fall back to one explicit dict-shaped API?
- Should `preview_path(None)` and `execute(None)` eventually mean “active plan” even in multi-robot modules, or should ambiguous `None` keep failing unless exactly one robot exists?
- Is current per-task execution start skew acceptable for the first dual-arm use case, or do we need a coordinator-level batch start in the same change?
