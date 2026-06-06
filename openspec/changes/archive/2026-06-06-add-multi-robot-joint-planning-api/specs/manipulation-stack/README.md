# Multi-robot joint planning technical summary

This change makes the manipulation planner store and operate on one **timed motion plan** for a single- or multi-robot motion. The important internal shift is from “per-robot cached paths/trajectories” to an authoritative active plan object that owns both the geometric paths and synchronized trajectories.

## Current internal model

Today, planning is centered on per-robot dictionaries:

```text
_planned_paths[robot_name]         -> JointPath
_planned_trajectories[robot_name]  -> JointTrajectory
```

For one robot this is fine. For dual-arm planning it is incomplete because two independently generated trajectories can have different durations and different `time_from_start` grids. That means two successful “plans” might not form one coordinated motion.

## Proposed internal model

Add an internal active plan representation, tentatively named `MotionPlan` or `PlannedMotion`:

```python
@dataclass
class MotionPlan:
    robot_names: list[RobotName]
    paths: dict[RobotName, JointPath]
    trajectories: dict[RobotName, JointTrajectory]
    duration: float
    created_from: Literal["joints", "pose"]
```

`MotionPlan` is the source of truth. Existing per-robot path/trajectory maps may remain as compatibility mirrors, but they should be updated atomically from the active plan and should not define multi-robot plan identity themselves.

## Multi-robot planning pipeline

For ordered multi-robot joint planning:

```text
robot_names                    # caller-provided order is normative
  -> robot_ids/configs
  -> current JointState per robot
  -> target JointState per robot
  -> concatenate start / goal / limits in robot_names order
  -> run RRT over combined joint vector
  -> for each sampled candidate:
       split candidate into per-robot JointState
       set all participating robots in one scratch world context
       run full-scene collision check
  -> combined JointPath
  -> combined JointTrajectory with one time grid
  -> split synchronized per-robot JointTrajectory objects
  -> store one MotionPlan
```

The key invariant is:

```text
combined path -> combined timed trajectory -> split synchronized trajectories
```

Never time-parameterize each robot independently for a coordinated plan.

## Public API shape

Reuse the existing two planning calls instead of adding a `multiple` API family:

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

Scalar inputs preserve current behavior. Ordered list inputs opt into coordinated multi-robot planning. `plan_to_pose` keeps existing semantics: solve IK once per target pose, then perform joint-space planning. It is not Cartesian path planning.

## Collision-checking model

Do not spread a `multiple` variant across every world method. The composite planner can mostly use existing world operations:

```text
get_joint_limits(robot_id)
get_joint_state(ctx, robot_id)
set_joint_state(ctx, robot_id, joint_state)
is_collision_free(ctx, robot_id)
```

For each composite candidate, all participating robot states are set in the same scratch context before collision checking. In the Drake backend, collision checking is already full-scene, so inter-arm collisions are caught once both robot states are installed in the context.

An optional small cleanup is `is_context_collision_free(ctx)` if using `is_collision_free(ctx, robot_ids[0])` reads too strangely.

## Preview and execution

`preview_path` and `execute` continue to be explicit user actions. They should read from the active `MotionPlan`:

```python
preview_path(duration, robot_name: str | list[str] | None)
execute(robot_name: str | list[str] | None)
```

Passing a string keeps existing single-robot behavior. Passing a list previews or executes the selected robot trajectories from the active plan. Passing `None` should keep current unambiguous single-robot default behavior; it should not silently choose a robot set in a multi-robot module.

Execution still submits split per-robot trajectories to existing coordinator trajectory tasks. This preserves coordinator architecture, but exact atomic start across multiple task RPCs is not guaranteed by this change.

## Manual QA blueprint

Add a mock dual-arm planner/coordinator blueprint named `dual-xarm6-mock-planner-coordinator`. It should be the primary no-hardware way to verify the implementation manually:

```bash
dimos run dual-xarm6-mock-planner-coordinator
python -i -m dimos.manipulation.planning.examples.demo_dual_arm_planning
```

The REPL example should call the same public manipulation RPC methods that users call directly:

```python
plan_to_joints([left_target, right_target], ["left_arm", "right_arm"])
preview_path(duration, ["left_arm", "right_arm"])
execute(["left_arm", "right_arm"])
```

It should also include a malformed-request helper that proves invalid list inputs fail without commanding motion or replacing the active coordinated plan.

## Non-goals

- No SRDF parsing.
- No named persistent planning groups.
- No true coupled multi-end-effector Cartesian IK.
- No coordinator redesign or atomic multi-task start in this change.
- No automatic execution after planning.
- No new MCP/LLM skill exposure in the first implementation.
