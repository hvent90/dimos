## Context

Keyboard arm teleoperation currently behaves like a small Cartesian pose controller. It reads coordinator joint state, depends on the robot FK model path and EEF joint id, seeds an absolute EEF pose, mutates that pose from key presses, and publishes `PoseStamped` targets to the coordinator. That couples a keyboard input module to robot-specific state that the coordinator task layer already owns.

DimOS also currently uses `Twist` for planar/base velocity conventions on `cmd_vel` and `twist_command`. A spatial EEF twist is semantically different: it represents translational and rotational end-effector velocity in 3D space and needs task routing. This change intentionally uses `TwistStamped` with `frame_id` as a temporary route key so the EEF path does not overload existing planar/base twist semantics.

## Goals / Non-Goals

### Goals

- Make keyboard arm teleop publish operator intent only: key mapping, speed scale, publish cadence, and stop/zero behavior.
- Move robot-derived EEF state, FK, target-pose integration, IK, safety checks, and timeout behavior into a coordinator-owned control task named `EEFTwistTask`.
- Add a routed spatial EEF twist stream using `TwistStamped.frame_id` to identify the destination task.
- Keep existing planar/base `twist_command` behavior unchanged.
- Update all current manipulator keyboard teleop blueprints so they no longer pass FK model and joint-state details into the keyboard module.

### Non-Goals

- Do not redesign global twist semantics across DimOS.
- Do not replace existing absolute Cartesian pose control or change `CartesianIKTask` behavior.
- Do not add MCP skills or agent-facing tools.
- Do not implement true differential IK in the initial version; integrate twist into a target pose and reuse the existing IK-style servo-position pattern.
- Do not make keyboard teleop responsible for robot workspace limits derived from geometry.

## DimOS Architecture

### Modules and Streams

The new manipulator keyboard flow is:

```text
KeyboardTeleopModule
  └─ coordinator_ee_twist_command: Out[TwistStamped]
       frame_id = target EEFTwistTask name
       linear/angular = spatial EEF twist intent

ControlCoordinator
  └─ coordinator_ee_twist_command: In[TwistStamped]
       routes by frame_id

EEFTwistTask
  └─ consumes routed spatial EEF twist
       uses CoordinatorState joint state
       FK seeds target pose
       twist integration updates target pose
       IK produces JointCommandOutput in SERVO_POSITION mode
```

The existing planar/base path remains separate:

```text
cmd_vel / twist_command: Twist
  └─ planar base twist convention: linear.x, linear.y, angular.z
```

`ControlCoordinator` should subscribe to the new EEF twist stream when configured tasks include `EEFTwistTask`. It should route `TwistStamped` messages by `frame_id`, matching the existing `PoseStamped.frame_id` routing pattern for Cartesian commands. Unmatched routed EEF twist commands should be ignored safely and logged/debuggable without affecting other tasks.

### Control Task

`EEFTwistTask` should follow existing control-task patterns:

- Configured through `TaskConfig` and discovered via the task registry package layout.
- Uses coordinator-provided state rather than adding new RPC/module references back into keyboard teleop.
- Produces `JointCommandOutput` using the normal coordinator arbitration path.
- Uses the same IK model inputs and joint-delta safety concepts as `CartesianIKTask` where applicable.

The task should expose a task-level twist command hook, either by extending the task protocol with an EEF twist hook or by using an internal coordinator route that only calls tasks that implement the method. The hook should accept `TwistStamped` plus coordinator time so timeout and integration semantics are deterministic.

### Blueprints, CLI, and Generated Registries

Current manipulator keyboard teleop blueprints should switch from `CartesianIKTask`-backed absolute pose teleop to `EEFTwistTask`-backed spatial EEF twist teleop. This includes xArm, Piper, OpenArm, and A-750 keyboard teleop blueprints. The keyboard module configuration should no longer include `model_path`, `ee_joint_id`, `joint_names`, `home_joints`, or joint-state timeout values. Robot-specific model details should move to the `EEFTwistTask` helper in manipulator common blueprints.

The new task package needs a registry entry so `dimos/control/tasks/registry.py` can discover the task type. If the new blueprint helper or task constants affect generated DimOS blueprint registries, the implementation tasks should include the repository's existing blueprint-generation validation.

No user-facing CLI command shape changes are expected; existing `dimos run keyboard_teleop_xarm*` style workflows should continue with revised internal wiring.

### Specs, Adapter Protocols, Skills, and MCP

No DimOS Spec Protocol or adapter Protocol changes are expected unless implementation discovers a need for typed cross-module RPC. The keyboard-to-coordinator connection should remain stream-based. No skills or MCP server/client surfaces should change.

## Decisions

1. **Use `TwistStamped` for spatial EEF twist.** `frame_id` routes to a task name, while the vector fields carry spatial EEF velocity intent.
2. **Keep a dedicated EEF twist coordinator stream for now.** This is known compatibility debt, but it avoids overloading existing planar/base twist semantics.
3. **Name the control task `EEFTwistTask`.** The term is specific enough to distinguish it from planar/base velocity tasks.
4. **Interpret initial spatial EEF twist in the coordinator/world frame.** This keeps the first version simpler and should be documented in code and tests.
5. **Use twist-integrated pose IK initially.** Seed from current FK on the first nonzero command, integrate velocity over elapsed coordinator time, solve IK, and output servo-position commands.
6. **Treat released movement keys as stop/reset intent.** A zero twist clears the active target; the next nonzero command should re-seed from current FK rather than resume from a stale integrated pose.

### `EEFTwistTask` State Machine

- **Idle:** no active twist and no integrated target. `is_active()` returns false.
- **First nonzero command:** stream callback stores the latest finite twist and update time only. The next `compute(state)` seeds the target from current FK using `CoordinatorState`, then integrates one bounded step.
- **Active command:** each `compute(state)` uses coordinator `state.t_now` and a bounded `dt` to integrate the latest twist into the target pose.
- **Zero command:** stream callback clears the latest twist and target; the task becomes inactive. The next nonzero command re-seeds from current FK.
- **Timeout:** if no fresh command arrives before the configured timeout, clear the latest twist and target and become inactive.
- **IK/safety rejection:** if IK output is non-finite, unconverged beyond the accepted error policy, or violates joint-delta safety, reject the output and do not commit an additional integrated target step farther into invalid space.

## Safety / Simulation / Replay

- The task should stop commanding when EEF twist commands time out, with an initial timeout in the 0.2-0.5s range.
- Zero twists and keyboard stop actions should clear the active twist target and prevent stale motion.
- IK failures, invalid target poses, and excessive joint deltas should reject the current command without crashing the coordinator.
- Spatial EEF twist inputs, integrated target poses, and IK outputs should be finite before use.
- Integration should bound elapsed `dt` and per-tick linear/angular step sizes to prevent large jumps after scheduler stalls or delayed callbacks.
- IK non-convergence should have an explicit final-error policy rather than blindly accepting every partial solution.
- IK/joint-delta rejection should avoid committing unsafe target advancement.
- Simulated manipulator stacks should exercise the same stream/task behavior as hardware stacks.
- Replay should remain usable for coordinator/task validation because robot state stays inside coordinator-managed state rather than keyboard-local FK state.
- Real hardware rollout should start with low speed defaults and existing manipulator safety constraints.

## Risks / Trade-offs

- **Temporary stream debt:** `coordinator_ee_twist_command` is a narrow compatibility path. A future refactor should unify twist routing/type semantics more cleanly.
- **Frame semantics:** Coordinator/world-frame interpretation is simple but may not match every operator expectation. Tool-frame support can be added later as an explicit capability.
- **Integrated pose drift:** Integrating twist into pose and solving IK is simpler than differential IK, but long-running commands may accumulate drift or hit IK limits.
- **Safety ownership moves:** Removing keyboard workspace clamps means task-level safety checks must be sufficient before hardware use.
- **Task routing mismatch:** Incorrect `frame_id` values can silently disable teleop unless logging/tests make mismatches visible.

## Migration / Rollout

1. Add the new routed EEF twist stream and task hook without changing existing planar/base twist handling.
2. Add and register `EEFTwistTask` with tests for routing, integration, timeout, reset, IK rejection, and output mode.
3. Refactor keyboard arm teleop to publish `TwistStamped` intent and remove robot-state/FK dependencies.
4. Update current manipulator keyboard teleop blueprints to use the new task helper and task name.
5. Validate existing absolute Cartesian IK users still pass.
6. Update relevant docs/glossary references and run OpenSpec validation plus targeted unit tests.

## Open Questions

- None for the initial implementation. Future work should revisit global twist routing/type semantics and optional tool-frame spatial EEF twist commands.
