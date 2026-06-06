# Implementation checklist

## 1. Plan artifact model

- [ ] 1.1 Add an internal timed motion plan type that records ordered robot names, per-robot geometric paths, per-robot timed trajectories, total duration, and source kind.
- [ ] 1.2 Make the timed motion plan the authoritative active plan stored by the manipulation module.
- [ ] 1.3 Preserve existing per-robot planned path and trajectory access where current methods or tests rely on it.
- [ ] 1.4 Ensure failed planning attempts do not partially replace the active plan.

## 2. Multi-robot joint planning

- [ ] 2.1 Extend `plan_to_joints` to accept scalar single-robot inputs and ordered multi-robot list inputs.
- [ ] 2.2 Validate multi-robot input shape, duplicate robot names, unknown robots, target joint counts, and missing current state before sampling.
- [ ] 2.3 Build composite start, goal, joint names, and limits in caller-provided robot order.
- [ ] 2.4 Add a composite joint-space planning path that reuses the existing joint planner algorithm while setting all participating robot states in one scratch context for collision validation.
- [ ] 2.5 Reject colliding composite starts, goals, and edges without returning partial robot paths.

## 3. Multi-robot pose planning

- [ ] 3.1 Extend `plan_to_pose` to accept scalar single-robot inputs and ordered multi-robot list inputs.
- [ ] 3.2 For multi-robot pose inputs, solve IK independently per robot using existing pose semantics.
- [ ] 3.3 Feed the solved per-robot joint goals into the coordinated multi-robot joint planning path.
- [ ] 3.4 Return a clear failure when any robot's IK solve fails and avoid replacing the active plan.

## 4. Synchronized trajectory generation

- [ ] 4.1 Generate one combined timed trajectory from the composite joint path before splitting by robot.
- [ ] 4.2 Split the combined trajectory into per-robot trajectories that share total duration and `time_from_start` values.
- [ ] 4.3 Store the split trajectories in the active timed motion plan.
- [ ] 4.4 Preserve coordinator joint-name translation during execution.

## 5. Preview and execution behavior

- [ ] 5.1 Extend `preview_path` to accept a single robot name or ordered robot-name list while preserving existing scalar behavior.
- [ ] 5.2 Extend `execute` to accept a single robot name or ordered robot-name list while preserving existing scalar behavior.
- [ ] 5.3 Keep ambiguous `None` behavior unchanged for multi-robot modules unless exactly one robot is configured.
- [ ] 5.4 Ensure planning and preview never initiate hardware motion.
- [ ] 5.5 Document or expose execution start-skew limitations if per-robot coordinator task submission remains non-atomic.

## 6. Tests

- [ ] 6.1 Add unit tests for scalar planning compatibility.
- [ ] 6.2 Add unit tests for multi-robot input validation and deterministic ordering.
- [ ] 6.3 Add unit tests for active plan atomicity on failure.
- [ ] 6.4 Add unit tests proving split per-robot trajectories share duration and `time_from_start` values.
- [ ] 6.5 Add world/planner tests proving composite collision checks set all participating robots in one context.
- [ ] 6.6 Add manipulation module tests for multi-robot joint planning, multi-robot pose planning failure behavior, preview list behavior, and execute list behavior.

## 7. Manual QA blueprint and example

- [ ] 7.1 Add `dual-xarm6-mock-planner-coordinator` that composes the dual mock XArm6 coordinator with the dual XArm6 manipulation planner and Meshcat visualization.
- [ ] 7.2 Add `dimos.manipulation.planning.examples.demo_dual_arm_planning` with REPL helpers for `dual_plan_joints`, `dual_preview`, `dual_execute`, and one malformed multi-robot request.
- [ ] 7.3 Keep execution explicit in the example; planning and preview helpers must not command motion.
- [ ] 7.4 Regenerate `dimos/robot/all_blueprints.py` and update blueprint registry tests for the new blueprint name.

## 8. Documentation

- [ ] 8.1 Update `dimos/manipulation/planning/README.md` with the timed plan concept, the new mock dual-arm blueprint, and the multi-robot joint planning example.
- [ ] 8.2 Update `docs/capabilities/manipulation/readme.md` with user-facing multi-robot planning usage after implementation confirms final syntax.
- [ ] 8.3 Update `docs/capabilities/manipulation/openarm_integration.md` if coordinated planning should replace independent left/right examples.
- [ ] 8.4 Document non-goals: no SRDF parsing, no named planning groups, no true coupled Cartesian IK, and no automatic execution after planning.

## 9. Verification and manual QA

- [ ] 9.1 Run `openspec validate add-multi-robot-joint-planning-api`.
- [ ] 9.2 Run focused manipulation unit tests for changed planning/module code.
- [ ] 9.3 Run Drake/manipulation integration tests if dependencies are available in the environment.
- [ ] 9.4 Run docs validation commands for changed markdown docs.
- [ ] 9.5 Manually QA through `dimos run dual-xarm6-mock-planner-coordinator` plus the dual-arm planning REPL: successful coordinated joint plan, successful preview, explicit execution, and one malformed multi-robot request.
- [ ] 9.6 Run `pytest dimos/robot/test_all_blueprints_generation.py` and confirm the generated registry includes `dual-xarm6-mock-planner-coordinator`.
