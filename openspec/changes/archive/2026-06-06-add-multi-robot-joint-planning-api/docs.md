# Documentation impact

## User-Facing Docs

- Update `dimos/manipulation/planning/README.md` to explain that a successful plan is a time-parameterized motion plan, not only a geometric path.
- Document `dual-xarm6-mock-planner-coordinator` as the preferred no-hardware manual QA blueprint for coordinated dual-arm planning.
- Update `docs/capabilities/manipulation/readme.md` with the multi-robot joint planning flow once implementation confirms the final callable syntax.
- Update `docs/capabilities/manipulation/openarm_integration.md` if OpenArm dual-arm examples should use coordinated planning instead of independent left/right planning.
- Include examples for:
  - scalar single-robot `plan_to_joints` / `plan_to_pose` compatibility,
  - ordered multi-robot joint targets,
  - ordered multi-robot pose targets using current IK-then-joint-plan semantics,
  - explicit preview and execution of a selected robot set.
- Include the manual verification REPL flow:
  - `dimos run dual-xarm6-mock-planner-coordinator`,
  - `python -i -m dimos.manipulation.planning.examples.demo_dual_arm_planning`,
  - `dual_plan_joints()`, `dual_preview()`, `dual_execute()`, and `bad_request()`.
- Document non-goals explicitly: no SRDF parsing, no named planning groups, no true coupled multi-end-effector Cartesian IK, and no automatic hardware motion after planning.

## Contributor Docs

- No new contributor workflow docs are required.
- Add the mock blueprint plus REPL sequence to existing manipulation testing or planning docs as the reusable manual QA procedure; do not create a separate contributor guide unless the procedure grows beyond manipulation planning.

## Coding-Agent Docs

- No `AGENTS.md` or `docs/coding-agents/` update is expected.
- If implementation establishes new conventions for manipulation planning APIs, add them to the manipulation planning docs rather than coding-agent-specific docs.

## Doc Validation

- Run `openspec validate add-multi-robot-joint-planning-api`.
- For changed markdown docs, run the repository's relevant markdown/doc validation from `docs/development/openspec.md` and `docs/development/writing_docs.md` if available.
- For runnable Python snippets added to docs, run `md-babel-py run <doc>` when the snippet is intended to execute in docs validation.

## No Docs Needed

- Not applicable. This change alters developer-facing manipulation planning behavior and needs user-facing manipulation docs.
