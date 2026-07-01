## User-Facing Docs

- Update manipulator keyboard teleop usage documentation if it describes absolute pose jogging, FK model parameters, or current `PoseStamped` command behavior.
- Document that keyboard arm teleop now sends spatial EEF twist intent and that releasing movement keys sends zero twist to clear active motion rather than re-syncing an absolute pose.
- If blueprint docs list task internals, update manipulator keyboard teleop examples to mention `EEFTwistTask` instead of `CartesianIKTask` for keyboard EEF teleop.

## Contributor Docs

- Update control/teleop contributor docs with the semantic split between:
  - planar base twist: existing `Twist` convention for base-like motion, and
  - spatial EEF twist: routed `TwistStamped` convention for end-effector motion.
- Document `coordinator_ee_twist_command` as a temporary compatibility stream and note the future debt around global twist routing/type semantics.
- Add or update task-development notes for `EEFTwistTask`: routed command handling, FK seeding, twist integration, timeout/reset behavior, and IK safety rejection.

## Coding-Agent Docs

- The root `CONTEXT.md` glossary already defines the canonical terms "planar base twist" and "spatial EEF twist". Keep it implementation-detail free.
- If `docs/coding-agents/` has control or teleop guidance, add a short note: keyboard teleop should publish operator intent only; robot state, FK, IK, workspace/safety checks, and timeouts belong in coordinator tasks.

## Doc Validation

- Run OpenSpec validation for the change after artifacts are complete.
- Run targeted documentation checks only if the edited docs have a specific checker in the repository.
- Use targeted tests from the implementation plan to validate that docs match behavior, especially keyboard stop semantics and planar/base twist compatibility.

## No Docs Needed

- No MCP/agent skill documentation is needed because this change does not add skills or agent-facing tools.
- No CLI reference update is expected unless implementation changes command names or flags; current blueprint run commands should remain stable.
