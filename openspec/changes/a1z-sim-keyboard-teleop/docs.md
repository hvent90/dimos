## User-Facing Docs

- Update the A1Z keyboard-teleoperation usage documentation (or add a focused A1Z simulation section) with `dimos --simulation run keyboard-teleop-a1z` and the `[` open / `]` close controls.
- State that this is a deterministic manual-validation scene and that process restart is the Stage 1 reset mechanism.

## Contributor Docs

- None. The implementation and OpenSpec design document capture the converter, bridge, and scene assumptions; no general contributor workflow changes are introduced.

## Coding-Agent Docs

- None. Existing `AGENTS.md` guidance already covers blueprints, generated registry validation, and MuJoCo simulation composition.

## Doc Validation

- Run the repository's applicable Markdown link/documentation validation for the changed file, if available.
- Verify documented commands and key mappings during the Stage 1 manual smoke test.

## No Docs Needed

Documentation is required because the existing public blueprint gains an opt-in `--simulation` behavior and new keyboard controls.
