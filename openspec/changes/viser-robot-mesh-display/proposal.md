## Why

The Viser manipulation view currently renders only the robot's visual geometry.
Operators debugging planning or scene issues cannot inspect the collision shape
without leaving the view or changing the underlying robot model.

Adding an in-panel display choice makes collision geometry inspectable during a
live session while preserving the visual robot view as the default. It is a
view-only diagnostic capability: planning and execution behavior do not
change.

## What Changes

- Add a Viser sidebar `Robot display` control with `Visual`, `Collision`, and
  `Both` modes for the primary robot.
- Display collision meshes in diagnostic magenta (`#D228DC`) at 35% opacity.
- Apply mode changes immediately and retain the selection when the primary
  robot visual is recreated during the same session.
- Preload visual and collision representations for deterministic switching.
- In `Collision` and `Both`, clearly indicate when primary collision geometry is
  unavailable and visual meshes remain shown with the diagnostic magenta,
  translucent collision treatment so the selected mode remains obvious; do not
  show the notice in `Visual` or when collision geometry exists.

## Affected DimOS Surfaces

- Modules/streams: Viser manipulation scene and panel GUI only; no stream
  contract changes.
- Blueprints/CLI: None.
- Skills/MCP: None.
- Hardware/simulation/replay: View-only behavior with identical semantics in
  hardware, simulation, and replay.
- Docs/generated registries: Update the manipulation capability guide; no
  generated registries.

## Capabilities

### New Capabilities

- `viser-robot-mesh-display`: Operator-visible robot visual, collision, and
  combined mesh display modes in the Viser manipulation panel.

### Modified Capabilities

- None.

## Impact

Viser users gain an immediate collision-geometry debugging view with no public
API, CLI, robot-command, or planning-world compatibility change. Initial Viser
scene setup may use more memory because both representations are preloaded.
Implementation must validate the installed Viser API for independently managed
collision meshes, add focused scene and GUI tests, and update the manipulation
visualization documentation. The fallback notice remains scoped to the primary
robot and does not alter target or preview-ghost rendering.
