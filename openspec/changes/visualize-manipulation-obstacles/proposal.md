## Why

Manipulation planning obstacles are currently accepted by the planning world without a read-only visual representation, making it difficult to verify the world used by the planner. Viser rendering will provide immediate visual feedback while preserving the planner as the source of truth.

This change is intentionally limited to obstacle add/remove visualization in the manipulation planning stack. It establishes predictable synchronization and failure feedback without introducing a general viewer architecture.

## What Changes

- Add optional Viser visualization for obstacles accepted into the manipulation planning world.
- When visualization is enabled, initialize the Viser backend before floor or any obstacle mutation.
- Give `WorldMonitor` explicit coordinated add/remove helpers that call `WorldSpec` first and then, when available, the optional Viser visualizer.
- Route `WorldObstacleMonitor` through those helpers so successful world mutations are forwarded exactly once and failed mutations are not forwarded.
- Do not add native-world hooks; `WorldMonitor` is the sole coordination seam.
- Keep visualization disabled as a true no-op with no visualizer calls.
- Render planner-parity box, sphere, cylinder, and mesh obstacles.
- Provide one local `manipulation.obstacles` visibility checkbox, default visible, that hides/shows obstacle entities without losing their render state.
- On mesh rendering failure, retain a local proxy and label it as failed rather than silently dropping the obstacle.
- Scope the initial behavior to add/remove only; pose updates are explicitly out of scope, and no general viewer architecture or broader mutation synchronization is introduced.

## Affected DimOS Surfaces

- Modules/streams: Manipulation planning-world obstacle module and its add/remove mutation path; no new inter-module stream is required.
- Blueprints/CLI: The xarm6 planner-only blueprint gains the optional visualization wiring/configuration; no CLI command changes.
- Skills/MCP: None.
- Hardware/simulation/replay: Read-only visualization of planner state; affects the manual xarm6 planner-only test and does not change robot actuation.
- Docs/generated registries: No general viewer documentation or generated registry changes are in scope.

## Capabilities

### New Capabilities

- `manipulation-obstacle-visualization`: Optional read-only Viser rendering and exact add/remove synchronization for accepted manipulation planning obstacles.

### Modified Capabilities

- None.

## Impact

Developers and operators can inspect the manipulation planning world visually when enabled, while existing behavior remains unchanged when disabled. The visualization backend introduces a Viser dependency/configuration surface and must be attached before world mutation. Planner geometry remains authoritative; mesh failures are visible through a local proxy and label. QA includes automated mutation/disabled-path coverage where applicable and a manual xarm6 planner-only check covering initialization order, add/remove parity, visibility state preservation, geometry parity, and mesh-failure feedback.
