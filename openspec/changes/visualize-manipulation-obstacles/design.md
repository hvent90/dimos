## Context

`ManipulationModule._initialize_planning()` creates a concrete `WorldSpec` through
`dimos/manipulation/planning/factory.py`, builds its `WorldMonitor`, adds the
configured robots, finalizes the world, and then adds the optional floor.  The
monitor currently delegates `add_obstacle()` and `remove_obstacle()` directly to
the world, while `VisualizationSpec` only covers scene startup, robot state, and
preview operations.  The existing in-process Viser implementation is under
`dimos/manipulation/visualization/viser/` and is selected by
`create_manipulation_visualization()`.

The planner world remains authoritative.  `Obstacle` carries the planner geometry
(`BOX`, `SPHERE`, `CYLINDER`, or `MESH`), pose, dimensions, color, and mesh path;
Viser must consume that same value after the world accepts it.  The current startup
ordering creates the visualization too late for the floor mutation, and no
visualization operation is tied to an individual world mutation.

## Goals / Non-Goals

**Goals:**

- When Viser is selected, initialize its server/scene before the floor or any
  other obstacle can be added.
- Add explicit coordinated `WorldMonitor` add/remove helpers. Each helper calls
  `WorldSpec` first and then, on success, calls the optional Viser visualizer
  synchronously and exactly once with the accepted obstacle/ID.
- Route `WorldObstacleMonitor` through the `WorldMonitor` helpers rather than
  calling the world or visualizer directly.
- Render planner-parity boxes, spheres, cylinders, and meshes under a local
  `manipulation.obstacles` scene namespace.
- Provide one local visibility checkbox, defaulting to visible, which changes
  entity visibility without deleting render handles or losing state.
- Show a local proxy and a failure label when a mesh cannot be rendered, rather than
  silently dropping an accepted planner obstacle.
- Preserve the existing world identity and preserve the no-visualization path as
  a true no-op.

**Non-Goals:**

- No proxy/decorator world, replacement world, mutation polling, event queue, or
  periodic obstacle reconciliation.
- No new inter-module stream, CLI command, skill, MCP surface, or general viewer
  architecture.
- No dynamic obstacle pose synchronization in this change.  Pose updates remain
  deferred because Drake's collision/visual pose behavior currently differs; an
  update must not be inferred from add/remove coordination.
- No change to robot visualization, planner collision semantics, or robot actuation.

## DimOS Architecture

The existing flow remains `ManipulationModule` → `WorldMonitor` → concrete
`DrakeWorld`/`RoboPlanWorld`.  `WorldSpec` and `VisualizationSpec` remain the
public contracts; no DimOS `Spec` Protocol or stream is added. `WorldMonitor`
owns the coordination seam: its explicit obstacle add/remove helpers invoke
`WorldSpec` and then an optional Viser visualizer. No native-world hook,
world decorator, or callback is installed on the concrete world.

Startup is reordered as follows:

1. Create the concrete world with the existing backend factory and create planning
   specs around that exact instance.
2. Construct the optional visualization from the existing visualization factory.
   With Viser enabled, eagerly start the Viser runtime and initialize its scene.
3. Construct the `WorldMonitor` with the visualization (or attach it before any
   obstacle mutation), add robots, and finalize the world as today.
4. Add the floor through the `WorldMonitor` coordinated add helper. The helper
   calls `WorldSpec` first and forwards only a successful mutation to Viser.
5. Start the normal monitors. All subsequent RPC and perception add/remove calls
   from `WorldObstacleMonitor` use the same `WorldMonitor` seam.

The `WorldMonitor` helpers invoke the `WorldSpec` mutation first. `add_obstacle`
supplies the accepted `Obstacle` and returned ID to Viser only on success;
`remove_obstacle` supplies the removed ID only when `WorldSpec` returns `True`.
Duplicate/no-op adds and missing-ID removes do not produce visualization calls.
Coordination is direct in the mutation call stack, not queued or polled.
Visualization errors are logged and isolated after the world has committed its
authoritative state.

The Viser scene owns an obstacle-handle map keyed by world obstacle ID beneath
`/manipulation/obstacles/<id>`.  Primitive dimensions, pose, color, and opacity
are mapped directly from `Obstacle`.  Meshes use the supplied path; if Viser
cannot load the mesh, the scene creates a small conspicuous proxy at the obstacle
pose and a sibling/local text label containing the failure.  Removing an obstacle
removes its handle, proxy, and label together.  A single checkbox named
`manipulation.obstacles` controls visibility for all handles in that namespace;
changing it only toggles `visible` and retains the handle map and render state.

The Viser visualizer gets explicit obstacle methods (or an equivalent internal
adapter) rather than expanding `VisualizationSpec` with backend-specific methods.
The existing `initialize_scene()` remains responsible for robot metadata; obstacle
startup state is not reconstructed by polling `get_obstacles()`.  Consequently the
floor and every later accepted add are observed in order, while a failed world
mutation never reaches Viser.

No skills or MCP tools are exposed.  The existing xArm6 planner-only blueprint
configuration is the manual integration surface; generated blueprint registries
are unchanged.  If configuration/dependency wiring is needed for the existing
optional Viser extra, it should use the current visualization config and package
conventions rather than adding a new CLI flag.

## Decisions

### Explicit monitor coordination

Keep the concrete `DrakeWorld`/`RoboPlanWorld` unchanged and let `WorldMonitor`
coordinate each supported mutation. This keeps `WorldSpec` authoritative,
preserves world identity, makes the optional visualizer dependency explicit, and
avoids native-world hooks, wrappers, and backend-internal coupling.

### Commit-before-forward and exact-once forwarding

Each `WorldMonitor` helper completes the `WorldSpec` add/remove first, then invokes
the matching visual operation once. This makes the planner the source of truth
and prevents failed operations from appearing in Viser. Duplicate and missing-
object no-ops are not visual mutations. The helper executes synchronously under
the existing monitor mutation lock, giving deterministic ordering without a
queue.

### Read-only Viser scene

Obstacle handles are created and removed by planner events only.  Viser controls
only local visibility; it never edits poses, dimensions, or planner state.  Pose
updates are intentionally not synchronized until Drake's dynamic-pose discrepancy is
resolved.

### Geometry and mesh failure behavior

Primitive geometry uses the same dimensions and world-frame pose as the planner.
Mesh load failures are visible and diagnosable through a proxy plus label, while
the accepted obstacle remains in the planner world.  Silently omitting a mesh was
rejected because it falsely suggests planner/visual parity.

### Lifecycle and threading

Viser startup occurs before the first obstacle mutation. Teardown stops obstacle
monitor activity before closing the Viser scene/runtime so late coordinated calls
cannot target closed handles. Direct scene calls are protected by the scene's
lifecycle/scene lock as needed, and visualizer failures do not hold up world
authority. The existing visualization thread continues to publish robot state
only; it does not poll or reconcile obstacles.

## Safety / Simulation / Replay

This is read-only visualization and does not send commands to a robot or alter
collision checking, so hardware safety behavior is unchanged.  Real hardware,
simulation, and replay use the same accepted-world mutation path; the viewer only
reflects the selected planning backend.  The Viser backend is optional and must be
explicitly enabled; disabled visualization creates no server, scene, or
Viser dependency access.

The manual xArm6 planner-only check should enable Viser and verify that startup
shows the floor, primitive and mesh obstacle parity, exact add/remove behavior,
the `manipulation.obstacles` checkbox, and mesh failure proxy/label feedback.  It
should also verify that planning and robot actuation are unchanged.  Automated
tests should cover both Drake and backend-independent coordination behavior where
available, without requiring a live browser.

## Risks / Trade-offs

- Viser API differences or unavailable mesh assets can prevent a native mesh
  handle.  Keep primitive rendering independent and use the proxy/label fallback;
  test scene calls with fakes.
- Direct visualizer calls run on obstacle-monitor/RPC threads and may briefly add Viser
  latency.  This is intentional for exact ordering; avoid blocking network or
  polling infrastructure and keep the scene operation bounded.
- A visualizer call can fail after a successful planner mutation. Log the
  failure and retain planner state; do not roll back native world state.  The mesh
  fallback specifically prevents silent loss for the expected asset-load failure.
- Lifecycle races could call a closed scene. Stop obstacle monitors before
  visualization teardown and guard the coordinated visualizer seam with the
  lifecycle lock.
- Existing Meshcat behavior must not regress; its existing visualization path
  remains unchanged and does not opt into the Viser visualizer seam.

## Migration / Rollout

The change is backward compatible for `none` and Meshcat configurations.  Existing
Viser configuration fields (host, port, browser, panel, and preview settings) are
retained; obstacle visibility is a local scene GUI state, not persisted planner
configuration.  No migration, generated registry regeneration, documentation
update, or CLI deployment step is required.  The implementation should be limited
to the manipulation world/monitor startup path, Viser scene/visualizer/config GUI
as needed, and focused tests; no other artifacts are changed.

Validation consists of the focused manipulation and Viser tests plus the manual
xArm6 planner-only check.  Rollback is simply selecting `backend="none"` (or
removing the Viser wiring); native world behavior remains available without the
the coordinated visualizer seam.

## Open Questions

- Confirm the exact Viser primitive handle constructors and text-label API against
  the pinned optional dependency while implementing the scene adapter; the
  observable fallback contract is fixed, but method names are not.
- Decide the concise label text and proxy dimensions/color that best communicate a
  failed mesh without obscuring nearby planner geometry.
- A future change may define a stable dynamic-pose event contract after Drake's
  collision and visual pose semantics are reconciled; it is intentionally not part
  of this design.
