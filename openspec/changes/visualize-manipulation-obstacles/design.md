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

- When Viser is selected, initialize its server/scene and install a direct mutation
  hook on the already-created concrete planning-world object before the floor or
  any other obstacle can be added.
- Forward each actual successful obstacle add and remove synchronously and exactly
  once, using the accepted obstacle/ID and the matching Viser scene operation.
- Render planner-parity boxes, spheres, cylinders, and meshes under a local
  `manipulation.obstacles` scene namespace.
- Provide one local visibility checkbox, defaulting to visible, which changes
  entity visibility without deleting render handles or losing state.
- Show a local proxy and a failure label when a mesh cannot be rendered, rather than
  silently dropping an accepted planner obstacle.
- Keep the concrete world identity intact and preserve the no-visualization path as
  a true no-op.

**Non-Goals:**

- No proxy/decorator world, replacement world, mutation polling, event queue, or
  periodic obstacle reconciliation.
- No new inter-module stream, CLI command, skill, MCP surface, or general viewer
  architecture.
- No dynamic obstacle pose synchronization in this change.  Pose updates remain
  deferred because Drake's collision/visual pose behavior currently differs; an
  update must not be inferred from add/remove hooks.
- No change to robot visualization, planner collision semantics, or robot actuation.

## DimOS Architecture

The existing flow remains `ManipulationModule` → `WorldMonitor` → concrete
`DrakeWorld`/`RoboPlanWorld`.  `WorldSpec` and `VisualizationSpec` remain the
public contracts; no DimOS `Spec` Protocol or stream is added.  The hook is an
internal, narrowly typed obstacle-mutation adapter/callback installed on the
concrete world instance, so the world continues to be passed to planners and
monitors by its original identity.

Startup is reordered as follows:

1. Create the concrete world with the existing backend factory and create planning
   specs around that exact instance.
2. Construct the optional visualization from the existing visualization factory.
   With Viser enabled, eagerly start the Viser runtime and initialize its scene.
3. Construct the `WorldMonitor` with the visualization (or attach it before any
   obstacle mutation), add robots, and finalize the world as today.
4. Install the obstacle hook on the concrete world before adding the floor.  The
   hook is absent for `NoManipulationVisualizationConfig` and for non-Viser
   visualization paths that do not opt into obstacle rendering.
5. Add the floor and start the normal monitors.  All subsequent monitor, RPC, and
   perception add/remove calls use the same world and therefore pass through the
   hook.

The concrete world implementation invokes the callback only after its own
mutation and bookkeeping succeed.  `add_obstacle` supplies the accepted
`Obstacle` and returned ID; `remove_obstacle` supplies the removed ID only when it
returns `True`.  Duplicate/no-op adds and missing-ID removes do not produce
visualization calls.  Callback invocation is direct in the mutation call stack,
not queued or polled.  Visualization errors are logged and isolated from the
planner mutation after the world has committed its authoritative state.

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

### Direct hook on the native world

Install a callback on the existing concrete `DrakeWorld`/`RoboPlanWorld` instance
before floor creation.  This preserves `world is planning_specs.world` and avoids
changing planner, monitor, or backend type checks.  A wrapper world was rejected
because it obscures identity and risks missing backend-internal mutations.

### Commit-before-forward and exact-once forwarding

Each backend completes its native add/remove first, then invokes the matching
visual operation once.  This makes the planner the source of truth and prevents
failed operations from appearing in Viser.  Duplicate and missing-object no-ops
are not visual mutations.  The callback executes synchronously under the existing
world/monitor mutation lock, giving deterministic ordering without a queue.

### Read-only Viser scene

Obstacle handles are created and removed by planner events only.  Viser controls
only local visibility; it never edits poses, dimensions, or planner state.  Pose
updates are intentionally not hooked until Drake's dynamic-pose discrepancy is
resolved.

### Geometry and mesh failure behavior

Primitive geometry uses the same dimensions and world-frame pose as the planner.
Mesh load failures are visible and diagnosable through a proxy plus label, while
the accepted obstacle remains in the planner world.  Silently omitting a mesh was
rejected because it falsely suggests planner/visual parity.

### Lifecycle and threading

Viser startup occurs before the first obstacle mutation; teardown removes the hook
before closing the Viser scene/runtime so late monitor callbacks cannot target
closed handles.  Direct scene calls are protected by the scene's lifecycle/scene
lock as needed, and callback failures do not hold up world authority.  The
existing visualization thread continues to publish robot state only; it does not
poll or reconcile obstacles.

## Safety / Simulation / Replay

This is read-only visualization and does not send commands to a robot or alter
collision checking, so hardware safety behavior is unchanged.  Real hardware,
simulation, and replay use the same accepted-world mutation path; the viewer only
reflects the selected planning backend.  The Viser backend is optional and must be
explicitly enabled; disabled visualization creates no server, scene, hook, or
Viser dependency access.

The manual xArm6 planner-only check should enable Viser and verify that startup
shows the floor, primitive and mesh obstacle parity, exact add/remove behavior,
the `manipulation.obstacles` checkbox, and mesh failure proxy/label feedback.  It
should also verify that planning and robot actuation are unchanged.  Automated
tests should cover both Drake and backend-independent hook behavior where
available, without requiring a live browser.

## Risks / Trade-offs

- Viser API differences or unavailable mesh assets can prevent a native mesh
  handle.  Keep primitive rendering independent and use the proxy/label fallback;
  test scene calls with fakes.
- Direct callbacks run on obstacle-monitor/RPC threads and may briefly add Viser
  latency.  This is intentional for exact ordering; avoid blocking network or
  polling infrastructure and keep the scene operation bounded.
- A visualization callback can fail after a successful planner mutation.  Log the
  failure and retain planner state; do not roll back native world state.  The mesh
  fallback specifically prevents silent loss for the expected asset-load failure.
- Lifecycle races could call a closed scene.  Install/remove the hook under the
  same lifecycle lock and stop obstacle monitors before visualization teardown.
- Existing Meshcat behavior must not regress; its world-native visualization path
  remains unchanged and does not install the Viser hook.

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
hook.

## Open Questions

- Confirm the exact Viser primitive handle constructors and text-label API against
  the pinned optional dependency while implementing the scene adapter; the
  observable fallback contract is fixed, but method names are not.
- Decide the concise label text and proxy dimensions/color that best communicate a
  failed mesh without obscuring nearby planner geometry.
- A future change may define a stable dynamic-pose event contract after Drake's
  collision and visual pose semantics are reconciled; it is intentionally not part
  of this design.
