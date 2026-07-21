## Context

The Viser manipulation visualizer renders the robot's visual URDF geometry for
the current robot, target, and transient preview representations. Operators
need a fast way to inspect the collision representation while debugging
planning and scene issues, without restarting visualization or changing the
planning world.

The control belongs in the enabled Viser sidebar. It affects only the primary
robot representation, not target or preview ghosts, and applies uniformly to
all of that robot's links.

## Goals / Non-Goals

**Goals:**

- Provide a `Robot display` sidebar section with a keyboard-accessible,
  text-labelled three-way selector: `Visual`, `Collision`, and `Both`.
- Default to `Visual` for every new visualization session.
- Apply a selection immediately while the robot is moving and preserve it when
  the primary robot visual is rebuilt during the same session.
- Preload visual and collision geometry so every selection change is immediate.
- Render collision geometry in diagnostic magenta (`#D228DC`) at fixed 35%
  opacity in both `Collision` and `Both` modes.
- Preserve the existing graceful behavior for models without collision meshes:
  retain the selected mode, render the available visual geometry with the
  diagnostic magenta (`#D228DC`) 35%-opacity collision treatment, and clearly
  tell the user that visual meshes remain shown in `Collision` and `Both`.
  Suppress that notice in `Visual` and whenever collision geometry exists.

**Non-Goals:**

- Selecting geometry independently for individual links.
- Changing target or preview-ghost rendering, planning collision semantics,
  collision exclusions, or robot model configuration.
- Persisting the choice across browser reloads or visualization sessions.
- Adding a public CLI, RPC, skill/MCP surface, or configuration option.

## DimOS Architecture

No module, stream, transport, blueprint, DimOS Python `Spec` Protocol, or
skill/MCP interface changes are required. The behavior is local to the
in-process Viser visualization:

```text
ViserPanelGui selector
        │ selection changed
        ▼
ViserManipulationScene display-mode state
        │ applies visibility/material state
        ▼
primary robot visual and collision mesh handles
```

`ViserPanelGui` owns the sidebar affordance. `ViserManipulationScene` owns the
session-scoped mode and the preloaded mesh handles, applies the mode whenever
the primary robot is created or replaced, and leaves target/preview handles
unchanged. `ViserManipulationVisualizer` continues to own lifecycle and joint
state publication.

The implementation must obtain the collision representation from the same
prepared robot model source used by Viser, without changing `RobotModelConfig`
or the planning world's collision model. The exact Viser/URDF loading seam is
an implementation decision to validate against the installed Viser API.

## Decisions

### A three-way selector represents a closed display state

The domain state is `visual | collision | both`. A segmented selector avoids
the invalid and ambiguous "neither mesh" state that two independent toggles
would permit. Tooltips explain each option:

- **Visual:** Render the robot's visual mesh.
- **Collision:** Render the robot's transparent collision mesh.
- **Both:** Overlay the transparent collision mesh on the visual mesh.

### Collision geometry is a readable debugging overlay

Collision geometry always uses diagnostic magenta (`#D228DC`) at 35% opacity.
In `Both`, visual geometry remains solid underneath. Magenta is distinct from
the existing orange feasible-target, red infeasible-target, and blue preview
semantics, without introducing another material-control surface.

### Mode is session state, not robot-model state

The selector defaults to `Visual`; it does not persist to the browser, robot
configuration, or planning world. A robot reload in the open session reapplies
the current selection so the UI remains the source of truth.

### Missing collision geometry follows existing fallback behavior

The interface does not disable choices. If no collision meshes are available,
`Collision` and `Both` gracefully render the available visual geometry using the
diagnostic magenta (`#D228DC`) at 35% opacity, preserving the selected mode's
visual meaning. The panel accurately indicates that collision geometry is
unavailable and visual meshes remain shown. This notice is suppressed in
`Visual` and when collision geometry exists, and applies only to the primary
robot; target and preview-ghost rendering remains unchanged.

### Geometry is preloaded

Both representations are prepared when the primary robot is initialized,
trading startup memory/load time for deterministic live switching. The design
does not allow a selector change to trigger model loading, scene reset, or
planning-world interaction.

## Safety / Simulation / Replay

This is a view-only debugging affordance. It must not change robot commands,
planning inputs, collision checking, execution, or the live Drake context.
It has identical behavior with hardware, simulation, and replay because it
only changes Viser scene visibility and material properties. Manual QA should
include switching modes while joint states are updating.

## Risks / Trade-offs

- Preloading collision meshes increases initial scene setup cost and memory
  use. Mitigate by retaining one mesh-handle set per primary robot only.
- Viser's public URDF helper may not expose collision-only handles. Validate
  the supported API before relying on private handles; isolate any required
  compatibility adapter in the scene layer.
- The conditional missing-collision notice and substituted-visual styling must
  remain accurate without becoming a warning in normal `Visual` mode or when
  collision geometry exists; tests must verify both notice/no-notice paths and
  the diagnostic material treatment.

## Migration / Rollout

No migration, configuration update, generated registry update, or deployment
step is needed. Add behavior tests for the selector, default, live mode
changes, reload preservation, opacity/material, and missing-collision fallback.
Update `docs/capabilities/manipulation/index.md` once the implementation is
released to describe the operator-facing panel control. Validate changed docs
with the repository's documentation-link check when available.

## Open Questions

- Which installed Viser API is the supported way to load or expose collision
  mesh handles independently of visual mesh handles?
