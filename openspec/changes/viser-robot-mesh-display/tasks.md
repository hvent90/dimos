## 1. Implementation

- [x] 1.1 Inspect the installed Viser version and its supported URDF/mesh APIs for independently managed collision geometry handles, documenting the chosen public loading seam or isolating any required compatibility adapter in `dimos/manipulation/visualization/viser/scene.py`.
- [x] 1.2 Add session-scoped `visual | collision | both` state to `ViserManipulationScene`, defaulting to `visual`, and represent/preload primary visual and collision mesh handles from the prepared robot model without changing planning or robot-model configuration.
- [x] 1.3 Apply visibility and material state uniformly to every primary-robot link: visual geometry for `visual`, diagnostic magenta `#D228DC` at 35% opacity for collision geometry in `collision` and `both`, including substituted visual geometry when collision meshes are unavailable, while leaving target and preview handles unchanged.
- [x] 1.4 Add the accessible, text-labelled `Robot display` selector with `Visual`, `Collision`, and `Both` options to `dimos/manipulation/visualization/viser/gui.py`, and connect its callback to the scene mode setter without blocking joint-state updates.
- [x] 1.5 Reapply the selected mode whenever the primary robot representation is created or recreated, preserve the selector value for the current session, and retain the available visual geometry with the diagnostic magenta translucent treatment and accurate panel indication when collision geometry is missing.
- [x] 1.6 Add focused scene and GUI coverage in `dimos/manipulation/visualization/viser/test_viser_visualization.py` and `dimos/manipulation/visualization/viser/test_gui_status.py` for the visual default, all display modes, collision and substituted-visual material/opacity, missing-collision indication, target/preview exclusion, immediate mode changes, and accessible selector callback.
- [x] 1.7 Extend lifecycle coverage in `dimos/manipulation/visualization/viser/test_visualizer_lifecycle.py` (and related scene tests as needed) to verify mode preservation across primary representation recreation and continued joint updates.

## 2. Documentation

- [x] 2.1 Update `docs/capabilities/manipulation/index.md` to describe the Viser `Robot display` control, its `Visual` default and `Collision`/`Both` modes, magenta 35% collision and substituted-visual rendering, view-only immediate behavior, session recreation preservation, unchanged target/preview ghosts, and the accurate panel indication for models without collision meshes.
- [x] 2.2 Do not update or regenerate the blueprint registry: no blueprint names, module classes, or generated-registry inputs change in this capability.

## 3. Verification

- [x] 3.1 Run `openspec validate viser-robot-mesh-display`.
- [x] 3.2 Run `uv run pytest dimos/manipulation/visualization/viser/test_viser_visualization.py dimos/manipulation/visualization/viser/test_gui_status.py dimos/manipulation/visualization/viser/test_visualizer_lifecycle.py`.
- [x] 3.3 Run `uv run mypy dimos/manipulation/visualization/viser/` if the implementation changes typed Python code in that package.
- [x] 3.4 Run `uv run doclinks --check docs/`.
- [x] 3.5 Manually QA the Viser manipulation panel: confirm the default and all three display modes, collision and substituted-visual material/opacity, missing-collision indication, unchanged target/preview ghosts, recreation preservation, and switching modes while joint updates are live.

## 4. Collision fallback indicator

- [x] 4.1 Add a primary-robot-scoped Viser fallback notice for unavailable collision geometry that is visible only in `Collision` and `Both`, clearly states that magenta translucent visual meshes remain shown, and leaves target/preview-ghost rendering unchanged.
- [x] 4.2 Add focused scene/GUI tests covering the fallback notice and magenta translucent substituted-visual treatment in `Collision` and `Both`, and their absence in `Visual` and when collision geometry exists.
- [x] 4.3 Run OpenSpec validation and the documentation-link check after updating the fallback-indicator requirements and user documentation.
