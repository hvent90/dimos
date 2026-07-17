## Why

Quest teleoperation can already record Piper demonstrations for offline LeRobot conversion, but operators lack a dedicated live camera and recording-status view. Collection also needs a clear, configurable task label and a workflow that keeps discarded takes out of training data without deleting raw evidence.

This change establishes a focused, hardware-backed Piper collection blueprint for producing a first RGB-and-joint-state imitation-learning dataset with reliable episode boundaries and live operational feedback.

## What Changes

- Add a dedicated Quest Piper data-collection blueprint with a RealSense RGB camera, collection recorder, episode monitor, and Rerun monitoring.
- Expose the task label as collection-recorder configuration and carry it into recorded episode metadata for LeRobot conversion.
- Configure two Quest controls: one toggle button starts and saves an episode; a second button discards an active episode.
- Show the live RGB camera feed and collection state in Rerun without depending on the manipulation or Viser stacks.
- Preserve raw native-rate streams in SQLite and convert kept episodes to a 30 Hz LeRobot dataset with RGB images, Piper arm-plus-gripper joint observations, and next absolute-joint actions.

## Affected DimOS Surfaces

- Modules/streams: collection recorder configuration; episode-status task metadata; RGB image, Piper coordinator joint-state, and episode-status streams; Rerun collection observability.
- Blueprints/CLI: a new built-in Quest Piper Rerun collection blueprint and its generated blueprint registry entry.
- Skills/MCP: none.
- Hardware/simulation/replay: physical Piper, Quest controllers, and RealSense hardware; no manipulation module, Viser dependency, simulation, or replay support is added.
- Docs/generated registries: collection-to-LeRobot workflow documentation and regenerated `dimos/robot/all_blueprints.py`.

## Capabilities

### New Capabilities
- `piper-imitation-data-collection`: collection of labeled Quest-to-Piper RGB and joint-state demonstrations, live monitoring, and LeRobot-ready conversion behavior.

### Modified Capabilities

- None.

## Impact

Operators gain a self-contained collection command with live visibility into camera and recording state. The new blueprint requires connected Piper, Quest, and RealSense hardware and adds Rerun as a runtime monitoring dependency. Existing collection blueprints and their behavior remain unchanged. QA must cover blueprint generation, episode save/discard semantics, configuration propagation, stream wiring, and a hardware-capable record-to-LeRobot smoke path.
