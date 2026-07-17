## Context

`learning_collect_quest_piper` already composes Quest-to-Piper teleoperation, a real-hardware RealSense camera, `EpisodeMonitorModule`, and `CollectionRecorder`. It records `color_image`, `coordinator_joint_state`, and `status` to SQLite. Offline dataprep extracts saved status ranges, resamples at 30 Hz by default, and uses the next joint-state sample as an absolute-joint action.

Rerun is already composed through `vis_module("rerun")`, which provides an LCM-backed bridge and viewer infrastructure. `Image` messages implement `to_rerun()`, so the camera stream can be monitored without the manipulation or Viser stacks. `EpisodeStatus` has no Rerun representation today, so recording-state monitoring requires a narrow collection-status visualization adapter.

## Goals / Non-Goals

**Goals:**
- Add a separate physical Piper collection blueprint that combines existing Quest teleoperation, RealSense RGB recording, collection lifecycle controls, and Rerun monitoring.
- Make a collection task label a `CollectionRecorderConfig` value and preserve it in recorded episode status for LeRobot task metadata.
- Record reusable raw RGB and continuous Piper arm-plus-gripper state, then support the existing 30 Hz LeRobot conversion contract: current joint state as observation and next absolute joint state as action.
- Give operators live RGB and episode-state feedback while leaving the recorder as the data authority.

**Non-Goals:**
- Changing the existing `learning_collect_quest_piper` blueprint.
- Adding Viser, manipulation visualization, robot-model rendering, MCP skills, depth images, multiple cameras, replay support, or a simulator-specific collection path.
- Redesigning existing Quest lower-button engagement controls or allowing a completed episode to be discarded retroactively.
- Binarizing gripper actions; that remains a future conversion concern.

## DimOS Architecture

The new built-in blueprint, `learning_collect_quest_piper_rerun`, will use `autoconnect(...)` to compose:

```text
Quest controllers
    │ teleop_buttons / controller pose
    ▼
teleop_quest_piper ──► Piper coordinator ──► coordinator_joint_state ─┐
RealSenseCamera ─────────────────────────────► color_image             ├► CollectionRecorder → SQLite
EpisodeMonitorModule ────────────────────────► status                  ┘
          │
          └─► collection-status Rerun adapter

color_image ─────────────────────────────────► vis_module("rerun")
```

- `teleop_quest_piper` retains its existing controller-to-coordinator remapping and produces `teleop_buttons` for episode control.
- Real hardware includes `RealSenseCamera.blueprint(enable_pointcloud=False)`. The image is both recorded and made visible to Rerun; no depth or point cloud is part of the collection contract.
- `CollectionRecorder` continues to consume `color_image`, `coordinator_joint_state`, and `status`. It gains a `task_label` configuration value. Blueprint composition must propagate that value to the episode monitor so each emitted status event carries the label; raw status remains the source used by dataprep.
- The top-button mapping resolves toggle to `start` while idle and `save` while recording. A separate button resolves to `discard` only while an episode is active. Saved and discarded events remain persisted in SQLite; dataprep writes only successful episodes to LeRobot.
- Rerun composition uses the standard `vis_module("rerun", rerun_config=...)` path and remains observational. It receives camera data through the existing generic bridge. A collection-specific status adapter converts `EpisodeStatus` transitions into visible Rerun state/count information without changing the recorded stream or introducing a manipulation dependency.
- No new DimOS Python `Spec` Protocol, RPC link, or skill/MCP surface is needed. Registry generation exposes the blueprint through the standard CLI discovery path.

## Decisions

1. **Add a separate Rerun collector rather than modify the existing Piper collector.** This preserves current users' topology and dependency behavior while providing a dedicated monitored workflow.
2. **Use Rerun, not Viser.** Rerun already has a reusable generic visual module and can display `Image` messages. Viser is manipulation-specific in the current repository and is out of scope.
3. **Record RGB plus continuous arm-and-gripper joints.** This is the smallest useful imitation-learning contract. The recorded joint stream supplies both state at time *t* and absolute action at *t+1* through the existing action shift.
4. **Keep task metadata at the collection-recorder configuration boundary.** Task identity is collection metadata, not a hard-coded blueprint concern. The status producer receives the configured value solely to serialize it alongside episode boundaries.
5. **Use two controller actions.** Toggle begins/saves an episode; discard aborts only the active take. This matches the existing state machine and avoids a new retroactive-deletion workflow.
6. **Target 30 Hz only during conversion.** SQLite retains native timestamps; dataprep resamples required streams on the RGB timeline with existing nearest-neighbor behavior and tolerance.

## Safety / Simulation / Replay

This is a physical-robot workflow: a connected Piper, Quest controller service, and RealSense are required for the intended path. The change does not alter coordinator motion commands, engagement behavior, lower-button controls, or robot safety limits; it observes existing teleoperation and records its outputs. Operators must retain existing teleoperation safety procedures and verify the RealSense view before beginning a take.

The new blueprint does not promise simulation or replay support. If composed in an existing simulation environment that supplies `color_image`, it may inherit the current collector's camera omission pattern, but that is not a supported validation target for this change.

Manual hardware QA must confirm camera visibility, episode transitions, save/discard exclusion, and that no Rerun component changes command or recorder streams.

## Risks / Trade-offs

- **Rerun status visibility has no existing generic `EpisodeStatus` conversion.** Mitigate with a focused adapter and a test that validates the emitted visualization data; keep the recorder independent from it.
- **Rerun's bridge observes LCM topics.** Confirm the composed stream transport makes both camera and status visualization available; a missing viewer topic must not affect recording.
- **Nearest-neighbor 30 Hz synchronization can drop samples when timing exceeds tolerance.** Use a pilot recording and dataset inspection to validate frame count and alignment before collecting a large corpus.
- **A task label configured at the recorder must be propagated to the monitor.** Test that saved episode metadata carries the value through SQLite and into LeRobot output.
- **The exact Quest button alias for “top” must be verified against current controller aliases.** Cover the selected mapping with the existing episode-monitor button test style.

## Migration / Rollout

Existing collection blueprints, recordings, and conversion commands remain compatible. Add the new blueprint, run `pytest dimos/robot/test_all_blueprints_generation.py` to regenerate `dimos/robot/all_blueprints.py`, and document the hardware prerequisites plus record-to-convert workflow. Rollback consists of not selecting the new blueprint; it introduces no migration of existing datasets.

## Open Questions

- None blocking. Implementation must select the repository's exact top-button alias and the minimal Rerun entity/archetype for collection status, then lock both down with focused tests.
