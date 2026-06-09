## Why

DimOS has the building blocks for robot learning data collection and rollout, but they are not yet joined into a coherent learning workflow. Memory2 can record typed timestamped streams, and the ControlCoordinator can execute joint trajectories with preemption, but there is no shared robot contract that maps raw recordings to LeRobot export rows, live streams to policy inputs, and policy actions back to coordinator-native commands.

This change establishes a robot learning rollout framework so developers can collect demonstrations, convert them to LeRobot-compatible datasets, and deploy trained policies with less train/rollout mismatch. The immediate need is to make data alignment, action chunk execution, and preemption behavior explicit before adding policy-specific implementations.

## What Changes

- Add behavior for recording robot learning episodes into Memory2 with enough observation, action, timing, task, and episode metadata to export fixed-FPS LeRobot datasets later.
- Add behavior for deriving policy and export views from raw Memory2 recordings or live streams through a shared robot contract, without making framed rows the recording format.
- Add behavior for a policy rollout module that periodically generates action chunks from transient robot-contract projections and submits them to coordinator-managed control.
- Add behavior for chunked joint trajectory rollout where preempted chunks are dropped entirely and reported as dropped/preempted rather than partially resumed.
- Add LeRobot export behavior that turns Memory2 rollout recordings into LeRobot frame rows with stable feature schemas, episode boundaries, task labels, timestamps, images/videos, and actions.
- No intentional **BREAKING** public API, CLI, or hardware-safety changes are proposed; rollout execution must integrate with existing coordinator safety and preemption semantics.

## Affected DimOS Surfaces

- Modules/streams: Memory2 recorders and stores; rollout episode metadata streams; robot contract projection surfaces; policy rollout module streams; `JointState`, image, pose/odometry, gripper, and action chunk streams; ControlCoordinator task invocation/status surfaces.
- Blueprints/CLI: robot learning recorder/rollout blueprint composition; optional CLI surfaces for recording, inspecting, and exporting rollout datasets; no existing blueprint behavior should change by default.
- Skills/MCP: optional future skills or MCP tools for starting/stopping recording and rollout; no agent skill behavior changes are required for the initial framework.
- Hardware/simulation/replay: coordinator-managed manipulation hardware; simulation and replay paths that consume Memory2 recordings; hardware safety depends on preserving existing ControlCoordinator arbitration, timeout, cancellation, and preemption behavior.
- Docs/generated registries: user/developer docs for robot learning collection, LeRobot export, policy rollout, and preemption semantics; possible generated registries if new blueprints or control task types are added.

## Capabilities

### New Capabilities

- `robot-learning-recording`: Covers Memory2-based recording of learning episodes, required observation/action streams, episode metadata, and LeRobot-exportable data completeness.
- `rollout-frame-alignment`: Covers robot-contract projection and timestamp alignment shared by live policy inference and offline dataset export while preserving raw recordings.
- `policy-rollout-control`: Covers policy modules that periodically generate action chunks and submit them to coordinator-managed control without directly writing hardware.
- `action-chunk-preemption`: Covers chunk execution status, whole-chunk dropping on preemption, and observable completion/preemption/drop reporting.
- `lerobot-dataset-export`: Covers conversion from Memory2 rollout recordings to LeRobot-compatible datasets, including feature schema, fixed-FPS frames, episode metadata, images/videos, and actions.

### Modified Capabilities

- None.

## Impact

Developers gain a structured path from demonstration collection to LeRobot training data and policy rollout while keeping DimOS stream, Memory2, replay, and ControlCoordinator concepts aligned. The main compatibility risk is semantic rather than API-breaking: recorded stream names, feature ordering, timestamp tolerances, and action chunk status must remain stable enough for exported datasets and trained policies to match live rollout.

The change may introduce optional dependencies or documented installation paths for LeRobot export, but core robot operation should not require LeRobot unless export is used. Documentation should cover collection workflows, dataset conversion, policy rollout, and hardware safety expectations. Test and QA scope should include Memory2 recording completeness, fixed-FPS export alignment, policy frame assembly parity between live and recorded data, coordinator chunk execution, and preemption cases where interrupted chunks are dropped entirely.
