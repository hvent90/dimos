## 1. Collection Metadata and Observability

- [x] 1.1 Add `task_label` to collection-recorder configuration and propagate it into emitted episode status so saved raw episodes retain collection task metadata.
- [x] 1.2 Add focused episode-monitor and dataprep tests for task-label persistence, toggle save behavior, active-take discard behavior, and exclusion of discarded episodes.
- [x] 1.3 Implement a narrow Rerun collection-status visualization adapter that exposes recording state and saved/discarded counts without altering recorder or teleoperation streams.
- [x] 1.4 Add focused Rerun adapter tests, including viewer/visualization failure isolation from recording behavior.

## 2. Piper Rerun Collection Blueprint

- [x] 2.1 Add `learning_collect_quest_piper_rerun` by composing the existing Quest Piper teleoperation stack, physical RealSense RGB camera with point clouds disabled, episode monitor, collection recorder, and `vis_module("rerun")`.
- [x] 2.2 Configure the blueprint's exact supported top-button alias as the start/save toggle and a distinct controller button as active-take discard; preserve lower-button engagement behavior.
- [x] 2.3 Configure Rerun to display the recorded `color_image` stream and collection-status adapter output, and verify the camera/recorder/Rerun transport topology is observational.
- [x] 2.4 Add blueprint-level tests for stream composition, RealSense inclusion on the hardware path, selected episode controls, and absence of manipulation/Viser dependencies.
- [x] 2.5 Run `pytest dimos/robot/test_all_blueprints_generation.py` and include the generated `dimos/robot/all_blueprints.py` update plus registry expectations for the new CLI blueprint.

## 3. LeRobot Conversion Workflow

- [x] 3.1 Add or document a Piper conversion configuration that maps RGB images and continuous arm-plus-gripper joint state into 30 Hz observations and next absolute-joint actions.
- [x] 3.2 Add an end-to-end fixture-based dataprep test proving that task-labeled saved takes appear in LeRobot output and discarded takes do not.
- [ ] 3.3 Validate the chosen synchronization tolerance and output schema with a short pilot collection before relying on the workflow for corpus-scale data capture.

## 4. Documentation

- [x] 4.1 Add a user-facing collection guide covering hardware prerequisites, new blueprint invocation, Rerun monitoring, top-button toggle/discard workflow, task-label configuration, and record-to-LeRobot conversion.
- [x] 4.2 Cross-link the collection guide from hosted Quest teleoperation documentation without changing engagement-control guidance.
- [x] 4.3 Run the repository documentation link and Markdown validation configured for the changed documentation.

## 5. Verification

- [x] 5.1 Run `openspec validate add-quest-piper-rerun-data-collection`.
- [x] 5.2 Run focused pytest targets for collection metadata, Rerun visualization, dataprep conversion, Quest controls, and blueprint registry generation.
- [ ] 5.3 Run the applicable formatter, linter, and type checks for changed Python modules.
- [ ] 5.4 Manually validate on connected Piper, Quest, and RealSense hardware: confirm Rerun RGB/status visibility, record one saved and one discarded take, stop cleanly, convert the session to LeRobot at 30 Hz, and inspect the resulting dataset.
