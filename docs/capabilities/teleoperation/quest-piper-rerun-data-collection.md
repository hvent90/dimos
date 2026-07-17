---
title: "Quest Piper Rerun Data Collection"
---

Collect supervised Piper demonstrations with a Quest headset, a RealSense color
camera, and the Rerun monitor. This guide covers the expected hardware workflow;
it does not replace the robot manufacturer's setup or safety procedures.

## Prerequisites

- A physical Piper arm and gripper, powered and connected to the computer that
  runs DimOS.
- A CAN adapter and a known-good CAN interface named `can0`.
- A Meta Quest headset with its browser available, on the same LAN as the DimOS
  computer. The headset must be able to reach the computer's LAN IP; `localhost`
  in a URL refers to the headset itself and is not the robot computer.
- A connected and positioned Intel RealSense camera, with a clear color view of
  the workspace.
- A stable network connection between the Quest and the DimOS computer, and a
  clear, supervised workspace with an accessible physical stop.

Before collecting demonstrations, verify the Piper, CAN wiring, camera view,
and Quest tracking with the arm disabled or otherwise in a safe non-moving
state.

## Start the collection stack

Run the expected collection command on the DimOS computer:

```bash
dimos --can-port can0 run learning-collect-quest-piper-rerun
```

The Quest teleoperation server reports a URL containing the DimOS computer's
LAN address (for example, `https://<robot-host-ip>:<port>`). Open that **LAN
URL**, not `localhost`, in the Quest browser. Accept the local development
certificate if the supervised lab setup requires it. Do not substitute a port
or URL that is not printed by the running stack.

The Rerun viewer is the monitoring surface for the collection run. Confirm that
the camera image, Piper state, and the operator's intended motion are updating
before enabling motion. Keep the viewer visible to the supervisor throughout
the run and stop if the image or state becomes stale.

> **Integration assumption:** this guide assumes the finalized
> `learning-collect-quest-piper-rerun` blueprint starts the Quest web server,
> recorder, and Rerun monitoring together. If the implementation exposes a
> different viewer-open mechanism, use only the command and URL printed by that
> implementation rather than adding an undocumented CLI flag.

## Record and discard episodes

The episode monitor is idle when the stack starts. It reacts to rising button
edges:

| Quest control | Effect |
|---|---|
| **B** (top button) | Toggle: start a recording while idle; save the active recording and return to idle when pressed again. |
| **Y** | Discard the active recording and return to idle. It has no discard effect while idle. |

Only press **Y** while an episode is active. A saved episode should contain the
complete demonstration, including a stable start and release/settle at the end.
Verify the save/discard status in the terminal or monitor before starting the
next episode.

## Task labels

The dedicated Rerun collector configures the task label as
`pick_and_place`. That value is propagated through the recorder configuration
to each episode-status event and is consumed later by DataPrep.

For a custom collection, set `CollectionRecorderConfig.task_label` at the
`CollectionRecorder` blueprint/configuration boundary. This is the supported
configuration boundary for collection task labels. No CLI override syntax is
provided; do not invent a module prefix or `-o` path.

## Recording output and DataPrep expectations

On shutdown, the recorder flushes a SQLite session database at:

```text
~/.local/state/dimos/recordings/session_piper_rerun_YYYYMMDD_HHMMSS.db
```

`XDG_STATE_HOME` changes the `~/.local/state` portion. Copy or archive the
database before cleanup. The 30 Hz LeRobot dataprep path expects a sustained,
timestamped color-image stream and a continuous arm/gripper joint stream. Each
sample must provide the current observation joints and the **next joint action**
(the action is aligned from the subsequent joint state, not inferred from a
single Quest pose). Avoid episodes with missing color frames, stopped joint
updates, or gaps that make that next-state alignment ambiguous.

## Hardware QA and safety

Every collection run is supervised hardware QA:

1. Keep a supervisor within reach of the physical stop and keep people and
   loose objects outside the arm's motion envelope.
2. Start with low-risk workspace geometry and confirm joint, gripper, camera,
   and Quest tracking telemetry before enabling the arm.
3. Test B start/save and Y discard with the arm in a safe position before
   collecting a real demonstration.
4. Stop immediately for unexpected motion, CAN errors, stale Rerun data,
   tracking loss, camera obstruction, overheating, or network disconnect.
5. After stopping, confirm that the SQLite file exists and that the final
   episode status is saved or discarded as intended before moving hardware or
   disconnecting the camera.

For hosted browser/Quest teleoperation background, see the
[hosted Quest teleop guide](/docs/capabilities/teleoperation/hosted.md). That guide's connection and engagement
guidance remains unchanged; this page adds the local collection workflow only.
