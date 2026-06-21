# Map Postprocessing

You recorded a run. The lidar map drifted. You want a clean one to compare against — ground truth, basically, without a motion-capture rig.

This is the offline fix. Point it at a recording, it bends the trajectory back into shape using AprilTags it saw along the way, then snaps the local geometry together with ICP. Out comes a corrected map written back into the same recording.

It runs on a `.db` after the fact. It is not part of the live nav stack and never touches the robot.

## What you need in the recording

A recording dir with `mem2.db` plus `camera_intrinsics.json`, and these streams inside the db:

- a camera stream (`color_image`)
- odometry (`pointlio_odometry`)
- world-registered lidar scans (`pointlio_lidar`)
- AprilTags physically in the scene, sized and known

The tags are the whole trick. Drift accumulates, but a tag you saw at minute 1 and again at minute 9 is the *same tag* — so the two sightings have to land in the same spot. That constraint is what pulls the map straight.

## The three steps

The scripts live in `dimos/navigation/nav_stack/modules/pgo/scripts/`.

**1. Detect the tags.** Run the camera frames through detection with *no* filtering — every detection, every frame, with its quality numbers attached.

```
python dimos/navigation/nav_stack/modules/pgo/scripts/detect_tags.py --rec=PATH
```

Writes `raw_april_tags`. Leaving it unfiltered matters: you tune the quality gates later in postprocessing without re-running detection, which is the slow part.

**2. Solve.** Two stages, one command:

```
python dimos/navigation/nav_stack/modules/pgo/scripts/post_process.py both --rec=PATH
```

- **Tag PGO (GTSAM).** Odometry between-poses are stiff on roll/pitch and z (gravity isn't drifting) and loose on yaw, where the real error lives. The tag sightings are landmark factors, weighted by how good each detection was. This fixes the big-picture drift.
- **ICP refinement.** Lidar submaps that are close in space but far apart in time get aligned to each other. This cleans up the local geometry the tags don't directly constrain.

It writes `gt_pointlio_odometry` and `gt_pointlio_lidar` back into the db, optionally a `.pc2.lcm` of the corrected cloud, and opens a comparison view. Run `odom`, `lidar`, or `both`.

**3. Look at it.** `post_process.py` opens the rrd for you, but you can rebuild it anytime:

```
python dimos/navigation/nav_stack/modules/pgo/scripts/make_rrd.py --rec=PATH
```

Raw cloud in red, every `gt_*` version in its own color, tag landmarks marked. Add another correction method and it shows up automatically — good for comparing approaches side by side.

## Knobs worth knowing

- `--no-icp` — tag PGO only, skip the ICP stage.
- `--no-lcm` / `--no-rrd` — skip the cloud export / the viewer.
- `--out=NAME` — output prefix, if you want to keep several corrections in one db.
- Tag quality gates (sharpness, reprojection error, distance, view angle, motion blur) live near the top of `post_process.py`. They're relaxed by default to keep more sightings. Tighten them if a bad tag pose is yanking the map around.

## When it won't help

No tags in the scene, or tags seen only once, means no loop constraints — you get ICP cleanup and not much else. Same story if the camera never got a clean look at a tag. Garbage detections in, garbage map out; that's what the gates are for.
