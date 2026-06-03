#!/usr/bin/env python
# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Post-process Go2 + Livox recordings: add AprilTag-corrected groundtruth + .rrd.

Thin orchestrator over dimos/robot/unitree/go2/recording/*. For every `mem2.db`
under a recordings directory it:
  1. prints a recording sanity check (rec_check),
  2. detects AprilTags -> `april_tags` stream                 (recording.apriltags),
  3. solves a drift-corrected trajectory -> `gtsam_odom`       (recording.gtsam_gt),
  4. re-anchors the lidar onto it -> `<lidar>_corrected`       (recording.lidar_reanchor),
  5. writes a Rerun `.rrd` visualization                        (recording.build_rrd).

A tag seen at several times pins the odometry chain and removes accumulated
drift. Also writes `gtsam_odom.tum` next to each db (relocalization groundtruth).

Run in a python env with dimos + cv2 + gtsam + scipy (from the dimos repo):

    uv run --no-sync python \
        dimos/robot/unitree/go2/scripts/go2_mid360_post_process.py [TARGET] [--force]

TARGET may be a `mem2.db`, a recording dir containing one, or a dir to scan for
recordings. With no TARGET it processes the most recently created recording
under --recordings-dir.
"""

from __future__ import annotations

import argparse
from pathlib import Path

from dimos.memory2.store.sqlite import SqliteStore
from dimos.robot.unitree.go2.recording import rec_check
from dimos.robot.unitree.go2.recording.apriltags import detect_apriltags
from dimos.robot.unitree.go2.recording.build_rrd import build_rrd
from dimos.robot.unitree.go2.recording.gtsam_gt import build_gtsam_gt, write_gtsam_odom
from dimos.robot.unitree.go2.recording.lidar_reanchor import reanchor_stream

# Lidar/odom pairs that may be re-anchored onto gtsam_odom — only when their odom
# is the same frame family gtsam was built from (so the re-anchor composes). The
# legacy Go2 onboard `lidar`/`odom` is a different estimator frame -> left as-is.
REANCHOR_PAIRS = [("go2_lidar", "go2_odom"), ("fastlio_lidar", "fastlio_odometry")]
DB_NAME = "mem2.db"


def _created_time(path: Path) -> float:
    """File creation time (st_birthtime on macOS/BSD; falls back to mtime)."""
    stat = path.stat()
    return getattr(stat, "st_birthtime", stat.st_mtime)


def _scan(root: Path) -> list[Path]:
    return sorted(path for path in root.rglob(DB_NAME) if "-wal" not in path.name)


def resolve_databases(target: str | None, recordings_dir: str) -> list[Path]:
    """Pick which mem2.db(s) to process.

    TARGET wins when given: a `mem2.db` file, a dir holding one (process just
    that recording), or any other dir (scan it recursively). With no TARGET,
    process only the most recently created recording under recordings_dir.
    """
    if target:
        path = Path(target)
        if path.name == DB_NAME:
            return [path]
        if (path / DB_NAME).exists():
            return [path / DB_NAME]
        databases = _scan(path)
        if not databases:
            raise SystemExit(f"no {DB_NAME} found under {path}")
        return databases

    databases = _scan(Path(recordings_dir))
    if not databases:
        raise SystemExit(f"no {DB_NAME} found under {recordings_dir}")
    most_recent = max(databases, key=_created_time)
    print(f"no target given — using most recent recording: {most_recent.parent}")
    return [most_recent]


def correct_db(db: Path, *, image_stream, apriltag_stream, gtsam_stream, marker_length, dictionary):
    """AprilTag detection -> GTSAM trajectory -> re-anchor lidar. Returns True if
    a corrected trajectory was written."""
    with SqliteStore(path=str(db)) as store:
        detections = detect_apriltags(
            store, image_stream, apriltag_stream, marker_length, dictionary
        )
    if not detections:
        print("   no AprilTags detected — skipping gtsam_odom (no landmark constraints)")
        return False
    trajectory = build_gtsam_gt(str(db), detections)
    with SqliteStore(path=str(db)) as store:
        write_gtsam_odom(store, trajectory, gtsam_stream, db.parent / "gtsam_odom.tum")
        stream_names = store.list_streams()
        for lidar_stream, odom_stream in REANCHOR_PAIRS:
            if lidar_stream in stream_names and odom_stream in stream_names:
                try:
                    reanchor_stream(
                        store,
                        str(db),
                        lidar_stream=lidar_stream,
                        odom_stream=odom_stream,
                        gtsam_stream=gtsam_stream,
                        out_stream=f"{lidar_stream}_corrected",
                    )
                except Exception as error:
                    print(f"   re-anchor {lidar_stream}_corrected failed: {error}")
    return True


def process_db(
    db: Path,
    *,
    image_stream,
    apriltag_stream,
    gtsam_stream,
    marker_length,
    dictionary,
    force,
    no_gtsam=False,
    make_rrd=True,
    camera_freq=30,
    map_voxel=0.1,
    cloud_stride=3,
    mid360_pitch=False,
):
    print(f">> {db}")
    try:
        rec_check.report(db.parent)
    except Exception as error:
        print(f"   rec_check skipped: {error}")

    with SqliteStore(path=str(db)) as store:
        already_corrected = gtsam_stream in store.list_streams()

    if no_gtsam:
        print("   --no-gtsam: skipping AprilTag/GTSAM/re-anchor")
    elif already_corrected and not force:
        print(f"   already has '{gtsam_stream}' — skipping AprilTag/GTSAM/re-anchor (use --force)")
    else:
        correct_db(
            db,
            image_stream=image_stream,
            apriltag_stream=apriltag_stream,
            gtsam_stream=gtsam_stream,
            marker_length=marker_length,
            dictionary=dictionary,
        )

    if make_rrd:
        try:
            build_rrd(
                str(db),
                str(db.parent / f"{db.parent.name}.rrd"),
                camera_stride=camera_freq,
                map_voxel=map_voxel,
                cloud_stride=cloud_stride,
                mid360_pitch=mid360_pitch,
            )
        except Exception as error:
            print(f"   rrd failed: {error}")


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "target",
        nargs="?",
        default=None,
        help="a mem2.db, a recording dir containing one, or a dir to scan. "
        "Omit to use the most recently created recording under --recordings-dir.",
    )
    parser.add_argument(
        "--recordings-dir",
        default="./go2_recordings",
        help="root searched when no target is given",
    )
    parser.add_argument("--image-stream", default="color_image")
    parser.add_argument("--apriltag-stream", default="april_tags")
    parser.add_argument("--gtsam-stream", default="gtsam_odom")
    parser.add_argument("--marker-length", type=float, default=0.10)
    parser.add_argument("--dictionary", default="DICT_APRILTAG_36h11")
    parser.add_argument("--force", action="store_true", help="reprocess even if gtsam_odom exists")
    parser.add_argument(
        "--no-gtsam",
        action="store_true",
        help="skip AprilTag/GTSAM/re-anchor (e.g. rebuild only the .rrd)",
    )
    parser.add_argument("--no-rrd", action="store_true", help="skip the .rrd visualization step")
    parser.add_argument(
        "--camera-freq",
        type=int,
        default=30,
        help="keep 1 of every N color frames in the .rrd (usually the biggest part)",
    )
    parser.add_argument(
        "--map-voxel",
        type=float,
        default=0.1,
        help="voxel size (m) for the .rrd clouds/maps; larger = smaller .rrd",
    )
    parser.add_argument(
        "--cloud-stride",
        type=int,
        default=3,
        help="keep 1 of every N per-frame lidar clouds in the .rrd",
    )
    parser.add_argument(
        "--mid360-pitch",
        action="store_true",
        help="apply the legacy mid360->camera 44deg pitch correction (old fastlio "
        "recordings; new go2_* data has correct transforms, leave off)",
    )
    args = parser.parse_args()

    databases = resolve_databases(args.target, args.recordings_dir)
    print(f"found {len(databases)} recording(s)")
    for db in databases:
        try:
            process_db(
                db,
                image_stream=args.image_stream,
                apriltag_stream=args.apriltag_stream,
                gtsam_stream=args.gtsam_stream,
                marker_length=args.marker_length,
                dictionary=args.dictionary,
                force=args.force,
                no_gtsam=args.no_gtsam,
                make_rrd=not args.no_rrd,
                camera_freq=args.camera_freq,
                map_voxel=args.map_voxel,
                cloud_stride=args.cloud_stride,
                mid360_pitch=args.mid360_pitch,
            )
        except Exception as error:
            print(f"   !! failed: {error}")
    print("done")


if __name__ == "__main__":
    main()
