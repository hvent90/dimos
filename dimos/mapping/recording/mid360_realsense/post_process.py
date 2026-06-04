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

"""Post-process RealSense + Mid-360 recordings: AprilTag-corrected groundtruth + .rrd.

Thin wrapper over dimos/mapping/recording/utils/post_process.py. Intrinsics come
from the recorded RealSense camera_info; the camera_optical mount is fixed by the
record blueprint (1cm forward + 1cm below the lidar).

    uv run --no-sync python \
        dimos/mapping/recording/mid360_realsense/post_process.py [TARGET] [--force]

TARGET may be a `mem2.db`, a recording dir containing one, or a dir to scan for
recordings. With no TARGET it processes the most recently created recording
under --recordings-dir.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

from dimos.mapping.recording.utils.post_process import CameraParams, run
from dimos.memory2.store.sqlite import SqliteStore
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo

# The RealSense + Mid-360 recording only has the fastlio lidar/odom pair; its odom
# is the same frame family gtsam was built from, so the re-anchor composes.
REANCHOR_PAIRS = [("fastlio_lidar", "fastlio_odometry")]
CAMERA_INFO_STREAM = "realsense_camera_info"

# camera_optical pose in mid360_link (the frame fastlio odom is anchored to),
# matching the camera mount in record.py: 1cm forward + 1cm below the lidar,
# then the REP-103 optical rotation.
REALSENSE_OPTICAL_IN_BASE = [0.01, 0.0, -0.01, -0.5, 0.5, -0.5, 0.5]


def load_camera(db: Path) -> CameraParams:
    """Read intrinsics/distortion/resolution from the recorded RealSense
    camera_info stream; the optical mount is fixed by the record blueprint."""
    with SqliteStore(path=str(db)) as store:
        if CAMERA_INFO_STREAM not in store.list_streams():
            raise SystemExit(f"no '{CAMERA_INFO_STREAM}' stream in {db} — cannot get intrinsics")
        info = next(iter(store.stream(CAMERA_INFO_STREAM, CameraInfo))).data
    intrinsics = np.array(info.K, dtype=np.float64).reshape(3, 3)
    distortion = np.array(info.D, dtype=np.float64)
    return intrinsics, distortion, REALSENSE_OPTICAL_IN_BASE, (info.width, info.height)


if __name__ == "__main__":
    run(description=__doc__, reanchor_pairs=REANCHOR_PAIRS, load_camera=load_camera)
