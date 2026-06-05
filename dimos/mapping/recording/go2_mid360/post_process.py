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

"""Post-process Go2 + Livox recordings: AprilTag-corrected groundtruth + .rrd.

Thin wrapper over dimos/mapping/recording/utils/post_process.py with the Go2
front-camera calibration and the lidar/odom pairs present in these recordings.

    uv run --no-sync python \
        dimos/mapping/recording/go2_mid360/post_process.py [TARGET] [--force]

TARGET may be a `mem2.db`, a recording dir containing one, or a dir to scan for
recordings. With no TARGET it processes the most recently created recording
under --recordings-dir.
"""

from __future__ import annotations

from pathlib import Path

from dimos.mapping.recording.utils import stream_names
from dimos.mapping.recording.utils.go2_align import adjust_go2_streams
from dimos.mapping.recording.utils.post_process import CameraParams, run
from dimos.robot.unitree.go2.config import (
    GO2_FRONT_CAMERA_DISTORTION,
    GO2_FRONT_CAMERA_INTRINSICS,
    GO2_FRONT_CAMERA_OPTICAL_IN_BASE,
    GO2_FRONT_CAMERA_RESOLUTION,
)

# (lidar, odom) pairs re-anchored onto gtsam_odom: each cloud's own odometry is
# subtracted and the corrected gtsam pose re-applied, so the source odom frame
# need not match gtsam's. Both the Go2 onboard and the mid360/FAST-LIO clouds.
REANCHOR_PAIRS = [
    (stream_names.LIDAR, stream_names.ODOM),
    (stream_names.FASTLIO_LIDAR, stream_names.FASTLIO_ODOM),
]


def load_camera(db: Path) -> CameraParams:
    return (
        GO2_FRONT_CAMERA_INTRINSICS,
        GO2_FRONT_CAMERA_DISTORTION,
        GO2_FRONT_CAMERA_OPTICAL_IN_BASE,
        GO2_FRONT_CAMERA_RESOLUTION,
    )


if __name__ == "__main__":
    run(
        description=__doc__,
        reanchor_pairs=REANCHOR_PAIRS,
        load_camera=load_camera,
        extra_steps=adjust_go2_streams,
    )
