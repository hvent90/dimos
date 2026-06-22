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

"""Static mount frames for the RealSense D435i + Mid-360 rig.

Published continuously onto tf while recording (see :class:`Mid360RealsenseStaticTf`)
so the mount geometry lands in the recording's tf stream.

Frame sources
-------------
RealSense D435i frame transforms are transcribed from the official
realsense2_description xacro (urdf/_d435.urdf.xacro + urdf/_d435i_imu_modules.urdf.xacro,
use_nominal_extrinsics=true).

Mid-360 geometry (manual): body is 65 x 65 x 60 mm; the point-cloud origin O lies on the
central vertical axis, ~47 mm above the base. The IMU chip is *not* on that axis. The
lidar-to-IMU extrinsic comes from the official Mid-360 config (extrinsic_T flipped gives
the IMU position in lidar coords).
"""

from __future__ import annotations

import math

from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.protocol.tf.static_tf_publisher import (
    FrameSpec,
    StaticTfPublisher,
    frames_to_edge_transforms,
)

CAMERA_ANGLE_UP = math.radians(10)

# Mid-360 box: pitched down from bottom_screw_frame, then offset back/up in that frame
BOX_PITCH_DOWN = math.radians(26) + CAMERA_ANGLE_UP
BOX_BACK = 0.085
BOX_UP = 0.037  # ~4cm up

# Physical constants from _d435.urdf.xacro (meters)
CAM_HEIGHT = 0.025
DEPTH_PY = 0.0175
DEPTH_PZ = CAM_HEIGHT / 2
MOUNT_FROM_CENTER_OFFSET = 0.0149
GLASS_TO_FRONT = 0.1e-3
ZERO_DEPTH_TO_GLASS = 4.2e-3
MESH_X_OFFSET = MOUNT_FROM_CENTER_OFFSET - GLASS_TO_FRONT - ZERO_DEPTH_TO_GLASS

DEPTH_TO_INFRA1_OFFSET = 0.0
DEPTH_TO_INFRA2_OFFSET = -0.050
DEPTH_TO_COLOR_OFFSET = 0.015
IMU_XYZ = (-0.01174, -0.00552, 0.0051)

# rpy that maps a sensor frame to its optical frame (z-forward, x-right, y-down)
OPTICAL_RPY = (-math.pi / 2, 0.0, -math.pi / 2)

# Mid-360 internal frames (manual: point-cloud origin O ~47mm above base, on central axis).
# Box center is 30mm above base, so O sits +17mm along box +z.
LIDAR_ABOVE_BOX_CENTER = 0.017
# IMU position in point-cloud (lidar) coordinates, from Livox Mid-360 extrinsics.
IMU_IN_LIDAR = (0.011, 0.02329, -0.04412)

# The physical mount tree (parent -> child). The gravity-flat "world" helper frame from
# the offline tooling is omitted here — during recording, world comes from odometry.
FRAMES: list[FrameSpec] = [
    ("bottom_screw_frame", None, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
    ("link", "bottom_screw_frame", (MESH_X_OFFSET, DEPTH_PY, DEPTH_PZ), (0.0, 0.0, 0.0)),
    ("depth_frame", "link", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
    ("depth_optical_frame", "depth_frame", (0.0, 0.0, 0.0), OPTICAL_RPY),
    ("infra1_frame", "link", (0.0, DEPTH_TO_INFRA1_OFFSET, 0.0), (0.0, 0.0, 0.0)),
    ("infra1_optical_frame", "infra1_frame", (0.0, 0.0, 0.0), OPTICAL_RPY),
    ("infra2_frame", "link", (0.0, DEPTH_TO_INFRA2_OFFSET, 0.0), (0.0, 0.0, 0.0)),
    ("infra2_optical_frame", "infra2_frame", (0.0, 0.0, 0.0), OPTICAL_RPY),
    ("color_frame", "link", (0.0, DEPTH_TO_COLOR_OFFSET, 0.0), (0.0, 0.0, 0.0)),
    ("color_optical_frame", "color_frame", (0.0, 0.0, 0.0), OPTICAL_RPY),
    ("accel_frame", "link", IMU_XYZ, (0.0, 0.0, 0.0)),
    ("accel_optical_frame", "accel_frame", (0.0, 0.0, 0.0), OPTICAL_RPY),
    ("gyro_frame", "link", IMU_XYZ, (0.0, 0.0, 0.0)),
    ("gyro_optical_frame", "gyro_frame", (0.0, 0.0, 0.0), OPTICAL_RPY),
    ("box_pitch_frame", "bottom_screw_frame", (0.0, 0.0, 0.0), (0.0, BOX_PITCH_DOWN, 0.0)),
    ("box_center", "box_pitch_frame", (-BOX_BACK, 0.0, BOX_UP), (0.0, 0.0, 0.0)),
    ("lidar_frame", "box_center", (0.0, 0.0, LIDAR_ABOVE_BOX_CENTER), (0.0, 0.0, 0.0)),
    ("imu_frame", "lidar_frame", IMU_IN_LIDAR, (0.0, 0.0, 0.0)),
]


class Mid360RealsenseStaticTf(StaticTfPublisher):
    """Publishes the RealSense/Mid-360 mount tree onto tf on a fixed interval."""

    def transforms(self) -> list[Transform]:
        return frames_to_edge_transforms(FRAMES)
