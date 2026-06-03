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

"""Go2 front RGB camera rig (1280x720) — shared intrinsics/extrinsic.

Same values as front_camera_720.yaml. Edit here for a different rig.
"""

from __future__ import annotations

import numpy as np

# Pinhole intrinsic matrix K (fx, fy, cx, cy).
CAMERA_INTRINSICS = np.array(
    [[797.47561649, 0.0, 643.53521678], [0.0, 796.48721128, 349.27836053], [0.0, 0.0, 1.0]],
    dtype=np.float64,
)
# Lens distortion coefficients (k1, k2, p1, p2).
CAMERA_DISTORTION = np.array([-0.07309429, -0.02341141, -0.00693059, 0.00923868], dtype=np.float64)
# Pose of the camera_optical frame expressed in base_link: [x, y, z, qx, qy, qz, qw].
CAMERA_OPTICAL_IN_BASE = [0.3, 0.0, 0.0, -0.5, 0.5, -0.5, 0.5]
