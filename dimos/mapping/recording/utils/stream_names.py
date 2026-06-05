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

"""Canonical mem2 stream names for go2 / mid360 recordings.

Centralized so renaming a recorded stream is a one-line change here instead of a
grep across the post-processing pipeline. Note the recorder's `In[...]` port
*attributes* in record.py must still be literal identifiers (Python can't name an
attribute from a constant) — keep those in sync with these values by hand.
"""

from __future__ import annotations

# Go2 onboard sensors (the legacy short names; were briefly `go2_odom`/`go2_lidar`).
ODOM = "odom"  # Go2 onboard leg odometry (PoseStamped)
LIDAR = "lidar"  # Go2 onboard L1 lidar (PointCloud2)

# Mid-360 + FAST-LIO lidar-inertial estimator.
FASTLIO_ODOM = "fastlio_odometry"
FASTLIO_LIDAR = "fastlio_lidar"
LIVOX_LIDAR = "livox_lidar"  # raw sensor-frame cloud (loop closure input)
LIVOX_IMU = "livox_imu"

# Camera / fiducials.
COLOR_IMAGE = "color_image"
APRIL_TAGS = "april_tags"

# Post-process outputs.
GTSAM_ODOM = "gtsam_odom"
CORRECTED_SUFFIX = "_corrected"  # reanchor_stream writes f"{lidar}{CORRECTED_SUFFIX}"
ADJUSTED_SUFFIX = "_adjusted"  # go2_align writes f"{stream}{ADJUSTED_SUFFIX}"


def corrected(stream: str) -> str:
    """The re-anchored variant name for a lidar stream."""
    return f"{stream}{CORRECTED_SUFFIX}"


def adjusted(stream: str) -> str:
    """The fastlio-frame-rebased variant name for a go2 stream."""
    return f"{stream}{ADJUSTED_SUFFIX}"
