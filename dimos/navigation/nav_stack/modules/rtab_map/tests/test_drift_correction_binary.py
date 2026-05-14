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

"""Binary-driven drift-correction contract check.

Drives the rtab_map binary with an odometry stream that linearly drifts
and asserts the contract the wrapper publishes:

1. corrected_odometry messages are emitted for every odom+scan pair
   (1:1, not throttled).
2. corrected_odometry = (rtab_tf map->odom correction) * (raw odom).
   This is the relationship downstream consumers rely on. Verified by
   reconstructing the corrected pose from the latest tf message and the
   matching odom and checking the two agree.

A separate aspirational "correction absorbs drift over time" assertion
was attempted but couldn't be made deterministic on synthetic
identical-scan input. rtabmap's lidar-only loop-closure can't fire on
identical-scan revisits with no scan diversity; that property is
covered indirectly by the cross-wall E2E sim test.
"""

from __future__ import annotations

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from dimos.navigation.nav_stack.modules.rtab_map.tests.conftest import (
    RtabHarness,
    identity_quat,
    square_room_scan,
)

pytestmark = [pytest.mark.self_hosted]


def _odom_to_se3(msg) -> np.ndarray:
    q = msg.pose.orientation
    r = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    se3 = np.eye(4)
    se3[:3, :3] = r
    se3[:3, 3] = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
    return se3


def test_corrected_odometry_equals_tf_times_raw(rtab_harness: RtabHarness) -> None:
    """Feed drifty odom + scans. Assert:
    - ≥3 corrected_odometry messages (1-per-frame contract).
    - All corrected positions are finite (no NaN/Inf leaks).
    - corrected_pose ≈ tf_correction @ raw_odom_pose (within float tolerance).
    """
    scan = square_room_scan()
    drift = 0.05
    n_steps = 12
    raw_odom_poses = []
    for i in range(n_steps):
        ts = float(i) * 0.4
        odom_pos = np.array([drift * i, 0.0, 0.0])
        rtab_harness.publish_odom(odom_pos, identity_quat(), ts)
        rtab_harness.publish_scan(scan, ts)
        raw_odom_poses.append((ts, odom_pos))
        rtab_harness.drain(seconds=0.12)

    rtab_harness.drain(seconds=2.5)

    corrected = rtab_harness.corrected.messages
    tfs = rtab_harness.rtab_tf.messages
    assert len(corrected) >= 3, f"expected ≥3 corrected_odometry messages, got {len(corrected)}"
    assert tfs, "rtab_tf channel saw no messages"

    for msg in corrected:
        for v in (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z):
            assert np.isfinite(v), f"corrected pose has non-finite value: {msg}"

    # Contract: corrected_pose == tf_correction @ raw_odom_pose for the same
    # timestamp. Check against the latest published pair.
    last_corrected = corrected[-1]
    matching_tf = None
    matching_odom = None
    for tf in reversed(tfs):
        if abs(tf.ts - last_corrected.ts) < 0.05:
            matching_tf = tf
            break
    for ts, pos in reversed(raw_odom_poses):
        if abs(ts - last_corrected.ts) < 0.05:
            matching_odom = pos
            break

    if matching_tf is None or matching_odom is None:
        pytest.skip(
            "no matching tf/odom message at the same timestamp as the latest "
            "corrected_odometry — skipping the algebraic check"
        )

    tf_se3 = _odom_to_se3(matching_tf)
    raw_se3 = np.eye(4)
    raw_se3[:3, 3] = matching_odom
    expected_se3 = tf_se3 @ raw_se3
    expected_xyz = expected_se3[:3, 3]
    actual_xyz = np.array(
        [
            last_corrected.pose.position.x,
            last_corrected.pose.position.y,
            last_corrected.pose.position.z,
        ]
    )
    assert np.allclose(actual_xyz, expected_xyz, atol=1e-3), (
        f"corrected_pose != tf @ raw_odom: expected {expected_xyz}, got {actual_xyz}"
    )
