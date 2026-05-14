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

"""Binary-driven loop-closure check.

Asserts the loop-closure machinery doesn't fire spuriously: a monotonic
forward trajectory through a feature-rich scene must keep the published
``rtab_tf`` correction at identity (no revisit, no closure, no map shift).

Note: an aspirational "closure fires on revisit and produces a non-identity
correction" test was attempted but couldn't be made deterministic on
synthetic identical-scan input. rtabmap's lidar-only proximity detection
needs real per-frame geometric variation to admit a closure hypothesis;
hand-crafted scans don't reliably reproduce. The real coverage for that
direction is the cross-wall E2E sim test
(``test_cross_wall_planning_rtab.py``), which drives the binary through
the full Unity sim and verifies the robot actually navigates.
"""

from __future__ import annotations

import numpy as np
import pytest

from dimos.navigation.nav_stack.modules.rtab_map.tests.conftest import (
    RtabHarness,
    identity_quat,
    square_room_scan,
)

pytestmark = [pytest.mark.self_hosted]


def _tf_offset_norm(tf_msg) -> float:
    p = tf_msg.pose.position
    return float(np.linalg.norm([p.x, p.y, p.z]))


def test_monotonic_forward_keeps_identity_correction(rtab_harness: RtabHarness) -> None:
    """No revisit → no closure → rtab_tf must stay at identity for the
    entire run. A spurious closure would show up as a non-zero translation
    in the published rtab_tf messages."""
    scan = square_room_scan()
    for i in range(8):
        ts = float(i) * 0.5
        rtab_harness.publish_odom(np.array([0.5 * i, 0.0, 0.0]), identity_quat(), ts)
        rtab_harness.publish_scan(scan, ts)
        rtab_harness.drain(seconds=0.15)

    rtab_harness.drain(seconds=2.0)

    tf_msgs = rtab_harness.rtab_tf.messages
    assert tf_msgs, "rtab_tf channel received no messages"
    max_offset = max(_tf_offset_norm(m) for m in tf_msgs)
    assert max_offset < 1e-3, (
        f"expected rtab_tf to stay at identity on a monotonic-forward run; "
        f"max norm was {max_offset:.6e}"
    )
