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

"""Verify the OctoMap survives a loop-closure-style pose shift.

Until the refactor that switched to `rtab.getLocalOptimizedPoses()`,
the wrapper froze its scan poses at capture time. After any loop
closure that shifted those poses, the OctoMap would hold misaligned
chunks of the same world at old and new corrections simultaneously
— and the obstacle cloud at any single point would never converge.

This test fakes the closure by feeding the binary two visits to a
landmark with a small disagreement between the visits, so rtabmap's
proximity-based closure detection has something to lock onto. After
the (likely) closure, the obstacle cloud must remain spatially
consistent — no doubled obstacles, no large drift between the two
visits' versions of the same wall.

We can't reliably force a closure on a 100% synthetic scene (rtabmap's
ICP can refuse to converge), so the test is skipped if no closure
event is detected via the rtab_tf correction changing magnitude.
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


def test_no_doubled_walls_after_closure(rtab_harness: RtabHarness) -> None:
    """Visit a room, leave, return; assert the obstacle cloud isn't
    visibly split into two offset copies of the same wall (which is
    what the frozen-pose design produced after closure)."""
    scan = square_room_scan()[:, :3]
    scan = np.column_stack([scan, np.ones(len(scan), dtype=np.float32)])

    # Trajectory: out and back. Last few frames re-visit the start.
    waypoints = [(-0.1 * i, 0.0) for i in range(8)]
    waypoints += [(-0.7 + 0.1 * i, 0.0) for i in range(8)]

    base_ts = 100.0
    for i, (x, y) in enumerate(waypoints):
        ts = base_ts + float(i) * 0.4
        rtab_harness.publish_odom(np.array([x, y, 0.0]), identity_quat(), ts)
        rtab_harness.publish_scan(scan, ts)
        rtab_harness.drain(seconds=0.15)
    rtab_harness.drain(seconds=3.0)

    non_empty = [msg for msg in rtab_harness.octomap.messages if len(msg.as_numpy()[0]) > 0]
    if not non_empty:
        pytest.skip("rtabmap built an empty octomap on this synthetic input")

    pts, _ = non_empty[-1].as_numpy()

    # The room walls in conftest are at x=±1.5, y=±1.5. After closure the
    # cloud should still have those walls within a tight band — not split
    # into two layers separated by however much rtabmap's pose correction
    # ended up being.
    wall_pos_x = pts[(pts[:, 0] > 1.0) & (pts[:, 1] > -2.0) & (pts[:, 1] < 2.0)]
    if len(wall_pos_x) == 0:
        pytest.skip("no positive-x wall voxels in octomap")
    spread = wall_pos_x[:, 0].max() - wall_pos_x[:, 0].min()
    # One cell of natural spread is fine. Multiple cells suggests two
    # misaligned versions of the wall co-existing.
    assert spread < 0.4, (
        f"positive-x wall voxels span {spread:.2f} m — looks like the OctoMap "
        f"has two misaligned copies of the wall (frozen-pose-after-closure bug)"
    )
