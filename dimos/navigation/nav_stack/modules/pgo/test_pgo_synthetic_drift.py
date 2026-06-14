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

"""Loop-closure behaviour of the native PGO, driven by a lockstep harness.

Each test composes a synthetic ``SyntheticLockstepReplay`` (ack-paced replay of
a precomputed trajectory) with a ``PGO`` blueprint and a ``GraphCapture``, the
same shape as the jnav loop-closure eval harness. Because every scan is paced
on PGO's ``corrected_odometry`` ack, coverage is deterministic regardless of
host speed — no fixed-rate sleeps in the replay.

Setup: a synthetic point-cloud "room"; the robot drives out-and-back along a
corridor while a linear drift is injected into the reported odometry. On the
return leg the robot is *physically* back at the start (so the body-frame scan
is byte-identical to the first scan), but the reported odom pose is offset by
several metres. With ``loop_search_radius=1.0m`` the position-based search
cannot match the two visits; Scan Context, which works on scan appearance
rather than pose, can.
"""

from __future__ import annotations

import math
import time

import pytest

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.navigation.nav_stack.modules.pgo.lockstep_harness import (
    DRIFT_AT_REVISIT_M,
    GraphCapture,
    SyntheticLockstepReplay,
    TrajectoryWaypoint,
    trajectory_payload,
    trajectory_reverse_loop,
    trajectory_with_drift,
)
from dimos.navigation.nav_stack.modules.pgo.pgo import PGO
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

pytestmark = [pytest.mark.self_hosted, pytest.mark.skipif_no_nix]

# Loop closure thresholds passed to the binary.
LOOP_SEARCH_RADIUS_M = 1.0
LOOP_TIME_THRESH_S = 5.0
MIN_LOOP_DETECT_DURATION_S = 1.0

# Drain after the replay reports finished, so PGO can flush any pending loop
# closure events (emitted just after corrected_odometry) before teardown.
POST_FEED_DRAIN_SEC = 3.0
POLL_INTERVAL_SEC = 0.25

# loop_closure_event SE(3) delta sanity bounds.
QUATERNION_UNIT_TOL = 0.05
TRANSLATION_MAX_M = 100.0


def _run_pgo(
    use_scan_context: bool,
    trajectory: list[TrajectoryWaypoint] | None = None,
) -> tuple[int, list[dict[str, object]]]:
    """Replay the trajectory through PGO; return (closure_count, closure_events)."""
    if trajectory is None:
        trajectory = trajectory_with_drift()

    replay_blueprint = SyntheticLockstepReplay.blueprint(
        trajectory=trajectory_payload(trajectory),
    )
    pgo_blueprint = PGO.blueprint(
        debug=True,
        use_scan_context=use_scan_context,
        key_pose_delta_trans=0.4,
        loop_search_radius=LOOP_SEARCH_RADIUS_M,
        loop_time_thresh=LOOP_TIME_THRESH_S,
        loop_score_thresh=1.0,
        loop_submap_half_range=5,
        submap_resolution=0.1,
        min_loop_detect_duration=MIN_LOOP_DETECT_DURATION_S,
        global_map_voxel_size=0.1,
        global_map_publish_rate=1.0,
        unregister_input=True,
        scan_context_max_range_m=30.0,
        scan_context_match_threshold=0.6,
        # The synthetic room is deliberately planar/simple, so it trips the
        # real-world loop-acceptance gates (degeneracy ~0, sparse occupancy).
        # Those guard open-grass false closures on real recordings, not this
        # appearance-vs-position test — disable them here.
        loop_min_degeneracy=0.0,
        loop_min_occupancy=0,
    )
    capture_blueprint = GraphCapture.blueprint()

    blueprint = autoconnect(replay_blueprint, pgo_blueprint, capture_blueprint)
    coordinator = ModuleCoordinator.build(blueprint)
    try:
        replay = coordinator.get_instance(SyntheticLockstepReplay)
        capture = coordinator.get_instance(GraphCapture)
        while not replay.is_finished():
            time.sleep(POLL_INTERVAL_SEC)
        replay_error = replay.error()
        assert replay_error is None, f"lockstep replay failed: {replay_error}"
        time.sleep(POST_FEED_DRAIN_SEC)
        return capture.closures(), capture.closure_events()
    finally:
        coordinator.stop()


def _validate_closure_event(event: dict[str, object], event_index: int) -> None:
    """Assert each transform has unit-norm rotation + finite, bounded translation."""
    transforms = event["transforms"]
    assert isinstance(transforms, list)
    assert len(transforms) > 0, f"event {event_index}: loop-closure event has no transforms"
    for transform_index, transform in enumerate(transforms):
        translation_x, translation_y, translation_z = transform["translation"]
        rotation_x, rotation_y, rotation_z, rotation_w = transform["rotation"]
        for value, name in [
            (translation_x, "translation_x"),
            (translation_y, "translation_y"),
            (translation_z, "translation_z"),
            (rotation_x, "rotation_x"),
            (rotation_y, "rotation_y"),
            (rotation_z, "rotation_z"),
            (rotation_w, "rotation_w"),
        ]:
            assert math.isfinite(value), (
                f"event {event_index} transform {transform_index}: {name}={value} not finite"
            )
        translation_norm = math.sqrt(translation_x**2 + translation_y**2 + translation_z**2)
        assert translation_norm < TRANSLATION_MAX_M, (
            f"event {event_index} transform {transform_index}: "
            f"|t|={translation_norm:.3f}m exceeds sanity cap {TRANSLATION_MAX_M}m"
        )
        quaternion_norm = math.sqrt(rotation_x**2 + rotation_y**2 + rotation_z**2 + rotation_w**2)
        assert abs(quaternion_norm - 1.0) < QUATERNION_UNIT_TOL, (
            f"event {event_index} transform {transform_index}: "
            f"|q|={quaternion_norm:.6f} drifts from unit (tol {QUATERNION_UNIT_TOL})"
        )


class TestPGOSyntheticDrift:
    """Scan Context catches the loop; position search misses it."""

    def test_scan_context_catches_drifted_loop(self) -> None:
        closures, closure_events = _run_pgo(use_scan_context=True)
        logger.info(f"[synthetic_drift] scan_context=true  → {closures} loop events")
        assert closures >= 1, (
            f"Scan Context should catch the loop at the revisit point "
            f"(drift={DRIFT_AT_REVISIT_M}m). Got {closures} events."
        )
        # The emitted SE(3) deltas must be well-formed (valid rotations + finite
        # translations), not just present.
        for event_index, event in enumerate(closure_events):
            _validate_closure_event(event, event_index)

    def test_position_search_misses_drifted_loop(self) -> None:
        closures, _ = _run_pgo(use_scan_context=False)
        logger.info(f"[synthetic_drift] scan_context=false → {closures} loop events")
        assert closures == 0, (
            f"Position-based search shouldn't fire when drift "
            f"({DRIFT_AT_REVISIT_M}m) >> loop_search_radius "
            f"({LOOP_SEARCH_RADIUS_M}m). Got {closures} events."
        )

    def test_scan_context_catches_reverse_loop(self) -> None:
        """Robot drives 8m east facing east, turns 180°, drives back facing west.

        Regression test for the init_guess fix in
        ``simple_pgo.cpp::searchForLoopPairs``: ICP must seed the yaw rotation
        about the source keyframe (not the world origin) for the rotated source
        cloud to stay co-located with the target.
        """
        closures, closure_events = _run_pgo(
            use_scan_context=True, trajectory=trajectory_reverse_loop()
        )
        logger.info(f"[reverse_loop] → {closures} loop events")
        assert closures >= 1, (
            "Scan Context + ICP should catch the 180° reverse-heading loop. "
            f"Got {closures} events. This regresses the init_guess fix in "
            "simple_pgo.cpp (rotation must be about the source keyframe, "
            "not the world origin)."
        )
        for event_index, event in enumerate(closure_events):
            _validate_closure_event(event, event_index)
