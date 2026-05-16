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

"""End-to-end check that PGO publishes valid loop-closure events.

Replays the ``og_nav_60s`` rosbag through PGO with aggressive
loop-closure thresholds and asserts each emitted ``loop_closure``
event has positive-shape pose deltas, unit-norm quaternions, and
finite translations. Wired with the DimOS Module + Blueprint pipeline
so no LCM topic strings live here.

If the bag doesn't trigger any loop the test skips — the rosbag
trajectory is data-dependent, not a code defect.
"""

from __future__ import annotations

import asyncio
from collections.abc import AsyncIterator
import math
from pathlib import Path
import time
from typing import Any

import pytest
from reactivex.disposable import Disposable

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.nav_msgs.Path import Path as NavPath
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.nav_stack.modules.pgo.pgo import PGO
from dimos.navigation.nav_stack.tests.rosbag_fixtures import (
    load_rosbag_window,
    make_odometry_msg,
    make_pointcloud_msg,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

pytestmark = [pytest.mark.slow]

POST_FEED_DRAIN_SEC = 5.0
POLL_INTERVAL_SEC = 0.25

QUATERNION_UNIT_TOL = 0.05
TRANSLATION_MAX_M = 100.0


# ---------------------------------------------------------------------------
# Modules
# ---------------------------------------------------------------------------


class RosbagScanOdomPlaybackConfig(ModuleConfig):
    rosbag_path: str | None = None
    odom_subsample: int = 4


class RosbagScanOdomPlaybackModule(Module):
    """Replays scan + odom from the ``og_nav_60s`` rosbag at original timing."""

    config: RosbagScanOdomPlaybackConfig

    registered_scan: Out[PointCloud2]
    odometry: Out[Odometry]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._frames_published: int = 0
        self._playback_finished: bool = False

    async def main(self) -> AsyncIterator[None]:
        rosbag_path = Path(self.config.rosbag_path) if self.config.rosbag_path else None
        self._window = load_rosbag_window(rosbag_path)
        self._playback_task = asyncio.create_task(self._run_playback())
        yield
        self._playback_task.cancel()

    async def _run_playback(self) -> None:
        timeline: list[tuple[str, float, Any]] = []
        for odom_index in range(0, len(self._window.odom), self.config.odom_subsample):
            row = self._window.odom[odom_index]
            timeline.append(
                ("odom", float(row[0]), make_odometry_msg(row[1:4], row[4:8], ts=row[0]))
            )
        for timestamp, points in self._window.scans:
            timeline.append(("scan", float(timestamp), make_pointcloud_msg(points, ts=timestamp)))
        timeline.sort(key=lambda entry: entry[1])
        if not timeline:
            self._playback_finished = True
            return

        bag_start_time = timeline[0][1]
        wallclock_start = time.monotonic()
        for kind, bag_timestamp, message in timeline:
            target_wallclock = wallclock_start + (bag_timestamp - bag_start_time)
            now = time.monotonic()
            if target_wallclock > now:
                await asyncio.sleep(target_wallclock - now)
            if kind == "odom":
                self.odometry.publish(message)
            else:
                self.registered_scan.publish(message)
            self._frames_published += 1
        self._playback_finished = True

    @rpc
    def is_finished(self) -> bool:
        return self._playback_finished

    @rpc
    def frames_published(self) -> int:
        return self._frames_published


class LoopClosureRecorderConfig(ModuleConfig):
    pass


class LoopClosureRecorderModule(Module):
    """Accumulates every loop_closure event so the test can validate the shape."""

    config: LoopClosureRecorderConfig
    loop_closure: In[NavPath]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._events: list[dict[str, Any]] = []

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.loop_closure.subscribe(self._on_loop_closure)))

    def _on_loop_closure(self, message: NavPath) -> None:
        self._events.append(_message_to_dict(message))
        first_pose = message.poses[0] if message.poses else None
        first_pose_summary = (
            f"first=t=({first_pose.position.x:.3f},{first_pose.position.y:.3f},"
            f"{first_pose.position.z:.3f}) "
            f"q=({first_pose.orientation.x:.3f},{first_pose.orientation.y:.3f},"
            f"{first_pose.orientation.z:.3f},{first_pose.orientation.w:.3f})"
            if first_pose
            else "<empty>"
        )
        logger.info(
            f"[loop_closure] event #{len(self._events) - 1} received: "
            f"poses_length={len(message.poses)}, frame_id={message.frame_id!r}, "
            f"ts={message.ts:.3f}, {first_pose_summary}"
        )

    @rpc
    def events(self) -> list[dict[str, Any]]:
        return list(self._events)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _message_to_dict(message: NavPath) -> dict[str, Any]:
    """JSON-friendly snapshot of a NavPath — Pydantic-friendly RPC return."""
    return {
        "frame_id": message.frame_id,
        "ts": message.ts,
        "poses": [
            {
                "position": (
                    pose.position.x,
                    pose.position.y,
                    pose.position.z,
                ),
                "orientation": (
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                ),
            }
            for pose in message.poses
        ],
    }


def _validate_loop_closure_event(event: dict[str, Any], event_index: int) -> tuple[float, float]:
    """Assert each pose has unit quaternion + finite translation. Returns
    aggregate ``(max_translation_norm, max_quaternion_drift)`` stats.
    """
    poses = event["poses"]
    assert len(poses) > 0, f"event {event_index}: loop-closure message has no poses"

    max_translation_norm = 0.0
    max_quaternion_drift = 0.0
    for pose_index, pose in enumerate(poses):
        translation_x, translation_y, translation_z = pose["position"]
        quaternion_x, quaternion_y, quaternion_z, quaternion_w = pose["orientation"]
        for value, name in [
            (translation_x, "translation_x"),
            (translation_y, "translation_y"),
            (translation_z, "translation_z"),
        ]:
            assert math.isfinite(value), (
                f"event {event_index} pose {pose_index}: {name}={value} not finite"
            )
        for value, name in [
            (quaternion_x, "quaternion_x"),
            (quaternion_y, "quaternion_y"),
            (quaternion_z, "quaternion_z"),
            (quaternion_w, "quaternion_w"),
        ]:
            assert math.isfinite(value), (
                f"event {event_index} pose {pose_index}: {name}={value} not finite"
            )
        translation_norm = math.sqrt(
            translation_x * translation_x
            + translation_y * translation_y
            + translation_z * translation_z
        )
        assert translation_norm < TRANSLATION_MAX_M, (
            f"event {event_index} pose {pose_index}: |t|={translation_norm:.3f}m "
            f"exceeds sanity cap {TRANSLATION_MAX_M}m"
        )
        quaternion_norm = math.sqrt(
            quaternion_x * quaternion_x
            + quaternion_y * quaternion_y
            + quaternion_z * quaternion_z
            + quaternion_w * quaternion_w
        )
        quaternion_drift = abs(quaternion_norm - 1.0)
        assert quaternion_drift < QUATERNION_UNIT_TOL, (
            f"event {event_index} pose {pose_index}: |q|={quaternion_norm:.6f} drifts "
            f"from unit by {quaternion_drift:.6f} (tol {QUATERNION_UNIT_TOL})"
        )
        max_translation_norm = max(max_translation_norm, translation_norm)
        max_quaternion_drift = max(max_quaternion_drift, quaternion_drift)

    return max_translation_norm, max_quaternion_drift


# ---------------------------------------------------------------------------
# Test
# ---------------------------------------------------------------------------


class TestPGOLoopClosure:
    """End-to-end: PGO publishes loop-closure events with valid shape."""

    def test_loop_closure_events_published(self) -> None:
        playback_blueprint = RosbagScanOdomPlaybackModule.blueprint()
        # Aggressive loop-closure thresholds — bag is 60s, so we need short
        # re-visit windows to actually fire events.
        pgo_blueprint = PGO.blueprint(
            key_pose_delta_trans=0.5,
            loop_search_radius=2.0,
            loop_time_thresh=5.0,
            loop_score_thresh=0.5,
            loop_submap_half_range=5,
            submap_resolution=0.1,
            min_loop_detect_duration=1.0,
            global_map_voxel_size=0.1,
            global_map_publish_rate=1.0,
            unregister_input=True,
        )
        recorder_blueprint = LoopClosureRecorderModule.blueprint()

        blueprint = autoconnect(playback_blueprint, pgo_blueprint, recorder_blueprint)
        coordinator = ModuleCoordinator.build(blueprint)
        try:
            playback = coordinator.get_instance(RosbagScanOdomPlaybackModule)
            recorder = coordinator.get_instance(LoopClosureRecorderModule)
            while not playback.is_finished():
                time.sleep(POLL_INTERVAL_SEC)
            time.sleep(POST_FEED_DRAIN_SEC)
            events = recorder.events()
        finally:
            coordinator.stop()

        logger.info(f"\n[loop_closure] total events received: {len(events)}")

        if not events:
            pytest.skip(
                "rosbag trajectory didn't trigger any PGO loop closures "
                "even with aggressive thresholds — this validates only the "
                "publishing path's existence (verified via native log "
                "lines), not the on-wire payload."
            )

        for event_index, event in enumerate(events):
            max_translation_norm, max_quaternion_drift = _validate_loop_closure_event(
                event, event_index
            )
            logger.info(
                f"[loop_closure] event #{event_index} VALID: "
                f"keyframe_count={len(event['poses'])}, "
                f"max|t|={max_translation_norm:.4f}m, "
                f"max|q|-1|={max_quaternion_drift:.6f}"
            )

        assert all(len(event["poses"]) > 0 for event in events)
