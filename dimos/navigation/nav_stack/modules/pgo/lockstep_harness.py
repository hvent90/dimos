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

"""Lockstep replay harness for PGO tests.

Mirrors the ack-paced replay + graph-capture design of the jnav loop-closure
eval (``dimos/navigation/jnav/modules/loop_closure/eval.py``), adapted to the
nav-stack PGO streams (``registered_scan``, ``corrected_odometry``).

Two modules compose with a PGO blueprint via ``autoconnect``:

* ``SyntheticLockstepReplay`` — generates synthetic room scans + drifted
  odometry from a precomputed trajectory and publishes them closed-loop: after
  each scan it waits for PGO's ``corrected_odometry`` ack before sending the
  next. Every scan is processed regardless of host speed (no fixed-rate
  sleeps), so coverage is deterministic.
* ``GraphCapture`` — records PGO's optimized pose graph and every
  ``loop_closure_event`` (with SE(3) deltas) for the test to assert on.
"""

from __future__ import annotations

import asyncio
from collections.abc import AsyncIterator
import math
from typing import Any

import numpy as np
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Graph3D import Graph3D
from dimos.msgs.nav_msgs.GraphDelta3D import GraphDelta3D
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Cross-trajectory drift injected at the revisit. Must be >> loop_search_radius
# so position-based search cannot accidentally find the loop.
DRIFT_AT_REVISIT_M = 5.0

# Bound on how long a single scan may wait for its corrected_odometry ack. Well
# above any sane PGO per-scan processing time; a real stall fails the test.
ACK_TIMEOUT_SEC = 20.0

# Frame 0 races PGO's subscription handshake (the native process subscribes to
# registered_scan slightly after the blueprint starts). Resend it on this
# interval until the first ack proves PGO is live, then run strict lockstep.
WARMUP_RESEND_INTERVAL_SEC = 0.2


def make_room_points(half_size: float = 20.0, density: float = 0.15) -> np.ndarray:
    """Sample points on the inside of a 4-wall square room."""
    points: list[np.ndarray] = []
    z_levels = np.arange(0.0, 3.0, density)
    wall_axis = np.arange(-half_size, half_size, density)

    for wall_y in (half_size, -half_size):
        grid_x, grid_z = np.meshgrid(wall_axis, z_levels)
        block = np.column_stack([grid_x.ravel(), np.full(grid_x.size, wall_y), grid_z.ravel()])
        points.append(block)
    for wall_x in (half_size, -half_size):
        grid_y, grid_z = np.meshgrid(wall_axis, z_levels)
        block = np.column_stack([np.full(grid_y.size, wall_x), grid_y.ravel(), grid_z.ravel()])
        points.append(block)

    # Distinctive interior columns so the scene isn't rotationally symmetric.
    column_radius = 0.5
    for column_center_x, column_center_y in [(5.0, 0.0), (-5.0, 8.0)]:
        angles = np.arange(0.0, 2.0 * math.pi, 0.2)
        column_z_levels = np.arange(0.0, 3.0, density)
        grid_angle, grid_z = np.meshgrid(angles, column_z_levels)
        column_x = column_center_x + column_radius * np.cos(grid_angle.ravel())
        column_y = column_center_y + column_radius * np.sin(grid_angle.ravel())
        points.append(np.column_stack([column_x, column_y, grid_z.ravel()]))

    return np.concatenate(points).astype(np.float32)


def make_pose(x: float, y: float, z: float, yaw: float) -> Pose:
    pose = Pose()
    pose.position = Vector3(x, y, z)
    half_yaw = yaw * 0.5
    pose.orientation = Quaternion(0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))
    return pose


def _yaw_rotation(yaw: float) -> np.ndarray:
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    return np.array(
        [[cos_yaw, -sin_yaw, 0.0], [sin_yaw, cos_yaw, 0.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )


def world_to_body(points_world: np.ndarray, position: np.ndarray, yaw: float) -> np.ndarray:
    rotation = _yaw_rotation(yaw).T
    return (points_world - position) @ rotation.T


def body_to_world(points_body: np.ndarray, position: np.ndarray, yaw: float) -> np.ndarray:
    rotation = _yaw_rotation(yaw)
    return points_body @ rotation.T + position


# A trajectory waypoint: (timestamp, true_position, true_yaw, drifted_position,
# drifted_yaw). The robot physically follows the true pose; PGO only sees the
# drifted pose, so the body-frame scan at a revisit is identical while the
# reported odometry is offset.
TrajectoryWaypoint = tuple[float, np.ndarray, float, np.ndarray, float]


def trajectory_with_drift(
    num_outbound: int = 20, num_inbound: int = 20, leg_length: float = 8.0
) -> list[TrajectoryWaypoint]:
    """Out-and-back trajectory that physically returns to the start.

    The drift is purely additive in (x, y) and ramps linearly with travelled
    distance, so by the time the robot returns to (0, 0) the reported odom pose
    is offset by ``DRIFT_AT_REVISIT_M``.
    """
    samples: list[TrajectoryWaypoint] = []
    # Start at timestamp=1.0 because Odometry(ts=0.0) is treated as "now" by the
    # constructor — using 0.0 would inject wall-clock time and break the
    # monotonic-ts assumption in PGO's scan handling.
    timestamp = 1.0
    time_step = 0.5
    total_steps = num_outbound + num_inbound
    for step in range(num_outbound + 1):
        progress = step / max(num_outbound, 1)
        x = progress * leg_length
        true_position = np.array([x, 0.0, 0.5])
        yaw = 0.0
        drift_amount = (step / total_steps) * DRIFT_AT_REVISIT_M
        drifted_position = true_position + np.array([0.0, drift_amount, 0.0])
        samples.append((timestamp, true_position, yaw, drifted_position, yaw))
        timestamp += time_step
    for step in range(1, num_inbound + 1):
        progress = step / max(num_inbound, 1)
        x = leg_length * (1.0 - progress)
        true_position = np.array([x, 0.0, 0.5])
        yaw = 0.0  # keep heading the same so descriptors are directly comparable
        drift_amount = ((num_outbound + step) / total_steps) * DRIFT_AT_REVISIT_M
        drifted_position = true_position + np.array([0.0, drift_amount, 0.0])
        samples.append((timestamp, true_position, yaw, drifted_position, yaw))
        timestamp += time_step
    return samples


def trajectory_reverse_loop(
    num_outbound: int = 20, num_inbound: int = 20, leg_length: float = 8.0
) -> list[TrajectoryWaypoint]:
    """Out-and-back where the robot turns 180° at the far end.

    Exercises ICP's yaw-around-source-keyframe init_guess in
    ``simple_pgo.cpp::searchForLoopPairs``.
    """
    samples: list[TrajectoryWaypoint] = []
    timestamp = 1.0
    time_step = 0.5
    for step in range(num_outbound + 1):
        progress = step / max(num_outbound, 1)
        x = progress * leg_length
        position = np.array([x, 0.0, 0.5])
        yaw = 0.0
        samples.append((timestamp, position, yaw, position.copy(), yaw))
        timestamp += time_step
    for step in range(1, num_inbound + 1):
        progress = step / max(num_inbound, 1)
        x = leg_length * (1.0 - progress)
        position = np.array([x, 0.0, 0.5])
        yaw = math.pi
        samples.append((timestamp, position, yaw, position.copy(), yaw))
        timestamp += time_step
    return samples


def trajectory_payload(trajectory: list[TrajectoryWaypoint]) -> list[list[float]]:
    """Flatten a trajectory into a JSON-serializable matrix for ModuleConfig.

    Each row is ``[timestamp, true_x, true_y, true_z, true_yaw, drifted_x,
    drifted_y, drifted_z, drifted_yaw]``.
    """
    rows: list[list[float]] = []
    for timestamp, true_position, true_yaw, drifted_position, drifted_yaw in trajectory:
        rows.append(
            [
                float(timestamp),
                float(true_position[0]),
                float(true_position[1]),
                float(true_position[2]),
                float(true_yaw),
                float(drifted_position[0]),
                float(drifted_position[1]),
                float(drifted_position[2]),
                float(drifted_yaw),
            ]
        )
    return rows


class SyntheticLockstepReplayConfig(ModuleConfig):
    trajectory: list[list[float]]
    room_half_size: float = 20.0
    room_density: float = 0.15
    ack_timeout_sec: float = ACK_TIMEOUT_SEC
    warmup_resend_interval_sec: float = WARMUP_RESEND_INTERVAL_SEC


class SyntheticLockstepReplay(Module):
    """Closed-loop synthetic replay paced on PGO's corrected_odometry acks."""

    config: SyntheticLockstepReplayConfig

    registered_scan: Out[PointCloud2]
    odometry: Out[Odometry]
    corrected_odometry: In[Odometry]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._ack_count: int = 0
        self._ack_event: asyncio.Event | None = None
        self._frames_published: int = 0
        self._finished: bool = False
        self._error: str | None = None

    async def handle_corrected_odometry(self, value: Odometry) -> None:
        self._ack_count += 1
        if self._ack_event is not None:
            self._ack_event.set()

    async def main(self) -> AsyncIterator[None]:
        self._messages = self._build_messages()
        self._replay_task = asyncio.create_task(self._replay())
        yield
        self._replay_task.cancel()

    def _build_messages(self) -> list[tuple[Odometry, PointCloud2]]:
        room_points = make_room_points(self.config.room_half_size, self.config.room_density)
        messages: list[tuple[Odometry, PointCloud2]] = []
        for row in self.config.trajectory:
            (
                timestamp,
                true_x,
                true_y,
                true_z,
                true_yaw,
                drifted_x,
                drifted_y,
                drifted_z,
                drifted_yaw,
            ) = row
            true_position = np.array([true_x, true_y, true_z])
            drifted_position = np.array([drifted_x, drifted_y, drifted_z])
            body_points = world_to_body(room_points, true_position, true_yaw)
            world_points = body_to_world(body_points, drifted_position, drifted_yaw)
            scan = PointCloud2.from_numpy(
                world_points.astype(np.float32), frame_id="map", timestamp=timestamp
            )
            odometry = Odometry(
                ts=timestamp,
                frame_id="odom",
                child_frame_id="base_link",
                pose=make_pose(drifted_x, drifted_y, drifted_z, drifted_yaw),
            )
            messages.append((odometry, scan))
        return messages

    async def _replay(self) -> None:
        try:
            for index, (odometry, scan) in enumerate(self._messages):
                acks_before = self._ack_count
                self._ack_event = asyncio.Event()
                self.odometry.publish(odometry)
                self.registered_scan.publish(scan)
                self._frames_published += 1
                if index == 0:
                    await self._warmup_until_first_ack(acks_before, odometry, scan)
                else:
                    await self._wait_for_ack(index, acks_before)
        except Exception as exc:
            self._error = f"{type(exc).__name__}: {exc}"
            raise
        finally:
            self._finished = True

    async def _warmup_until_first_ack(
        self, acks_before: int, odometry: Odometry, scan: PointCloud2
    ) -> None:
        deadline = asyncio.get_event_loop().time() + self.config.ack_timeout_sec
        assert self._ack_event is not None
        while self._ack_count == acks_before:
            remaining = deadline - asyncio.get_event_loop().time()
            if remaining <= 0:
                raise RuntimeError(
                    "PGO did not ack the first scan within "
                    f"{self.config.ack_timeout_sec:.1f}s — it never came up. "
                    "Bump ack_timeout_sec if PGO needs longer to start on this host."
                )
            try:
                await asyncio.wait_for(
                    self._ack_event.wait(),
                    timeout=min(self.config.warmup_resend_interval_sec, remaining),
                )
            except asyncio.TimeoutError:
                self.odometry.publish(odometry)
                self.registered_scan.publish(scan)

    async def _wait_for_ack(self, index: int, acks_before: int) -> None:
        deadline = asyncio.get_event_loop().time() + self.config.ack_timeout_sec
        assert self._ack_event is not None
        while self._ack_count == acks_before:
            remaining = deadline - asyncio.get_event_loop().time()
            if remaining <= 0:
                raise RuntimeError(
                    f"PGO did not ack scan {index} within "
                    f"{self.config.ack_timeout_sec:.1f}s — it stalled mid-replay."
                )
            try:
                await asyncio.wait_for(self._ack_event.wait(), timeout=remaining)
            except asyncio.TimeoutError:
                continue

    @rpc
    def is_finished(self) -> bool:
        return self._finished

    @rpc
    def frames_published(self) -> int:
        return self._frames_published

    @rpc
    def error(self) -> str | None:
        return self._error


class GraphCapture(Module):
    """Records PGO's optimized pose graph and every loop_closure_event.

    pose_graph keeps the latest full keyframe list; loop_closure_event
    accumulates per-closure SE(3) deltas (subscribed manually so no event is
    dropped by LATEST coalescing). Exposed over RPC for the host to assert on.
    """

    pose_graph: In[Graph3D]
    loop_closure_event: In[GraphDelta3D]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._keyframes: int = 0
        self._closure_events: list[dict[str, Any]] = []

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(
            Disposable(self.loop_closure_event.subscribe(self._on_loop_closure_event))
        )

    async def handle_pose_graph(self, value: Graph3D) -> None:
        self._keyframes = len(value.nodes)

    def _on_loop_closure_event(self, message: GraphDelta3D) -> None:
        self._closure_events.append(
            {
                "ts": message.ts,
                "transforms": [
                    {
                        "translation": (
                            transform.translation.x,
                            transform.translation.y,
                            transform.translation.z,
                        ),
                        "rotation": (
                            transform.rotation.x,
                            transform.rotation.y,
                            transform.rotation.z,
                            transform.rotation.w,
                        ),
                    }
                    for transform in message.transforms
                ],
            }
        )
        logger.info(
            f"[graph_capture] loop_closure_event #{len(self._closure_events) - 1}: "
            f"node_count={len(message.nodes)}, ts={message.ts:.3f}"
        )

    @rpc
    def closures(self) -> int:
        return len(self._closure_events)

    @rpc
    def keyframes(self) -> int:
        return self._keyframes

    @rpc
    def closure_events(self) -> list[dict[str, Any]]:
        return list(self._closure_events)
