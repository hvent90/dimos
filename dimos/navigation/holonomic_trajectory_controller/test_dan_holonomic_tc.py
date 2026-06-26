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

"""``DanHolonomicTC`` stream wiring.

These exercise the module surface that routes the ``path`` / ``odometry`` /
``stop_movement`` inputs into the control core and forwards ``nav_cmd_vel`` /
``goal_reached``: an empty path stops, a non-empty path hot-swaps or starts,
``stop_movement`` cancels, and arrival publishes ``goal_reached(True)``. The
control law itself is covered in ``test_holonomic_path_follower``.
"""

from __future__ import annotations

from collections.abc import Callable, Iterator
from contextlib import contextmanager
from dataclasses import dataclass, field
import math
import time
from typing import Any

from dimos_lcm.std_msgs import Bool  # type: ignore[import-untyped]

from dimos.core.stream import Stream, Transport
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.nav_msgs.Path import Path
from dimos.navigation.base import NavigationState
from dimos.navigation.holonomic_trajectory_controller.module import DanHolonomicTC


class _DirectTransport(Transport):  # type: ignore[type-arg]
    """Synchronous in-process transport so ``start()`` can wire the inputs.

    Delivers each broadcast straight to the subscribed handlers on the calling
    thread, which keeps the wiring assertions deterministic.
    """

    def __init__(self) -> None:
        self._subscribers: list[Callable[[Any], Any]] = []

    def broadcast(self, _selfstream: Any, value: Any) -> None:
        for callback in list(self._subscribers):
            callback(value)

    def subscribe(
        self, callback: Callable[[Any], Any], _selfstream: Stream[Any] | None = None
    ) -> Callable[[], None]:
        self._subscribers.append(callback)

        def _unsubscribe() -> None:
            if callback in self._subscribers:
                self._subscribers.remove(callback)

        return _unsubscribe

    def start(self) -> None: ...

    def stop(self) -> None:
        self._subscribers.clear()


@dataclass
class _Captured:
    cmd_vel: list[Twist] = field(default_factory=list)
    goal_reached: list[Bool] = field(default_factory=list)


def _yaw_quaternion(yaw_rad: float) -> Quaternion:
    return Quaternion(0.0, 0.0, math.sin(yaw_rad / 2.0), math.cos(yaw_rad / 2.0))


def _odometry(x: float, y: float, yaw_rad: float, *, ts: float = 1.0) -> Odometry:
    return Odometry(
        ts=ts,
        frame_id="map",
        pose=Pose(position=[x, y, 0.0], orientation=_yaw_quaternion(yaw_rad)),
    )


def _path_from_points(points: list[tuple[float, float]]) -> Path:
    poses: list[PoseStamped] = []
    for index, point in enumerate(points):
        if index + 1 < len(points):
            next_point = points[index + 1]
            yaw = math.atan2(next_point[1] - point[1], next_point[0] - point[0])
        else:
            prev_point = points[index - 1]
            yaw = math.atan2(point[1] - prev_point[1], point[0] - prev_point[0])
        poses.append(
            PoseStamped(
                ts=1.0,
                frame_id="map",
                position=[point[0], point[1], 0.0],
                orientation=_yaw_quaternion(yaw),
            )
        )
    return Path(frame_id="map", poses=poses)


def _is_zero_twist(cmd: Twist) -> bool:
    return (
        abs(float(cmd.linear.x)) < 1e-9
        and abs(float(cmd.linear.y)) < 1e-9
        and abs(float(cmd.angular.z)) < 1e-9
    )


def _wait_until(predicate: Callable[[], bool], timeout: float = 2.0) -> bool:
    deadline = time.time() + timeout
    while time.time() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return predicate()


class _ModuleHarness:
    def __init__(
        self, module: DanHolonomicTC, captured: _Captured, unsubs: list[Callable[[], None]]
    ) -> None:
        self.module = module
        self.captured = captured
        self._unsubs = unsubs

    @property
    def core(self) -> Any:
        return self.module._core

    def feed_odom(self, x: float, y: float, yaw_rad: float, *, ts: float = 1.0) -> None:
        self.module.odometry.transport.broadcast(None, _odometry(x, y, yaw_rad, ts=ts))

    def feed_path(self, path: Path) -> None:
        self.module.path.transport.broadcast(None, path)

    def feed_empty_path(self) -> None:
        self.module.path.transport.broadcast(None, Path(frame_id="map", poses=[]))

    def feed_stop(self, value: bool = True) -> None:
        self.module.stop_movement.transport.broadcast(None, Bool(value))

    def close(self) -> None:
        for unsub in self._unsubs:
            unsub()
        self.module.stop()


@contextmanager
def _running_module(**config: Any) -> Iterator[_ModuleHarness]:
    module = DanHolonomicTC(**config)
    module.path.transport = _DirectTransport()
    module.odometry.transport = _DirectTransport()
    module.stop_movement.transport = _DirectTransport()
    captured = _Captured()
    unsubs = [
        module.nav_cmd_vel.subscribe(captured.cmd_vel.append),
        module.goal_reached.subscribe(captured.goal_reached.append),
    ]
    module.start()
    harness = _ModuleHarness(module, captured, unsubs)
    try:
        yield harness
    finally:
        harness.close()


def test_empty_path_publishes_zero_twist_and_clears_route() -> None:
    with _running_module(goal_tolerance=0.2) as h:
        h.feed_odom(0.0, 0.0, 0.0)
        h.feed_path(_path_from_points([(0.0, 0.0), (2.0, 0.0)]))
        thread = h.core._thread
        assert thread is not None  # a route is active

        h.feed_empty_path()
        thread.join(timeout=1.0)

        assert not thread.is_alive()
        assert h.core._path is None
        assert h.core._thread is None
        assert h.core.get_state() == NavigationState.IDLE
        assert h.captured.cmd_vel
        assert _is_zero_twist(h.captured.cmd_vel[-1])
        assert not h.captured.goal_reached


def test_stop_movement_publishes_zero_twist_and_clears_route() -> None:
    with _running_module(goal_tolerance=0.2) as h:
        h.feed_odom(0.0, 0.0, 0.0)
        h.feed_path(_path_from_points([(0.0, 0.0), (2.0, 0.0)]))
        thread = h.core._thread
        assert thread is not None

        h.feed_stop(True)
        thread.join(timeout=1.0)

        assert not thread.is_alive()
        assert h.core._path is None
        assert h.core._thread is None
        assert h.core.get_state() == NavigationState.IDLE
        assert h.captured.cmd_vel
        assert _is_zero_twist(h.captured.cmd_vel[-1])


def test_stop_movement_false_does_not_clear_route() -> None:
    with _running_module(goal_tolerance=0.2) as h:
        h.feed_odom(0.0, 0.0, 0.0)
        h.feed_path(_path_from_points([(0.0, 0.0), (2.0, 0.0)]))
        thread = h.core._thread

        h.feed_stop(False)

        assert h.core._thread is thread
        assert h.core._path is not None


def test_hot_update_path_swaps_route_without_restarting_thread() -> None:
    with _running_module(goal_tolerance=0.2) as h:
        h.feed_odom(0.0, 0.0, 0.0)
        h.feed_path(_path_from_points([(0.0, 0.0), (2.0, 0.0)]))
        thread_before = h.core._thread
        assert thread_before is not None

        h.feed_path(_path_from_points([(0.0, 0.0), (0.0, 2.0)]))

        # Same control thread: the route was hot-swapped, not restarted.
        assert h.core._thread is thread_before
        swapped = [(float(p.position.x), float(p.position.y)) for p in h.core._path.poses]
        assert swapped == [(0.0, 0.0), (0.0, 2.0)]


def test_arrival_publishes_goal_reached_without_final_spin() -> None:
    # Odom sits on the goal with a misaligned heading; align_goal_yaw=False must
    # report arrival on position alone, never spinning to align the final yaw.
    with _running_module(goal_tolerance=0.2, align_goal_yaw=False) as h:
        h.feed_odom(1.0, 0.0, 1.2)
        h.feed_path(_path_from_points([(0.0, 0.0), (1.0, 0.0)]))

        assert _wait_until(lambda: bool(h.captured.goal_reached))
        assert [bool(b.data) for b in h.captured.goal_reached] == [True]
        assert h.core.is_goal_reached() is True
        # No final rotation: every command published was a body-yaw rate of zero.
        assert h.captured.cmd_vel
        assert all(abs(float(cmd.angular.z)) < 1e-6 for cmd in h.captured.cmd_vel)
