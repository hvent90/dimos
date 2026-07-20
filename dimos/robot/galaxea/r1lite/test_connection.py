# Copyright 2025-2026 Dimensional Inc.
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

"""Deterministic unit tests for R1LiteConnection.

The connection is exercised without ROS or hardware: RawROS is faked and the
ROS message types the handlers import are injected into sys.modules.
"""

from __future__ import annotations

import ast
from pathlib import Path
import sys
import threading
import time
import types
from typing import Any

import pytest

from dimos.robot.galaxea.r1lite import connection as conn_mod
from dimos.robot.galaxea.r1lite.connection import R1LiteConnection, R1LiteConnectionConfig

_CONN_SRC = Path(conn_mod.__file__)


class _Msg:
    def __init__(self, **kw: Any) -> None:
        self.__dict__.update(kw)


class _FakeRos:
    def __init__(self) -> None:
        self.published: list[tuple[Any, Any]] = []

    def now_stamp(self) -> Any:
        return _Msg(sec=0, nanosec=0)

    def publish(self, topic: Any, message: Any) -> None:
        self.published.append((topic, message))


@pytest.fixture(autouse=True)
def _fake_ros_msgs(monkeypatch: Any) -> None:
    """Inject the ROS message modules the handlers import lazily."""

    class _Header:
        def __init__(self) -> None:
            self.stamp = None

    class _RosJointState:
        def __init__(self) -> None:
            self.header = _Header()
            self.name: list[str] = []
            self.position: list[float] = []
            self.velocity: list[float] = []
            self.effort: list[float] = []

    sensor_mod = types.ModuleType("sensor_msgs")
    sensor_msg_mod = types.ModuleType("sensor_msgs.msg")
    sensor_msg_mod.JointState = _RosJointState  # type: ignore[attr-defined]
    sensor_mod.msg = sensor_msg_mod  # type: ignore[attr-defined]
    monkeypatch.setitem(sys.modules, "sensor_msgs", sensor_mod)
    monkeypatch.setitem(sys.modules, "sensor_msgs.msg", sensor_msg_mod)


def _bare() -> R1LiteConnection:
    """A connection instance with only the fields the handlers touch."""
    c = R1LiteConnection.__new__(R1LiteConnection)
    c.config = R1LiteConnectionConfig()
    c._lock = threading.Lock()
    c._ros = _FakeRos()
    c._cmd_left_topic = "left"
    c._cmd_right_topic = "right"
    c._cmd_gripper_left_topic = "gl"
    c._cmd_gripper_right_topic = "gr"
    c._torso_cmd_warned = False
    c._stale_logged = False
    now = time.monotonic()
    c._torso_seen = c._left_seen = c._right_seen = True
    c._torso_ts = c._left_ts = c._right_ts = now
    return c


def _motor_cmd(num_joints: int = 16) -> Any:
    q = [float(i) for i in range(num_joints)]
    return _Msg(num_joints=num_joints, q=q, dq=[0.0] * num_joints)


def test_no_module_level_ros_import() -> None:
    tree = ast.parse(_CONN_SRC.read_text())
    for node in tree.body:
        names = []
        if isinstance(node, ast.Import):
            names = [a.name for a in node.names]
        elif isinstance(node, ast.ImportFrom):
            names = [node.module or ""]
        for n in names:
            assert not n.startswith(("rclpy", "sensor_msgs", "geometry_msgs", "std_msgs")), (
                f"module-level ROS import {n} would break `import dimos` without ROS"
            )


def test_copy_segment_rejects_wrong_length() -> None:
    dst = [0.0] * 6
    ok = R1LiteConnection._copy_segment(
        _Msg(position=[1.0] * 5, velocity=[], effort=[]), dst, [0.0] * 6, [0.0] * 6
    )
    assert ok is False
    assert dst == [0.0] * 6


def test_copy_segment_accepts_exact_length() -> None:
    dst = [0.0] * 6
    ok = R1LiteConnection._copy_segment(
        _Msg(position=[1.0] * 6, velocity=[], effort=[]), dst, [0.0] * 6, [0.0] * 6
    )
    assert ok is True
    assert dst == [1.0] * 6


def test_tracking_velocities_zero_maps_to_speed() -> None:
    c = _bare()
    c.config.tracking_speed = 0.7
    assert c._tracking_velocities([0.0, 0.0]) == [0.7, 0.7]
    assert c._tracking_velocities([1.5]) == [1.5]


def test_stale_segments() -> None:
    c = _bare()
    c._left_ts = time.monotonic() - 10.0
    torso_stale, left_stale, right_stale = c._stale_segments(time.monotonic())
    assert left_stale and not torso_stale and not right_stale


def test_motor_command_wrong_count_ignored() -> None:
    c = _bare()
    c._on_motor_command(_motor_cmd(num_joints=12))
    assert c._ros.published == []


def test_motor_command_maps_arms_and_drops_torso() -> None:
    c = _bare()
    c._on_motor_command(_motor_cmd())
    topics = {t for t, _ in c._ros.published}
    assert topics == {"left", "right"}
    by_topic = {t: m for t, m in c._ros.published}
    assert by_topic["left"].position == [4.0, 5.0, 6.0, 7.0, 8.0, 9.0]
    assert by_topic["right"].position == [10.0, 11.0, 12.0, 13.0, 14.0, 15.0]


def test_motor_command_dropped_when_arm_feedback_stale() -> None:
    c = _bare()
    c._left_ts = time.monotonic() - 10.0
    c._on_motor_command(_motor_cmd())
    assert c._ros.published == []


def test_stream_chassis_zero_publishes_multiple_ticks() -> None:
    c = _bare()
    calls: list[Any] = []
    c._publish_chassis_command = lambda twist: calls.append(twist)  # type: ignore[method-assign]
    c._last_chassis_speed = 0.0
    c.config.publish_rate_hz = 100.0
    c._stream_chassis_zero(0.05)
    assert len(calls) >= 2


def test_gripper_out_of_range_ignored() -> None:
    c = _bare()
    c._on_gripper_command("left", _Msg(position=[250.0]))
    assert c._ros.published == []
