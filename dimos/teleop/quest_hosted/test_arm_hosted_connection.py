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

"""Unit tests for ArmHostedConnection's operator-command handling.

No coordinator / no WebRTC: ``ArmHostedConnection.__init__`` builds a whole
Module, so instances are assembled bare (``object.__new__``) with only the
fields the tested paths touch, and output ports are mocked.
"""

from __future__ import annotations

import json
import threading
from types import SimpleNamespace
from typing import Any
from unittest.mock import MagicMock

from dimos_lcm.geometry_msgs import PoseStamped as LCMPoseStamped
from dimos_lcm.sensor_msgs import Joy as LCMJoy
import numpy as np
import pytest

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.teleop.quest.quest_types import Hand, QuestControllerState
from dimos.teleop.quest_hosted.arm_hosted_connection import ArmHostedConnection
from dimos.teleop.utils.stream_stats import LiveStreamStats


def _bare_connection() -> ArmHostedConnection:
    """An ArmHostedConnection with only the fields the command paths need."""
    s = object.__new__(ArmHostedConnection)
    s._lock = threading.RLock()
    s._is_engaged = {Hand.LEFT: False, Hand.RIGHT: False}
    s._initial_poses = {Hand.LEFT: None, Hand.RIGHT: None}
    s._current_poses = {Hand.LEFT: None, Hand.RIGHT: None}
    s._controllers = {Hand.LEFT: None, Hand.RIGHT: None}
    s._decoders = {
        LCMPoseStamped._get_packed_fingerprint(): s._on_pose_bytes,
        LCMJoy._get_packed_fingerprint(): s._on_joy_bytes,
    }
    s._estopped = False
    s._cmd_stats = LiveStreamStats()
    s._task_names = {Hand.RIGHT: "teleop_xarm"}
    s._mux_init(["cam1", "cam2"])
    s.config = SimpleNamespace(
        task_names={"right": "teleop_xarm"},
        control_loop_hz=50.0,
        telemetry_hz=3.0,
        video_max_width=0,
        video_max_fps=0.0,
        latency_stamp=False,
    )
    s.left_controller_output = MagicMock()
    s.right_controller_output = MagicMock()
    s.buttons = MagicMock()
    s.telemetry_out = MagicMock()
    s.mux_image = MagicMock()
    s.video_stats = MagicMock()
    return s


@pytest.fixture
def station() -> ArmHostedConnection:
    return _bare_connection()


def _pose_bytes(frame_id: str, ts: float = 1.0) -> bytes:
    return PoseStamped(ts=ts, frame_id=frame_id).lcm_encode()


def _tick(s: ArmHostedConnection) -> None:
    """One control-loop iteration (the loop body, without the thread)."""
    with s._lock:
        s._handle_engage()
        for hand in Hand:
            if not s._should_publish(hand):
                continue
            output_pose = s._get_output_pose(hand)
            if output_pose is not None:
                s._publish_msg(hand, output_pose)


def _sent_acks(s: ArmHostedConnection) -> list[dict[str, Any]]:
    return [
        json.loads(call.args[0])
        for call in s.telemetry_out.publish.call_args_list
        if json.loads(call.args[0]).get("type") == "cmd_ack"
    ]


def _img(width: int, height: int) -> Image:
    return Image(
        data=np.zeros((height, width, 3), dtype=np.uint8),
        format=ImageFormat.RGB,
        frame_id="test",
    )


# ─── Command plane: pose dispatch ──────────────────────────────────────


def test_cmd_raw_pose_routes_to_hand(station: ArmHostedConnection) -> None:
    station._on_cmd_raw(_pose_bytes("right"))
    assert station._current_poses[Hand.RIGHT] is not None
    assert station._current_poses[Hand.LEFT] is None


def test_cmd_raw_bad_frame_id_dropped(station: ArmHostedConnection) -> None:
    station._on_cmd_raw(_pose_bytes("torso"))
    assert station._current_poses[Hand.LEFT] is None
    assert station._current_poses[Hand.RIGHT] is None


def test_cmd_raw_foreign_bytes_ignored(station: ArmHostedConnection) -> None:
    station._on_cmd_raw(b"\x00\x01\x02\x03garbage-frame")
    assert station._current_poses[Hand.RIGHT] is None


def test_cmd_raw_accepts_str(station: ArmHostedConnection) -> None:
    # A str payload can't match an LCM fingerprint, but must not crash.
    station._on_cmd_raw("hello")


def test_pose_feeds_cmd_stats(station: ArmHostedConnection) -> None:
    assert station._cmd_stats.snapshot() is None
    station._on_cmd_raw(_pose_bytes("right"))
    station._on_cmd_raw(_pose_bytes("right", ts=1.02))
    assert station._cmd_stats.snapshot() is not None


# ─── Engage → publish with task-name routing ───────────────────────────


def _engage_right(s: ArmHostedConnection) -> None:
    s._on_cmd_raw(_pose_bytes("right"))
    s._controllers[Hand.RIGHT] = QuestControllerState(is_left=False, primary=True)
    _tick(s)


def test_engage_publishes_task_routed_pose(station: ArmHostedConnection) -> None:
    _engage_right(station)
    assert station._is_engaged[Hand.RIGHT]
    station.right_controller_output.publish.assert_called()
    out = station.right_controller_output.publish.call_args.args[0]
    assert out.frame_id == "teleop_xarm"
    station.left_controller_output.publish.assert_not_called()


def test_release_disengages(station: ArmHostedConnection) -> None:
    _engage_right(station)
    station._controllers[Hand.RIGHT] = QuestControllerState(is_left=False, primary=False)
    _tick(station)
    assert not station._is_engaged[Hand.RIGHT]


# ─── E-STOP latch ──────────────────────────────────────────────────────


def test_estop_disengages_blocks_publish_and_acks(station: ArmHostedConnection) -> None:
    _engage_right(station)
    station.right_controller_output.publish.reset_mock()

    station._on_state_json(b'{"type": "estop", "nonce": 7}')

    assert station._estopped
    assert not station._is_engaged[Hand.RIGHT]
    _tick(station)  # primary still held — must NOT re-engage or publish
    assert not station._is_engaged[Hand.RIGHT]
    station.right_controller_output.publish.assert_not_called()
    assert _sent_acks(station) == [{"type": "cmd_ack", "nonce": 7, "ok": True}]


def test_estop_clear_rearms_but_does_not_resume(station: ArmHostedConnection) -> None:
    _engage_right(station)
    station._on_state_json(b'{"type": "estop", "nonce": 1}')
    station.right_controller_output.publish.reset_mock()

    station._on_state_json(b'{"type": "estop_clear", "nonce": 2}')
    assert not station._estopped

    # Button still held from before the estop: engaging again is allowed only
    # because a held primary re-engages from the CURRENT pose (delta zero) —
    # the arm doesn't jump. What must not happen is publishing while latched.
    _tick(station)
    assert station._is_engaged[Hand.RIGHT]


def test_operator_lost_disengages(station: ArmHostedConnection) -> None:
    _engage_right(station)
    station._on_state_json(b'{"type": "operator_lost"}')
    assert not station._is_engaged[Hand.RIGHT]
    assert not station._estopped  # loss is not an estop; re-engage allowed


# ─── State plane: camera select / video stats / malformed input ────────


def test_camera_select_switches_and_republish(station: ArmHostedConnection) -> None:
    station._on_cam("cam2", _img(64, 48))  # cached, not selected → no publish
    station.mux_image.publish.assert_not_called()
    station._on_state_json(b'{"type": "camera_select", "cams": ["cam2"]}')
    assert station._mux_state() == ["cam2"]
    station.mux_image.publish.assert_called_once()  # immediate republish


def test_camera_select_unknown_falls_back(station: ArmHostedConnection) -> None:
    station._on_state_json(b'{"type": "camera_select", "cams": ["cam9"]}')
    assert station._mux_state() == ["cam1"]


def test_video_stats_published(station: ArmHostedConnection) -> None:
    station._on_state_json(b'{"type": "video_stats", "fps": 30}')
    station.video_stats.publish.assert_called_once()
    assert station.video_stats.publish.call_args.args[0].fps == 30.0


def test_malformed_json_ignored(station: ArmHostedConnection) -> None:
    station._on_state_json(b"{broken")
    station._on_state_json(b"\x00binary")
    station._on_state_json(b'{"type": "unknown_thing"}')


# ─── Camera mux ────────────────────────────────────────────────────────


def test_mux_two_cams_hstack_scaled(station: ArmHostedConnection) -> None:
    station._set_cam_selection(["cam1", "cam2"])
    station.mux_image.publish.reset_mock()
    station._on_cam("cam1", _img(640, 480))
    station._on_cam("cam2", _img(320, 120))
    out = station.mux_image.publish.call_args.args[0]
    # Both tiles scaled to the min height (120); cam1 640x480 → 160x120.
    assert out.data.shape[0] == 120
    assert out.data.shape[1] == 160 + 320


def test_mux_width_cap(station: ArmHostedConnection) -> None:
    station.config.video_max_width = 320
    station._on_cam("cam1", _img(640, 480))
    out = station.mux_image.publish.call_args.args[0]
    assert out.data.shape[1] == 320
    assert out.data.shape[0] == 240


# ─── Telemetry payload ─────────────────────────────────────────────────


def test_telemetry_payload_shape(station: ArmHostedConnection) -> None:
    _engage_right(station)
    p = station._telemetry_payload()
    assert p["type"] == "robot_telemetry"
    assert p["state"]["engaged"] == {"left": False, "right": True}
    assert p["state"]["cams"] == ["cam1"]
    assert p["state"]["estopped"] is False
    assert p["robot_ts"] > 0
