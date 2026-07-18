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

"""Unit tests for the TrajectoryReplayModule record/replay state machine.

The module is built normally; only its boot side effects (asyncio loop + RPC
transport from `Module.__init__`) are patched out, and its two Out ports are
mocks so published messages can be inspected. Handlers are driven directly.
"""

from __future__ import annotations

from collections.abc import Callable, Iterator

import pytest
import pytest_mock

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.protocol.rpc.pubsubrpc import LCMRPC
from dimos.teleop.quest.quest_types import Buttons
from dimos.teleop.utils.trajectory_replay_module import TrajectoryReplayModule

TASK = "teleop_xarm"


@pytest.fixture
def make_module(
    mocker: pytest_mock.MockerFixture,
) -> Iterator[Callable[..., TrajectoryReplayModule]]:
    mocker.patch("dimos.core.module.get_loop", return_value=(mocker.MagicMock(), None))
    mocker.patch.object(LCMRPC, "__init__", return_value=None)
    mocker.patch.object(LCMRPC, "serve_module_rpc", return_value=None)
    mocker.patch.object(LCMRPC, "start", return_value=None)
    mocker.patch.object(LCMRPC, "stop", return_value=None)

    built: list[TrajectoryReplayModule] = []

    def _make(**config: object) -> TrajectoryReplayModule:
        m = TrajectoryReplayModule(**config)
        m.cartesian_out = mocker.MagicMock()  # type: ignore[assignment]
        m.buttons_out = mocker.MagicMock()  # type: ignore[assignment]
        m.replay_progress = mocker.MagicMock()  # type: ignore[assignment]
        built.append(m)
        return m

    yield _make
    for m in built:
        m._stop_replay.set()
        t = m._replay_thread
        if t is not None:
            t.join(timeout=2.0)


def _delta(x: float) -> PoseStamped:
    return PoseStamped(position=[x, 0.0, 0.0], orientation=[0, 0, 0, 1], frame_id=TASK)


def _buttons(*, trigger: float = 0.0, **attrs: bool) -> Buttons:
    b = Buttons()
    for name, value in attrs.items():
        b.set_attribute(name, value)
    if trigger:
        b.pack_analog_triggers(left=0.0, right=trigger)
    return b


def _cartesian_out(m: TrajectoryReplayModule) -> list[PoseStamped]:
    return [call.args[0] for call in m.cartesian_out.publish.call_args_list]  # type: ignore[attr-defined]


def _buttons_out(m: TrajectoryReplayModule) -> list[Buttons]:
    return [call.args[0] for call in m.buttons_out.publish.call_args_list]  # type: ignore[attr-defined]


def test_live_teleop_forwarded_when_primary_held(
    make_module: Callable[..., TrajectoryReplayModule],
) -> None:
    m = make_module()
    m._on_buttons(_buttons(right_primary=True))  # X/A engaged (live teleop)
    d = _delta(0.1)
    m._on_controller(d)
    assert _cartesian_out(m) == [d]  # forwarded unchanged


def test_pose_without_primary_or_draw_is_dropped(
    make_module: Callable[..., TrajectoryReplayModule],
) -> None:
    # A pose arriving while neither X/A nor grip is held must not reach the arm —
    # guards against draw poses leaking to the arm before the latch flips.
    m = make_module()
    m._on_buttons(_buttons())  # nothing held
    m._on_controller(_delta(0.1))
    assert _cartesian_out(m) == []


def test_grip_records_without_forwarding(
    make_module: Callable[..., TrajectoryReplayModule],
) -> None:
    m = make_module()
    m._on_buttons(_buttons())  # release baseline
    m._on_buttons(_buttons(right_grip=True))  # grip rising → record
    m._on_controller(_delta(0.1))
    m._on_controller(_delta(0.2))
    # Arm stays still while drawing: nothing forwarded.
    assert _cartesian_out(m) == []
    assert len(m._buffer) == 2
    m._on_buttons(_buttons())  # grip falling → freeze
    assert m._recording is False
    assert len(m._buffer) == 2


def test_replay_emits_engage_then_deltas_then_disengage(
    make_module: Callable[..., TrajectoryReplayModule],
) -> None:
    m = make_module(replay_speed=1.0)
    # Record two deltas.
    m._on_buttons(_buttons())
    m._on_buttons(_buttons(right_grip=True))
    m._on_controller(_delta(0.1))
    m._on_controller(_delta(0.2))
    m._on_buttons(_buttons())  # freeze

    # Press B → replay (runs on a background thread).
    m._on_buttons(_buttons(right_secondary=True))
    t = m._replay_thread
    assert t is not None
    t.join(timeout=3.0)
    assert not t.is_alive()

    # The replay thread publishes exactly one engage (right_primary True) edge
    # and one disengage (False) edge; the live buttons forwarded before replay
    # all have right_primary False. So the sequence of primary states must end
    # ...False, True (engage), False (disengage).
    btns = _buttons_out(m)
    engage_states = [b.right_primary for b in btns]
    assert True in engage_states, "replay never engaged the task"
    engage_idx = engage_states.index(True)
    assert engage_states[-1] is False, "replay must disengage at the end"
    assert engage_idx < len(engage_states) - 1

    # The two recorded deltas were republished (after the engage) with the same
    # task frame_id, in order.
    replayed = _cartesian_out(m)
    assert len(replayed) == 2
    assert [round(p.position.x, 3) for p in replayed] == [0.1, 0.2]
    assert all(p.frame_id == TASK for p in replayed)


def test_engage_press_cancels_replay_and_resumes_live(
    make_module: Callable[..., TrajectoryReplayModule],
) -> None:
    import time

    # Slow replay so the worker is still running when we press A.
    m = make_module(replay_speed=0.01)
    m._on_buttons(_buttons())
    m._on_buttons(_buttons(right_grip=True))
    # Give the samples a real time gap so the worker sleeps between them.
    m._buffer = [
        (0.0, _delta(0.1), 0.0),
        (0.5, _delta(0.2), 0.0),
        (1.0, _delta(0.3), 0.0),
    ]
    m._recording = False
    m._on_buttons(_buttons())  # baseline (grip released)

    m._on_buttons(_buttons(right_secondary=True))  # B → replay
    t = m._replay_thread
    assert t is not None
    time.sleep(0.1)
    assert t.is_alive(), "replay should still be running"

    # Press A → cancel. Rising edge requires a prior non-primary sample, which
    # the baseline above provided.
    m._on_buttons(_buttons(right_primary=True))
    t.join(timeout=2.0)
    assert not t.is_alive()
    assert m._replaying is False

    # Live teleop now owns control: a pose forwards to the arm (primary held).
    n_before = len(_cartesian_out(m))
    m._on_controller(_delta(0.9))
    assert len(_cartesian_out(m)) == n_before + 1


def test_replay_reproduces_recorded_gripper(
    make_module: Callable[..., TrajectoryReplayModule],
) -> None:
    m = make_module(replay_speed=1.0)
    m._on_buttons(_buttons())
    # Grip pressed with the trigger squeezed → gripper value captured per sample.
    m._on_buttons(_buttons(right_grip=True, trigger=0.75))
    m._on_controller(_delta(0.1))
    m._on_controller(_delta(0.2))
    m._on_buttons(_buttons())  # freeze (release grip; trigger back to 0)

    m._on_buttons(_buttons(right_secondary=True))  # B → replay
    t = m._replay_thread
    assert t is not None
    t.join(timeout=3.0)

    # Some replayed Buttons must carry the recorded ~0.75 gripper trigger while
    # engaged, proving the gripper is reproduced alongside the arm motion.
    btns = _buttons_out(m)
    engaged_triggers = [round(b.right_trigger_analog, 2) for b in btns if b.right_primary]
    assert any(t >= 0.7 for t in engaged_triggers), engaged_triggers


def test_replay_publishes_progress(
    make_module: Callable[..., TrajectoryReplayModule],
) -> None:
    m = make_module(replay_speed=1.0)
    m._on_buttons(_buttons())
    m._on_buttons(_buttons(right_grip=True))
    m._on_controller(_delta(0.1))
    m._on_controller(_delta(0.2))
    m._on_buttons(_buttons())

    m._on_buttons(_buttons(right_secondary=True))
    t = m._replay_thread
    assert t is not None
    t.join(timeout=3.0)

    permilles = [call.args[0].data for call in m.replay_progress.publish.call_args_list]
    assert permilles, "no progress published"
    assert permilles[-1] == 1000  # ends fully consumed
    assert all(0 <= p <= 1000 for p in permilles)


def test_play_with_too_few_points_does_not_replay(
    make_module: Callable[..., TrajectoryReplayModule],
) -> None:
    m = make_module(min_points=2)
    m._on_buttons(_buttons())
    m._on_buttons(_buttons(right_grip=True))
    m._on_controller(_delta(0.1))  # only one point
    m._on_buttons(_buttons())  # freeze (1 point)
    m._on_buttons(_buttons(right_secondary=True))  # B → should be a no-op
    assert m._replay_thread is None
    assert _cartesian_out(m) == []


def test_live_input_gated_during_replay(
    make_module: Callable[..., TrajectoryReplayModule],
) -> None:
    m = make_module()
    m._replaying = True  # simulate an in-flight replay
    m._on_controller(_delta(0.9))  # live delta arrives mid-replay
    assert _cartesian_out(m) == []  # dropped, doesn't fight the replay
