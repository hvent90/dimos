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

"""Unit tests for the EpisodeMonitor state machine.

Drives the button/keyboard handlers directly and captures published
EpisodeStatus events via a stubbed `status` Out port. The module is built with
`object.__new__` + the subclass' own state init so the test exercises just the
state machine, not the RPC/transport machinery a full `Module()` boots up.
Mirrors the offline `extract_episodes` state machine these events feed.
"""

from __future__ import annotations

import threading

from dimos.learning.collection.episode_monitor import (
    EpisodeMonitorModule,
    EpisodeMonitorModuleConfig,
    EpisodeStatus,
    KeyPress,
)
from dimos.teleop.quest.quest_types import BUTTON_ALIASES, Buttons


class FakeStatusOut:
    """Stand-in for the `status` Out port that records published events."""

    def __init__(self) -> None:
        self.events: list[EpisodeStatus] = []

    def publish(self, status: EpisodeStatus) -> None:
        self.events.append(status)


def _monitor(**config: object) -> tuple[EpisodeMonitorModule, FakeStatusOut]:
    m = EpisodeMonitorModule.__new__(EpisodeMonitorModule)
    m.config = EpisodeMonitorModuleConfig(**config)  # type: ignore[assignment]
    m._state = "idle"
    m._saved = 0
    m._discarded = 0
    m._last_event = "init"
    m._prev_bits = {}
    m._lock = threading.Lock()
    out = FakeStatusOut()
    m.status = out  # type: ignore[assignment]
    return m, out


def _press(monitor: EpisodeMonitorModule, alias: str, ts: float) -> None:
    """Rising edge: release-then-press the given Quest button alias."""
    attr = BUTTON_ALIASES[alias]
    released = Buttons()
    pressed = Buttons()
    setattr(pressed, attr, True)
    monitor._on_buttons(released)
    monitor._on_buttons(pressed)


def test_toggle_starts_then_saves() -> None:
    m, out = _monitor()  # default map: toggle=B, discard=Y
    _press(m, "B", ts=1.0)  # idle → recording
    _press(m, "B", ts=2.0)  # recording → idle (saved)

    events = [e.last_event for e in out.events]
    assert events == ["start", "save"]
    assert out.events[-1].state == "idle"
    assert out.events[-1].episodes_saved == 1
    assert out.events[-1].episodes_discarded == 0


def test_discard_does_not_count_as_saved() -> None:
    m, out = _monitor()
    _press(m, "B", ts=1.0)  # start
    _press(m, "Y", ts=2.0)  # discard

    assert out.events[-1].state == "idle"
    assert out.events[-1].episodes_saved == 0
    assert out.events[-1].episodes_discarded == 1


def test_start_while_recording_autocommits_previous() -> None:
    # toggle (start), then an explicit start via keyboard while still recording:
    # the in-progress episode auto-commits (matches the offline extractor).
    m, out = _monitor(keyboard_map={"start": "r"})
    _press(m, "B", ts=1.0)  # recording
    m._on_keyboard(KeyPress(key="r", ts=2.0))  # start again → auto-commit prior

    assert out.events[-1].last_event == "start"
    assert out.events[-1].state == "recording"
    assert out.events[-1].episodes_saved == 1  # the auto-committed one


def test_no_event_without_rising_edge() -> None:
    m, out = _monitor()
    pressed = Buttons()
    pressed.right_secondary = True  # B held
    m._on_buttons(pressed)
    m._on_buttons(pressed)  # still held — no new edge
    assert [e.last_event for e in out.events] == ["start"]


def test_published_status_is_internally_consistent() -> None:
    # Every published event's counters/state must match the event it carries —
    # the snapshot is taken under the same lock as the mutation.
    m, out = _monitor()
    _press(m, "B", 1.0)  # start
    _press(m, "B", 2.0)  # save  (1)
    _press(m, "B", 3.0)  # start
    _press(m, "B", 4.0)  # save  (2)
    _press(m, "B", 5.0)  # start
    _press(m, "Y", 6.0)  # discard (1)

    for e in out.events:
        if e.last_event == "start":
            assert e.state == "recording"
        elif e.last_event in ("save", "discard"):
            assert e.state == "idle"
    assert out.events[-1].episodes_saved == 2
    assert out.events[-1].episodes_discarded == 1


def test_reset_counters() -> None:
    m, out = _monitor()
    _press(m, "B", 1.0)
    _press(m, "B", 2.0)
    status = m.reset_counters()
    assert status.episodes_saved == 0
    assert status.episodes_discarded == 0
    assert status.state == "idle"
    assert status.last_event == "init"
