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

"""Unit tests for Go2HostedConnection's operator-command handling.

No robot / no WebRTC: ``Go2HostedConnection.__init__`` builds a whole Module,
so we exercise the pure command logic on a bare instance (``object.__new__``)
with a mocked ``connection`` and only the attributes the tested methods touch.
Covers the security-relevant paths — the sport-command allow-list and the
stale / out-of-order cmd_vel drop on the unreliable wire.
"""

from __future__ import annotations

import time
from types import SimpleNamespace
from typing import Any
from unittest.mock import MagicMock

import pytest

from dimos.robot.unitree.go2.hosted_connection import (
    ALLOWED_SPORT_CMDS,
    Go2HostedConnection,
)


def _bare_connection() -> Go2HostedConnection:
    """A Go2HostedConnection with only the fields the command paths need."""
    conn = object.__new__(Go2HostedConnection)
    conn.connection = MagicMock()
    conn._last_cmd_ts = 0.0
    conn.config = SimpleNamespace(cmd_stale_after_sec=0.5)
    return conn


def _twist(ts: float) -> Any:
    return SimpleNamespace(
        ts=ts, linear=SimpleNamespace(x=0, y=0, z=0), angular=SimpleNamespace(x=0, y=0, z=0)
    )


# ─── sport-command allow-list ────────────────────────────────────────


@pytest.mark.parametrize("name", list(ALLOWED_SPORT_CMDS))
def test_allowed_sport_cmd_dispatched(name: str, monkeypatch: pytest.MonkeyPatch) -> None:
    """Every allow-listed name maps to its api_id and calls sport_command."""
    conn = _bare_connection()
    acks: list[tuple[Any, bool]] = []
    monkeypatch.setattr(conn, "_send_ack", lambda nonce, ok: acks.append((nonce, ok)))
    conn.connection.sport_command.return_value = True

    conn._handle_sport_cmd({"name": name, "nonce": 7})
    # runs on a worker thread — wait for the ack instead of sleeping blindly.
    for _ in range(200):
        if acks:
            break
        time.sleep(0.005)

    conn.connection.sport_command.assert_called_once_with(ALLOWED_SPORT_CMDS[name])
    assert acks == [(7, True)]


@pytest.mark.parametrize("name", ["Backflip", "", "sport_command", None, 1013])
def test_disallowed_sport_cmd_rejected(name: Any, monkeypatch: pytest.MonkeyPatch) -> None:
    """Unknown / non-allow-listed names are rejected with ok=False, no call."""
    conn = _bare_connection()
    acks: list[tuple[Any, bool]] = []
    monkeypatch.setattr(conn, "_send_ack", lambda nonce, ok: acks.append((nonce, ok)))

    conn._handle_sport_cmd({"name": name, "nonce": 9})

    conn.connection.sport_command.assert_not_called()
    assert acks == [(9, False)]


def test_standready_is_not_a_raw_sport_command(monkeypatch: pytest.MonkeyPatch) -> None:
    """StandReady routes to the combo, never a single sport_command call."""
    conn = _bare_connection()
    called: list[Any] = []
    monkeypatch.setattr(conn, "_stand_ready", lambda nonce: called.append(nonce))

    conn._handle_sport_cmd({"name": "StandReady", "nonce": 3})

    assert called == [3]
    conn.connection.sport_command.assert_not_called()


# ─── cmd_vel stale / out-of-order drop ───────────────────────────────


def test_move_drops_stale_cmd() -> None:
    """A twist older than cmd_stale_after_sec is dropped, robot not moved."""
    conn = _bare_connection()
    old = time.time() - 1.0  # > 0.5s stale threshold
    assert conn.move(_twist(old)) is False


def test_move_drops_out_of_order_cmd() -> None:
    """A twist with ts <= the newest seen is dropped (reorder guard)."""
    conn = _bare_connection()
    now = time.time()
    conn._last_cmd_ts = now
    assert conn.move(_twist(now)) is False  # equal → drop
    assert conn.move(_twist(now - 0.1)) is False  # older → drop


def test_move_accepts_fresh_in_order_cmd() -> None:
    """A fresh, newer twist is forwarded and advances _last_cmd_ts."""
    conn = _bare_connection()
    conn.connection.move.return_value = True
    ts = time.time()

    assert conn.move(_twist(ts)) is True
    assert conn._last_cmd_ts == ts
    conn.connection.move.assert_called_once()


# ─── speed-mode / rage toggle ────────────────────────────────────────


def _wait_ack(acks: list[Any]) -> None:
    for _ in range(200):
        if acks:
            return
        time.sleep(0.005)


def test_set_mode_unknown_rejected(monkeypatch: pytest.MonkeyPatch) -> None:
    conn = _bare_connection()
    conn._rage_active = False
    acks: list[tuple[Any, bool]] = []
    monkeypatch.setattr(conn, "_send_ack", lambda nonce, ok: acks.append((nonce, ok)))

    conn._handle_set_mode({"mode": "ludicrous", "nonce": 1})

    assert acks == [(1, False)]
    conn.connection.set_rage_mode.assert_not_called()


@pytest.mark.parametrize("mode", ["normal", "high"])
def test_set_mode_non_rage_does_not_touch_firmware(
    mode: str, monkeypatch: pytest.MonkeyPatch
) -> None:
    """normal/high are browser-side scale only — acked, no firmware toggle."""
    conn = _bare_connection()
    conn._rage_active = False
    acks: list[tuple[Any, bool]] = []
    monkeypatch.setattr(conn, "_send_ack", lambda nonce, ok: acks.append((nonce, ok)))

    conn._handle_set_mode({"mode": mode, "nonce": 2})

    assert acks == [(2, True)]
    conn.connection.set_rage_mode.assert_not_called()


def test_set_mode_rage_boundary_toggles_firmware(monkeypatch: pytest.MonkeyPatch) -> None:
    """Crossing into rage calls set_rage_mode(True) and flips _rage_active."""
    conn = _bare_connection()
    conn._rage_active = False
    conn.connection.set_rage_mode.return_value = True
    acks: list[tuple[Any, bool]] = []
    monkeypatch.setattr(conn, "_send_ack", lambda nonce, ok: acks.append((nonce, ok)))

    conn._handle_set_mode({"mode": "rage", "nonce": 4})
    _wait_ack(acks)

    conn.connection.set_rage_mode.assert_called_once_with(True)
    assert conn._rage_active is True
    assert acks == [(4, True)]


def test_set_mode_already_in_rage_short_circuits(monkeypatch: pytest.MonkeyPatch) -> None:
    """Re-selecting rage while already active acks without a firmware call."""
    conn = _bare_connection()
    conn._rage_active = True
    acks: list[tuple[Any, bool]] = []
    monkeypatch.setattr(conn, "_send_ack", lambda nonce, ok: acks.append((nonce, ok)))

    conn._handle_set_mode({"mode": "rage", "nonce": 5})

    assert acks == [(5, True)]
    conn.connection.set_rage_mode.assert_not_called()
