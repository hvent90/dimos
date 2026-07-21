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

from __future__ import annotations

import builtins
import sys
from types import ModuleType
from typing import Any

import pytest

from dimos.hardware.manipulators.piper.adapter import PiperAdapter


class _FakePiper:
    def __init__(self, **_: object) -> None:
        self.gripper_calls: list[tuple[int, int, int, int]] = []

    def ConnectPort(self, **_: object) -> None:
        pass

    def GetArmStatus(self) -> object:
        return object()

    def MotionCtrl_1(self, *_: int) -> None:
        pass

    def MotionCtrl_2(self, **_: int) -> None:
        pass

    def JointCtrl(self, *_: int) -> None:
        pass

    def GripperCtrl(self, position: int, speed: int, code: int, param: int) -> None:
        self.gripper_calls.append((position, speed, code, param))
        if len(self.gripper_calls) == 1:
            raise RuntimeError("gripper unavailable")


def test_connect_reports_missing_sdk(
    monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    real_import = builtins.__import__

    def missing_piper_sdk(name: str, *args: Any, **kwargs: Any) -> ModuleType:
        if name == "piper_sdk":
            raise ImportError("piper_sdk unavailable")
        return real_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", missing_piper_sdk)

    assert not PiperAdapter().connect()
    assert "Piper SDK not installed" in capsys.readouterr().out


def test_connect_continues_when_gripper_startup_fails(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    sdk_module = ModuleType("piper_sdk")
    sdk_module.C_PiperInterface_V2 = _FakePiper  # type: ignore[attr-defined]
    monkeypatch.setitem(sys.modules, "piper_sdk", sdk_module)
    monkeypatch.setattr("dimos.hardware.manipulators.piper.adapter.time.sleep", lambda _: None)

    adapter = PiperAdapter()

    assert adapter.connect()
    assert adapter.is_connected()
    assert not adapter._gripper_initialized
