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

"""Focused tests for Recorder pose resolution."""

from __future__ import annotations

import asyncio
from collections.abc import Awaitable, Callable
from types import SimpleNamespace

from dimos.memory2.module import Recorder, RecorderConfig


class _Input:
    def pure_observable(self) -> object:
        return object()


class _Stream:
    def __init__(self) -> None:
        self.appended: list[tuple[object, dict[str, object]]] = []

    def append(self, msg: object, **kwargs: object) -> None:
        self.appended.append((msg, kwargs))


class _Transform:
    def to_pose(self) -> str:
        return "pose"


class _Tf:
    def __init__(self) -> None:
        self.calls: list[tuple[str, str, float, float]] = []

    def get(self, root: str, frame: str, *, time_point: float, time_tolerance: float) -> _Transform:
        self.calls.append((root, frame, time_point, time_tolerance))
        return _Transform()


class _TestRecorder(Recorder):
    callback: Callable[[object], Awaitable[None]]

    def process_observable(
        self, observable: object, async_cb: Callable[[object], Awaitable[None]]
    ) -> None:
        del observable
        self.callback = async_cb


def _recorder(pose_independent_streams: set[str]) -> tuple[_TestRecorder, _Tf]:
    recorder = _TestRecorder.__new__(_TestRecorder)
    recorder.config = RecorderConfig(
        pose_independent_streams=pose_independent_streams,
        record_tf=False,
    )
    recorder._pose_setters = {}
    tf = _Tf()
    recorder._tf = tf
    return recorder, tf


def test_pose_independent_stream_appends_without_tf_lookup() -> None:
    recorder, tf = _recorder({"color_image"})
    stream = _Stream()
    message = SimpleNamespace(ts=12.5, frame_id="camera")

    recorder._port_to_stream("color_image", _Input(), stream)
    asyncio.run(recorder.callback(message))

    assert tf.calls == []
    assert stream.appended == [(message, {"ts": 12.5, "pose": None})]


def test_unconfigured_stream_retains_tf_lookup() -> None:
    recorder, tf = _recorder(set())
    stream = _Stream()
    message = SimpleNamespace(ts=12.5, frame_id="camera")

    recorder._port_to_stream("status", _Input(), stream)
    asyncio.run(recorder.callback(message))

    assert tf.calls == [("world", "camera", 12.5, 0.5)]
    assert stream.appended == [(message, {"ts": 12.5, "pose": "pose"})]
