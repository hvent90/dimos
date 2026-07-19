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

import asyncio
from queue import Queue
import string
import threading

import pytest
import reactivex as rx
from reactivex import operators as ops

from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import Out
from dimos.core.transport import pLCMTransport


class StartModule(Module):
    uppercase: Out[str]

    @rpc
    def start(self) -> None:
        super().start()

        observable = rx.interval(0.1).pipe(
            ops.take(len(string.ascii_lowercase)),
            ops.map(lambda i: string.ascii_lowercase[i]),
        )

        self.process_observable(observable, self.handle_letter)

    async def handle_letter(self, letter: str) -> None:
        self.uppercase.publish(letter.upper())


@pytest.fixture
def start_module():
    blueprint = StartModule.blueprint()
    coordinator = ModuleCoordinator.build(blueprint)
    yield
    coordinator.stop()


@pytest.fixture
def get_collected_letters():
    uppercase_transport = pLCMTransport("/uppercase")
    uppercase_transport.start()
    queue = Queue()
    uppercase_transport.subscribe(queue.put)

    def _get_collected_letters() -> list[str]:
        return "".join([queue.get(timeout=4) for _ in range(26)])

    yield _get_collected_letters

    uppercase_transport.stop()


def test_async_module_process_observable(get_collected_letters, start_module):
    """
    Tests that process_observable correctly processes items from an observable
    in an async manner.

    Most of the logic is in get_collected_letters, because we need to setup the
    subscription to the result before starting the module. This is because the
    module emits from the start method.

    The strict equality below also locks down the serial-delivery contract: the
    per-subscription dispatcher must invoke `handle_letter` once per item in the
    order they were emitted (the source emits at 100ms intervals, slower than the
    near-zero handler runtime, so no LATEST coalescing should occur).
    """
    collected = get_collected_letters()
    assert len(collected) == 26
    assert collected == "ABCDEFGHIJKLMNOPQRSTUVWXYZ"


def test_async_dispatch_dispose_drains_active_handler() -> None:
    loop = asyncio.new_event_loop()
    loop_thread = threading.Thread(target=loop.run_forever)
    loop_thread.start()
    module = Module.__new__(Module)
    module._loop = loop
    started = threading.Event()
    finished = threading.Event()

    async def handler(_message: object) -> None:
        started.set()
        try:
            await asyncio.sleep(60)
        finally:
            finished.set()

    try:
        on_message, dispatcher = module._make_async_dispatch(handler)
        on_message("pending")
        assert started.wait(timeout=2)

        dispose_thread = threading.Thread(target=dispatcher.dispose)
        dispose_thread.start()
        dispose_thread.join(timeout=2)

        assert not dispose_thread.is_alive()
        assert finished.is_set()
    finally:
        loop.call_soon_threadsafe(loop.stop)
        loop_thread.join(timeout=2)
        loop.close()


def test_async_dispatch_dispose_from_owning_loop_does_not_block() -> None:
    loop = asyncio.new_event_loop()
    loop_thread = threading.Thread(target=loop.run_forever)
    loop_thread.start()
    module = Module.__new__(Module)
    module._loop = loop
    started = threading.Event()
    finished = threading.Event()
    disposed = threading.Event()

    async def handler(_message: object) -> None:
        started.set()
        try:
            await asyncio.sleep(60)
        finally:
            finished.set()

    def dispose_on_loop() -> None:
        dispatcher.dispose()
        disposed.set()

    try:
        on_message, dispatcher = module._make_async_dispatch(handler)
        on_message("pending")
        assert started.wait(timeout=2)
        loop.call_soon_threadsafe(dispose_on_loop)

        assert disposed.wait(timeout=2)
        assert finished.wait(timeout=2)
    finally:
        loop.call_soon_threadsafe(loop.stop)
        loop_thread.join(timeout=2)
        loop.close()
