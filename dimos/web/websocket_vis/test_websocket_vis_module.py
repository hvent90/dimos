import asyncio
import inspect
import threading

import pytest

from dimos.core.module import Module
from dimos.web.websocket_vis.websocket_vis_module import WebsocketVisModule


class _SocketIo:
    def __init__(self) -> None:
        self.disconnected = threading.Event()

    async def disconnect(self) -> None:
        self.disconnected.set()

    async def emit(self, event: str, data: object) -> None:
        await asyncio.sleep(0)


def _module() -> WebsocketVisModule:
    module = WebsocketVisModule.__new__(WebsocketVisModule)
    module.sio = _SocketIo()
    module._broadcast_loop = asyncio.new_event_loop()
    module._broadcast_thread = None
    module._uvicorn_server = None
    module._uvicorn_server_thread = None
    module._pending_coroutines = set()
    module._pending_coroutines_lock = threading.RLock()
    return module


def test_stop_waits_for_disconnect_before_stopping_loop(monkeypatch: pytest.MonkeyPatch) -> None:
    module = _module()
    thread = threading.Thread(target=module._broadcast_loop.run_forever)
    thread.start()
    monkeypatch.setattr(Module, "stop", lambda self: None)

    module.stop()

    thread.join(timeout=1)
    assert module.sio.disconnected.is_set()
    assert not thread.is_alive()
    assert module._pending_coroutines == set()


def test_stop_closes_coroutine_when_loop_submission_races_shutdown(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    module = _module()
    module._broadcast_loop = type(
        "LoopRace",
        (),
        {
            "is_closed": lambda self: False,
            "call_soon_threadsafe": lambda self, callback: None,
            "stop": lambda self: None,
        },
    )()
    monkeypatch.setattr(Module, "stop", lambda self: None)

    submitted = []

    def fail_submission(coroutine, loop):
        submitted.append(coroutine)
        raise RuntimeError("loop closed")

    monkeypatch.setattr(asyncio, "run_coroutine_threadsafe", fail_submission)

    module.stop()

    assert len(submitted) == 1
    assert inspect.getcoroutinestate(submitted[0]) == inspect.CORO_CLOSED
