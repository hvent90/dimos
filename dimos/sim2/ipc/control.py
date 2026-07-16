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

"""Small Unix-socket control plane for sim2 lockstep notifications."""

from __future__ import annotations

from collections.abc import Iterator
import json
from pathlib import Path
import queue
import socket
import threading
from typing import Any


class SimControlServer:
    def __init__(self, path: Path) -> None:
        self.path = path
        self._socket: socket.socket | None = None
        self._clients: set[socket.socket] = set()
        self._clients_lock = threading.Lock()
        self._clients_condition = threading.Condition(self._clients_lock)
        self._acks: set[tuple[int, int]] = set()
        self._ack_condition = threading.Condition()
        self._last_observation: dict[str, Any] | None = None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        self.path.unlink(missing_ok=True)
        server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        server.bind(str(self.path))
        server.listen()
        server.settimeout(0.2)
        self._socket = server
        self._thread = threading.Thread(target=self._accept_loop, name="sim2-control", daemon=True)
        self._thread.start()

    def publish_observation(
        self,
        *,
        episode_id: int,
        control_tick: int,
        sim_time: float,
        control_dt: float,
        reset: bool = False,
    ) -> None:
        message = {
            "event": "observation",
            "episode_id": episode_id,
            "control_tick": control_tick,
            "sim_time": sim_time,
            "control_dt": control_dt,
            "reset": reset,
        }
        self._last_observation = message
        self._broadcast(message)

    def wait_for_action(self, episode_id: int, control_tick: int, timeout: float) -> bool:
        key = (episode_id, control_tick)
        with self._ack_condition:
            ready = self._ack_condition.wait_for(
                lambda: key in self._acks or self._stop.is_set(),
                timeout=timeout,
            )
            if not ready or self._stop.is_set():
                return False
            self._acks.remove(key)
            return True

    def wait_for_controller(self) -> bool:
        """Block until a control client attaches or the server closes."""
        with self._clients_condition:
            self._clients_condition.wait_for(lambda: bool(self._clients) or self._stop.is_set())
            return bool(self._clients) and not self._stop.is_set()

    def _accept_loop(self) -> None:
        assert self._socket is not None
        while not self._stop.is_set():
            try:
                client, _ = self._socket.accept()
            except TimeoutError:
                continue
            except OSError:
                break
            with self._clients_condition:
                self._clients.add(client)
                self._clients_condition.notify_all()
                if self._last_observation is not None:
                    try:
                        client.sendall(self._encode(self._last_observation))
                    except OSError:
                        self._clients.discard(client)
                        client.close()
                        continue
            threading.Thread(
                target=self._client_loop,
                args=(client,),
                name="sim2-control-client",
                daemon=True,
            ).start()

    def _client_loop(self, client: socket.socket) -> None:
        try:
            with client.makefile("r", encoding="utf-8") as reader:
                for line in reader:
                    message = json.loads(line)
                    if message.get("op") != "action_ready":
                        continue
                    key = (int(message["episode_id"]), int(message["control_tick"]))
                    with self._ack_condition:
                        self._acks.add(key)
                        self._ack_condition.notify_all()
        except (OSError, ValueError, KeyError, json.JSONDecodeError):
            pass
        finally:
            with self._clients_condition:
                self._clients.discard(client)
                self._clients_condition.notify_all()
            try:
                client.close()
            except OSError:
                pass

    def _broadcast(self, message: dict[str, Any]) -> None:
        payload = self._encode(message)
        stale: list[socket.socket] = []
        with self._clients_lock:
            for client in self._clients:
                try:
                    client.sendall(payload)
                except OSError:
                    stale.append(client)
            for client in stale:
                self._clients.discard(client)
                client.close()

    @staticmethod
    def _encode(message: dict[str, Any]) -> bytes:
        return (json.dumps(message, separators=(",", ":")) + "\n").encode()

    def close(self) -> None:
        self._stop.set()
        with self._ack_condition:
            self._ack_condition.notify_all()
        if self._socket is not None:
            self._socket.close()
        with self._clients_condition:
            clients = list(self._clients)
            self._clients.clear()
            self._clients_condition.notify_all()
        for client in clients:
            client.close()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self.path.unlink(missing_ok=True)


class SimControlClient:
    def __init__(self, path: Path, timeout: float = 30.0) -> None:
        client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        client.settimeout(timeout)
        client.connect(str(path))
        client.settimeout(None)
        self._socket = client
        self._events: queue.Queue[dict[str, Any]] = queue.Queue()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._read_loop, name="sim2-events", daemon=True)
        self._thread.start()

    def events(self) -> Iterator[dict[str, Any]]:
        while not self._stop.is_set():
            event = self._events.get()
            if event.get("event") == "closed":
                return
            yield event

    def next_observation(self, timeout: float | None = None) -> dict[str, Any]:
        event = self._events.get(timeout=timeout)
        if event.get("event") == "closed":
            raise ConnectionError("sim2 control connection closed")
        return event

    def action_ready(self, episode_id: int, control_tick: int) -> None:
        payload = {
            "op": "action_ready",
            "episode_id": episode_id,
            "control_tick": control_tick,
        }
        self._socket.sendall((json.dumps(payload, separators=(",", ":")) + "\n").encode())

    def _read_loop(self) -> None:
        try:
            with self._socket.makefile("r", encoding="utf-8") as reader:
                for line in reader:
                    self._events.put(json.loads(line))
        except (OSError, json.JSONDecodeError):
            pass
        finally:
            self._stop.set()
            self._events.put({"event": "closed"})

    def close(self) -> None:
        self._stop.set()
        try:
            self._socket.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        self._socket.close()
        self._thread.join(timeout=2.0)
