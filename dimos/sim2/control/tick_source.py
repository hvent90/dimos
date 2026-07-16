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

from __future__ import annotations

import queue
import threading

from dimos.control.clock import ControlTick
from dimos.sim2.ipc.control import SimControlClient
from dimos.sim2.ipc.registry import SimRegistry


class SimTickSource:
    def __init__(self, sim_id: str, registry: SimRegistry | None = None) -> None:
        self.sim_id = sim_id
        self.registry = registry or SimRegistry()
        self._client: SimControlClient | None = None

    def start(self) -> None:
        self._client = SimControlClient(self.registry.resolve_socket(self.sim_id))

    def wait_next(self, stop_event: threading.Event) -> ControlTick | None:
        if self._client is None:
            raise RuntimeError("SimTickSource is not started")
        while not stop_event.is_set():
            try:
                event = self._client.next_observation(timeout=0.2)
            except queue.Empty:
                continue
            except ConnectionError:
                if stop_event.is_set():
                    return None
                raise
            return ControlTick(
                t_now=float(event["sim_time"]),
                dt=float(event["control_dt"]),
                episode_id=int(event["episode_id"]),
                tick=int(event["control_tick"]),
                reset=bool(event.get("reset", False)),
            )
        return None

    def complete(self, tick: ControlTick) -> None:
        if self._client is None:
            raise RuntimeError("SimTickSource is not started")
        self._client.action_ready(tick.episode_id, tick.tick)

    def stop(self) -> None:
        if self._client is not None:
            self._client.close()
        self._client = None
