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

"""Optional external clock contract for ControlCoordinator."""

from __future__ import annotations

from dataclasses import dataclass
import threading
from typing import Protocol


@dataclass(frozen=True)
class ControlTick:
    t_now: float
    dt: float
    episode_id: int
    tick: int
    reset: bool = False


class TickSource(Protocol):
    def start(self) -> None: ...

    def wait_next(self, stop_event: threading.Event) -> ControlTick | None: ...

    def complete(self, tick: ControlTick) -> None: ...

    def stop(self) -> None: ...
