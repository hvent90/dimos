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

"""Small episode-oriented facade over SimModule's test RPC surface."""

from __future__ import annotations

from typing import Protocol

from dimos.sim2.spec import WorldStateFrame


class SimControl(Protocol):
    def reset(self, seed: int | None = None) -> WorldStateFrame: ...

    def step(
        self,
        control_ticks: int = 1,
        await_sensors: list[str] | None = None,
        sensor_timeout: float = 5.0,
    ) -> WorldStateFrame: ...

    def pause(self) -> None: ...

    def run(self) -> None: ...


class SimTestClient:
    def __init__(self, control: SimControl) -> None:
        self._control = control

    def reset(self, seed: int | None = None) -> WorldStateFrame:
        return self._control.reset(seed)

    def step(
        self,
        control_ticks: int = 1,
        *,
        await_sensors: tuple[str, ...] = (),
        sensor_timeout: float = 5.0,
    ) -> WorldStateFrame:
        return self._control.step(
            control_ticks,
            list(await_sensors),
            sensor_timeout,
        )

    def run(self) -> None:
        self._control.run()

    def pause(self) -> None:
        self._control.pause()
