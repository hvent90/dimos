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
from dimos.control.task import BaseControlTask, CoordinatorState, ResourceClaim
from dimos.control.tick_loop import TickLoop


class QueueTickSource:
    def __init__(self) -> None:
        self.ticks: queue.Queue[ControlTick | None] = queue.Queue()
        self.completed = threading.Event()
        self.last_completed: ControlTick | None = None

    def start(self) -> None:
        pass

    def wait_next(self, stop_event: threading.Event) -> ControlTick | None:
        del stop_event
        return self.ticks.get()

    def complete(self, tick: ControlTick) -> None:
        self.last_completed = tick
        self.completed.set()

    def stop(self) -> None:
        self.ticks.put(None)


class RecordingTask(BaseControlTask):
    def __init__(self) -> None:
        self.states: list[CoordinatorState] = []
        self.reset_count = 0

    @property
    def name(self) -> str:
        return "recording"

    def claim(self) -> ResourceClaim:
        return ResourceClaim(joints=frozenset())

    def is_active(self) -> bool:
        return True

    def compute(self, state: CoordinatorState):
        self.states.append(state)
        return None

    def on_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        del by_task, joints

    def reset_runtime_state(self, reactivate: bool | None = None) -> bool:
        del reactivate
        self.reset_count += 1
        return True


def test_external_tick_source_drives_simulation_time_and_reset() -> None:
    source = QueueTickSource()
    task = RecordingTask()
    lock = threading.Lock()
    loop = TickLoop(
        tick_rate=100.0,
        hardware={},
        hardware_lock=lock,
        tasks={task.name: task},
        task_lock=lock,
        joint_to_hardware={},
        tick_source=source,
    )
    loop.start()
    try:
        tick = ControlTick(t_now=1.25, dt=0.02, episode_id=2, tick=4, reset=True)
        source.ticks.put(tick)

        assert source.completed.wait(timeout=1.0)
        assert source.last_completed == tick
        assert len(task.states) == 1
        assert task.states[0].t_now == 1.25
        assert task.states[0].dt == 0.02
        assert task.states[0].joints.timestamp == 1.25
        assert task.reset_count == 1
    finally:
        loop.stop()
