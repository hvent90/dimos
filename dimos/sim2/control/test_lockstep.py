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

from pathlib import Path
import threading

from dimos.control.components import HardwareComponent, HardwareType, make_twist_base_joints
from dimos.control.hardware_interface import ConnectedTwistBase
from dimos.control.task import (
    BaseControlTask,
    ControlMode,
    CoordinatorState,
    JointCommandOutput,
    ResourceClaim,
)
from dimos.control.tick_loop import TickLoop
from dimos.sim2.backend.kinematic import KinematicBackend
from dimos.sim2.control.adapters.twist_base.adapter import SimTwistBaseAdapter
from dimos.sim2.control.tick_source import SimTickSource
from dimos.sim2.ipc.registry import SimRegistry
from dimos.sim2.runtime import SimRuntime
from dimos.sim2.spec import (
    ControlInterface,
    ExecutionConfig,
    ExecutionMode,
    SimConfig,
    SimRobotSpec,
    WorldStateFrame,
)


class ConstantVelocityTask(BaseControlTask):
    def __init__(self, joints: list[str]) -> None:
        self.joints = joints
        self.times: list[float] = []

    @property
    def name(self) -> str:
        return "constant_velocity"

    def claim(self) -> ResourceClaim:
        return ResourceClaim(frozenset(self.joints), mode=ControlMode.VELOCITY)

    def is_active(self) -> bool:
        return True

    def compute(self, state: CoordinatorState) -> JointCommandOutput:
        self.times.append(state.t_now)
        return JointCommandOutput(
            joint_names=self.joints,
            velocities=[1.0, 0.0, 0.0],
            mode=ControlMode.VELOCITY,
        )

    def on_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        del by_task, joints


def test_lockstep_runtime_and_tick_loop_advance_as_one_transaction(tmp_path: Path) -> None:
    registry = SimRegistry(run_id="coordinator-lockstep", root=tmp_path)
    reached = threading.Event()
    latest: list[WorldStateFrame] = []

    def on_world_state(frame: WorldStateFrame) -> None:
        latest[:] = [frame]
        if frame.control_tick >= 3:
            reached.set()

    runtime = SimRuntime(
        SimConfig(
            sim_id="main",
            backend=KinematicBackend(),
            robots=(SimRobotSpec("base", ControlInterface.TWIST_BASE, 3),),
            execution=ExecutionConfig(
                mode=ExecutionMode.LOCKSTEP,
                physics_dt=0.01,
                control_decimation=2,
                autostart=False,
                action_timeout=1.0,
            ),
        ),
        registry=registry,
        world_state_callback=on_world_state,
    )
    runtime.start()
    adapter = SimTwistBaseAdapter(dof=3, hardware_id="base", registry=registry)
    joints = make_twist_base_joints("base")
    component = HardwareComponent(
        hardware_id="base",
        hardware_type=HardwareType.BASE,
        joints=joints,
        adapter_type="sim",
    )
    assert adapter.connect()
    connected = ConnectedTwistBase(adapter=adapter, component=component)
    task = ConstantVelocityTask(joints)
    loop = TickLoop(
        tick_rate=50.0,
        hardware={"base": connected},
        hardware_lock=threading.Lock(),
        tasks={task.name: task},
        task_lock=threading.Lock(),
        joint_to_hardware={joint: "base" for joint in joints},
        tick_source=SimTickSource("main", registry=registry),
    )
    loop.start()
    runtime.run()
    try:
        assert reached.wait(timeout=2.0)
        runtime.pause()

        assert latest[0].entities[0].position[0] >= 0.06
        assert task.times[:3] == [0.0, 0.02, 0.04]
    finally:
        loop.stop()
        adapter.disconnect()
        runtime.close()
