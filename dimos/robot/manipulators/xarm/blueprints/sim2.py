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

"""xArm7 MuJoCo stack using the sim2 runtime and generic adapter."""

from dimos.control.coordinator import ControlCoordinator
from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.manipulators.common.blueprints import trajectory_task
from dimos.robot.manipulators.xarm.sim2_profile import (
    xarm7_sim2_robot,
    xarm7_sim2_world,
)
from dimos.sim2.backend.mujoco import MujocoBackend
from dimos.sim2.control.profile import sim_hardware
from dimos.sim2.module import SimModule
from dimos.sim2.spec import ExecutionConfig, SimConfig

_SIM_ID = "main"
_XARM7 = xarm7_sim2_robot()
_XARM7_HARDWARE = sim_hardware(_XARM7, sim_id=_SIM_ID, gripper=True)

xarm7_sim2 = autoconnect(
    SimModule.blueprint(
        sim=SimConfig(
            sim_id=_SIM_ID,
            backend=MujocoBackend(),
            robots=(_XARM7,),
            world=xarm7_sim2_world(),
            execution=ExecutionConfig(physics_dt=0.002, control_decimation=5),
        )
    ),
    ControlCoordinator.blueprint(
        tick_rate=100.0,
        hardware=[_XARM7_HARDWARE],
        tasks=[trajectory_task(_XARM7_HARDWARE)],
    ),
).global_config(simulation="mujoco", robot_model="xarm7")
