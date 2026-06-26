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

"""Config and resolved plan models for benchmark runtime demos."""

from __future__ import annotations

from pathlib import Path

from dimos_runtime_protocol import RuntimeDescription, check_compatible
from pydantic import BaseModel, ConfigDict, PositiveInt


class StrictModel(BaseModel):
    model_config = ConfigDict(extra="forbid")


class BenchmarkEpisodeConfig(StrictModel):
    """Backend-facing declaration of one benchmark episode intent."""

    backend: str = "fake"
    episode_id: str = "fake-smoke"
    task_id: str = "fake-motor-smoke"
    runtime_host: str = "127.0.0.1"
    runtime_port: PositiveInt = 8765
    robot_id: str = "fakebot"
    dof: PositiveInt = 3
    control_step_hz: PositiveInt = 100
    ticks: PositiveInt = 30
    target_position: float = 0.2
    seed: int | None = None
    artifact_dir: Path = Path("artifacts/benchmark/fake-runtime-smoke")
    env_name: str = "Lift"
    robot_model: str = "Panda"
    controller: str = "JOINT_POSITION"
    horizon: PositiveInt = 200
    camera_name: str = "agentview"
    visualize: bool = False


class ResolvedRuntimePlan(StrictModel):
    """Concrete launch material derived from a benchmark episode config."""

    episode_id: str
    task_id: str
    backend: str
    runtime_base_url: str
    shm_key: str
    robot_id: str
    motor_names: list[str]
    control_step_hz: PositiveInt
    ticks: PositiveInt
    target_position: float
    artifact_dir: Path


def resolve_runtime_plan(
    config: BenchmarkEpisodeConfig,
    description: RuntimeDescription,
) -> ResolvedRuntimePlan:
    """Validate sidecar metadata and derive a concrete runtime plan."""

    compatibility = check_compatible(description.protocol)
    if not compatibility.compatible:
        raise ValueError(f"incompatible sidecar protocol: {compatibility.reason}")
    if description.backend != config.backend:
        raise ValueError(f"backend mismatch: config={config.backend} sidecar={description.backend}")
    matching_surfaces = [
        surface for surface in description.robot_surfaces if surface.robot_id == config.robot_id
    ]
    if not matching_surfaces:
        raise ValueError(f"sidecar did not report robot surface {config.robot_id!r}")
    surface = matching_surfaces[0]
    motor_names = [motor.name for motor in sorted(surface.motors, key=lambda motor: motor.index)]
    if len(motor_names) != config.dof:
        raise ValueError(f"expected {config.dof} motors, sidecar reported {len(motor_names)}")
    return ResolvedRuntimePlan(
        episode_id=config.episode_id,
        task_id=config.task_id,
        backend=config.backend,
        runtime_base_url=f"http://{config.runtime_host}:{config.runtime_port}",
        shm_key=f"{config.episode_id}-{config.robot_id}",
        robot_id=config.robot_id,
        motor_names=motor_names,
        control_step_hz=config.control_step_hz,
        ticks=config.ticks,
        target_position=config.target_position,
        artifact_dir=config.artifact_dir,
    )
