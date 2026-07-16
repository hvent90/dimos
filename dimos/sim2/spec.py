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

"""Backend-neutral contracts for the sim2 runtime."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
import json
from pathlib import Path
from typing import Any, ClassVar, Literal, Protocol, runtime_checkable

from dimos.simulation.scene_assets.spec import ScenePackage


class ControlInterface(str, Enum):
    TWIST_BASE = "twist_base"
    MANIPULATOR = "manipulator"
    WHOLE_BODY = "whole_body"


class ExecutionMode(str, Enum):
    LIVE = "live"
    LOCKSTEP = "lockstep"


class SensorImplementation(str, Enum):
    NATIVE = "native"
    PORTABLE = "portable"


EntityKind = Literal["dynamic", "kinematic", "static"]
ShapeHint = Literal["mesh", "box", "sphere", "cylinder"]


@runtime_checkable
class SceneControl(Protocol):
    """Backend-neutral scenario verbs used by integration tests."""

    def set_agent_position(self, x: float, y: float, z: float = 0.0) -> None: ...

    def add_wall(self, x1: float, y1: float, x2: float, y2: float) -> None: ...

    def publish_goal(self, x: float, y: float) -> None: ...


@dataclass(frozen=True)
class ExecutionConfig:
    mode: ExecutionMode = ExecutionMode.LIVE
    physics_dt: float = 0.002
    control_decimation: int = 10
    realtime_factor: float = 1.0
    autostart: bool = True
    action_timeout: float = 5.0

    def __post_init__(self) -> None:
        if self.physics_dt <= 0.0:
            raise ValueError("physics_dt must be positive")
        if self.control_decimation < 1:
            raise ValueError("control_decimation must be at least 1")
        if self.realtime_factor <= 0.0:
            raise ValueError("realtime_factor must be positive")
        if self.action_timeout <= 0.0:
            raise ValueError("action_timeout must be positive")

    @property
    def control_dt(self) -> float:
        return self.physics_dt * self.control_decimation


@dataclass(frozen=True)
class SpawnPose:
    position: tuple[float, float, float] = (0.0, 0.0, 0.0)
    quaternion_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)


@dataclass(frozen=True)
class RaycastLidarSpec:
    sensor_id: str = "lidar"
    frame_id: str = "world"
    implementation: SensorImplementation = SensorImplementation.NATIVE
    camera_names: tuple[str, ...] = ()
    width: int = 64
    height: int = 32
    rate_hz: float = 1.0
    min_range: float = 0.2
    max_range: float = 3.0
    max_height: float = 1.2
    geom_groups: tuple[int, ...] = ()
    robot_exclusion_radius: float = 0.0
    voxel_size: float = 0.005

    def __post_init__(self) -> None:
        if not self.sensor_id or not self.frame_id:
            raise ValueError("sensor_id and frame_id must not be empty")
        if self.width < 1 or self.height < 1 or self.rate_hz <= 0.0:
            raise ValueError("lidar dimensions and rate must be positive")
        if self.min_range < 0.0 or self.max_range <= self.min_range:
            raise ValueError("lidar range must satisfy 0 <= min_range < max_range")
        if self.robot_exclusion_radius < 0.0:
            raise ValueError("robot_exclusion_radius must not be negative")
        if self.voxel_size <= 0.0:
            raise ValueError("voxel_size must be positive")


@dataclass(frozen=True)
class SimRobotSpec:
    robot_id: str
    control_interface: ControlInterface
    dof: int
    joint_names: tuple[str, ...] = ()
    model_path: Path | None = None
    spawn: SpawnPose = field(default_factory=SpawnPose)
    capabilities: frozenset[str] = field(default_factory=frozenset)
    sensors: tuple[RaycastLidarSpec, ...] = ()
    backend_options: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.robot_id:
            raise ValueError("robot_id must not be empty")
        if self.dof < 1:
            raise ValueError("dof must be at least 1")
        if self.joint_names and len(self.joint_names) != self.dof:
            raise ValueError(
                f"robot '{self.robot_id}' has dof={self.dof} but "
                f"{len(self.joint_names)} joint names"
            )
        sensor_ids = [sensor.sensor_id for sensor in self.sensors]
        if len(sensor_ids) != len(set(sensor_ids)):
            raise ValueError(f"robot '{self.robot_id}' sensor IDs must be unique")


@dataclass(frozen=True)
class EntityDescriptor:
    entity_id: str
    kind: EntityKind = "kinematic"
    backend_name: str | None = None
    mesh_ref: str = ""
    shape_hint: ShapeHint = "mesh"
    extents: tuple[float, ...] = ()
    mass: float = 0.0
    rgba: tuple[float, float, float, float] | None = None

    def __post_init__(self) -> None:
        if not self.entity_id:
            raise ValueError("entity_id must not be empty")
        if self.mass < 0.0:
            raise ValueError("entity mass must not be negative")

    def to_wire(self) -> dict[str, Any]:
        value: dict[str, Any] = {
            "entity_id": self.entity_id,
            "kind": self.kind,
            "backend_name": self.backend_name,
            "mesh_ref": self.mesh_ref,
            "shape_hint": self.shape_hint,
            "extents": list(self.extents),
            "mass": self.mass,
        }
        if self.rgba is not None:
            value["rgba"] = list(self.rgba)
        return value

    @classmethod
    def from_wire(cls, value: dict[str, Any]) -> EntityDescriptor:
        rgba = value.get("rgba")
        return cls(
            entity_id=str(value["entity_id"]),
            kind=value.get("kind", "kinematic"),
            backend_name=(
                str(value["backend_name"]) if value.get("backend_name") is not None else None
            ),
            mesh_ref=str(value.get("mesh_ref", "")),
            shape_hint=value.get("shape_hint", "mesh"),
            extents=tuple(float(item) for item in value.get("extents", ())),
            mass=float(value.get("mass", 0.0)),
            rgba=tuple(float(item) for item in rgba) if rgba is not None else None,  # type: ignore[arg-type]
        )


@dataclass(frozen=True)
class WorldSpec:
    scene: ScenePackage | None = None
    revision: str = "empty"
    entities: tuple[EntityDescriptor, ...] = ()

    def __post_init__(self) -> None:
        if not self.revision:
            raise ValueError("world revision must not be empty")
        ids = [entity.entity_id for entity in self.entities]
        if len(ids) != len(set(ids)):
            raise ValueError("world entity IDs must be unique")


@dataclass(frozen=True)
class SimConfig:
    sim_id: str
    backend: Any
    robots: tuple[SimRobotSpec, ...]
    primary_robot_id: str | None = None
    world: WorldSpec = field(default_factory=WorldSpec)
    execution: ExecutionConfig = field(default_factory=ExecutionConfig)

    def __post_init__(self) -> None:
        if not self.sim_id:
            raise ValueError("sim_id must not be empty")
        robot_ids = [robot.robot_id for robot in self.robots]
        if not robot_ids:
            raise ValueError("at least one robot is required")
        if len(set(robot_ids)) != len(robot_ids):
            raise ValueError("robot IDs must be unique within a simulation")
        if self.primary_robot_id is not None and self.primary_robot_id not in robot_ids:
            raise ValueError("primary_robot_id must name a configured robot")

    @property
    def primary_robot(self) -> str:
        return self.primary_robot_id or self.robots[0].robot_id


@dataclass(frozen=True)
class EntityState:
    entity_id: str
    position: tuple[float, float, float]
    quaternion_xyzw: tuple[float, float, float, float]
    frame_id: str = "world"
    linear_velocity: tuple[float, float, float] = (0.0, 0.0, 0.0)
    angular_velocity: tuple[float, float, float] = (0.0, 0.0, 0.0)

    def to_wire(self) -> dict[str, Any]:
        return {
            "entity_id": self.entity_id,
            "frame_id": self.frame_id,
            "position": list(self.position),
            "quaternion_xyzw": list(self.quaternion_xyzw),
            "linear_velocity": list(self.linear_velocity),
            "angular_velocity": list(self.angular_velocity),
        }

    @classmethod
    def from_wire(cls, value: dict[str, Any]) -> EntityState:
        return cls(
            entity_id=str(value["entity_id"]),
            frame_id=str(value.get("frame_id", "world")),
            position=tuple(float(item) for item in value["position"]),  # type: ignore[arg-type]
            quaternion_xyzw=tuple(  # type: ignore[arg-type]
                float(item) for item in value["quaternion_xyzw"]
            ),
            linear_velocity=tuple(  # type: ignore[arg-type]
                float(item) for item in value.get("linear_velocity", (0.0, 0.0, 0.0))
            ),
            angular_velocity=tuple(  # type: ignore[arg-type]
                float(item) for item in value.get("angular_velocity", (0.0, 0.0, 0.0))
            ),
        )


@dataclass(frozen=True)
class WorldStateFrame:
    msg_name: ClassVar[str] = "sim2.WorldStateFrame"

    episode_id: int
    physics_tick: int
    control_tick: int
    sim_time: float
    scene_revision: str
    entities: tuple[EntityState, ...]

    def lcm_encode(self) -> bytes:
        return json.dumps(
            {
                "version": 2,
                "episode_id": self.episode_id,
                "physics_tick": self.physics_tick,
                "control_tick": self.control_tick,
                "sim_time": self.sim_time,
                "scene_revision": self.scene_revision,
                "entities": [entity.to_wire() for entity in self.entities],
            },
            separators=(",", ":"),
        ).encode("utf-8")

    @classmethod
    def lcm_decode(cls, data: bytes, **_: Any) -> WorldStateFrame:
        value = json.loads(data)
        if value.get("version") not in (1, 2):
            raise ValueError(f"unsupported WorldStateFrame version: {value.get('version')}")
        return cls(
            episode_id=int(value["episode_id"]),
            physics_tick=int(value["physics_tick"]),
            control_tick=int(value["control_tick"]),
            sim_time=float(value["sim_time"]),
            scene_revision=str(value["scene_revision"]),
            entities=tuple(EntityState.from_wire(entity) for entity in value["entities"]),
        )


@dataclass(frozen=True)
class WorldManifest:
    msg_name: ClassVar[str] = "sim2.WorldManifest"

    scene_revision: str
    frame_id: str
    entities: tuple[EntityDescriptor, ...]

    def lcm_encode(self) -> bytes:
        return json.dumps(
            {
                "version": 1,
                "scene_revision": self.scene_revision,
                "frame_id": self.frame_id,
                "entities": [entity.to_wire() for entity in self.entities],
            },
            separators=(",", ":"),
        ).encode("utf-8")

    @classmethod
    def lcm_decode(cls, data: bytes, **_: Any) -> WorldManifest:
        value = json.loads(data)
        if value.get("version") != 1:
            raise ValueError(f"unsupported WorldManifest version: {value.get('version')}")
        return cls(
            scene_revision=str(value["scene_revision"]),
            frame_id=str(value.get("frame_id", "world")),
            entities=tuple(EntityDescriptor.from_wire(entity) for entity in value["entities"]),
        )


def scene_entity_descriptors(scene: ScenePackage) -> tuple[EntityDescriptor, ...]:
    descriptors: list[EntityDescriptor] = []
    for entity in scene.entities:
        raw = entity.get("descriptor")
        if not isinstance(raw, dict):
            continue
        descriptor = dict(raw)
        entity_id = str(descriptor.get("entity_id") or entity.get("id") or "")
        if not entity_id:
            continue
        mesh_ref = str(descriptor.get("mesh_ref") or entity.get("visual_path") or "")
        if mesh_ref and not Path(mesh_ref).is_absolute():
            mesh_ref = str((scene.package_dir / mesh_ref).resolve())
        descriptor["entity_id"] = entity_id
        descriptor["mesh_ref"] = mesh_ref
        descriptor["backend_name"] = f"entity:{entity_id}"
        descriptors.append(EntityDescriptor.from_wire(descriptor))
    return tuple(descriptors)


@dataclass(frozen=True)
class SensorReady:
    msg_name: ClassVar[str] = "sim2.SensorReady"

    sensor_id: str
    episode_id: int
    source_tick: int
    sim_time: float
    sequence: int

    def lcm_encode(self) -> bytes:
        return json.dumps(
            {
                "version": 1,
                "sensor_id": self.sensor_id,
                "episode_id": self.episode_id,
                "source_tick": self.source_tick,
                "sim_time": self.sim_time,
                "sequence": self.sequence,
            },
            separators=(",", ":"),
        ).encode("utf-8")

    @classmethod
    def lcm_decode(cls, data: bytes, **_: Any) -> SensorReady:
        value = json.loads(data)
        if value.get("version") != 1:
            raise ValueError(f"unsupported SensorReady version: {value.get('version')}")
        return cls(
            sensor_id=str(value["sensor_id"]),
            episode_id=int(value["episode_id"]),
            source_tick=int(value["source_tick"]),
            sim_time=float(value["sim_time"]),
            sequence=int(value["sequence"]),
        )
