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

"""Unified MuJoCo simulation Module.

Owns a single ``MujocoEngine`` and publishes:
- camera streams (Out ports), replacing ``MujocoCamera``
- joint state via shared memory, consumed by ``ShmMujocoAdapter`` inside
  ``ControlCoordinator``

This avoids the prior pattern of sharing engines via a global in-process
registry, which was fragile when ``WorkerManager`` places the adapter and
the camera in different worker processes.
"""

from __future__ import annotations

import math
from pathlib import Path
import shutil
import subprocess
import sys
import threading
import time
from typing import Any, Literal

import mujoco
import numpy as np
import open3d as o3d  # type: ignore[import-untyped]
from pydantic import Field
import reactivex as rx
from reactivex.disposable import Disposable
from scipy.spatial.transform import Rotation as R

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.simulation.backend.mujoco.engine import (
    CameraConfig,
    CameraFrame,
    MujocoEngine,
    RaycastLidarConfig,
)
from dimos.simulation.backend.mujoco.robot_sim_binding import RobotSimSpec
from dimos.simulation.backend.mujoco.shm import (
    ManipShmWriter,
    shm_key_from_path,
)
from dimos.simulation.legacy.mujoco.constants import (
    LIDAR_RESOLUTION,
    MAX_HEIGHT,
    MAX_RANGE,
    MIN_RANGE,
)
from dimos.simulation.scene.entity import EntityDescriptor, EntityStateBatch
from dimos.spec import perception
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


_RX180 = R.from_euler("x", 180, degrees=True)
_LIDAR_GEOM_GROUPS = (0, 0, 1, 1, 0, 0)
_CMD_VEL_STALE_SEC = 0.5
_ENTITY_STATE_MIN_INTERVAL_SEC = 1.0 / 30.0
_ENGINE_CONNECT_TIMEOUT_SEC = 30.0
_PUBLISH_THREAD_JOIN_TIMEOUT_SEC = 2.0
_ENGINE_CONNECT_POLL_SEC = 0.1
_STALE_FRAME_POLL_FRACTION = 0.5
_RGBD_POINTCLOUD_VOXEL_SIZE = 0.005


def _default_identity_transform() -> Transform:
    return Transform(
        translation=Vector3(0.0, 0.0, 0.0),
        rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
    )


def _imu_from_mujoco_wxyz(
    quaternion: tuple[float, float, float, float],
    gyroscope: tuple[float, float, float],
    accelerometer: tuple[float, float, float],
    *,
    frame_id: str,
    ts: float,
) -> Imu:
    w, x, y, z = quaternion
    return Imu(
        orientation=Quaternion(x, y, z, w),
        angular_velocity=Vector3(*gyroscope),
        linear_acceleration=Vector3(*accelerometer),
        frame_id=frame_id,
        ts=ts,
    )


class MujocoSimModuleConfig(ModuleConfig):
    """Configuration for the unified MuJoCo simulation module.

    Two ways to specify the model:

    * ``address`` (legacy): path to a pre-built MJCF/MJB containing both
      scene and robot. Used by blueprints that pre-cooked a wrapper at
      build time. The address is loaded as-is.
    * ``robot_mjcf`` + ``scene_xml`` (preferred): robot-agnostic scene
      package + a separately-specified robot MJCF. The module composes
      ``MjSpec(scene) + MjSpec(robot) + entities`` at start time, so the
      same scene package works with any robot. ``scene_xml`` may be
      omitted for "robot only on a flat floor" runs.
    """

    address: str | Path = ""
    # New compose-at-start path.
    scene_xml: str | Path | None = None
    robot_mjcf: str | Path | None = None
    robot_meshdir: str | Path | None = None
    robot_id: str = ""
    meshdir: str | None = None
    headless: bool = False
    dof: int = 7

    # Camera config (matches former MujocoCameraConfig).
    camera_name: str = "wrist_camera"
    width: int = 640
    height: int = 480
    fps: int = 15
    base_frame_id: str = "link7"
    base_transform: Transform | None = Field(default_factory=_default_identity_transform)
    align_depth_to_color: bool = True
    enable_color: bool = True
    enable_depth: bool = True
    enable_pointcloud: bool = False
    pointcloud_fps: float = 5.0
    camera_info_fps: float = 1.0
    lidar_camera_names: list[str] = Field(default_factory=list)
    lidar_camera_width: int = 640
    lidar_camera_height: int = 360
    lidar_voxel_size: float = 0.05
    # Optional MuJoCo-native lidar: cast rays from one or more named cameras
    # and publish world-frame PointCloud2 points on ``pointcloud``.
    enable_mujoco_lidar: bool = False
    mujoco_lidar_camera_names: list[str] = Field(default_factory=list)
    mujoco_lidar_voxel_size: float = LIDAR_RESOLUTION
    mujoco_lidar_geom_groups: list[int] = Field(default_factory=lambda: [0, 1, 2, 3, 4, 5])
    mujoco_lidar_raycast_width: int = 64
    mujoco_lidar_raycast_height: int = 32
    mujoco_lidar_min_range: float = MIN_RANGE
    mujoco_lidar_max_range: float = MAX_RANGE
    mujoco_lidar_max_height: float = MAX_HEIGHT
    mujoco_lidar_robot_exclusion_radius: float = 0.0
    renderer_max_geom: int = 0
    enable_kinematic_base_control: bool = False
    enable_kinematic_joint_hold: bool = False
    inject_legacy_assets: bool = False
    support_floor: bool = False
    support_floor_z: float = 0.0
    support_floor_group: int = 2
    support_floor_friction: tuple[float, float, float] = (1.0, 0.05, 0.001)
    spawn_xy: tuple[float, float] | None = None
    spawn_z: float | None = None
    spawn_yaw: float | None = None
    reset_joint_positions: list[float] | None = None
    robot_sim_spec: RobotSimSpec | None = None
    # Scene-package entity metadata (scene.meta.json ``entities`` entries).
    # When the loaded model contains matching ``entity:<id>`` bodies (see
    # dimos.simulation.backend.mujoco.scene_package_entity_composer), their world poses are published
    # as an EntityStateBatch — MuJoCo is the entity authority in this mode.
    scene_entities: list[dict[str, Any]] = Field(default_factory=list)
    imu_gyro_sensor_names: list[str] = Field(
        default_factory=lambda: [
            "imu-pelvis-angular-velocity",
            "imu-torso-angular-velocity",
            "gyro_pelvis",
            "imu_gyro",
        ]
    )
    imu_accel_sensor_names: list[str] = Field(
        default_factory=lambda: [
            "imu-pelvis-linear-acceleration",
            "imu-torso-linear-acceleration",
            "accelerometer_pelvis",
            "imu_accel",
        ]
    )
    imu_linvel_sensor_names: list[str] = Field(
        default_factory=lambda: [
            "body-linear-vel",
            "imu-linear-velocity",
            "imu_linvel",
        ]
    )
    engine_mode: Literal["thread", "subprocess"] = "thread"


class MujocoSimModule(
    Module,
    perception.DepthCamera,
):
    """Single Module that owns a MujocoEngine, publishes camera streams, and
    exposes joint state/commands to a ``ShmMujocoAdapter`` via shared memory.

    The adapter attaches to the same SHM buffers using the MJCF path as the
    discovery key - no RPC, no globals. From ControlCoordinator's perspective
    the adapter is an ordinary ``ManipulatorAdapter``; SHM is its transport.
    """

    config: MujocoSimModuleConfig
    color_image: Out[Image]
    depth_image: Out[Image]
    pointcloud: Out[PointCloud2]
    camera_info: Out[CameraInfo]
    depth_camera_info: Out[CameraInfo]
    imu: Out[Imu]
    odom: Out[PoseStamped]
    # Per-tick snapshot of scene-package entities simulated by MuJoCo
    # (``entity:<id>`` bodies). Same message/topic the browser publishes in
    # browser-physics mode — consumers don't care who the authority is.
    entity_state_batch: Out[EntityStateBatch]
    cmd_vel: In[Twist]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._engine: MujocoEngine | None = None
        self._entity_bodies: list[tuple[EntityDescriptor, int]] = []
        self._last_entity_state_pub = 0.0
        self._shm: ManipShmWriter | None = None
        self._sim_hooks: Any | None = None
        self._engine_proc: subprocess.Popen[Any] | None = None
        self._gripper_idx: int | None = None
        self._gripper_ctrl_range: tuple[float, float] = (0.0, 1.0)
        self._gripper_joint_range: tuple[float, float] = (0.0, 1.0)
        self._stop_event = threading.Event()
        self._publish_thread: threading.Thread | None = None
        self._camera_info_base: CameraInfo | None = None
        self._cmd_vel_lock = threading.Lock()
        self._cmd_vel = Twist.zero()
        self._last_cmd_vel_time = 0.0
        self._kinematic_base_z: float | None = None
        self._shm_ready_signaled = False
        self._imu_quat_slice: slice | None = None
        self._imu_gyro_slice: slice | None = None
        self._imu_accel_slice: slice | None = None
        self._imu_linvel_slice: slice | None = None
        self._imu_base_qpos_slice: slice | None = None
        self._root_base_qpos_adr: int | None = None
        self._root_spawn_clearance_z: float | None = None

    @property
    def _camera_enabled(self) -> bool:
        return self.config.enable_color or self.config.enable_depth or self.config.enable_pointcloud

    @property
    def _primary_camera_needed(self) -> bool:
        return (
            self.config.enable_color
            or self.config.enable_depth
            or (
                self.config.enable_pointcloud
                and not self.config.lidar_camera_names
                and not self.config.enable_mujoco_lidar
            )
        )

    @property
    def _camera_link(self) -> str:
        return f"{self.config.camera_name}_link"

    @property
    def _color_frame(self) -> str:
        return f"{self.config.camera_name}_color_frame"

    @property
    def _color_optical_frame(self) -> str:
        return f"{self.config.camera_name}_color_optical_frame"

    @property
    def _depth_frame(self) -> str:
        return f"{self.config.camera_name}_depth_frame"

    @property
    def _depth_optical_frame(self) -> str:
        return f"{self.config.camera_name}_depth_optical_frame"

    @rpc
    def get_color_camera_info(self) -> CameraInfo | None:
        if self._camera_info_base is None:
            return None
        return self._camera_info_base.with_ts(time.time())

    @rpc
    def get_depth_camera_info(self) -> CameraInfo | None:
        if self._camera_info_base is None:
            return None
        return self._camera_info_base.with_ts(time.time())

    @rpc
    def get_depth_scale(self) -> float:
        return 1.0

    @rpc
    def start(self) -> None:
        super().start()
        if not self.config.address and not self.config.robot_mjcf:
            raise RuntimeError(
                "MujocoSimModule: either config.robot_mjcf (preferred) "
                "or config.address (legacy MJCF path) is required"
            )

        # SHM discovery key — robot_mjcf wins when set, address otherwise.
        shm_key_source = str(self.config.robot_mjcf or self.config.address)
        shm_key = shm_key_from_path(shm_key_source)
        if self.config.engine_mode == "subprocess":
            self._start_subprocess(shm_key)
            return

        self._shm = ManipShmWriter(shm_key)
        self._shm_ready_signaled = False
        camera_configs = self._make_camera_configs()
        engine_assets: dict[str, bytes] | None = None
        if self.config.inject_legacy_assets:
            from dimos.simulation.backend.mujoco.assets import get_assets

            engine_assets = get_assets()

        # MuJoCo-native raycast lidar (main feature): build one RaycastLidarConfig
        # per named camera; rendered cameras stay in _make_camera_configs().
        raycast_lidars: list[RaycastLidarConfig] = []
        if self.config.enable_pointcloud and self.config.enable_mujoco_lidar:
            for camera_name in self._mujoco_lidar_camera_names():
                raycast_lidars.append(
                    RaycastLidarConfig(
                        name=camera_name,
                        width=self.config.mujoco_lidar_raycast_width,
                        height=self.config.mujoco_lidar_raycast_height,
                        fps=self.config.pointcloud_fps,
                        min_range=self.config.mujoco_lidar_min_range,
                        max_range=self.config.mujoco_lidar_max_range,
                        max_height=self.config.mujoco_lidar_max_height,
                        geom_groups=tuple(self.config.mujoco_lidar_geom_groups),
                        robot_exclusion_radius=self.config.mujoco_lidar_robot_exclusion_radius,
                    )
                )

        engine_kwargs: dict[str, Any] = dict(
            headless=self.config.headless,
            cameras=camera_configs,
            raycast_lidars=raycast_lidars or None,
            meshdir=self.config.meshdir,
            on_before_step=self._apply_shm_commands,
            on_after_step=self._after_step,
            assets=engine_assets,
            spawn_xy=self.config.spawn_xy,
            spawn_z=self.config.spawn_z,
            spawn_yaw=self.config.spawn_yaw,
            reset_joint_positions=self.config.reset_joint_positions,
            robot_sim_spec=self.config.robot_sim_spec,
        )
        if self.config.robot_mjcf:
            engine_kwargs["model"] = self._compose_model()
            engine_kwargs["config_path"] = Path(self.config.robot_mjcf)
        else:
            engine_kwargs["config_path"] = Path(self.config.address)
        self._engine = MujocoEngine(**engine_kwargs)

        dof = self.config.dof
        joint_names = list(self._engine.joint_names)
        self._detect_gripper(joint_names)
        self._resolve_imu_slices()
        self._create_sim_hooks(dof)

        if not self._engine.connect():
            raise RuntimeError("MujocoSimModule: engine.connect() failed")

        self._resolve_entity_bodies()
        self._stop_event.clear()

        self._start_kinematic_base_control()
        self._start_camera_publishers()
        self._start_pointcloud_publisher()

        logger.info(
            "MujocoSimModule started",
            address=self.config.address,
            dof=dof,
            camera=self.config.camera_name,
            camera_enabled=self._camera_enabled,
            shm_key=shm_key,
        )

    def _compose_model(self) -> mujoco.MjModel:
        """Compose scene (optional) + entities + robot into one ``MjModel``.

        This is the runtime side of "scene packages are robot-agnostic":
        the cooked scene wrapper has no robot, the robot MJCF has no
        scene, and ``MjSpec.attach`` stitches them together with optional
        body-name prefixing keyed on ``robot_id`` (empty prefix by
        default — single-robot scenes don't need renaming).
        """
        from dimos.simulation.backend.mujoco.scene_package_entity_composer import (
            add_scene_package_entities_to_spec,
        )

        if self.config.scene_xml:
            spec_scene = mujoco.MjSpec.from_file(str(self.config.scene_xml))
        else:
            spec_scene = mujoco.MjSpec()
        if self.config.support_floor:
            spec_scene.worldbody.add_geom(
                name="locomotion_support_floor",
                type=mujoco.mjtGeom.mjGEOM_PLANE,
                pos=[0.0, 0.0, float(self.config.support_floor_z)],
                size=[0.0, 0.0, 0.01],
                group=int(self.config.support_floor_group),
                rgba=[0.0, 0.0, 0.0, 0.0],
                friction=list(self.config.support_floor_friction),
            )
        spec_robot = mujoco.MjSpec.from_file(str(self.config.robot_mjcf))
        if self.config.robot_meshdir:
            spec_robot.meshdir = str(self.config.robot_meshdir)
        # Preserve the robot dynamics contract when attaching into a
        # robot-agnostic scene. Otherwise the scene's default MuJoCo options
        # can silently change the policy/control timing.
        spec_scene.option.timestep = spec_robot.option.timestep

        spawn_xy = self.config.spawn_xy or (0.0, 0.0)
        spawn_z = self.config.spawn_z if self.config.spawn_z is not None else 0.0
        frame = spec_scene.worldbody.add_frame(
            pos=[float(spawn_xy[0]), float(spawn_xy[1]), float(spawn_z)],
        )
        prefix = f"{self.config.robot_id}-" if self.config.robot_id else None
        spec_scene.attach(spec_robot, prefix=prefix, frame=frame)
        if self.config.scene_entities:
            add_scene_package_entities_to_spec(spec_scene, self.config.scene_entities)
        return spec_scene.compile()

    def _resolve_entity_bodies(self) -> None:
        """Map configured scene entities to ``entity:<id>`` bodies in the model."""
        self._entity_bodies = []
        if not self.config.scene_entities or self._engine is None:
            return
        from dimos.simulation.backend.mujoco.scene_package_entity_composer import (
            scene_package_entity_body_name,
        )

        for raw in self.config.scene_entities:
            try:
                descriptor = EntityDescriptor.from_wire(raw["descriptor"])
            except (KeyError, TypeError, ValueError) as exc:
                logger.warning("MujocoSimModule: bad scene entity metadata: %s", exc)
                continue
            body_id = self._engine.body_id(scene_package_entity_body_name(descriptor.entity_id))
            if body_id is None:
                logger.warning(
                    "MujocoSimModule: entity %s not in model (compose_entity_model not used?)",
                    descriptor.entity_id,
                )
                continue
            self._entity_bodies.append((descriptor, body_id))
        if self._entity_bodies:
            logger.info(
                "MujocoSimModule: publishing entity states for %d scene entities",
                len(self._entity_bodies),
            )

    def _make_camera_configs(self) -> list[CameraConfig]:
        camera_configs: list[CameraConfig] = []
        max_geom = self.config.renderer_max_geom or None
        if self._primary_camera_needed:
            camera_configs.append(
                CameraConfig(
                    name=self.config.camera_name,
                    width=self.config.width,
                    height=self.config.height,
                    fps=float(self.config.fps),
                    render_rgb=self.config.enable_color
                    or (self.config.enable_pointcloud and not self.config.lidar_camera_names),
                    render_depth=self.config.enable_depth
                    or (self.config.enable_pointcloud and not self.config.lidar_camera_names),
                    max_geom=max_geom,
                )
            )

        lidar_scene_option = mujoco.MjvOption()
        geomgroup = lidar_scene_option.geomgroup  # type: ignore[attr-defined]
        for group_id, enabled in enumerate(_LIDAR_GEOM_GROUPS):
            geomgroup[group_id] = enabled
        for lidar_name in self.config.lidar_camera_names:
            if lidar_name == self.config.camera_name and self._primary_camera_needed:
                continue
            camera_configs.append(
                CameraConfig(
                    name=lidar_name,
                    width=self.config.lidar_camera_width,
                    height=self.config.lidar_camera_height,
                    fps=float(self.config.pointcloud_fps),
                    render_rgb=False,
                    render_depth=True,
                    scene_option=lidar_scene_option,
                    max_geom=max_geom,
                )
            )
        return camera_configs

    def _detect_gripper(self, joint_names: list[str]) -> None:
        dof = self.config.dof
        if len(joint_names) <= dof:
            return
        assert self._engine is not None
        ctrl_range = self._engine.get_actuator_ctrl_range(dof)
        joint_range = self._engine.get_joint_range(dof)
        if ctrl_range is None or joint_range is None:
            raise ValueError(f"Gripper joint at index {dof} missing ctrl/joint range in MJCF")
        self._gripper_idx = dof
        self._gripper_ctrl_range = ctrl_range
        self._gripper_joint_range = joint_range
        logger.info(
            "MujocoSimModule: gripper detected",
            idx=dof,
            ctrl_range=ctrl_range,
            joint_range=joint_range,
        )

    def _resolve_imu_slices(self) -> None:
        assert self._engine is not None
        binding = self._engine.robot_binding
        if binding is not None:
            self._imu_quat_slice = binding.imu_quat_slice
            self._imu_gyro_slice = binding.imu_gyro_slice
            self._imu_accel_slice = binding.imu_accel_slice
            self._imu_linvel_slice = binding.imu_linvel_slice
            self._imu_base_qpos_slice = (
                slice(binding.root_qpos_adr + 3, binding.root_qpos_adr + 7)
                if binding.root_qpos_adr is not None
                else None
            )
            return

        self._imu_quat_slice = None
        self._imu_gyro_slice = self._engine.find_sensor_slice(
            *self.config.imu_gyro_sensor_names, dim=3
        )
        self._imu_accel_slice = self._engine.find_sensor_slice(
            *self.config.imu_accel_sensor_names, dim=3
        )
        self._imu_linvel_slice = self._engine.find_sensor_slice(
            *self.config.imu_linvel_sensor_names, dim=3
        )
        root_qpos_adr = self._engine.root_qpos_adr
        if root_qpos_adr is not None:
            self._imu_base_qpos_slice = slice(root_qpos_adr + 3, root_qpos_adr + 7)
        else:
            self._imu_base_qpos_slice = None
        self._root_base_qpos_adr = self._engine.root_qpos_adr
        self._root_spawn_clearance_z = self._compute_root_spawn_clearance_z()

    def _create_sim_hooks(self, dof: int) -> None:
        assert self._shm is not None
        from dimos.simulation.backend.mujoco.wholebody_sim_hooks import WholeBodySimHooks

        self._sim_hooks = WholeBodySimHooks(
            self._shm,
            dof=dof,
            gripper_idx=self._gripper_idx,
            gripper_ctrl_range=self._gripper_ctrl_range,
            gripper_joint_range=self._gripper_joint_range,
        )

    def _start_subprocess(self, shm_key: str) -> None:
        if self._camera_enabled:
            raise RuntimeError(
                "MujocoSimModule(engine_mode='subprocess') does not support cameras. "
                "Disable color/depth/pointcloud or use engine_mode='thread'."
            )
        interp = (
            (shutil.which("mjpython") or shutil.which("python"))
            if sys.platform == "darwin"
            else sys.executable
        )
        if interp is None:
            raise RuntimeError(
                "MujocoSimModule(engine_mode='subprocess'): no Python interpreter found"
            )

        cmd = [
            interp,
            "-m",
            "dimos.simulation.backend.mujoco.engine_proc",
            str(self.config.address),
            shm_key,
            str(self.config.dof),
        ]
        if not self.config.headless:
            cmd.append("--view")
        if not self.config.inject_legacy_assets:
            cmd.append("--no-asset-inject")

        self._engine_proc = subprocess.Popen(cmd)
        time.sleep(0.2)
        returncode = self._engine_proc.poll()
        if returncode is not None:
            self._engine_proc = None
            raise RuntimeError(
                "MujocoSimModule engine subprocess exited immediately "
                f"(returncode={returncode}, address={self.config.address})"
            )
        logger.info(
            "MujocoSimModule spawned engine subprocess",
            pid=self._engine_proc.pid,
            interp=interp,
            address=self.config.address,
            shm_key=shm_key,
        )

    def _start_kinematic_base_control(self) -> None:
        if not self.config.enable_kinematic_base_control:
            return
        assert self._engine is not None
        if not self._engine.has_root_freejoint:
            logger.warning("Kinematic base control requested, but MJCF has no freejoint root")
        root_pose = self._engine.get_root_pose()
        self._kinematic_base_z = None if root_pose is None else float(root_pose[0][2])
        self.register_disposable(Disposable(self.cmd_vel.subscribe(self._on_cmd_vel)))

    def _start_camera_publishers(self) -> None:
        if not self._primary_camera_needed:
            return
        self._build_camera_info()

        self._publish_thread = threading.Thread(
            target=self._publish_loop, daemon=True, name="MujocoSimPublish"
        )
        self._publish_thread.start()

        interval_sec = 1.0 / self.config.camera_info_fps
        self.register_disposable(
            rx.interval(interval_sec).subscribe(
                on_next=lambda _: self._publish_camera_info(),
                on_error=lambda e: logger.error("CameraInfo publish error", error=str(e)),
            )
        )

    def _start_pointcloud_publisher(self) -> None:
        if not self.config.enable_pointcloud:
            return
        if not (
            self._primary_camera_needed
            or self.config.lidar_camera_names
            or self.config.enable_mujoco_lidar
        ):
            return
        pc_interval = 1.0 / self.config.pointcloud_fps
        self.register_disposable(
            rx.interval(pc_interval).subscribe(
                on_next=lambda _: self._generate_pointcloud(),
                on_error=lambda e: logger.error("Pointcloud error", error=str(e)),
            )
        )

    def _compose_model(self) -> mujoco.MjModel:
        """Compose optional scene package MJCF + robot MJCF + entities."""
        from dimos.simulation.backend.mujoco.scene_package_entity_composer import (
            add_scene_package_entities_to_spec,
            find_scene_package_entity_spawn_penetrators,
        )

        if self.config.robot_mjcf is None:
            raise RuntimeError("MujocoSimModule: robot_mjcf is required for composition")

        def build_spec(*, force_static: frozenset[str] = frozenset()) -> mujoco.MjSpec:
            if self.config.scene_xml is not None:
                spec_scene = mujoco.MjSpec.from_file(str(self.config.scene_xml))
            else:
                spec_scene = mujoco.MjSpec()

            spec_robot = mujoco.MjSpec.from_file(str(self.config.robot_mjcf))
            if self.config.robot_meshdir is not None:
                spec_robot.meshdir = str(self.config.robot_meshdir)

            # Keep the robot controller timing stable when attached to a scene
            # package whose wrapper may have different default options.
            spec_scene.option.timestep = spec_robot.option.timestep

            spawn_xy = self.config.spawn_xy or (0.0, 0.0)
            spawn_z = self.config.spawn_z if self.config.spawn_z is not None else 0.0
            frame_kwargs: dict[str, Any] = {
                "pos": [float(spawn_xy[0]), float(spawn_xy[1]), float(spawn_z)],
            }
            if self.config.spawn_yaw is not None:
                yaw = float(self.config.spawn_yaw)
                frame_kwargs["quat"] = [math.cos(yaw * 0.5), 0.0, 0.0, math.sin(yaw * 0.5)]
            frame = spec_scene.worldbody.add_frame(
                **frame_kwargs,
            )
            prefix = f"{self.config.robot_id}-" if self.config.robot_id else None
            spec_scene.attach(spec_robot, prefix=prefix, frame=frame)

            if self.config.scene_entities:
                add_scene_package_entities_to_spec(
                    spec_scene, self.config.scene_entities, force_static=force_static
                )
            return spec_scene

        spec_scene = build_spec()
        model = spec_scene.compile()
        if self.config.scene_entities:
            penetrators = find_scene_package_entity_spawn_penetrators(model)
            if penetrators:
                logger.warning(
                    "MujocoSimModule: scene entities spawn in deep contact; welding static",
                    count=len(penetrators),
                    samples=sorted(penetrators)[:20],
                )
                model = build_spec(force_static=penetrators).compile()
        return model

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        if self._publish_thread and self._publish_thread.is_alive():
            self._publish_thread.join(timeout=_PUBLISH_THREAD_JOIN_TIMEOUT_SEC)
        self._publish_thread = None

        errors: list[tuple[str, BaseException]] = []
        if self._engine is not None:
            try:
                self._engine.disconnect()
                self._engine = None
            except Exception as exc:
                logger.error("engine.disconnect() failed", error=str(exc))
                errors.append(("engine.disconnect", exc))
        if self._engine_proc is not None and self._engine_proc.poll() is None:
            try:
                self._engine_proc.terminate()
                self._engine_proc.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                logger.warning(
                    f"engine subprocess pid={self._engine_proc.pid} did not exit; killing"
                )
                self._engine_proc.kill()
            except Exception as exc:
                logger.error("engine subprocess terminate raised", error=str(exc))
                errors.append(("engine_proc.terminate", exc))
            finally:
                self._engine_proc = None
        if self._shm is not None:
            try:
                self._shm.signal_stop()
                self._shm.cleanup()
                self._shm = None
            except Exception as exc:
                logger.error("SHM cleanup failed", error=str(exc))
                errors.append(("shm.cleanup", exc))

        self._sim_hooks = None
        self._camera_info_base = None
        self._sim_hooks = None
        self._shm_ready_signaled = False
        super().stop()

        if errors:
            op, err = errors[0]
            raise RuntimeError(f"MujocoSimModule.stop() failed during {op}: {err}") from err

    @rpc
    def respawn(self) -> bool:
        engine = self._engine
        if engine is None:
            return False
        with self._cmd_vel_lock:
            self._cmd_vel = Twist.zero()
            self._last_cmd_vel_time = 0.0
        if self._sim_hooks is not None:
            self._sim_hooks.clear_latched_commands()
        applied = engine.request_reset(wait=True)
        logger.info("MujocoSimModule: respawn requested", applied=applied)
        return applied

    def reset(self) -> bool:
        """Reset the running simulation to its current configured spawn pose."""
        engine = self._engine
        if engine is None:
            return False
        if self._sim_hooks is not None:
            self._sim_hooks.clear_latched_commands()
        applied = engine.request_reset(wait=True)
        logger.info("MujocoSimModule: reset requested", applied=applied)
        return applied

    def _compute_root_spawn_clearance_z(self) -> float | None:
        engine = self._engine
        qpos_adr = self._root_base_qpos_adr
        if engine is None or qpos_adr is None:
            return None

        qpos = engine.read_qpos(slice(qpos_adr, qpos_adr + 3))
        root_x = float(qpos[0])
        root_y = float(qpos[1])
        root_z = float(qpos[2])
        ground_z = engine.ground_height_at(root_x, root_y)
        if ground_z is not None:
            return root_z - ground_z
        if self.config.spawn_z is not None:
            return root_z - float(self.config.spawn_z)
        return root_z

    @rpc
    def respawn_at(
        self,
        x: float,
        y: float,
        z: float | None = None,
        yaw: float | None = None,
    ) -> bool:
        engine = self._engine
        if engine is None:
            return False
        with self._cmd_vel_lock:
            self._cmd_vel = Twist.zero()
            self._last_cmd_vel_time = 0.0
        if self._sim_hooks is not None:
            self._sim_hooks.clear_latched_commands()
        ground_z = None
        spawn_z = None if z is None else float(z)
        if spawn_z is None:
            ground_z = engine.ground_height_at(float(x), float(y))
            if ground_z is not None and self._root_spawn_clearance_z is not None:
                spawn_z = ground_z + self._root_spawn_clearance_z
            elif ground_z is not None and self.config.spawn_z is not None:
                spawn_z = ground_z + float(self.config.spawn_z)
            else:
                spawn_z = self.config.spawn_z
        applied = engine.request_reset_to(
            spawn_xy=(float(x), float(y)),
            spawn_z=spawn_z,
            spawn_yaw=None if yaw is None else float(yaw),
            wait=True,
        )
        logger.info(
            "MujocoSimModule: respawn_at requested",
            x=x,
            y=y,
            z=z,
            ground_z=ground_z,
            spawn_z=spawn_z,
            yaw=yaw,
            applied=applied,
        )
        return applied

    def _apply_shm_commands(self, engine: MujocoEngine) -> None:
        if self._sim_hooks is not None:
            self._sim_hooks.pre_step(engine)

    def _on_cmd_vel(self, msg: Twist) -> None:
        with self._cmd_vel_lock:
            self._cmd_vel = Twist(msg)
            self._last_cmd_vel_time = time.monotonic()

    def _apply_cmd_vel(self, engine: MujocoEngine) -> None:
        if not self.config.enable_kinematic_base_control:
            return
        with self._cmd_vel_lock:
            cmd = Twist(self._cmd_vel)
            age = time.monotonic() - self._last_cmd_vel_time
        if age > _CMD_VEL_STALE_SEC:
            cmd = Twist.zero()
        engine.apply_root_twist(
            cmd.linear.x,
            cmd.linear.y,
            cmd.angular.z,
            fixed_z=self._kinematic_base_z,
        )

    def _after_step(self, engine: MujocoEngine) -> None:
        self._apply_cmd_vel(engine)
        if self.config.enable_kinematic_joint_hold:
            engine.enforce_position_targets()
        self._publish_state(engine)

    def _publish_state(self, engine: MujocoEngine) -> None:
        shm = self._shm
        if shm is None:
            return
        if self._sim_hooks is not None:
            self._sim_hooks.post_step(engine)

        root_pose = engine.get_root_pose()
        if root_pose is not None:
            position, quat_xyzw = root_pose
            self.odom.publish(
                PoseStamped(
                    ts=time.time(),
                    frame_id="world",
                    position=Vector3(position),
                    orientation=Quaternion(quat_xyzw),
                )
            )
        self._publish_entity_states(engine)
        self._publish_imu(engine)
        if not self._shm_ready_signaled:
            shm.signal_ready(num_joints=len(engine.joint_names))
            self._shm_ready_signaled = True

    def _publish_entity_states(self, engine: MujocoEngine) -> None:
        """Publish the EntityStateBatch snapshot (throttled — display/lidar
        consumers, not control)."""
        if not self._entity_bodies:
            return
        now = time.monotonic()
        if now - self._last_entity_state_pub < _ENTITY_STATE_MIN_INTERVAL_SEC:
            return
        self._last_entity_state_pub = now

        poses = engine.get_body_world_poses([body_id for _, body_id in self._entity_bodies])
        entries: list[tuple[EntityDescriptor, Pose]] = []
        for (descriptor, _), (pos, wxyz) in zip(self._entity_bodies, poses, strict=True):
            pose = Pose()
            pose.position = Vector3(float(pos[0]), float(pos[1]), float(pos[2]))
            pose.orientation = Quaternion(
                float(wxyz[1]), float(wxyz[2]), float(wxyz[3]), float(wxyz[0])
            )
            entries.append((descriptor, pose))
        self.entity_state_batch.publish(EntityStateBatch(entries=entries))

    def _publish_imu(self, engine: MujocoEngine) -> None:
        shm = self._shm
        if shm is None:
            return
        if (
            self._imu_quat_slice is None
            and self._imu_gyro_slice is None
            and self._imu_accel_slice is None
            and self._imu_linvel_slice is None
            and self._imu_base_qpos_slice is None
        ):
            return

        if self._imu_quat_slice is not None:
            q = engine.read_sensor_data(self._imu_quat_slice)
            quat = (float(q[0]), float(q[1]), float(q[2]), float(q[3]))
        elif self._imu_base_qpos_slice is not None:
            q = engine.read_qpos(self._imu_base_qpos_slice)
            quat = (float(q[0]), float(q[1]), float(q[2]), float(q[3]))
        else:
            quat = (1.0, 0.0, 0.0, 0.0)
        if self._imu_gyro_slice is not None:
            g = engine.read_sensor_data(self._imu_gyro_slice)
            gyro = (float(g[0]), float(g[1]), float(g[2]))
        else:
            gyro = (0.0, 0.0, 0.0)
        if self._imu_accel_slice is not None:
            a = engine.read_sensor_data(self._imu_accel_slice)
            accel = (float(a[0]), float(a[1]), float(a[2]))
        else:
            accel = (0.0, 0.0, 0.0)
        if self._imu_linvel_slice is not None:
            v = engine.read_sensor_data(self._imu_linvel_slice)
            linvel = (float(v[0]), float(v[1]), float(v[2]))
        else:
            linvel = (0.0, 0.0, 0.0)

        shm.write_imu(
            quaternion=quat,
            gyroscope=gyro,
            accelerometer=accel,
            linear_velocity=linvel,
        )
        self.imu.publish(
            Imu(
                ts=time.time(),
                frame_id="pelvis",
                orientation=Quaternion(quat[1], quat[2], quat[3], quat[0]),
                angular_velocity=Vector3(gyro[0], gyro[1], gyro[2]),
                linear_acceleration=Vector3(accel[0], accel[1], accel[2]),
            )
        )

    def _build_camera_info(self) -> None:
        if self._engine is None:
            return
        fovy_deg = self._engine.get_camera_fovy(self.config.camera_name)
        if fovy_deg is None:
            logger.error("Camera not found in MJCF", camera_name=self.config.camera_name)
            return
        h = self.config.height
        w = self.config.width
        fovy_rad = math.radians(fovy_deg)
        fy = h / (2.0 * math.tan(fovy_rad / 2.0))
        fx = fy  # square pixels
        self._camera_info_base = CameraInfo.from_intrinsics(
            fx=fx,
            fy=fy,
            cx=w / 2.0,
            cy=h / 2.0,
            width=w,
            height=h,
            frame_id=self._color_optical_frame,
        )

    def _publish_loop(self) -> None:
        """Poll engine for rendered frames and publish at configured FPS."""
        engine = self._engine
        if engine is None:
            return

        interval = 1.0 / self.config.fps
        last_timestamp = 0.0
        published_count = 0

        # Wait for engine to actually be connected (sim thread may take a tick).
        deadline = time.monotonic() + _ENGINE_CONNECT_TIMEOUT_SEC
        while not self._stop_event.is_set() and not engine.connected:
            if time.monotonic() > deadline:
                logger.error("MujocoSimModule: timed out waiting for engine to connect")
                return
            self._stop_event.wait(timeout=_ENGINE_CONNECT_POLL_SEC)

        if self._stop_event.is_set():
            return

        while not self._stop_event.is_set():
            try:
                frame = engine.read_camera(self.config.camera_name)
            except RuntimeError as exc:
                logger.error(
                    "MuJoCo render failed; stopping publish loop",
                    camera_name=self.config.camera_name,
                    error=str(exc),
                    exc_info=True,
                )
                return

            if frame is None or frame.timestamp <= last_timestamp:
                self._stop_event.wait(timeout=interval * _STALE_FRAME_POLL_FRACTION)
                continue
            last_timestamp = frame.timestamp
            ts = time.time()

            if self.config.enable_color and frame.rgb is not None:
                color_img = Image(
                    data=frame.rgb,
                    format=ImageFormat.RGB,
                    frame_id=self._color_optical_frame,
                    ts=ts,
                )
                self.color_image.publish(color_img)

            if self.config.enable_depth and frame.depth is not None:
                depth_img = Image(
                    data=frame.depth,
                    format=ImageFormat.DEPTH,
                    frame_id=self._color_optical_frame,
                    ts=ts,
                )
                self.depth_image.publish(depth_img)

            self._publish_tf(ts, frame)

            published_count += 1
            if published_count == 1:
                logger.info(
                    "MujocoSimModule first frame published",
                    rgb_shape=frame.rgb.shape if frame.rgb is not None else None,
                    depth_shape=frame.depth.shape if frame.depth is not None else None,
                )

            elapsed = time.time() - ts
            sleep_time = interval - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def _publish_camera_info(self) -> None:
        base = self._camera_info_base
        if base is None:
            return
        ts = time.time()
        info = CameraInfo(
            height=base.height,
            width=base.width,
            distortion_model=base.distortion_model,
            D=base.D,
            K=base.K,
            P=base.P,
            frame_id=base.frame_id,
            ts=ts,
        )
        self.camera_info.publish(info)
        self.depth_camera_info.publish(info)

    def _publish_tf(self, ts: float, frame: CameraFrame | None) -> None:
        if frame is None:
            return
        mj_rot = R.from_matrix(frame.cam_mat.reshape(3, 3))
        optical_rot = mj_rot * _RX180
        q = optical_rot.as_quat()  # xyzw
        pos = Vector3(
            float(frame.cam_pos[0]),
            float(frame.cam_pos[1]),
            float(frame.cam_pos[2]),
        )
        rot = Quaternion(float(q[0]), float(q[1]), float(q[2]), float(q[3]))
        self.tf.publish(
            Transform(
                translation=pos,
                rotation=rot,
                frame_id="world",
                child_frame_id=self._color_optical_frame,
                ts=ts,
            ),
            Transform(
                translation=pos,
                rotation=rot,
                frame_id="world",
                child_frame_id=self._depth_optical_frame,
                ts=ts,
            ),
            Transform(
                translation=pos,
                rotation=rot,
                frame_id="world",
                child_frame_id=self._camera_link,
                ts=ts,
            ),
        )

    def _generate_pointcloud(self) -> None:
        if self._engine is None:
            return
        if self.config.enable_mujoco_lidar:
            self._generate_mujoco_lidar_pointcloud()
            return
        if self.config.lidar_camera_names:
            self._generate_lidar_pointcloud()
            return
        # Back-project the primary camera's depth image.
        if self._camera_info_base is None:
            return
        frame = self._engine.read_camera(self.config.camera_name)
        if frame is None or frame.rgb is None or frame.depth is None:
            return
        try:
            color_img = Image(
                data=frame.rgb,
                format=ImageFormat.RGB,
                frame_id=self._color_optical_frame,
                ts=frame.timestamp,
            )
            depth_img = Image(
                data=frame.depth,
                format=ImageFormat.DEPTH,
                frame_id=self._color_optical_frame,
                ts=frame.timestamp,
            )
            pcd = PointCloud2.from_rgbd(
                color_image=color_img,
                depth_image=depth_img,
                camera_info=self._camera_info_base,
                depth_scale=1.0,
            )
            pcd = pcd.voxel_downsample(_RGBD_POINTCLOUD_VOXEL_SIZE)
            self.pointcloud.publish(pcd)
        except Exception as exc:
            logger.error("Pointcloud generation error", error=str(exc))

    def _generate_lidar_pointcloud(self) -> None:
        if self._engine is None:
            return
        try:
            from dimos.simulation.backend.mujoco.depth_camera import depth_image_to_point_cloud

            all_points: list[np.ndarray] = []
            latest_ts = 0.0
            for camera_name in self.config.lidar_camera_names:
                frame = self._engine.read_camera(camera_name)
                if frame is None or frame.depth is None:
                    continue
                points = depth_image_to_point_cloud(
                    frame.depth,
                    frame.cam_pos,
                    frame.cam_mat.reshape(3, 3),
                    fov_degrees=frame.fovy,
                )
                if points.size:
                    all_points.append(points)
                latest_ts = max(latest_ts, frame.timestamp)
            if not all_points:
                return
            cloud = o3d.geometry.PointCloud()
            cloud.points = o3d.utility.Vector3dVector(np.vstack(all_points))
            cloud = cloud.voxel_down_sample(self.config.lidar_voxel_size)
            self.pointcloud.publish(
                PointCloud2(pointcloud=cloud, ts=latest_ts or time.time(), frame_id="world")
            )
        except Exception as exc:
            logger.error("Multi-camera lidar fusion error", error=str(exc))

    def _mujoco_lidar_camera_names(self) -> list[str]:
        names = self.config.mujoco_lidar_camera_names
        return list(names) if names else [self.config.camera_name]

    def _generate_mujoco_lidar_pointcloud(self) -> None:
        engine = self._engine
        if engine is None:
            return

        all_points: list[NDArray[Any]] = []
        latest_ts = 0.0
        for camera_name in self._mujoco_lidar_camera_names():
            frame = engine.read_raycast_lidar(camera_name)
            if frame is None:
                continue
            if frame.points.size > 0:
                all_points.append(frame.points)
                latest_ts = max(latest_ts, frame.timestamp)

        if not all_points:
            return

        try:
            pcd = PointCloud2.from_numpy(
                np.vstack(all_points),
                frame_id="world",
                timestamp=latest_ts or time.time(),
            )
            pcd = pcd.voxel_downsample(self.config.mujoco_lidar_voxel_size)
            self.pointcloud.publish(pcd)
        except Exception as exc:
            logger.error("MuJoCo lidar pointcloud generation error", error=str(exc))
