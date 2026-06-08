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

"""AgiBot X2 MuJoCo policy sim with Babylon visualization."""

from __future__ import annotations

from functools import lru_cache
import os
from pathlib import Path
from typing import Any
import xml.etree.ElementTree as ET

from dimos.control.components import HardwareComponent, HardwareType
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.core.transport import LCMTransport
from dimos.hardware.whole_body.spec import WholeBodyConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.agibot.x2_ultra.policy_constants import (
    X2_DEFAULT_POSITIONS,
    X2_JOINTS,
    X2_KD,
    X2_KP,
)
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule
from dimos.utils.data import LfsPath
from dimos.visualization.babylon_scene_viewer import BabylonSceneViewerModule

_X2_ROOT = Path(__file__).resolve().parent.parent.parent
_X2_ROBOT_MJCF_PATH = _X2_ROOT / "x2_ultra.xml"
_X2_MESH_DIR = _X2_ROOT / "meshes"
_DEFAULT_POLICY_ONNX = LfsPath("mujoco_sim/agibot_x2_policy.onnx")

_X2_SIM_TICK_RATE_HZ = 250.0
_CMD_VEL_TOPIC = "/x2/cmd_vel"
_JOINT_STATE_TOPIC = "/x2/coordinator/joint_state"
_ODOM_TOPIC = "/x2/odom"


def _env_float(name: str, default: float) -> float:
    raw = os.environ.get(name)
    return default if raw is None or raw == "" else float(raw)


def _env_int(name: str, default: int) -> int:
    raw = os.environ.get(name)
    return default if raw is None or raw == "" else int(raw)


_X2_SPAWN_Z_M = _env_float("DIMOS_X2_SPAWN_Z", 0.68)


def _policy_onnx_path() -> Path:
    override = os.environ.get("DIMOS_X2_POLICY_ONNX")
    return Path(override).expanduser() if override else _DEFAULT_POLICY_ONNX


@lru_cache(maxsize=1)
def _scene_package_config() -> Any | None:
    scene = os.environ.get("DIMOS_SCENE_PACKAGE_PATH") or global_config.scene

    from dimos.simulation.scenes.catalog import resolve_scene_package

    return resolve_scene_package(scene)


@lru_cache(maxsize=1)
def _x2_mujoco_scene_xml() -> Path | None:
    """Path to the scene-only MuJoCo wrapper, or None if no scene is set.

    The robot is attached at runtime via ``MjSpec.attach()`` inside
    ``MujocoSimModule.start``; this only needs the scene wrapper.
    """
    scene_package = _scene_package_config()
    if scene_package is None or scene_package.mujoco_scene_path is None:
        return None
    return Path(scene_package.mujoco_scene_path)


def _flat_world_mjcf() -> Path:
    """Generate a cached X2 MJCF with a ground plane for MuJoCo policy sim."""
    cache_dir = Path(os.environ.get("XDG_CACHE_HOME", Path.home() / ".cache")) / "dimos" / "x2"
    cache_dir.mkdir(parents=True, exist_ok=True)
    out_path = cache_dir / "x2_ultra_flat.xml"

    root = ET.parse(_X2_ROBOT_MJCF_PATH).getroot()
    compiler = root.find("compiler")
    if compiler is None:
        compiler = ET.Element("compiler")
        root.insert(0, compiler)
    compiler.set("meshdir", str(_X2_MESH_DIR.resolve()))

    worldbody = root.find("worldbody")
    if worldbody is None:
        worldbody = ET.SubElement(root, "worldbody")
    if worldbody.find("./geom[@name='floor']") is None:
        floor = ET.Element(
            "geom",
            {
                "name": "floor",
                "type": "plane",
                "size": "20 20 0.05",
                "rgba": "0.2 0.24 0.26 1",
                "condim": "3",
                "friction": "0.9 0.02 0.001",
            },
        )
        worldbody.insert(0, floor)

    ET.ElementTree(root).write(out_path, encoding="unicode")
    return out_path


_scene_package = _scene_package_config()
_x2_scene_xml = _x2_mujoco_scene_xml()
_viewer_kwargs: dict[str, Any] = {
    "mjcf_path": str(_X2_ROBOT_MJCF_PATH),
    "camera_name": "rgbd_head_front",
    "vehicle_height": _X2_SPAWN_Z_M,
    "pointcloud_hz": _env_float("DIMOS_BABYLON_POINTCLOUD_HZ", 2.0),
    "pointcloud_max_points": _env_int("DIMOS_BABYLON_POINTCLOUD_MAX_POINTS", 70000),
}
if _scene_package is not None and _scene_package.visual_path is not None:
    _viewer_kwargs.update(
        scene_path=str(_scene_package.visual_path),
        scene_scale=_scene_package.alignment.scale,
        scene_translation=_scene_package.alignment.translation,
        scene_rotation_zyx_deg=_scene_package.alignment.rotation_zyx_deg,
        scene_y_up=_scene_package.alignment.y_up,
        browser_collision_path=(
            str(_scene_package.browser_collision_path)
            if _scene_package.browser_collision_path is not None
            else None
        ),
        initial_entities=_scene_package.entities,
    )


agibot_x2_policy_sim = (
    autoconnect(
        MujocoSimModule.blueprint(
            scene_xml=(str(_x2_scene_xml) if _x2_scene_xml is not None else None),
            robot_mjcf=str(_X2_ROBOT_MJCF_PATH),
            robot_meshdir=str(_X2_MESH_DIR),
            scene_entities=_scene_package.entities if _scene_package else [],
            headless=True,
            dof=len(X2_JOINTS),
            enable_color=False,
            enable_depth=False,
            enable_pointcloud=False,
            spawn_z=_X2_SPAWN_Z_M,
            reset_joint_positions=X2_DEFAULT_POSITIONS,
            imu_gyro_sensor_names=["body-angular-velocity"],
            imu_accel_sensor_names=["body-linear-acceleration"],
            imu_linvel_sensor_names=["body-linear-vel"],
        ),
        ControlCoordinator.blueprint(
            tick_rate=_X2_SIM_TICK_RATE_HZ,
            publish_joint_state=True,
            joint_state_frame_id="coordinator",
            hardware=[
                HardwareComponent(
                    hardware_id="x2",
                    hardware_type=HardwareType.WHOLE_BODY,
                    joints=X2_JOINTS,
                    adapter_type="sim_mujoco_x2",
                    # SHM key matches MujocoSimModule's robot_mjcf source.
                    address=str(_X2_ROBOT_MJCF_PATH),
                    auto_enable=True,
                    wb_config=WholeBodyConfig(kp=tuple(X2_KP), kd=tuple(X2_KD)),
                ),
            ],
            tasks=[
                TaskConfig(
                    name="x2_rsl_rl_wbc",
                    type="x2_rsl_rl_wbc",
                    joint_names=X2_JOINTS,
                    priority=50,
                    auto_start=True,
                    params={
                        "policy_onnx": _policy_onnx_path(),
                        "hardware_id": "x2",
                        "auto_arm": True,
                        "auto_dry_run": False,
                        "decimation": 1,
                    },
                ),
            ],
        ),
        BabylonSceneViewerModule.blueprint(**_viewer_kwargs),
    )
    .transports(
        {
            ("joint_state", JointState): LCMTransport(_JOINT_STATE_TOPIC, JointState),
            ("odom", PoseStamped): LCMTransport(_ODOM_TOPIC, PoseStamped),
            ("cmd_vel", Twist): LCMTransport(_CMD_VEL_TOPIC, Twist),
            ("twist_command", Twist): LCMTransport(_CMD_VEL_TOPIC, Twist),
        }
    )
    .global_config(n_workers=4, robot_model="agibot_x2_ultra")
)

__all__ = ["agibot_x2_policy_sim"]
