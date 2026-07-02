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

"""Mesh-backed simulated RGBD camera with textured rendering.

Ray-casts the cooked scene mesh from the robot's camera pose to produce a
real camera feed — **color and depth** — so downstream perception
(``ObjectTracking`` / ``TemporalMemory`` / object registration) sees the
loaded environment instead of a black placeholder. Depth is published in
the same ``ImageFormat.DEPTH`` (float metres, registered to the color
frame) wire format ``MujocoSimModule`` uses, so a perception module is
identical whichever backend rendered the frame.

Color is **textured**: the scene is loaded with its glTF materials
(via trimesh, which tolerates real-world GLBs better than ASSIMP), and
each ray hit samples the material's ``baseColorTexture`` at the
barycentric-interpolated UV. Per-geometry fallback chain when a texture
is absent: vertex colors → ``baseColorFactor`` → neutral grey. Scans
carry their lighting baked into albedo, so sampling albedo directly
reproduces the captured appearance without a shading model — and the
whole pipeline stays pure CPU (Open3D BVH + numpy gathers): no GL/EGL
context, no render-thread affinity, runs identically on macOS dev boxes
and headless CI. (An Open3D Filament offscreen path was evaluated and
rejected: the macOS wheel only supports EGL-headless, and Cocoa pins GUI
rendering to the main thread — the same trap the splat camera documents.)

Pose pipeline per tick:
  1. FK the robot off ``/coordinator/joint_state`` + ``/odom`` to find
     the camera body (same code path ``SplatCameraModule`` uses).
  2. ``RaycastingScene.cast_rays`` → hit distance + triangle + barycentric.
  3. Texture/vertex-color/factor sample per pixel → ``(H, W, 3)`` uint8.
  4. Publish ``color_image``/``depth_image`` (+ ``camera_info`` on a slow
     timer) and a ``world -> frame_id`` TF for projection consumers.

Perf: ~20 ms/frame at 640x480 on M-series (BVH + gathers), so 10 Hz is
comfortable even at VGA.
"""

from __future__ import annotations

from pathlib import Path as FilePath
import threading
import time
from typing import Any

import mujoco
import numpy as np
from pydantic import Field
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.simulation.scene.mesh_scene import (
    SceneMeshAlignment,
    _world_rotation,
    load_scene_mesh,
)
from dimos.simulation.sensors.rig import CameraSpec, g1_d435_default, world_pose
from dimos.simulation.sensors.robot_fk import (
    apply_state,
    dimos_joint_to_mjcf,
    load_robot_meshes,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_SKY_MAT = -1_000_000
"""Sentinel material id for pixels whose rays miss all geometry."""


class MeshCameraConfig(ModuleConfig):
    """Configuration for ``MeshCameraModule``."""

    scene_path: str = ""
    """Path to a ``.glb`` / ``.obj`` / ``.ply`` / ``.stl`` scene mesh.
    Empty disables the publisher."""

    mjcf_path: str = ""
    """MJCF used for camera-body forward kinematics.  Should match the
    one ``MujocoSimModule`` simulates against."""

    scene_scale: float = 1.0
    scene_translation: tuple[float, float, float] = (0.0, 0.0, 0.0)
    scene_rotation_zyx_deg: tuple[float, float, float] = (0.0, 0.0, 0.0)
    scene_y_up: bool = True

    render_hz: float = 10.0
    info_hz: float = 1.0
    frame_id: str = "camera_optical"

    publish_tf: bool = True
    """Publish a ``world -> frame_id`` transform each rendered frame so
    projection consumers can place the pixels."""

    sky_color: tuple[int, int, int] = (135, 175, 215)
    """RGB 0..255 for ray-misses.  Soft sky-blue by default."""


class _RenderAssets:
    """Immutable per-scene raycast + shading tables built once at start."""

    def __init__(
        self,
        raycasting_scene: Any,
        tri_uv: np.ndarray,
        tri_mat: np.ndarray,
        tri_vertices: np.ndarray,
        vertex_colors: np.ndarray | None,
        textures: list[np.ndarray],
        flat_colors: np.ndarray,
    ) -> None:
        self.scene = raycasting_scene
        self.tri_uv = tri_uv  # (Nt, 3, 2) float32
        self.tri_mat = tri_mat  # (Nt,) int32: >=0 texture idx, -1 vertex colors, <=-2 flat
        self.tri_vertices = tri_vertices  # (Nt, 3) int64 — for vertex-color lerp
        self.vertex_colors = vertex_colors  # (Nv, 3) float32 in [0,1] or None
        self.textures = textures  # list of (Ht, Wt, 3) uint8
        self.flat_colors = flat_colors  # (Nf, 3) uint8, indexed by (-mat - 2)


class MeshCameraModule(Module):
    """Ray-cast the cooked scene mesh from the robot camera pose as RGBD."""

    config: MeshCameraConfig = Field(default_factory=MeshCameraConfig)

    color_image: Out[Image]
    depth_image: Out[Image]
    camera_info: Out[CameraInfo]
    depth_camera_info: Out[CameraInfo]
    joint_state: In[JointState]
    odom: In[PoseStamped]

    def __init__(
        self,
        *,
        camera_spec: CameraSpec | None = None,
        **kwargs: Any,
    ) -> None:
        super().__init__(**kwargs)
        self._camera_spec = camera_spec if camera_spec is not None else g1_d435_default()

        self._state_lock = threading.Lock()
        self._latest_joints: dict[str, float] = {}
        self._latest_base_pos: np.ndarray | None = None
        self._latest_base_wxyz: np.ndarray | None = None

        self._assets: _RenderAssets | None = None
        self._ray_dirs_cam: np.ndarray | None = None
        self._cam_body_id: int | None = None
        self._cam_info_msg: CameraInfo | None = None
        self._render_thread: threading.Thread | None = None
        self._info_thread: threading.Thread | None = None
        self._stop_event = threading.Event()

    @rpc
    def start(self) -> None:
        super().start()

        cfg = self.config
        if not cfg.scene_path or not cfg.mjcf_path:
            logger.info("MeshCameraModule: scene_path or mjcf_path empty, publisher disabled")
            return

        scene_file = FilePath(cfg.scene_path).expanduser()
        alignment = SceneMeshAlignment(
            scale=cfg.scene_scale,
            rotation_zyx_deg=cfg.scene_rotation_zyx_deg,
            translation=cfg.scene_translation,
            y_up=cfg.scene_y_up,
        )
        logger.info(f"MeshCameraModule: loading scene mesh {scene_file}")
        try:
            self._assets = self._build_textured_assets(scene_file, alignment)
        except Exception as e:
            logger.warning(
                f"MeshCameraModule: textured load failed ({e}); falling back to vertex-color loader"
            )
            self._assets = self._build_vertex_color_assets(scene_file, alignment)

        # Robot meshes — for FK to extract camera body pose each tick.
        # Inline asset walk: avoids importing splat_camera (and through
        # it the heavy gsplat / splat-apple machinery) just to reuse one
        # helper, and avoids dimos.simulation.backend.mujoco.assets.get_assets
        # which pulls mujoco_playground → warp → torch.
        from dimos.utils.data import get_data

        data_dir = FilePath(str(get_data("mujoco_sim")))
        person_dir = FilePath(str(get_data("person")))
        assets: dict[str, bytes] = {}
        for root, pattern in [
            (data_dir, "*.xml"),
            (data_dir, "*.obj"),
            (data_dir / "scene_office1" / "textures", "*.png"),
            (data_dir / "scene_office1" / "office_split", "*.obj"),
            (person_dir, "*.obj"),
            (person_dir, "*.png"),
        ]:
            if root.exists():
                for f in root.glob(pattern):
                    if f.is_file():
                        assets[f.name] = f.read_bytes()
        # Bundled menagerie (G1 / Go1 meshes) via importlib so we don't
        # exec ``mujoco_playground.__init__``.
        import importlib.util

        spec = importlib.util.find_spec("mujoco_playground")
        if spec is not None and spec.submodule_search_locations:
            menagerie = (
                FilePath(next(iter(spec.submodule_search_locations)))
                / "external_deps"
                / "mujoco_menagerie"
            )
            for sub in ("unitree_go1/assets", "unitree_g1/assets"):
                root = menagerie / sub
                if root.exists():
                    for f in root.iterdir():
                        if f.is_file():
                            assets[f.name] = f.read_bytes()
        self._robot = load_robot_meshes(FilePath(cfg.mjcf_path), assets=assets)

        cam_body_id = mujoco.mj_name2id(
            self._robot.model, mujoco.mjtObj.mjOBJ_BODY, self._camera_spec.body_name
        )
        if cam_body_id < 0:
            logger.error(
                f"MeshCameraModule: camera mount body '{self._camera_spec.body_name}' "
                f"not in MJCF; module will publish nothing"
            )
            return
        self._cam_body_id = cam_body_id

        self._cam_info_msg = self._build_camera_info()
        self._ray_dirs_cam = self._build_ray_directions_cam_frame()

        try:
            unsub = self.joint_state.subscribe(self._on_joint_state)
            self.register_disposable(Disposable(unsub))
        except Exception as e:
            logger.warning(f"MeshCameraModule: joint_state subscribe failed: {e}")

        try:
            unsub = self.odom.subscribe(self._on_odom)
            self.register_disposable(Disposable(unsub))
        except Exception as e:
            logger.warning(f"MeshCameraModule: odom subscribe failed: {e}")

        self._render_thread = threading.Thread(
            target=self._render_loop, name="mesh-camera-render", daemon=True
        )
        self._render_thread.start()
        self._info_thread = threading.Thread(
            target=self._info_loop, name="mesh-camera-info", daemon=True
        )
        self._info_thread.start()

        spec = self._camera_spec
        n_tex = len(self._assets.textures) if self._assets else 0
        logger.info(
            f"MeshCameraModule publishing color+depth at {cfg.render_hz} Hz "
            f"({spec.width}x{spec.height} @ vfov={spec.vfov_deg}°, {n_tex} textures)"
        )

    @rpc
    def stop(self) -> None:
        if hasattr(self, "_stop_event"):
            self._stop_event.set()
            for t in (self._render_thread, self._info_thread):
                if t and t.is_alive():
                    t.join(timeout=2.0)
        super().stop()

    # ----- asset building -----

    def _alignment_matrix(self, alignment: SceneMeshAlignment) -> np.ndarray:
        """Homogeneous source→world transform: world = R @ (scale · v) + t."""
        rotation = _world_rotation(alignment)
        matrix = np.eye(4, dtype=np.float64)
        matrix[:3, :3] = rotation * alignment.scale
        matrix[:3, 3] = np.asarray(alignment.translation, dtype=np.float64)
        return matrix

    def _build_textured_assets(
        self, scene_file: FilePath, alignment: SceneMeshAlignment
    ) -> _RenderAssets:
        """Load with trimesh, keeping glTF materials/UVs for texture sampling."""
        import open3d as o3d
        import trimesh

        loaded = trimesh.load(str(scene_file), force="scene", process=False)
        world_from_source = self._alignment_matrix(alignment)

        verts_parts: list[np.ndarray] = []
        tris_parts: list[np.ndarray] = []
        uv_parts: list[np.ndarray] = []
        mat_parts: list[np.ndarray] = []
        vcol_parts: list[np.ndarray] = []
        textures: list[np.ndarray] = []
        flat_colors: list[np.ndarray] = []
        vertex_offset = 0
        any_vertex_colors = False

        for node in loaded.graph.nodes_geometry:
            node_transform, geom_name = loaded.graph[node]
            geom = loaded.geometry[geom_name]
            faces = np.asarray(geom.faces, dtype=np.int64)
            if faces.size == 0:
                continue
            verts = trimesh.transformations.transform_points(geom.vertices, node_transform)
            verts = trimesh.transformations.transform_points(verts, world_from_source)

            visual = getattr(geom, "visual", None)
            material = getattr(visual, "material", None)
            uv = getattr(visual, "uv", None)
            texture = getattr(material, "baseColorTexture", None)

            n_verts = len(geom.vertices)
            geom_vcols = np.full((n_verts, 3), 0.7, dtype=np.float32)
            if uv is not None and len(uv) == n_verts and texture is not None:
                mat_id = len(textures)
                textures.append(np.asarray(texture.convert("RGB"), dtype=np.uint8))
                uv_tri = np.asarray(uv, dtype=np.float32)[faces]
            else:
                uv_tri = np.zeros((len(faces), 3, 2), dtype=np.float32)
                vertex_colors = getattr(visual, "vertex_colors", None)
                if vertex_colors is not None and len(vertex_colors) == n_verts:
                    mat_id = -1
                    geom_vcols = (
                        np.asarray(vertex_colors, dtype=np.float32)[:, :3] / 255.0
                    ).astype(np.float32)
                    any_vertex_colors = True
                else:
                    factor = getattr(material, "baseColorFactor", None)
                    color = (
                        np.asarray(factor[:3], dtype=np.uint8)
                        if factor is not None
                        else np.array([180, 180, 180], dtype=np.uint8)
                    )
                    mat_id = -2 - len(flat_colors)
                    flat_colors.append(color)

            verts_parts.append(verts.astype(np.float32))
            tris_parts.append(faces + vertex_offset)
            uv_parts.append(uv_tri)
            mat_parts.append(np.full(len(faces), mat_id, dtype=np.int32))
            vcol_parts.append(geom_vcols)
            vertex_offset += n_verts

        if not tris_parts:
            raise RuntimeError(f"no triangles in {scene_file}")

        vertices = np.concatenate(verts_parts)
        triangles = np.concatenate(tris_parts)
        raycasting_scene = o3d.t.geometry.RaycastingScene()
        raycasting_scene.add_triangles(
            o3d.core.Tensor(vertices), o3d.core.Tensor(triangles.astype(np.uint32))
        )
        return _RenderAssets(
            raycasting_scene=raycasting_scene,
            tri_uv=np.concatenate(uv_parts),
            tri_mat=np.concatenate(mat_parts),
            tri_vertices=triangles,
            vertex_colors=np.concatenate(vcol_parts) if any_vertex_colors else None,
            textures=textures,
            flat_colors=(
                np.stack(flat_colors) if flat_colors else np.zeros((0, 3), dtype=np.uint8)
            ),
        )

    def _build_vertex_color_assets(
        self, scene_file: FilePath, alignment: SceneMeshAlignment
    ) -> _RenderAssets:
        """Format fallback: the original Open3D loader, vertex colors only."""
        import open3d as o3d

        mesh = load_scene_mesh(scene_file, alignment=alignment)
        triangles = np.asarray(mesh.triangles, dtype=np.int64)
        if mesh.has_vertex_colors():
            vertex_colors = np.asarray(mesh.vertex_colors, dtype=np.float32)
        else:
            vertex_colors = np.full((len(mesh.vertices), 3), 0.7, dtype=np.float32)
        raycasting_scene = o3d.t.geometry.RaycastingScene()
        raycasting_scene.add_triangles(
            o3d.core.Tensor(np.asarray(mesh.vertices, dtype=np.float32)),
            o3d.core.Tensor(triangles.astype(np.uint32)),
        )
        return _RenderAssets(
            raycasting_scene=raycasting_scene,
            tri_uv=np.zeros((len(triangles), 3, 2), dtype=np.float32),
            tri_mat=np.full(len(triangles), -1, dtype=np.int32),
            tri_vertices=triangles,
            vertex_colors=vertex_colors,
            textures=[],
            flat_colors=np.zeros((0, 3), dtype=np.uint8),
        )

    # ----- helpers -----

    def _build_camera_info(self) -> CameraInfo:
        spec = self._camera_spec
        f = spec.focal_pixels()
        cx, cy = spec.cx(), spec.cy()
        return CameraInfo(
            frame_id=self.config.frame_id,
            height=spec.height,
            width=spec.width,
            distortion_model="plumb_bob",
            D=[0.0, 0.0, 0.0, 0.0, 0.0],
            K=[f, 0.0, cx, 0.0, f, cy, 0.0, 0.0, 1.0],
            R=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            P=[f, 0.0, cx, 0.0, 0.0, f, cy, 0.0, 0.0, 0.0, 1.0, 0.0],
        )

    def _build_ray_directions_cam_frame(self) -> np.ndarray:
        """Per-pixel ray directions in *camera* frame (+x right, +y down, +z fwd).

        Returns a contiguous ``(H*W, 3)`` float32 array, normalized.
        """
        spec = self._camera_spec
        f = spec.focal_pixels()
        cx, cy = spec.cx(), spec.cy()
        ys, xs = np.meshgrid(
            np.arange(spec.height, dtype=np.float32),
            np.arange(spec.width, dtype=np.float32),
            indexing="ij",
        )
        x = (xs + 0.5 - cx) / f
        y = (ys + 0.5 - cy) / f
        z = np.ones_like(x, dtype=np.float32)
        d = np.stack([x, y, z], axis=-1).reshape(-1, 3)
        d /= np.linalg.norm(d, axis=1, keepdims=True)
        return d.astype(np.float32)

    def _on_joint_state(self, msg: JointState) -> None:
        names = list(msg.name)
        positions = list(msg.position)
        if not names or len(names) != len(positions):
            return
        with self._state_lock:
            for n, q in zip(names, positions, strict=False):
                self._latest_joints[dimos_joint_to_mjcf(n)] = float(q)

    def _on_odom(self, msg: PoseStamped) -> None:
        with self._state_lock:
            self._latest_base_pos = np.array(
                [msg.position.x, msg.position.y, msg.position.z], dtype=np.float64
            )
            self._latest_base_wxyz = np.array(
                [msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z],
                dtype=np.float64,
            )

    def _camera_pose(self) -> tuple[np.ndarray, np.ndarray]:
        """FK the robot and return the camera (pos, wxyz) in world frame."""
        with self._state_lock:
            joints = dict(self._latest_joints)
            base_pos = None if self._latest_base_pos is None else self._latest_base_pos.copy()
            base_wxyz = None if self._latest_base_wxyz is None else self._latest_base_wxyz.copy()
        apply_state(self._robot, base_pos=base_pos, base_wxyz=base_wxyz, joint_positions=joints)
        body_pos = self._robot.data.xpos[self._cam_body_id]
        body_wxyz = self._robot.data.xquat[self._cam_body_id]
        return world_pose(body_pos, body_wxyz, self._camera_spec)

    def _publish_tf_pose(self, cam_pos: np.ndarray, cam_wxyz: np.ndarray, ts: float) -> None:
        if not self.config.publish_tf:
            return
        try:
            self.tf.publish(
                Transform(
                    translation=Vector3(float(cam_pos[0]), float(cam_pos[1]), float(cam_pos[2])),
                    # world_pose returns wxyz; Quaternion stores xyzw.
                    rotation=Quaternion(
                        float(cam_wxyz[1]),
                        float(cam_wxyz[2]),
                        float(cam_wxyz[3]),
                        float(cam_wxyz[0]),
                    ),
                    frame_id="world",
                    child_frame_id=self.config.frame_id,
                    ts=ts,
                )
            )
        except Exception as e:
            logger.debug(f"MeshCameraModule: tf publish failed: {e}")

    def _info_loop(self) -> None:
        period = 1.0 / self.config.info_hz if self.config.info_hz > 0 else 1.0
        while not self._stop_event.is_set():
            if self._cam_info_msg is not None:
                self._cam_info_msg.ts = time.time()
                try:
                    # Depth is registered to the color frame, so both infos are
                    # the same intrinsics — mirrors MujocoSimModule's RGBD pair.
                    self.camera_info.publish(self._cam_info_msg)
                    self.depth_camera_info.publish(self._cam_info_msg)
                except Exception as e:
                    logger.debug(f"MeshCameraModule: camera_info publish failed: {e}")
            self._stop_event.wait(period)

    # ----- render loop -----

    def _shade(self, tid: np.ndarray, bary_uv: np.ndarray, hit_mask: np.ndarray) -> np.ndarray:
        """Per-ray uint8 RGB from triangle hits (texture / vertex / flat)."""
        assets = self._assets
        assert assets is not None
        tid_safe = np.where(hit_mask, tid, 0)
        u = bary_uv[:, 0:1].astype(np.float32)
        v = bary_uv[:, 1:2].astype(np.float32)
        w = 1.0 - u - v

        material_ids = np.where(hit_mask, assets.tri_mat[tid_safe], _SKY_MAT)
        out = np.zeros((len(tid), 3), dtype=np.uint8)

        # Texture UV per hit (only meaningful where mat >= 0).
        uv_tri = assets.tri_uv[tid_safe]  # (N, 3, 2)
        uv_hit = w * uv_tri[:, 0] + u * uv_tri[:, 1] + v * uv_tri[:, 2]
        uv_hit -= np.floor(uv_hit)  # glTF REPEAT wrap

        for material_id in np.unique(material_ids):
            selected = material_ids == material_id
            if material_id == _SKY_MAT:
                out[selected] = np.array(self.config.sky_color, dtype=np.uint8)
            elif material_id >= 0:
                texture = assets.textures[material_id]
                tex_h, tex_w = texture.shape[:2]
                px = np.clip((uv_hit[selected, 0] * (tex_w - 1)).astype(np.int32), 0, tex_w - 1)
                py = np.clip(
                    ((1.0 - uv_hit[selected, 1]) * (tex_h - 1)).astype(np.int32), 0, tex_h - 1
                )
                out[selected] = texture[py, px]
            elif material_id == -1 and assets.vertex_colors is not None:
                tris = assets.tri_vertices[tid_safe[selected]]
                c0 = assets.vertex_colors[tris[:, 0]]
                c1 = assets.vertex_colors[tris[:, 1]]
                c2 = assets.vertex_colors[tris[:, 2]]
                colors = w[selected] * c0 + u[selected] * c1 + v[selected] * c2
                out[selected] = np.clip(colors * 255.0, 0.0, 255.0).astype(np.uint8)
            else:
                out[selected] = assets.flat_colors[-2 - material_id]
        return out

    def _render_loop(self) -> None:
        import open3d.core as o3c

        spec = self._camera_spec
        assets = self._assets
        if assets is None or self._ray_dirs_cam is None:
            return
        period = 1.0 / self.config.render_hz if self.config.render_hz > 0 else 0.1

        next_tick = time.monotonic()
        while not self._stop_event.is_set():
            try:
                cam_pos, cam_wxyz = self._camera_pose()

                # Camera-to-world rotation from wxyz quaternion.
                rot9 = np.zeros(9, dtype=np.float64)
                mujoco.mju_quat2Mat(rot9, np.asarray(cam_wxyz, dtype=np.float64))
                r_world_cam = rot9.reshape(3, 3).astype(np.float32)

                dirs_world = (r_world_cam @ self._ray_dirs_cam.T).T  # (H*W, 3)
                origins = np.broadcast_to(np.asarray(cam_pos, dtype=np.float32), dirs_world.shape)
                rays = np.concatenate([origins, dirs_world], axis=-1).astype(np.float32)
                hit = assets.scene.cast_rays(o3c.Tensor(rays, dtype=o3c.Dtype.Float32))

                t_hit = hit["t_hit"].numpy()
                tri_id = hit["primitive_ids"].numpy().astype(np.int64)
                bary_uv = hit["primitive_uvs"].numpy()  # (N, 2) barycentric
                hit_mask = np.isfinite(t_hit)

                ts = time.time()
                rgb = self._shade(tri_id, bary_uv, hit_mask).reshape(spec.height, spec.width, 3)

                # Depth: project the ray hit distance onto the optical axis
                # (Z = t_hit · dir_z, dir normalized in camera frame), float
                # metres with misses = 0 — the same DEPTH wire format
                # MujocoSimModule publishes.
                depth_m = (t_hit * self._ray_dirs_cam[:, 2]).astype(np.float32)
                depth_m[~hit_mask] = 0.0

                self.color_image.publish(
                    Image(ts=ts, frame_id=self.config.frame_id, format=ImageFormat.RGB, data=rgb)
                )
                self.depth_image.publish(
                    Image(
                        ts=ts,
                        frame_id=self.config.frame_id,
                        format=ImageFormat.DEPTH,
                        data=depth_m.reshape(spec.height, spec.width),
                    )
                )
                self._publish_tf_pose(cam_pos, cam_wxyz, ts)
            except Exception as e:
                logger.debug(f"MeshCameraModule render tick failed: {e}")

            next_tick += period
            sleep_for = next_tick - time.monotonic()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                next_tick = time.monotonic()


mesh_camera = MeshCameraModule.blueprint

__all__ = ["MeshCameraConfig", "MeshCameraModule", "mesh_camera"]
