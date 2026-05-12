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

"""Relocalize: ICP-based pose correction against a prior PCD map.

Port of the CMU autonomy stack's localizer_node
(``FASTLIO2_ROS2/localizer``): two-stage ICP (rough → refine) of live scans
against a pre-recorded map. Publishes a map → local-map TF correction so
downstream navigation operates in the prior map's frame.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import threading
import time
from typing import Any

import numpy as np
import open3d as o3d  # type: ignore[import-untyped]
from reactivex.disposable import Disposable
from scipy.spatial.transform import Rotation

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.nav_stack.frames import FRAME_MAP, FRAME_ODOM
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


@dataclass
class ICPParams:
    """Two-stage ICP parameters mirroring the CMU localizer config."""

    rough_scan_resolution: float = 0.25
    rough_map_resolution: float = 0.25
    rough_max_iteration: int = 5
    rough_score_thresh: float = 0.2

    refine_scan_resolution: float = 0.1
    refine_map_resolution: float = 0.1
    refine_max_iteration: int = 10
    refine_score_thresh: float = 0.1


def voxel_downsample(cloud: o3d.geometry.PointCloud, resolution: float) -> o3d.geometry.PointCloud:
    """Voxel-grid downsample; pass-through when resolution <= 0."""
    if resolution > 0:
        return cloud.voxel_down_sample(resolution)
    return cloud


def load_prior_pcd(
    path: str | Path, params: ICPParams
) -> tuple[o3d.geometry.PointCloud, o3d.geometry.PointCloud]:
    """Read a PCD file and return (rough_target, refine_target) downsampled clouds."""
    pcd_path = Path(path)
    if not pcd_path.exists():
        raise FileNotFoundError(f"Relocalize: PCD not found: {pcd_path}")
    cloud = o3d.io.read_point_cloud(str(pcd_path))
    if len(cloud.points) == 0:
        raise ValueError(f"Relocalize: PCD has no points: {pcd_path}")
    return (
        voxel_downsample(cloud, params.rough_map_resolution),
        voxel_downsample(cloud, params.refine_map_resolution),
    )


def two_stage_icp(
    source: o3d.geometry.PointCloud,
    rough_target: o3d.geometry.PointCloud,
    refine_target: o3d.geometry.PointCloud,
    initial_guess: np.ndarray,
    params: ICPParams,
) -> tuple[bool, np.ndarray]:
    """Run a two-stage ICP and return (converged, 4x4 transform).

    The transform takes points from the source frame (live scan, in local-map
    coordinates after applying initial_guess) into the target map frame.
    """
    if initial_guess.shape != (4, 4):
        raise ValueError(f"initial_guess must be 4x4, got {initial_guess.shape}")

    estimator = o3d.pipelines.registration.TransformationEstimationPointToPoint()

    src_rough = voxel_downsample(source, params.rough_scan_resolution)
    rough = o3d.pipelines.registration.registration_icp(
        src_rough,
        rough_target,
        max_correspondence_distance=max(params.rough_scan_resolution, 0.05) * 2.0,
        init=initial_guess,
        estimation_method=estimator,
        criteria=o3d.pipelines.registration.ICPConvergenceCriteria(
            max_iteration=params.rough_max_iteration
        ),
    )
    if rough.inlier_rmse <= 0.0 or rough.inlier_rmse > params.rough_score_thresh:
        return False, initial_guess

    src_refine = voxel_downsample(source, params.refine_scan_resolution)
    refine = o3d.pipelines.registration.registration_icp(
        src_refine,
        refine_target,
        max_correspondence_distance=max(params.refine_scan_resolution, 0.02) * 2.0,
        init=rough.transformation,
        estimation_method=estimator,
        criteria=o3d.pipelines.registration.ICPConvergenceCriteria(
            max_iteration=params.refine_max_iteration
        ),
    )
    if refine.inlier_rmse <= 0.0 or refine.inlier_rmse > params.refine_score_thresh:
        return False, rough.transformation
    return True, np.asarray(refine.transformation)


def pose_matrix(x: float, y: float, z: float, roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Build a 4x4 transform from translation + ZYX-extrinsic Euler angles."""
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_euler("ZYX", [yaw, pitch, roll]).as_matrix()
    matrix[:3, 3] = [x, y, z]
    return matrix


class RelocalizeConfig(ModuleConfig):
    # Prior map
    pcd_path: str = ""
    auto_relocalize: bool = False

    # Initial pose guess (applied when auto_relocalize=True or as default for relocalize())
    initial_x: float = 0.0
    initial_y: float = 0.0
    initial_z: float = 0.0
    initial_roll: float = 0.0
    initial_pitch: float = 0.0
    initial_yaw: float = 0.0

    # Frames
    map_frame: str = FRAME_MAP
    local_frame: str = FRAME_ODOM

    # Two-stage ICP knobs
    rough_scan_resolution: float = 0.25
    rough_map_resolution: float = 0.25
    rough_max_iteration: int = 5
    rough_score_thresh: float = 0.2

    refine_scan_resolution: float = 0.1
    refine_map_resolution: float = 0.1
    refine_max_iteration: int = 10
    refine_score_thresh: float = 0.1

    # ICP runs at this rate; in between, the last correction is rebroadcast.
    update_hz: float = 1.0


class Relocalize(Module):
    """ICP-based pose correction against a prior PCD map.

    Subscribes to live ``body_cloud`` + ``lio_odom``, runs two-stage ICP
    against a prior PCD at ``update_hz``, and publishes a ``map → local_frame``
    TF correction via ``self.tf``.
    """

    config: RelocalizeConfig

    body_cloud: In[PointCloud2]
    lio_odom: In[Odometry]
    map_cloud: Out[PointCloud2]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = threading.Lock()
        self._running = False
        self._thread: threading.Thread | None = None

        self._last_cloud: np.ndarray | None = None
        self._last_cloud_ts: float = 0.0
        self._last_local_pose: np.ndarray = np.eye(4)
        self._have_cloud = False
        self._have_odom = False

        self._rough_target: o3d.geometry.PointCloud | None = None
        self._refine_target: o3d.geometry.PointCloud | None = None
        self._map_published = False

        self._initial_guess: np.ndarray = np.eye(4)
        self._service_received: bool = False
        self._localize_success: bool = False
        self._auto_reloc_triggered: bool = False
        self._last_offset: np.ndarray = np.eye(4)

    @rpc
    def build(self) -> None:
        super().build()
        if self.config.pcd_path:
            self._load_map(self.config.pcd_path)

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.body_cloud.subscribe(self._on_cloud)))
        self.register_disposable(Disposable(self.lio_odom.subscribe(self._on_odom)))

        self._running = True
        self._thread = threading.Thread(target=self._relocalize_loop, daemon=True)
        self._thread.start()
        logger.info("Relocalize started (auto_relocalize=%s)", self.config.auto_relocalize)

    @rpc
    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
            self._thread = None
        super().stop()

    @rpc
    def relocalize(
        self,
        x: float,
        y: float,
        z: float,
        roll: float,
        pitch: float,
        yaw: float,
        pcd_path: str = "",
    ) -> str:
        """Set a new initial guess (and optionally swap in a new prior map)."""
        if pcd_path:
            self._load_map(pcd_path)
        with self._lock:
            self._initial_guess = pose_matrix(x, y, z, roll, pitch, yaw)
            self._service_received = True
            self._localize_success = False
        return f"Relocalize armed at ({x:.2f}, {y:.2f}, {z:.2f})"

    @rpc
    def relocalize_check(self) -> bool:
        """Return whether the most recent ICP converged."""
        with self._lock:
            return self._localize_success

    def _load_map(self, path: str) -> None:
        rough, refine = load_prior_pcd(path, self._icp_params())
        with self._lock:
            self._rough_target = rough
            self._refine_target = refine
            self._map_published = False
        logger.info(
            "Relocalize loaded prior map %s (rough=%d pts, refine=%d pts)",
            path,
            len(rough.points),
            len(refine.points),
        )

    def _icp_params(self) -> ICPParams:
        cfg = self.config
        return ICPParams(
            rough_scan_resolution=cfg.rough_scan_resolution,
            rough_map_resolution=cfg.rough_map_resolution,
            rough_max_iteration=cfg.rough_max_iteration,
            rough_score_thresh=cfg.rough_score_thresh,
            refine_scan_resolution=cfg.refine_scan_resolution,
            refine_map_resolution=cfg.refine_map_resolution,
            refine_max_iteration=cfg.refine_max_iteration,
            refine_score_thresh=cfg.refine_score_thresh,
        )

    def _on_cloud(self, msg: PointCloud2) -> None:
        points, _ = msg.as_numpy()
        if points.size == 0:
            return
        ts = msg.ts if msg.ts is not None else time.time()
        with self._lock:
            self._last_cloud = points[:, :3].astype(np.float32, copy=False)
            self._last_cloud_ts = ts
            self._have_cloud = True

    def _on_odom(self, msg: Odometry) -> None:
        # Build the 4x4 pose from the Odometry message (local-map → lidar/body).
        pose = msg.pose
        x = pose.position.x
        y = pose.position.y
        z = pose.position.z
        qx = pose.orientation.x
        qy = pose.orientation.y
        qz = pose.orientation.z
        qw = pose.orientation.w
        matrix = np.eye(4)
        matrix[:3, :3] = Rotation.from_quat([qx, qy, qz, qw]).as_matrix()
        matrix[:3, 3] = [x, y, z]
        with self._lock:
            self._last_local_pose = matrix
            self._have_odom = True

    def _relocalize_loop(self) -> None:
        period = 1.0 / max(self.config.update_hz, 0.01)
        last_tick = 0.0
        while self._running:
            now = time.monotonic()
            if now - last_tick < period:
                time.sleep(min(0.05, period))
                continue
            last_tick = now

            self._step()

    def _step(self) -> None:
        # Trigger auto-relocalization once we have the first cloud.
        with self._lock:
            ready = self._have_cloud and self._have_odom
        if not ready:
            return

        if self.config.auto_relocalize and not self._auto_reloc_triggered:
            self._auto_reloc_triggered = True
            self.relocalize(
                self.config.initial_x,
                self.config.initial_y,
                self.config.initial_z,
                self.config.initial_roll,
                self.config.initial_pitch,
                self.config.initial_yaw,
            )

        with self._lock:
            cloud_np = self._last_cloud
            local_pose = self._last_local_pose
            ts = self._last_cloud_ts
            last_offset = self._last_offset
            initial_guess = (
                self._initial_guess if self._service_received else last_offset @ local_pose
            )
            rough = self._rough_target
            refine = self._refine_target

        if rough is None or refine is None or cloud_np is None:
            self._publish_correction(last_offset, ts)
            return

        source = o3d.geometry.PointCloud()
        source.points = o3d.utility.Vector3dVector(cloud_np.astype(np.float64))

        params = self._icp_params()
        converged, transform = two_stage_icp(source, rough, refine, initial_guess, params)

        if converged:
            new_offset = transform @ np.linalg.inv(local_pose)
            with self._lock:
                self._last_offset = new_offset
                self._localize_success = True
            self._publish_correction(new_offset, ts)
        else:
            with self._lock:
                self._localize_success = False
            self._publish_correction(last_offset, ts)

        self._maybe_publish_map(ts)

    def _publish_correction(self, offset: np.ndarray, ts: float) -> None:
        translation = offset[:3, 3]
        rotation = Rotation.from_matrix(offset[:3, :3]).as_quat()  # x, y, z, w
        self.tf.publish(
            Transform(
                translation=Vector3(
                    float(translation[0]), float(translation[1]), float(translation[2])
                ),
                rotation=Quaternion(
                    float(rotation[0]), float(rotation[1]), float(rotation[2]), float(rotation[3])
                ),
                frame_id=self.config.map_frame,
                child_frame_id=self.config.local_frame,
                ts=ts,
            )
        )

    def _maybe_publish_map(self, ts: float) -> None:
        if self._map_published or self._refine_target is None:
            return
        # Only publish the prior map once (or once per swap) for visualization.
        points = np.asarray(self._refine_target.points, dtype=np.float32)
        if points.size == 0:
            return
        self.map_cloud.publish(
            PointCloud2.from_numpy(points, frame_id=self.config.map_frame, timestamp=ts)
        )
        self._map_published = True
