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

"""Object sightings from color + world-frame lidar + odom recordings.

The depth-based scan lane (``SceneScanner``/``WorldBelief``) needs RGBD
pairs, which go2 recordings don't have. This module lifts 2D detections to
3D by projecting the recording's world-frame lidar into the camera instead
(the ``Detection3DPC.from_2d`` path): for each sampled color frame, the
nearest odom pose plus the static base->camera_optical mount gives the
world->camera extrinsic, and the nearest lidar window supplies the points.
"""

from __future__ import annotations

from collections.abc import Iterator
from dataclasses import dataclass
from typing import Any

import numpy as np

from dimos.memory2.store.base import Store
from dimos.memory2.type.observation import Observation
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.perception.detection.detectors.base import Detector
from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox
from dimos.perception.detection.type.detection3d.pointcloud import Detection3DPC
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

COLOR_STREAMS = ["color_image"]
LIDAR_STREAMS = ["go2_lidar", "lidar"]
ODOM_STREAMS = ["go2_odom", "odom"]


@dataclass(frozen=True)
class LidarSighting:
    """One 3D-positioned object observation from a single frame."""

    name: str
    ts: float
    position: tuple[float, float, float]  # world frame
    confidence: float
    track_id: int  # -1 when the detector has no track for it
    n_points: int  # lidar points supporting the 3D position
    # World AABB (x_min, y_min, z_min, x_max, y_max, z_max) of the supporting
    # points — the visible portion of the object, not its full footprint.
    extent: tuple[float, float, float, float, float, float] | None = None


@dataclass
class LidarScanFrame:
    """Per-frame scan output, with intermediates kept for debug rendering."""

    ts: float
    image: Image
    robot_xy: tuple[float, float]
    world_to_optical: Transform
    lidar: PointCloud2
    detections_2d: list[Detection2DBBox]
    sightings: list[LidarSighting]


def _pick_stream(store: Store, candidates: list[str]) -> str:
    available = store.list_streams()
    name = next((s for s in candidates if s in available), None)
    if name is None:
        raise LookupError(f"None of streams {candidates} in store; found {available}")
    return name


def _nearest(stream_obs: list[Observation[Any]], ts: float) -> Observation[Any]:
    return min(stream_obs, key=lambda o: abs(o.ts - ts))


def iter_lidar_scan(
    store: Store,
    detector: Detector,
    camera_info: CameraInfo,
    base_to_optical: Transform,
    *,
    sample_period_s: float = 0.5,
    odom_tolerance_s: float = 0.15,
    lidar_tolerance_s: float = 2.0,
    time_range: tuple[float, float] | None = None,
) -> Iterator[LidarScanFrame]:
    """Scan a recording, yielding per-frame 3D-positioned detections.

    Args:
        store: memory2 store holding color/lidar/odom streams (go2 naming).
        detector: 2D detector; prompt it before calling for open-vocabulary.
        camera_info: intrinsics of the color camera.
        base_to_optical: static base_link -> camera_optical mount transform.
        sample_period_s: min spacing between processed color frames.
        odom_tolerance_s: max |dt| between a frame and its odom pose; frames
            without a close pose are skipped.
        lidar_tolerance_s: max |dt| between a frame and its lidar window.
        time_range: optional (t0, t1) window of frame timestamps to scan.
    """
    color_name = _pick_stream(store, COLOR_STREAMS)
    lidar_name = _pick_stream(store, LIDAR_STREAMS)
    odom_name = _pick_stream(store, ODOM_STREAMS)

    odom_obs: list[Observation[Any]] = store.stream(odom_name).order_by("ts").to_list()
    if not odom_obs:
        raise LookupError(f"Stream {odom_name!r} is empty")

    color = store.stream(color_name, Image).order_by("ts")
    if time_range is not None:
        color = color.time_range(*time_range)

    last_ts: float | None = None
    for obs in color:
        if last_ts is not None and obs.ts - last_ts < sample_period_s:
            continue
        nearest_odom = _nearest(odom_obs, obs.ts)
        if abs(nearest_odom.ts - obs.ts) > odom_tolerance_s:
            continue
        lidar_candidates = store.stream(lidar_name, PointCloud2).at(
            obs.ts, tolerance=lidar_tolerance_s
        )
        lidar_list = lidar_candidates.to_list()
        if not lidar_list:
            continue
        lidar_obs = _nearest(lidar_list, obs.ts)
        last_ts = obs.ts

        pose = nearest_odom.pose_stamped
        if pose is None:
            pose = nearest_odom.data
        pose.frame_id = "world"
        world_from_base = Transform.from_pose("base_link", pose)
        world_to_optical = (world_from_base + base_to_optical).inverse()

        image: Image = obs.data
        detections = detector.process_image(image)
        lidar_pc: PointCloud2 = lidar_obs.data
        sightings: list[LidarSighting] = []
        for det in detections.detections:
            det3d = Detection3DPC.from_2d(
                det,
                world_pointcloud=lidar_pc,
                camera_info=camera_info,
                world_to_optical_transform=world_to_optical,
            )
            if det3d is None:
                continue
            center = det3d.center
            pts, _ = det3d.pointcloud.as_numpy()
            mins, maxs = pts.min(axis=0), pts.max(axis=0)
            sightings.append(
                LidarSighting(
                    name=det.name,
                    ts=obs.ts,
                    position=(float(center.x), float(center.y), float(center.z)),
                    confidence=float(det.confidence),
                    track_id=int(det.track_id) if det.track_id is not None else -1,
                    n_points=len(det3d.pointcloud),
                    extent=(
                        float(mins[0]),
                        float(mins[1]),
                        float(mins[2]),
                        float(maxs[0]),
                        float(maxs[1]),
                        float(maxs[2]),
                    ),
                )
            )
        yield LidarScanFrame(
            ts=obs.ts,
            image=image,
            robot_xy=(float(pose.position.x), float(pose.position.y)),
            world_to_optical=world_to_optical,
            lidar=lidar_pc,
            detections_2d=list(detections.detections),
            sightings=sightings,
        )


def project_points(
    points_world: np.ndarray,
    world_to_optical: Transform,
    camera_info: CameraInfo,
) -> tuple[np.ndarray, np.ndarray]:
    """Project world points into the image; returns (uv, depth) for visible ones.

    Debug-render helper matching ``Detection3DPC.from_2d`` geometry (pinhole,
    distortion ignored).
    """
    fx, fy = camera_info.K[0], camera_info.K[4]
    cx, cy = camera_info.K[2], camera_info.K[5]
    m = world_to_optical.to_matrix()
    cam = (m @ np.hstack([points_world, np.ones((len(points_world), 1))]).T).T
    in_front = cam[:, 2] > 0
    cam = cam[in_front]
    u = cam[:, 0] / cam[:, 2] * fx + cx
    v = cam[:, 1] / cam[:, 2] * fy + cy
    visible = (u >= 0) & (u < camera_info.width) & (v >= 0) & (v < camera_info.height)
    uv = np.stack([u[visible], v[visible]], axis=1)
    return uv, cam[visible, 2]
