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

"""ArUco / AprilTag detection as a memory2 transformer.

Wraps the pure helpers in :mod:`dimos.perception.fiducial.marker_tf_module`
and emits one :class:`Detection3DMarker` observation per detected marker, with
``.pose`` composed into world frame from the upstream observation's
camera-in-world pose. The companion module :class:`MarkerTfModule` remains
the right choice for live TF publication; this transformer is for offline /
mem2-stream composition.

Skips frames where the upstream observation has no ``.pose`` (debug log):
without a camera-in-world pose, we can't honor the "always world-frame"
output contract.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

from dimos.memory2.transform import Transformer
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.perception.detection.type.detection3d.marker import Detection3DMarker
from dimos.perception.fiducial.marker_tf_module import (
    camera_info_to_cv_matrices,
    create_aruco_detector,
    estimate_marker_pose,
    rvec_tvec_to_transform,
)
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from collections.abc import Iterator

    from dimos.memory2.type.observation import Observation

logger = setup_logger()


def _pose_tuple_to_transform(
    pose: tuple[float, float, float, float, float, float, float],
    *,
    frame_id: str,
    child_frame_id: str,
    ts: float,
) -> Transform:
    x, y, z, qx, qy, qz, qw = pose
    return Transform(
        translation=Vector3(x, y, z),
        rotation=Quaternion(qx, qy, qz, qw),
        frame_id=frame_id,
        child_frame_id=child_frame_id,
        ts=ts,
    )


class DetectMarkers(Transformer[Image, Detection3DMarker]):
    """Detect fiducial markers and emit one world-pose observation per marker."""

    def __init__(
        self,
        camera_info: CameraInfo,
        marker_length_m: float,
        aruco_dictionary: str = "DICT_APRILTAG_36h11",
        world_frame: str = "world",
    ) -> None:
        if marker_length_m <= 0:
            raise ValueError(f"marker_length_m must be > 0, got {marker_length_m}")
        self.camera_info = camera_info
        self.marker_length_m = marker_length_m
        self.aruco_dictionary = aruco_dictionary
        self.world_frame = world_frame
        self._detector = create_aruco_detector(aruco_dictionary)
        self._cam_mtx, self._dist = camera_info_to_cv_matrices(camera_info)

    def __call__(
        self, upstream: Iterator[Observation[Image]]
    ) -> Iterator[Observation[Detection3DMarker]]:
        info = self.camera_info
        marker_size = Vector3(self.marker_length_m, self.marker_length_m, 0.0)

        for obs in upstream:
            if obs.pose is None:
                logger.debug("DetectMarkers: obs %s has no .pose; skipping", obs.id)
                continue

            image = obs.data
            if (
                info.width
                and info.height
                and (image.width != info.width or image.height != info.height)
            ):
                logger.debug(
                    "DetectMarkers: image %sx%s != CameraInfo %sx%s; skip",
                    image.width,
                    image.height,
                    info.width,
                    info.height,
                )
                continue

            gray = image.to_grayscale().as_numpy()
            corners, ids, _ = self._detector.detectMarkers(gray)
            if ids is None or len(ids) == 0:
                continue

            t_world_optical = _pose_tuple_to_transform(
                obs.pose,
                frame_id=self.world_frame,
                child_frame_id="optical",
                ts=obs.ts,
            )

            for corner_set, mid_arr in zip(corners, ids, strict=True):
                mid = int(mid_arr[0])
                pose = estimate_marker_pose(
                    corner_set,
                    self.marker_length_m,
                    self._cam_mtx,
                    self._dist,
                    distortion_model=info.distortion_model,
                )
                if pose is None:
                    continue
                rvec, tvec = pose
                t_optical_marker = rvec_tvec_to_transform(
                    rvec,
                    tvec,
                    frame_id="optical",
                    child_frame_id=f"marker_{mid}",
                    ts=obs.ts,
                )
                t_world_marker = t_world_optical + t_optical_marker

                corners_2d = corner_set.reshape(4, 2).astype(np.float32)
                xy_min = corners_2d.min(axis=0)
                xy_max = corners_2d.max(axis=0)
                bbox = (float(xy_min[0]), float(xy_min[1]), float(xy_max[0]), float(xy_max[1]))

                det = Detection3DMarker(
                    bbox=bbox,
                    track_id=mid,
                    class_id=mid,
                    confidence=1.0,
                    name=f"marker_{mid}",
                    ts=obs.ts,
                    image=image,
                    center=t_world_marker.translation,
                    size=marker_size,
                    transform=t_world_optical,
                    frame_id=self.world_frame,
                    orientation=t_world_marker.rotation,
                    marker_id=mid,
                    corners_px=corners_2d,
                    dictionary=self.aruco_dictionary,
                )

                yield obs.derive(data=det, pose=t_world_marker).tag(marker_id=mid)
