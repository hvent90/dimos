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

"""AprilTag detection over a mem2.db color stream.

Detects tags in the `color_image` stream, keeps only close head-on views, and
clusters same-id detections in time into a single medoid representative each.
(Re)writes the `april_tags` PoseStamped stream of tag-in-camera relative poses
(solvePnP, marker_id in each observation's tags) and returns those
representatives for the GTSAM solve.
"""

from __future__ import annotations

from collections import defaultdict
import math

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.Image import Image


def make_detector(dictionary_name: str):
    d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary_name))
    return cv2.aruco.ArucoDetector(d, cv2.aruco.DetectorParameters())


def _object_points(marker_length_m: float) -> np.ndarray:
    h = marker_length_m / 2.0
    return np.array([[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]], dtype=np.float32)


def estimate_marker_pose(corners_pixels, marker_length_m, intrinsics, distortion):
    """solvePnP a single tag -> (rotation_vector, translation_vector) in the
    camera_optical frame, or None if it failed."""
    image_corners = corners_pixels.reshape(4, 1, 2).astype(np.float32)
    found, rotation_vector, translation_vector = cv2.solvePnP(
        _object_points(marker_length_m),
        image_corners,
        intrinsics,
        distortion,
        flags=cv2.SOLVEPNP_IPPE_SQUARE,
    )
    return (rotation_vector, translation_vector) if found else None


def view_quality(t_cam_marker: list[float]) -> tuple[float, float]:
    """(distance_m, view_angle_deg) for a tag pose in the camera optical frame.

    distance is the camera->tag range; view_angle is the angle between the line
    of sight and the tag's surface normal (0 = perfectly head-on)."""
    translation = np.array(t_cam_marker[:3], dtype=np.float64)
    distance = float(np.linalg.norm(translation))
    normal = Rotation.from_quat(t_cam_marker[3:7]).as_matrix()[:, 2]
    line_of_sight = translation / (distance + 1e-9)
    cos_angle = abs(float(np.dot(line_of_sight, normal)))
    view_angle = math.degrees(math.acos(min(1.0, cos_angle)))
    return distance, view_angle


def cluster_by_time(detections: list[dict], gap_sec: float) -> list[list[dict]]:
    """Group same-marker detections into clusters. A new cluster begins whenever
    the time gap to the previous same-marker detection exceeds gap_sec."""
    clusters: list[list[dict]] = []
    by_marker: dict[int, list[dict]] = defaultdict(list)
    for detection in detections:
        by_marker[detection["marker_id"]].append(detection)
    for marker_detections in by_marker.values():
        marker_detections.sort(key=lambda detection: detection["ts"])
        current = [marker_detections[0]]
        for detection in marker_detections[1:]:
            if detection["ts"] - current[-1]["ts"] > gap_sec:
                clusters.append(current)
                current = [detection]
            else:
                current.append(detection)
        clusters.append(current)
    return clusters


def _pose_distance(a: list[float], b: list[float], rotation_weight_m_per_rad: float) -> float:
    translation = float(np.linalg.norm(np.array(a[:3]) - np.array(b[:3])))
    rotation = 2.0 * math.acos(min(1.0, abs(float(np.dot(a[3:7], b[3:7])))))
    return translation + rotation_weight_m_per_rad * rotation


def cluster_medoid(cluster: list[dict], rotation_weight_m_per_rad: float) -> dict:
    """The detection whose pose is most central (min total spatial+rotational
    distance to the rest) — a robust representative of the cluster."""
    poses = [detection["t_cam_marker"] for detection in cluster]
    best_index, best_cost = 0, float("inf")
    for i in range(len(poses)):
        cost = sum(
            _pose_distance(poses[i], poses[j], rotation_weight_m_per_rad)
            for j in range(len(poses))
            if j != i
        )
        if cost < best_cost:
            best_cost, best_index = cost, i
    return cluster[best_index]


def detect_apriltags(
    store,
    intrinsics,
    distortion,
    image_stream="color_image",
    stream_name="april_tags",
    marker_length=0.10,
    dictionary="DICT_APRILTAG_36h11",
    *,
    max_distance_m=1.0,
    max_view_angle_deg=45.0,
    cluster_gap_sec=5.0,
    rotation_weight_m_per_rad=0.5,
):
    """Detect tags in `image_stream`, keep only close, head-on views, cluster
    same-id detections by time, and (re)write the `april_tags` stream from one
    medoid representative per cluster. Returns that list of representatives."""
    detector = make_detector(dictionary)
    raw_detections: list[dict] = []
    image_count = 0
    for image_obs in store.stream(image_stream, Image):
        image_count += 1
        image = image_obs.data
        bgr = image.numpy() if hasattr(image, "numpy") else np.asarray(image.data)
        all_corners, marker_ids, _ = detector.detectMarkers(bgr)
        if marker_ids is None:
            continue
        for corners, marker_id in zip(all_corners, marker_ids.flatten(), strict=False):
            pose = estimate_marker_pose(corners, marker_length, intrinsics, distortion)
            if pose is None:
                continue
            rotation_vector, translation_vector = pose
            quaternion = Rotation.from_rotvec(rotation_vector.reshape(3)).as_quat()  # x,y,z,w
            translation = translation_vector.reshape(3)
            tag_in_camera = [
                float(translation[0]),
                float(translation[1]),
                float(translation[2]),
                float(quaternion[0]),
                float(quaternion[1]),
                float(quaternion[2]),
                float(quaternion[3]),
            ]
            raw_detections.append(
                {
                    "ts": float(image_obs.ts),
                    "marker_id": int(marker_id),
                    "t_cam_marker": tag_in_camera,
                }
            )

    # Quality gate: drop distant or oblique glimpses, keep close head-on views.
    kept = []
    for detection in raw_detections:
        distance, view_angle = view_quality(detection["t_cam_marker"])
        if distance <= max_distance_m and view_angle <= max_view_angle_deg:
            kept.append(detection)

    # One representative (medoid) per time-clustered group of same-id detections.
    detections = []
    for cluster in cluster_by_time(kept, cluster_gap_sec):
        detections.append(
            {**cluster_medoid(cluster, rotation_weight_m_per_rad), "n_observations": len(cluster)}
        )
    detections.sort(key=lambda detection: detection["ts"])

    if stream_name in store.list_streams():
        store.delete_stream(stream_name)
    april_tag_stream = store.stream(stream_name, PoseStamped)
    for detection in detections:
        tag_in_camera = detection["t_cam_marker"]
        pose_stamped = PoseStamped(
            ts=detection["ts"], position=tag_in_camera[:3], orientation=tag_in_camera[3:]
        )
        april_tag_stream.append(
            pose_stamped,
            ts=detection["ts"],
            pose=tuple(tag_in_camera),
            tags={"marker_id": detection["marker_id"]},
        )

    found_ids = sorted({detection["marker_id"] for detection in detections})
    print(
        f"   april_tags: {len(raw_detections)} raw -> {len(kept)} in-spec "
        f"-> {len(detections)} clusters, markers {found_ids} (over {image_count} images)"
    )
    return detections
