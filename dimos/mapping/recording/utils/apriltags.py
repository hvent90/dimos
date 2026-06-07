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

Detects tags in the `color_image` stream and rejects bad glimpses through several
independent gates — motion blur (per-tag sharpness), PnP misfit (reprojection
error), too-small / too-far / too-oblique views, and fast camera motion — then
clusters same-id detections in time and reduces each cluster to one robust pose
via a Huber-weighted refinement seeded at the cluster medoid. (Re)writes the
`april_tags` PoseStamped stream of tag-in-camera relative poses (solvePnP,
marker_id in each observation's tags) and returns those representatives for the
GTSAM solve.
"""

from __future__ import annotations

from collections import defaultdict
from itertools import pairwise
import math
from typing import Any

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.Image import Image

DEFAULT_MAX_DISTANCE_M = 1.0
DEFAULT_MAX_VIEW_ANGLE_DEG = 45.0
DEFAULT_CLUSTER_GAP_SEC = 5.0
DEFAULT_ROTATION_WEIGHT_M_PER_RAD = 0.5
# A blurry tag still solves a pose; these reject the bad glimpses up front.
DEFAULT_MIN_SHARPNESS = 60.0  # Laplacian variance over the tag ROI
DEFAULT_MAX_REPROJ_PX = 2.0  # RMS solvePnP corner reprojection error
DEFAULT_MIN_TAG_PX = 24.0  # tag side length in pixels (sqrt of quad area)
DEFAULT_MAX_LINEAR_SPEED_MPS = 0.5
DEFAULT_MAX_ANGULAR_SPEED_DPS = 50.0
DEFAULT_MIN_OBSERVATIONS = 3  # clusters thinner than this are unreliable
DEFAULT_HUBER_DELTA_M = 0.05  # residual past which a sample is down-weighted
_HUBER_ITERATIONS = 5


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


def reprojection_error_px(
    corners_pixels, rotation_vector, translation_vector, marker_length_m, intrinsics, distortion
) -> float:
    """RMS pixel distance between detected corners and the solvePnP pose reprojected
    back onto the image — a direct measure of how well the pose explains the tag."""
    projected, _ = cv2.projectPoints(
        _object_points(marker_length_m), rotation_vector, translation_vector, intrinsics, distortion
    )
    measured = corners_pixels.reshape(4, 2).astype(np.float64)
    diff = projected.reshape(4, 2) - measured
    return float(np.sqrt(np.mean(np.sum(diff * diff, axis=1))))


def tag_pixel_size(corners_pixels) -> float:
    """Tag side length in pixels (sqrt of the quad's image area); small = unreliable."""
    quad = corners_pixels.reshape(4, 2).astype(np.float32)
    return float(math.sqrt(abs(cv2.contourArea(quad))))


def tag_sharpness(gray: np.ndarray, corners_pixels) -> float:
    """Laplacian variance over the tag's bounding box — low under motion blur."""
    quad = corners_pixels.reshape(4, 2)
    x_min, y_min = np.floor(quad.min(0)).astype(int)
    x_max, y_max = np.ceil(quad.max(0)).astype(int)
    height, width = gray.shape[:2]
    x_min, y_min = max(int(x_min), 0), max(int(y_min), 0)
    x_max, y_max = min(int(x_max), width), min(int(y_max), height)
    if x_max - x_min < 2 or y_max - y_min < 2:
        return 0.0
    return float(cv2.Laplacian(gray[y_min:y_max, x_min:x_max], cv2.CV_64F).var())


def _pose_xyz_quat(pose: Any) -> np.ndarray | None:
    """Best-effort (x,y,z,qx,qy,qz,qw) from an observation pose (tuple/list or a msg
    with .position/.orientation); None if it isn't a usable pose."""
    if pose is None:
        return None
    if hasattr(pose, "position") and hasattr(pose, "orientation"):
        position, orientation = pose.position, pose.orientation
        return np.array(
            [
                position.x,
                position.y,
                position.z,
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w,
            ]
        )
    try:
        values = [float(component) for component in pose]
    except TypeError:
        return None
    return np.array(values[:7]) if len(values) >= 7 else None


def _camera_speeds(images: list[Any]) -> tuple[dict[float, tuple[float, float]], bool]:
    """Per-image (linear m/s, angular deg/s) from consecutive posed frames. The second
    return is False when too few frames carry poses to estimate motion (gate disabled)."""
    posed = [
        (float(obs.ts), pose)
        for obs in images
        if (pose := _pose_xyz_quat(getattr(obs, "pose", None))) is not None
    ]
    posed.sort(key=lambda item: item[0])
    speeds: dict[float, tuple[float, float]] = {}
    for (timestamp_a, pose_a), (timestamp_b, pose_b) in pairwise(posed):
        dt = timestamp_b - timestamp_a
        if dt <= 0:
            continue
        linear = float(np.linalg.norm(pose_b[:3] - pose_a[:3])) / dt
        cos_half = min(1.0, abs(float(np.dot(pose_a[3:7], pose_b[3:7]))))
        angular = math.degrees(2.0 * math.acos(cos_half)) / dt
        speeds[timestamp_b] = (linear, angular)
    return speeds, len(posed) >= 2


def _huber_weights(residuals: np.ndarray, delta: float) -> np.ndarray:
    """IRLS Huber weights: 1 inside `delta`, decaying as delta/r past it."""
    weights = np.ones_like(residuals)
    outside = residuals > delta
    weights[outside] = delta / residuals[outside]
    return weights


def robust_cluster_pose(
    cluster: list[dict], rotation_weight_m_per_rad: float, huber_delta_m: float
) -> dict:
    """Cluster representative: the medoid, then refined by Huber-weighted IRLS — a
    weighted-mean translation and weighted quaternion mean (Markley eigen method),
    re-weighting each iteration so a lingering bad glimpse keeps losing influence."""
    medoid = cluster_medoid(cluster, rotation_weight_m_per_rad)
    if len(cluster) < 2:
        return medoid
    poses = np.array([detection["t_cam_marker"] for detection in cluster], dtype=np.float64)
    translations, quaternions = poses[:, :3], poses[:, 3:7]
    reference = np.array(medoid["t_cam_marker"][3:7])
    signs = np.sign(quaternions @ reference)
    signs[signs == 0] = 1.0
    quaternions = quaternions * signs[:, None]
    estimate_translation = np.array(medoid["t_cam_marker"][:3])
    estimate_quaternion = reference.copy()
    delta_rad = huber_delta_m / max(rotation_weight_m_per_rad, 1e-9)
    for _ in range(_HUBER_ITERATIONS):
        weights_t = _huber_weights(
            np.linalg.norm(translations - estimate_translation, axis=1), huber_delta_m
        )
        estimate_translation = (weights_t[:, None] * translations).sum(0) / weights_t.sum()
        angular_residual = 2.0 * np.arccos(
            np.clip(np.abs(quaternions @ estimate_quaternion), 0.0, 1.0)
        )
        weights_r = _huber_weights(angular_residual, delta_rad)
        scatter = (
            weights_r[:, None, None] * np.einsum("ni,nj->nij", quaternions, quaternions)
        ).sum(0)
        estimate_quaternion = np.linalg.eigh(scatter)[1][:, -1]
        if estimate_quaternion @ reference < 0:
            estimate_quaternion = -estimate_quaternion
    return {
        **medoid,
        "t_cam_marker": [*estimate_translation.tolist(), *estimate_quaternion.tolist()],
    }


def detect_apriltags(
    store,
    intrinsics,
    distortion,
    image_stream="color_image",
    stream_name="april_tags",
    marker_length=0.10,
    dictionary="DICT_APRILTAG_36h11",
    *,
    max_distance_m=DEFAULT_MAX_DISTANCE_M,
    max_view_angle_deg=DEFAULT_MAX_VIEW_ANGLE_DEG,
    cluster_gap_sec=DEFAULT_CLUSTER_GAP_SEC,
    rotation_weight_m_per_rad=DEFAULT_ROTATION_WEIGHT_M_PER_RAD,
    min_sharpness=DEFAULT_MIN_SHARPNESS,
    max_reproj_px=DEFAULT_MAX_REPROJ_PX,
    min_tag_px=DEFAULT_MIN_TAG_PX,
    max_linear_speed_mps=DEFAULT_MAX_LINEAR_SPEED_MPS,
    max_angular_speed_dps=DEFAULT_MAX_ANGULAR_SPEED_DPS,
    min_observations=DEFAULT_MIN_OBSERVATIONS,
    huber_delta_m=DEFAULT_HUBER_DELTA_M,
):
    """Detect tags in `image_stream`, reject bad glimpses (blur, PnP misfit, small/
    far/oblique views, fast motion), cluster same-id detections by time, drop thin
    clusters, and (re)write the `april_tags` stream from one Huber-refined medoid
    representative per cluster. Returns that list of representatives."""
    detector = make_detector(dictionary)
    raw_detections: list[dict] = []
    images = store.stream(image_stream, Image).to_list()
    speed_by_ts, speed_available = _camera_speeds(images)
    for image_obs in images:
        image = image_obs.data
        bgr = image.numpy() if hasattr(image, "numpy") else np.asarray(image.data)
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY) if bgr.ndim == 3 else bgr
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
                    "sharpness": tag_sharpness(gray, corners),
                    "reproj_px": reprojection_error_px(
                        corners,
                        rotation_vector,
                        translation_vector,
                        marker_length,
                        intrinsics,
                        distortion,
                    ),
                    "tag_px": tag_pixel_size(corners),
                    "speed": speed_by_ts.get(float(image_obs.ts)),
                }
            )

    # Per-glimpse gates; count rejections per reason so thresholds are tunable.
    rejected = defaultdict(int)
    kept = []
    for detection in raw_detections:
        if detection["sharpness"] < min_sharpness:
            rejected["blur"] += 1
            continue
        if detection["reproj_px"] > max_reproj_px:
            rejected["reproj"] += 1
            continue
        if detection["tag_px"] < min_tag_px:
            rejected["small"] += 1
            continue
        distance, view_angle = view_quality(detection["t_cam_marker"])
        if distance > max_distance_m:
            rejected["far"] += 1
            continue
        if view_angle > max_view_angle_deg:
            rejected["oblique"] += 1
            continue
        speed = detection["speed"]
        if speed is not None and (
            speed[0] > max_linear_speed_mps or speed[1] > max_angular_speed_dps
        ):
            rejected["motion"] += 1
            continue
        kept.append(detection)

    # One Huber-refined representative per time-clustered group; drop thin clusters.
    detections = []
    thin_clusters = 0
    for cluster in cluster_by_time(kept, cluster_gap_sec):
        if len(cluster) < min_observations:
            thin_clusters += 1
            continue
        detections.append(
            {
                **robust_cluster_pose(cluster, rotation_weight_m_per_rad, huber_delta_m),
                "n_observations": len(cluster),
            }
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
    gate_summary = ", ".join(f"{reason}={count}" for reason, count in sorted(rejected.items()))
    if not speed_available:
        gate_summary += (", " if gate_summary else "") + "motion-gate-off(no poses)"
    print(
        f"   april_tags: {len(raw_detections)} raw -> {len(kept)} in-spec "
        f"-> {len(detections)} clusters (dropped {thin_clusters} thin), "
        f"markers {found_ids} (over {len(images)} images)"
    )
    if gate_summary:
        print(f"   april_tags rejected: {gate_summary}")
    return detections
