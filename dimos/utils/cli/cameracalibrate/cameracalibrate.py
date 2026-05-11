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

"""Interactive camera calibration for dimos (ROS CameraInfo YAML output).

Placeholder module for the `dimos cameracalibrate` CLI; behavior is filled in by
later T3 subtasks.
"""

from __future__ import annotations

import cv2
import numpy as np
import yaml


def write_camera_info_yaml(
    path: str,
    *,
    image_width: int,
    image_height: int,
    camera_name: str,
    frame_id: str,
    K: np.ndarray,
    D: np.ndarray,
    R: np.ndarray | None = None,
    P: np.ndarray | None = None,
    distortion_model: str = "plumb_bob",
) -> None:
    """Write ROS-style sensor_msgs/CameraInfo YAML for use with ``load_camera_info``.

    ``frame_id`` is part of the keyword API for symmetry with ``load_camera_info``; the
    ROS YAML schema does not store it (pass ``frame_id`` when calling ``load_camera_info``).
    """
    k = np.asarray(K, dtype=np.float64).reshape(3, 3)
    d = np.asarray(D, dtype=np.float64).ravel()
    k_flat = k.ravel(order="C").tolist()
    d_flat = d.tolist()

    if R is None:
        r_flat = np.eye(3, dtype=np.float64).ravel(order="C").tolist()
    else:
        r_flat = np.asarray(R, dtype=np.float64).reshape(3, 3).ravel(order="C").tolist()

    if P is None:
        fx = k_flat[0]
        fy = k_flat[4]
        cx = k_flat[2]
        cy = k_flat[5]
        p_flat = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    else:
        p_flat = np.asarray(P, dtype=np.float64).reshape(3, 4).ravel(order="C").tolist()

    n_dist = len(d_flat)
    payload = {
        "image_width": int(image_width),
        "image_height": int(image_height),
        "camera_name": camera_name,
        "distortion_model": distortion_model,
        "camera_matrix": {"rows": 3, "cols": 3, "data": k_flat},
        "distortion_coefficients": {"rows": 1, "cols": int(n_dist), "data": d_flat},
        "rectification_matrix": {"rows": 3, "cols": 3, "data": r_flat},
        "projection_matrix": {"rows": 3, "cols": 4, "data": p_flat},
    }

    with open(path, "w", encoding="utf-8") as f:
        yaml.safe_dump(payload, f, default_flow_style=False, sort_keys=False)


def find_chessboard_corners(gray: np.ndarray, cols: int, rows: int) -> np.ndarray | None:
    """Detect inner chessboard corners and refine them with sub-pixel accuracy.

    ``cols`` and ``rows`` are the counts of **inner** corners along each axis, matching
    ``cv2.findChessboardCorners(..., patternSize=(cols, rows))``.

    Returns:
        Float array of shape ``(cols * rows, 1, 2)`` on success, else ``None``.
    """
    g = np.asarray(gray)
    pattern_size = (cols, rows)
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
    ok, corners = cv2.findChessboardCorners(g, pattern_size, flags)
    if not ok or corners is None:
        return None

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    refined = cv2.cornerSubPix(g, corners, (11, 11), (-1, -1), criteria)
    return refined


def calibrate_from_frames(
    frames: list[np.ndarray],
    cols: int,
    rows: int,
    square_size_m: float,
) -> dict[str, object]:
    """Calibrate intrinsics from grayscale or BGR frames containing a chessboard.

    Each frame where ``find_chessboard_corners`` succeeds contributes one view.
    All frames must share the same resolution.

    Returns:
        ``{"K", "D", "rms", "image_size", "n_used"}`` with ``K`` (3x3) and ``D`` (1-d),
        ``rms`` reprojection RMSE from OpenCV, ``image_size`` ``(width, height)``, and
        ``n_used`` the number of frames that yielded detections.
    """
    if not frames:
        raise ValueError("frames must be non-empty")

    objp = np.zeros((rows * cols, 3), dtype=np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2).astype(np.float32)
    objp *= float(square_size_m)

    objpoints: list[np.ndarray] = []
    imgpoints: list[np.ndarray] = []

    first = np.asarray(frames[0])
    h0, w0 = first.shape[:2]

    for frame in frames:
        f = np.asarray(frame)
        if f.shape[:2] != (h0, w0):
            raise ValueError("All frames must have the same shape.")
        if f.ndim == 3:
            gray = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)
        else:
            gray = f
        corners = find_chessboard_corners(gray, cols, rows)
        if corners is None:
            continue
        objpoints.append(objp)
        imgpoints.append(corners.astype(np.float32))

    if not objpoints:
        raise ValueError("Chessboard not found in any frame.")

    rms, camera_matrix, dist_coeffs, _rvecs, _tvecs = cv2.calibrateCamera(
        objpoints,
        imgpoints,
        (w0, h0),
        None,
        None,
    )
    K = np.asarray(camera_matrix, dtype=np.float64)
    D = np.asarray(dist_coeffs, dtype=np.float64).reshape(-1)

    return {
        "K": K,
        "D": D,
        "rms": float(rms),
        "image_size": (int(w0), int(h0)),
        "n_used": len(objpoints),
    }


def main() -> None:
    """CLI entry point (stub)."""
    pass
