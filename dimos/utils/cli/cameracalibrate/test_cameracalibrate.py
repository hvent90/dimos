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

import os
import tempfile

import cv2
import numpy as np

from dimos.perception.common.utils import load_camera_info, load_camera_info_opencv
from dimos.utils.cli.cameracalibrate.cameracalibrate import (
    calibrate_from_frames,
    find_chessboard_corners,
    main,
    write_camera_info_yaml,
)


def _synthetic_chessboard_gray(
    width: int,
    height: int,
    cols: int,
    rows: int,
    square_px: int,
) -> np.ndarray:
    """Build a binary chessboard; ``cols`` x ``rows`` inner corners need ``cols+1`` x ``rows+1`` squares."""
    img = np.full((height, width), 255, dtype=np.uint8)
    board_w = (cols + 1) * square_px
    board_h = (rows + 1) * square_px
    ox = (width - board_w) // 2
    oy = (height - board_h) // 2
    for yi in range(rows + 1):
        for xi in range(cols + 1):
            color = 0 if (xi + yi) % 2 == 0 else 255
            x0 = ox + xi * square_px
            y0 = oy + yi * square_px
            cv2.rectangle(
                img,
                (x0, y0),
                (x0 + square_px - 1, y0 + square_px - 1),
                int(color),
                thickness=-1,
            )
    return img


def test_main_stub_runs() -> None:
    main()


def test_find_chessboard_corners_synthetic_board_returns_expected_count() -> None:
    cols, rows = 9, 6
    gray = _synthetic_chessboard_gray(640, 480, cols, rows, square_px=40)
    corners = find_chessboard_corners(gray, cols, rows)
    assert corners is not None
    assert corners.shape == (cols * rows, 1, 2)


def test_calibrate_from_frames_recovers_K_within_five_percent_on_synthetic_views() -> None:
    """Synthetic views from a known intrinsics matrix; recovered ``K`` within 5% (T3.5)."""
    cols, rows = 9, 6
    width, height = 640, 480
    square_size_m = 0.025
    square_px = 40

    # Zero skew; comparing skew ratio vs near-zero truth is ill-conditioned (T3.5 checks fx, fy, cx, cy).
    K_true = np.array(
        [[512.0, 0.0, 318.5], [0.0, 508.0, 242.3], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    D_zero = np.zeros(5, dtype=np.float64)

    gray_flat = _synthetic_chessboard_gray(width, height, cols, rows, square_px=square_px)
    corners_flat = find_chessboard_corners(gray_flat, cols, rows)
    assert corners_flat is not None
    src = corners_flat.reshape(-1, 2).astype(np.float32)

    objp = np.zeros((rows * cols, 3), dtype=np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2).astype(np.float32)
    objp *= float(square_size_m)

    rng = np.random.default_rng(42)
    frames: list[np.ndarray] = []
    for _ in range(18):
        rvec = rng.uniform(-0.22, 0.22, size=3).astype(np.float64)
        tvec = np.array(
            [
                rng.uniform(-0.04, 0.04),
                rng.uniform(-0.04, 0.04),
                rng.uniform(0.38, 0.52),
            ],
            dtype=np.float64,
        )
        imgpts, _ = cv2.projectPoints(objp, rvec, tvec, K_true, D_zero)
        dst = imgpts.reshape(-1, 2).astype(np.float32)
        H, _ = cv2.findHomography(src, dst, cv2.RANSAC, 2.0)
        if H is None:
            continue
        warped = cv2.warpPerspective(gray_flat, H, (width, height))
        corners_w = find_chessboard_corners(warped, cols, rows)
        if corners_w is not None:
            frames.append(warped)

    assert len(frames) >= 8

    out = calibrate_from_frames(frames, cols, rows, square_size_m)
    assert out["n_used"] == len(frames)
    assert out["image_size"] == (width, height)
    assert isinstance(out["rms"], float)

    K_est = np.asarray(out["K"], dtype=np.float64).reshape(3, 3)
    denom = np.maximum(np.abs(K_true), 1e-9)
    rel = np.abs(K_est - K_true) / denom
    assert np.all(rel < 0.05)

def test_write_camera_info_yaml_round_trip_matches_k_d_size_and_model() -> None:
    K = np.array([[500.0, 0.0, 320.0], [0.0, 510.0, 240.0], [0.0, 0.0, 1.0]])
    D = np.array([-0.1, 0.05, 0.0, 0.0, 0.0])
    with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False) as f:
        path = f.name
    try:
        write_camera_info_yaml(
            path,
            image_width=640,
            image_height=480,
            camera_name="test_cam",
            frame_id="camera_optical_frame",
            K=K,
            D=D,
            distortion_model="plumb_bob",
        )
        info = load_camera_info(path, frame_id="camera_link")
        assert info.width == 640
        assert info.height == 480
        assert info.distortion_model == "plumb_bob"
        assert np.allclose(np.asarray(info.K, dtype=np.float64).reshape(3, 3), K)
        assert np.allclose(np.asarray(info.D, dtype=np.float64).ravel(), D.ravel())
    finally:
        os.unlink(path)


def test_write_camera_info_yaml_round_trip_load_camera_info_and_opencv() -> None:
    """YAML written by ``write_camera_info_yaml`` round-trips through both loaders (T3.3)."""
    K = np.array([[600.0, 0.5, 400.0], [0.0, 605.0, 300.5], [0.0, 0.0, 1.0]], dtype=np.float64)
    D = np.array([-0.12, 0.08, 0.002, -0.001, 0.0], dtype=np.float64)
    R = np.array([[0.999, -0.01, 0.0], [0.01, 0.999, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)
    P = np.array(
        [[600.0, 0.0, 400.0, 0.1], [0.0, 605.0, 300.5, 0.2], [0.0, 0.0, 1.0, 0.0]],
        dtype=np.float64,
    )
    with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False) as f:
        path = f.name
    try:
        write_camera_info_yaml(
            path,
            image_width=800,
            image_height=600,
            camera_name="synthetic",
            frame_id="camera_optical",
            K=K,
            D=D,
            R=R,
            P=P,
            distortion_model="plumb_bob",
        )
        info = load_camera_info(path, frame_id="camera_optical")
        K_cv, D_cv = load_camera_info_opencv(path)

        assert info.width == 800
        assert info.height == 600
        assert info.distortion_model == "plumb_bob"
        assert info.header.frame_id == "camera_optical"
        assert np.allclose(np.asarray(info.K, dtype=np.float64).reshape(3, 3), K)
        assert np.allclose(np.asarray(info.D, dtype=np.float64).ravel(), D.ravel())
        assert np.allclose(np.asarray(info.R, dtype=np.float64).reshape(3, 3), R)
        assert np.allclose(np.asarray(info.P, dtype=np.float64).reshape(3, 4), P)
        assert np.allclose(K_cv, K)
        assert np.allclose(np.asarray(D_cv, dtype=np.float64).ravel(), D.ravel())
    finally:
        os.unlink(path)


def test_write_camera_info_yaml_custom_r_p_and_distortion_model() -> None:
    K = np.array([[400.0, 1.0, 160.0], [0.0, 401.0, 120.0], [0.0, 0.0, 1.0]])
    D = np.array([-0.05, 0.02, 0.001, -0.0005])
    R = np.eye(3)
    P = np.array([[400.0, 0.0, 160.0, 0.01], [0.0, 401.0, 120.0, 0.02], [0.0, 0.0, 1.0, 0.0]])
    with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False) as f:
        path = f.name
    try:
        write_camera_info_yaml(
            path,
            image_width=320,
            image_height=240,
            camera_name="narrow",
            frame_id="cam0",
            K=K,
            D=D,
            R=R,
            P=P,
            distortion_model="rational_polynomial",
        )
        info = load_camera_info(path)
        assert info.width == 320
        assert info.height == 240
        assert info.distortion_model == "rational_polynomial"
        assert np.allclose(np.asarray(info.K, dtype=np.float64).reshape(3, 3), K)
        assert np.allclose(np.asarray(info.D, dtype=np.float64).ravel(), D.ravel())
        assert np.allclose(np.asarray(info.R, dtype=np.float64).reshape(3, 3), R)
        assert np.allclose(np.asarray(info.P, dtype=np.float64).reshape(3, 4), P)
    finally:
        os.unlink(path)
