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

from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np
import pytest

pytest.importorskip("cv2.aruco")

from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.perception.fiducial.fixture_verification import (
    BoardLayout,
    FrameVerificationResult,
    apparent_scale_bin,
    board_completeness_class,
    generated_apriltag_board_layout,
    image_footprint_bin,
    load_manifest,
    median_tag_edge_percent,
    validate_detection_expectation,
    verify_board_layout_geometry,
    verify_fixture_frame,
    visible_board_layout_area_percent,
    visible_image_hull_area_percent,
)
from dimos.perception.fiducial.marker_tf_module import MarkerTfModule

ROOT = Path(__file__).resolve().parents[3]
MANIFEST_PATH = (
    ROOT / "dimos/perception/fiducial/blueprints/fixtures/apriltag_fixture_manifest.yaml"
)


@pytest.fixture(scope="module")
def manifest() -> dict[str, Any]:
    return load_manifest(MANIFEST_PATH)


@pytest.fixture(scope="module")
def layout() -> BoardLayout:
    return generated_apriltag_board_layout(list(range(12)), marker_length_m=0.05, page_size="a4")


@pytest.fixture(scope="module")
def fixture_results(
    manifest: dict[str, Any],
    layout: BoardLayout,
) -> list[FrameVerificationResult]:
    return [
        verify_fixture_frame(frame, repo_root=ROOT, manifest=manifest, layout=layout)
        for frame in manifest["frames"]
    ]


def test_fixture_layout_matches_dimos_apriltag_generator_for_a4_50mm_3x4(
    layout: BoardLayout,
) -> None:
    assert (layout.cols, layout.rows) == (3, 4)
    expected_bottom_left_mm = {
        0: (20.0, 218.8),
        1: (80.0, 218.8),
        2: (140.0, 218.8),
        3: (20.0, 159.6),
        4: (80.0, 159.6),
        5: (140.0, 159.6),
        6: (20.0, 100.4),
        7: (80.0, 100.4),
        8: (140.0, 100.4),
        9: (20.0, 41.2),
        10: (80.0, 41.2),
        11: (140.0, 41.2),
    }
    for tag_id, (x_mm, y_mm) in expected_bottom_left_mm.items():
        tag = layout.tags[tag_id]
        assert (tag.col, tag.row) == (tag_id % 3, tag_id // 3)
        np.testing.assert_allclose(tag.bottom_left_m[:2] * 1000.0, [x_mm, y_mm], atol=0.05)
        np.testing.assert_allclose(
            tag.center_m[:2] * 1000.0,
            [x_mm + 25.0, y_mm + 25.0],
            atol=0.05,
        )
        assert tag.corners_m.shape == (4, 3)


def test_all_manifest_frames_run_opencv_detection_and_match_visibility(
    manifest: dict[str, Any],
    fixture_results: list[FrameVerificationResult],
) -> None:
    failures: list[str] = []
    for frame, result in zip(manifest["frames"], fixture_results, strict=True):
        if result.metrics.image_width_px != manifest["camera"]["image_width_px"]:
            failures.append(
                f"{result.frame_id}: image width {result.metrics.image_width_px} "
                f"!= manifest {manifest['camera']['image_width_px']}"
            )
        if result.metrics.image_height_px != manifest["camera"]["image_height_px"]:
            failures.append(
                f"{result.frame_id}: image height {result.metrics.image_height_px} "
                f"!= manifest {manifest['camera']['image_height_px']}"
            )
        try:
            validate_detection_expectation(frame, result.detected_ids)
        except ValueError as exc:
            failures.append(f"{result.frame_id}: detected {result.detected_ids}: {exc}")

        metric_values = [
            result.metrics.median_tag_edge_percent,
            result.metrics.visible_image_hull_area_percent,
            result.metrics.visible_board_layout_area_percent,
            result.metrics.board_layout_error_px_p50,
            result.metrics.board_layout_error_px_p95,
        ]
        if not all(np.isfinite(value) for value in metric_values):
            failures.append(f"{result.frame_id}: non-finite detector-derived metrics")

        if frame["operator_planned_class"] == "none":
            if result.accepted:
                failures.append(f"{result.frame_id}: negative fixture row was accepted")
        elif not result.accepted:
            failures.append(f"{result.frame_id}: rejected fixture row: {result.reject_reasons}")

    assert not failures, "Detector-derived frame verification failed:\n" + "\n".join(failures)


def test_apparent_scale_bins_use_normalized_tag_edge_percent() -> None:
    corners_by_id = {0: _square_corners(10.0, 10.0, 80.0)}
    assert median_tag_edge_percent(corners_by_id, (1000, 500)) == pytest.approx(16.0)
    assert apparent_scale_bin(5.0) == "small_tag"
    assert apparent_scale_bin(10.0) == "medium_tag"
    assert apparent_scale_bin(20.0) == "large_tag"
    assert apparent_scale_bin(3.99) == "reject"
    assert apparent_scale_bin(35.01) == "reject"


def test_image_footprint_bins_use_visible_image_hull_area_percent() -> None:
    low = {0: _rect_corners(100.0, 100.0, 300.0, 200.0)}
    medium = {0: _rect_corners(100.0, 100.0, 450.0, 450.0)}
    high = {0: _rect_corners(100.0, 100.0, 700.0, 700.0)}
    assert visible_image_hull_area_percent(low, (1000, 1000)) == pytest.approx(2.0)
    assert image_footprint_bin(visible_image_hull_area_percent(low, (1000, 1000))) == (
        "low_image_footprint"
    )
    assert image_footprint_bin(visible_image_hull_area_percent(medium, (1000, 1000))) == (
        "medium_image_footprint"
    )
    assert image_footprint_bin(visible_image_hull_area_percent(high, (1000, 1000))) == (
        "high_image_footprint"
    )
    assert image_footprint_bin(0.5) == "reject"


def test_board_completeness_uses_generated_layout_area_percent(layout: BoardLayout) -> None:
    assert visible_board_layout_area_percent(layout, list(range(12))) == pytest.approx(100.0)
    assert board_completeness_class(layout, list(range(12))) == "full_board"
    assert board_completeness_class(layout, list(range(9))) == "partial_board_large"
    assert board_completeness_class(layout, list(range(6))) == "partial_board_medium"
    assert board_completeness_class(layout, [0, 3]) == "partial_board_small"
    assert board_completeness_class(layout, [9]) == "insufficient_board"
    assert board_completeness_class(layout, []) == "no_board"


def test_board_layout_geometry_accepts_detected_fixture_corners_from_generated_pdf_layout(
    fixture_results: list[FrameVerificationResult],
) -> None:
    failures = [
        f"{result.frame_id}: p95={result.metrics.board_layout_error_px_p95:.2f}px"
        for result in fixture_results
        if result.detected_ids
        and (
            result.board_layout_geometry is None
            or not result.board_layout_geometry.ok
            or result.metrics.board_layout_error_px_p95 > 3.0
        )
    ]
    assert not failures, "PDF-layout homography verification failed:\n" + "\n".join(failures)


def test_board_layout_geometry_rejects_swapped_detected_ids(layout: BoardLayout) -> None:
    corners_by_id = _layout_image_corners(layout, list(range(12)))
    corners_by_id[0], corners_by_id[1] = corners_by_id[1], corners_by_id[0]
    result = verify_board_layout_geometry(corners_by_id, layout)
    assert not result.ok
    assert result.layout_error_px_p95 > 3.0


def test_marker_tf_replay_publishes_visible_ids_only(
    manifest: dict[str, Any],
) -> None:
    mod = MarkerTfModule(
        marker_length_m=manifest["fixture"]["marker_length_m"],
        marker_namespace_prefix="fixture",
        max_freq=60.0,
    )
    failures: list[str] = []
    try:
        for index, frame in enumerate(manifest["frames"]):
            ts = 10_000.0 + index * 10.0
            image = Image.from_file(ROOT / frame["image_path"])
            image.ts = ts
            image.frame_id = manifest["camera"]["frame_id"]
            mod.tf.publish(
                Transform(
                    translation=Vector3(0.0, 0.0, 0.0),
                    rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
                    frame_id="world",
                    child_frame_id="base_link",
                    ts=ts,
                ),
                Transform(
                    translation=Vector3(0.0, 0.0, 0.0),
                    rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
                    frame_id="base_link",
                    child_frame_id=manifest["camera"]["frame_id"],
                    ts=ts,
                ),
            )
            mod._latest_camera_info = _camera_info_from_manifest(manifest, ts=ts)
            mod._process_color_image(image)

            marker_parent = "fixture/markers"
            if (
                frame["expected_visible_ids"]
                and mod.tf.get("world", marker_parent, ts, 0.1) is None
            ):
                failures.append(f"{frame['frame_id']}: missing world -> {marker_parent}")

            published_poses: dict[int, np.ndarray] = {}
            for tag_id in manifest["fixture"]["ids"]:
                transform = mod.tf.get(marker_parent, f"fixture/marker_{tag_id}", ts, 0.1)
                if transform is not None:
                    published_poses[tag_id] = transform.to_matrix()

            try:
                validate_detection_expectation(frame, sorted(published_poses))
            except ValueError as exc:
                failures.append(
                    f"{frame['frame_id']}: marker TF IDs {sorted(published_poses)}: {exc}"
                )

            for tag_id, pose in published_poses.items():
                if not np.all(np.isfinite(pose)):
                    failures.append(f"{frame['frame_id']}: marker_{tag_id} TF is non-finite")
    finally:
        mod.stop()

    assert not failures, "MarkerTfModule fixture replay failed:\n" + "\n".join(failures)


def _square_corners(x: float, y: float, edge: float) -> np.ndarray:
    return _rect_corners(x, y, x + edge, y + edge)


def _rect_corners(x0: float, y0: float, x1: float, y1: float) -> np.ndarray:
    return np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]], dtype=np.float32)


def _layout_image_corners(layout: BoardLayout, visible_ids: list[int]) -> dict[int, np.ndarray]:
    return {
        tag_id: (layout.tags[tag_id].corners_m[:, :2] * 1000.0 + np.array([100.0, 50.0])).astype(
            np.float32
        )
        for tag_id in visible_ids
    }


def _camera_info_from_manifest(manifest: dict[str, Any], *, ts: float) -> CameraInfo:
    camera = manifest["camera"]
    return CameraInfo(
        height=camera["image_height_px"],
        width=camera["image_width_px"],
        distortion_model=camera["distortion_model"],
        D=camera["distortion_coefficients"]["data"],
        K=camera["camera_matrix"]["data"],
        R=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        P=[
            camera["camera_matrix"]["data"][0],
            0.0,
            camera["camera_matrix"]["data"][2],
            0.0,
            0.0,
            camera["camera_matrix"]["data"][4],
            camera["camera_matrix"]["data"][5],
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
        ],
        frame_id=camera["frame_id"],
        ts=ts,
    )
