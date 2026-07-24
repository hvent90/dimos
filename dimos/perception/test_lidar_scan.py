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

"""End-to-end lidar-scan test on a synthetic recording with a stub detector.

Builds a store with color/lidar/odom streams where a point cluster sits 2 m
in front of a robot at the origin, and checks the 2D detection is lifted to
the cluster's 3D world position — no GPU or model download involved.
"""

from pathlib import Path

import numpy as np
import pytest

from dimos.memory2.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.perception.detection.detectors.base import Detector
from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D
from dimos.perception.lidar_scan import (
    LidarSighting,
    corroborated_sightings,
    iter_lidar_scan,
    project_points,
)

T0 = 1_000_000.0
CLUSTER_CENTER = np.array([2.0, 0.0, 0.3])

# base_link -> camera_optical without any mount offset: the standard ROS
# optical rotation (base +x forward == optical +z).
BASE_TO_OPTICAL = Transform(
    translation=Vector3(0.0, 0.0, 0.0),
    rotation=Quaternion(-0.5, 0.5, -0.5, 0.5),
    frame_id="base_link",
    child_frame_id="camera_optical",
)

CAMERA = CameraInfo(
    K=[100.0, 0.0, 64.0, 0.0, 100.0, 48.0, 0.0, 0.0, 1.0],
    width=128,
    height=96,
)


class StubDetector(Detector):
    """Returns one fixed full-ish bbox around the projected cluster."""

    def process_image(self, image: Image) -> ImageDetections2D[Detection2DBBox]:
        det = Detection2DBBox(
            bbox=(44.0, 13.0, 84.0, 53.0),  # cluster projects near (64, 33)
            track_id=3,
            class_id=0,
            confidence=0.9,
            name="box",
            ts=image.ts,
            image=image,
        )
        return ImageDetections2D(image=image, detections=[det])


@pytest.fixture()
def synthetic_store(tmp_path: Path) -> str:
    rng = np.random.default_rng(7)
    cluster = CLUSTER_CENTER + rng.normal(0.0, 0.03, size=(400, 3))
    db = tmp_path / "rec.db"
    with SqliteStore(path=str(db)) as store:
        images = store.stream("color_image", Image)
        lidar = store.stream("lidar", PointCloud2)
        odom = store.stream("odom", PoseStamped)
        frame = Image.from_numpy(np.zeros((96, 128, 3), dtype=np.uint8), ts=T0)
        images.append(frame, ts=T0)
        cloud = PointCloud2.from_numpy(cluster, frame_id="world", timestamp=T0)
        lidar.append(cloud, ts=T0)
        pose = PoseStamped(ts=T0, position=(0.0, 0.0, 0.0), orientation=(0.0, 0.0, 0.0, 1.0))
        odom.append(pose, ts=T0, pose=pose)
    return str(db)


def test_projection_sanity() -> None:
    world_to_optical = BASE_TO_OPTICAL.inverse()  # robot at origin, identity
    uv, depth = project_points(CLUSTER_CENTER[None, :], world_to_optical, CAMERA)
    assert uv.shape == (1, 2)
    np.testing.assert_allclose(uv[0], [64.0, 33.0], atol=0.5)
    np.testing.assert_allclose(depth[0], 2.0, atol=1e-6)


def test_iter_lidar_scan_lifts_detection_to_cluster(synthetic_store: str) -> None:
    with SqliteStore(path=synthetic_store, must_exist=True) as store:
        frames = list(iter_lidar_scan(store, StubDetector(), CAMERA, BASE_TO_OPTICAL))
    assert len(frames) == 1
    frame = frames[0]
    assert frame.ts == T0
    assert frame.robot_xy == (0.0, 0.0)
    assert len(frame.detections_2d) == 1
    assert len(frame.sightings) == 1
    s = frame.sightings[0]
    assert s.name == "box"
    assert s.track_id == 3
    assert s.confidence == 0.9
    # Hidden-point removal keeps only the camera-facing shell of the cluster.
    assert s.n_points > 10
    np.testing.assert_allclose(s.position, CLUSTER_CENTER, atol=0.08)
    assert s.extent is not None
    x_min, y_min, z_min, x_max, y_max, z_max = s.extent
    assert x_min <= CLUSTER_CENTER[0] <= x_max
    assert y_min <= CLUSTER_CENTER[1] <= y_max
    assert z_min <= CLUSTER_CENTER[2] <= z_max
    # The cluster is a 3-sigma=0.09m ball; its shell AABB must stay tight.
    assert max(x_max - x_min, y_max - y_min, z_max - z_min) < 0.5


def test_iter_lidar_scan_skips_frames_without_odom(synthetic_store: str, tmp_path: Path) -> None:
    # A frame whose nearest odom is farther than the tolerance is skipped.
    db = tmp_path / "no_odom.db"
    with SqliteStore(path=str(db)) as store:
        store.stream("color_image", Image).append(
            Image.from_numpy(np.zeros((96, 128, 3), dtype=np.uint8), ts=T0), ts=T0
        )
        store.stream("lidar", PointCloud2).append(
            PointCloud2.from_numpy(CLUSTER_CENTER[None, :], frame_id="world", timestamp=T0),
            ts=T0,
        )
        pose = PoseStamped(ts=T0 + 10.0, position=(0.0, 0.0, 0.0))
        store.stream("odom", PoseStamped).append(pose, ts=T0 + 10.0, pose=pose)
    with SqliteStore(path=str(db), must_exist=True) as store:
        frames = list(iter_lidar_scan(store, StubDetector(), CAMERA, BASE_TO_OPTICAL))
    assert frames == []


def test_iter_lidar_scan_missing_stream_raises(tmp_path: Path) -> None:
    db = tmp_path / "empty.db"
    with SqliteStore(path=str(db)) as store:
        store.stream("color_image", Image)
    with SqliteStore(path=str(db), must_exist=True) as store:
        with pytest.raises(LookupError, match="lidar"):
            list(iter_lidar_scan(store, StubDetector(), CAMERA, BASE_TO_OPTICAL))


def _row(name: str, ts: float, x: float, y: float) -> LidarSighting:
    return LidarSighting(
        name=name, ts=ts, position=(x, y, 0.0), confidence=0.2, track_id=-1, n_points=10
    )


def test_corroboration_drops_one_frame_flickers() -> None:
    rows = [
        _row("couch", T0 + 0.0, 1.0, 1.0),
        _row("couch", T0 + 0.5, 1.1, 1.0),
        _row("couch", T0 + 1.0, 1.0, 1.1),
        _row("surfboard", T0 + 0.5, 3.0, 3.0),  # one-frame junk match
        _row("toilet", T0 + 1.0, 5.0, 5.0),  # ditto
    ]
    kept, dropped = corroborated_sightings(rows, {}, radius_m=0.75, min_sightings=3, min_frames=2)
    assert [r.name for r in kept] == ["couch", "couch", "couch"]
    assert dropped == {"surfboard": 1, "toilet": 1}


def test_corroboration_needs_distinct_frames() -> None:
    # Three same-frame boxes (e.g. per-class NMS survivors) are one vantage.
    rows = [_row("chair", T0, 1.0 + 0.1 * i, 1.0) for i in range(3)]
    kept, dropped = corroborated_sightings(rows, {}, radius_m=0.75, min_sightings=3, min_frames=2)
    assert kept == []
    assert dropped == {"chair": 3}


def test_corroboration_passes_existing_objects_through() -> None:
    # A single re-sighting near a known node keeps flowing — that object
    # already earned its corroboration in an earlier scan.
    rows = [_row("couch", T0, 1.2, 1.0), _row("couch", T0, 9.0, 9.0)]
    kept, dropped = corroborated_sightings(
        rows, {"couch": [(1.0, 1.0)]}, radius_m=0.75, min_sightings=3, min_frames=2
    )
    assert [(r.name, r.position[0]) for r in kept] == [("couch", 1.2)]
    assert dropped == {"couch": 1}


def test_corroboration_separates_spatial_clusters() -> None:
    # Same name, two places: the re-fired cluster confirms, the flicker dies.
    rows = [
        _row("chair", T0 + 0.0, 1.0, 1.0),
        _row("chair", T0 + 0.5, 1.1, 1.0),
        _row("chair", T0 + 1.0, 1.0, 1.1),
        _row("chair", T0 + 0.5, 6.0, 6.0),
    ]
    kept, dropped = corroborated_sightings(rows, {}, radius_m=0.75, min_sightings=3, min_frames=2)
    assert all(r.position[0] < 2.0 for r in kept) and len(kept) == 3
    assert dropped == {"chair": 1}


def test_corroboration_disabled_at_one_one() -> None:
    rows = [_row("surfboard", T0, 3.0, 3.0)]
    kept, dropped = corroborated_sightings(rows, {}, radius_m=0.75, min_sightings=1, min_frames=1)
    assert kept == rows
    assert dropped == {}
