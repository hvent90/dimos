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

"""Re-base the Go2 onboard odom/lidar onto the fastlio frame.

The Go2's onboard `odom` (and the `lidar` clouds anchored to it) live in their
own estimator frame, offset from the lidar-inertial `fastlio_odometry` frame.
This rigidly re-bases them: at the first timestamp both streams cover, force
`odom` to coincide with `fastlio_odometry`, then carry that single fixed
transform forward for every later sample (anything before the overlap is
dropped). Writes `odom_adjusted` (PoseStamped) and `lidar_adjusted`
(PointCloud2).

    T = fastlio(t0) . odom(t0)^-1           (go2 frame -> fastlio frame, fixed)
    odom_adjusted(t) = T . odom(t)          for t >= t0
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from dimos.mapping.recording.utils import stream_names
from dimos.mapping.recording.utils.lidar_reanchor import _load_poses, _mat, _nearest, _to_transform
from dimos.memory2.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

GO2_ODOM = stream_names.ODOM
FASTLIO_ODOM = stream_names.FASTLIO_ODOM
GO2_LIDAR = stream_names.LIDAR
GO2_ODOM_ADJUSTED = stream_names.adjusted(stream_names.ODOM)
GO2_LIDAR_ADJUSTED = stream_names.adjusted(stream_names.LIDAR)


def _pose7_from_mat(matrix: np.ndarray) -> list[float]:
    quaternion = Rotation.from_matrix(matrix[:3, :3]).as_quat()
    return [
        float(matrix[0, 3]),
        float(matrix[1, 3]),
        float(matrix[2, 3]),
        float(quaternion[0]),
        float(quaternion[1]),
        float(quaternion[2]),
        float(quaternion[3]),
    ]


def adjust_go2_streams(db: Path) -> None:
    """Write `odom_adjusted` + `lidar_adjusted`, re-based onto the fastlio frame
    at the first overlapping timestamp. No-op if the inputs are missing."""
    db_path = str(db)
    with SqliteStore(path=db_path) as store:
        stream_names = store.list_streams()
        if GO2_ODOM not in stream_names or FASTLIO_ODOM not in stream_names:
            print(f"   go2-align: need '{GO2_ODOM}' + '{FASTLIO_ODOM}' — skipping")
            return

        go2_timestamps, go2_poses = _load_poses(db_path, GO2_ODOM)
        fastlio_timestamps, fastlio_poses = _load_poses(db_path, FASTLIO_ODOM)

        overlap_start = max(go2_timestamps[0], fastlio_timestamps[0])
        overlap_end = min(go2_timestamps[-1], fastlio_timestamps[-1])
        start_index = int(np.searchsorted(go2_timestamps, overlap_start))
        if start_index >= len(go2_timestamps) or go2_timestamps[start_index] > overlap_end:
            print("   go2-align: no overlapping timestamps — skipping")
            return

        anchor_timestamp = float(go2_timestamps[start_index])
        fastlio_index = _nearest(fastlio_timestamps, anchor_timestamp)
        # T: go2 frame -> fastlio frame, fixed at the first overlapping timestamp.
        anchor_transform = _mat(fastlio_poses[fastlio_index]) @ np.linalg.inv(
            _mat(go2_poses[start_index])
        )

        if GO2_ODOM_ADJUSTED in stream_names:
            store.delete_stream(GO2_ODOM_ADJUSTED)
        odom_out = store.stream(GO2_ODOM_ADJUSTED, PoseStamped)
        for index in range(start_index, len(go2_timestamps)):
            timestamp = float(go2_timestamps[index])
            pose7 = _pose7_from_mat(anchor_transform @ _mat(go2_poses[index]))
            odom_out.append(
                PoseStamped(ts=timestamp, position=pose7[:3], orientation=pose7[3:7]),
                ts=timestamp,
                pose=tuple(pose7),
            )
        dropped = start_index
        print(
            f"   go2-align: '{GO2_ODOM_ADJUSTED}' {len(go2_timestamps) - start_index} poses "
            f"(dropped {dropped} pre-overlap), anchored at t={anchor_timestamp:.3f}"
        )

        if GO2_LIDAR not in stream_names:
            print(f"   go2-align: no '{GO2_LIDAR}' — skipping lidar")
            return
        if GO2_LIDAR_ADJUSTED in stream_names:
            store.delete_stream(GO2_LIDAR_ADJUSTED)
        lidar_out = store.stream(GO2_LIDAR_ADJUSTED, PointCloud2)
        transform = _to_transform(anchor_transform)
        written = 0
        for observation in store.stream(GO2_LIDAR, PointCloud2).to_list():
            if observation.ts < anchor_timestamp:
                continue
            cloud = observation.data
            adjusted = cloud.transform(transform)  # go2 world -> fastlio world (open3d)
            new_cloud = PointCloud2.from_numpy(
                adjusted.points_f32(), timestamp=observation.ts, intensities=cloud.intensities_f32()
            )
            nearest_pose = anchor_transform @ _mat(
                go2_poses[_nearest(go2_timestamps, observation.ts)]
            )
            lidar_out.append(
                new_cloud, ts=observation.ts, pose=tuple(_pose7_from_mat(nearest_pose))
            )
            written += 1
        print(f"   go2-align: '{GO2_LIDAR_ADJUSTED}' {written} clouds re-based onto fastlio frame")
