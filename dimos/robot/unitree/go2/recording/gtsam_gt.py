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

"""GTSAM landmark-SLAM groundtruth from AprilTag observations.

Treats the FAST-LIO / odom pose chain as locally correct and the AprilTags as
static landmarks; a tag seen at several times pins the chain and removes
accumulated odometry drift. Trusts tag POSITION (solvePnP is metric) but
distrusts tag ORIENTATION (a small planar tag is yaw/pitch ambiguous), and wraps
tag factors in a Huber kernel so a bad detection can't dominate.
"""

from __future__ import annotations

import sqlite3

import numpy as np

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.robot.unitree.go2.recording.camera import CAMERA_OPTICAL_IN_BASE


def _pose_from7(pose7):
    """[x y z qx qy qz qw] -> gtsam.Pose3."""
    import gtsam

    return gtsam.Pose3(
        gtsam.Rot3.Quaternion(pose7[6], pose7[3], pose7[4], pose7[5]),
        gtsam.Point3(pose7[0], pose7[1], pose7[2]),
    )


def _pose_to7(pose3):
    """gtsam.Pose3 -> [x y z qx qy qz qw]."""
    quaternion = pose3.rotation().toQuaternion()
    translation = pose3.translation()
    return [
        translation[0],
        translation[1],
        translation[2],
        quaternion.x(),
        quaternion.y(),
        quaternion.z(),
        quaternion.w(),
    ]


def pick_pose_stream(connection) -> str:
    """The odom stream to use as the pose chain (go2_odom / fastlio_odometry preferred)."""
    stream_names = [row[0] for row in connection.execute("SELECT name FROM _streams").fetchall()]
    candidates = [name for name in ["go2_odom", "fastlio_odometry"] if name in stream_names]
    candidates += [
        name for name in stream_names if "odom" in name.lower() and name not in candidates
    ]
    for name in candidates:
        try:
            populated = connection.execute(
                f'SELECT count(*) FROM "{name}" WHERE pose_qw IS NOT NULL'
            ).fetchone()[0]
        except sqlite3.OperationalError:
            continue
        if populated > 0:
            return name
    raise SystemExit(f"no odom stream with populated pose columns among {candidates}")


def build_gtsam_gt(
    db_path,
    markers,
    *,
    node_stride=3,
    odom_rot_sig=0.004,
    odom_trans_sig=0.02,
    tag_rot_sig=1.0,
    tag_trans_sig=0.1,
    tag_huber=0.5,
):
    """Landmark-SLAM the odom chain + AprilTag landmarks. Returns [(ts, pose7), ...]."""
    import gtsam
    from gtsam import BetweenFactorPose3, PriorFactorPose3
    from gtsam.symbol_shorthand import L, X

    connection = sqlite3.connect(db_path)
    pose_stream = pick_pose_stream(connection)
    pose_rows = connection.execute(
        f"SELECT ts,pose_x,pose_y,pose_z,pose_qx,pose_qy,pose_qz,pose_qw "
        f'FROM "{pose_stream}" WHERE pose_qw IS NOT NULL ORDER BY ts'
    ).fetchall()
    connection.close()
    pose_rows = pose_rows[::node_stride]
    node_timestamps = np.array([row[0] for row in pose_rows])
    node_poses = [_pose_from7(row[1:8]) for row in pose_rows]
    num_nodes = len(pose_rows)
    print(
        f"   gtsam: pose stream '{pose_stream}', {num_nodes} nodes (stride {node_stride}), "
        f"{len(markers)} tag obs"
    )

    base_to_optical = _pose_from7(CAMERA_OPTICAL_IN_BASE)

    def nearest_node(timestamp):
        node_index = int(np.searchsorted(node_timestamps, timestamp))
        node_index = min(max(node_index, 0), num_nodes - 1)
        if node_index > 0 and abs(node_timestamps[node_index - 1] - timestamp) < abs(
            node_timestamps[node_index] - timestamp
        ):
            node_index -= 1
        return node_index

    graph = gtsam.NonlinearFactorGraph()
    initial = gtsam.Values()
    prior_noise = gtsam.noiseModel.Diagonal.Sigmas(np.full(6, 1e-4))
    odom_noise = gtsam.noiseModel.Diagonal.Sigmas(
        np.array([odom_rot_sig] * 3 + [odom_trans_sig] * 3)
    )
    tag_noise_base = gtsam.noiseModel.Diagonal.Sigmas(
        np.array([tag_rot_sig] * 3 + [tag_trans_sig] * 3)
    )
    tag_noise = gtsam.noiseModel.Robust.Create(
        gtsam.noiseModel.mEstimator.Huber.Create(tag_huber), tag_noise_base
    )

    for node_index in range(num_nodes):
        initial.insert(X(node_index), node_poses[node_index])
    graph.add(PriorFactorPose3(X(0), node_poses[0], prior_noise))
    for node_index in range(num_nodes - 1):
        relative = node_poses[node_index].between(node_poses[node_index + 1])
        graph.add(BetweenFactorPose3(X(node_index), X(node_index + 1), relative, odom_noise))

    landmark_ids = set()
    for detection in markers:
        marker_id = int(detection["marker_id"])
        node_index = nearest_node(detection["ts"])
        tag_in_body = base_to_optical.compose(_pose_from7(detection["t_cam_marker"]))
        if marker_id not in landmark_ids:
            initial.insert(L(marker_id), node_poses[node_index].compose(tag_in_body))
            landmark_ids.add(marker_id)
        graph.add(BetweenFactorPose3(X(node_index), L(marker_id), tag_in_body, tag_noise))

    params = gtsam.LevenbergMarquardtParams()
    params.setMaxIterations(100)
    optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial, params)
    result = optimizer.optimize()
    corrections = [
        np.linalg.norm(
            result.atPose3(X(node_index)).translation() - node_poses[node_index].translation()
        )
        for node_index in range(num_nodes)
    ]
    print(
        f"   gtsam: landmarks {sorted(landmark_ids)} | correction max {max(corrections):.2f} m, "
        f"mean {np.mean(corrections):.2f} m ({optimizer.iterations()} iters)"
    )
    return [
        (float(node_timestamps[node_index]), _pose_to7(result.atPose3(X(node_index))))
        for node_index in range(num_nodes)
    ]


def write_gtsam_odom(store, trajectory, stream_name, tum_path):
    """Write the corrected trajectory as a PoseStamped stream + a .tum file."""
    if stream_name in store.list_streams():
        store.delete_stream(stream_name)
    odom_stream = store.stream(stream_name, PoseStamped)
    with open(tum_path, "w") as tum_file:
        for timestamp, pose in trajectory:
            odom_stream.append(
                PoseStamped(ts=timestamp, position=pose[:3], orientation=pose[3:7]),
                ts=timestamp,
                pose=tuple(pose),
            )
            tum_file.write(f"{timestamp:.9f} " + " ".join(f"{value:.9f}" for value in pose) + "\n")
    print(f"   wrote '{stream_name}' stream ({len(trajectory)} poses) + {tum_path}")
