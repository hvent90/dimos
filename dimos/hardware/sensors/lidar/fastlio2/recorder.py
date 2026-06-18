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

"""Record FAST-LIO odometry + lidar into a memory2 SQLite db.

A ``TfRecorder`` whose ``odometry`` / ``lidar`` In ports auto-connect to a
FastLio2's same-named outputs. It records them under configurable stream names,
replacing only its own streams when appending (``force``). Poses come straight
from the odometry stream (``@pose_setter_for``): each lidar frame is stamped with
the latest odometry pose so ``fastlio_lidar`` carries the trajectory and ``dimos
map global`` can register it.
"""

from __future__ import annotations

from pathlib import Path

from dimos.core.stream import In
from dimos.memory2.module import OnExisting
from dimos.memory2.tf_recorder import TfRecorder, TfRecorderConfig, pose_setter_for
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


class FastLio2RecorderConfig(TfRecorderConfig):
    """Target db + stream names for the FAST-LIO recorder."""

    # db stream/table names the FastLio2 outputs are recorded under.
    odom_stream_name: str = "fastlio_odometry"
    lidar_stream_name: str = "fastlio_lidar"
    # Drop pre-existing odom/lidar streams instead of refusing to overwrite.
    force: bool = False
    # Append into a populated db (keep other streams); replace only our two.
    on_existing: OnExisting = OnExisting.APPEND


class FastLio2Recorder(TfRecorder):
    config: FastLio2RecorderConfig

    odometry: In[Odometry]
    lidar: In[PointCloud2]

    _last_odom_pose: Pose | None = None

    def _stream_name(self, port_name: str) -> str:
        if port_name == "odometry":
            return self.config.odom_stream_name
        if port_name == "lidar":
            return self.config.lidar_stream_name
        return port_name

    def _prepare_streams(self) -> None:
        cfg = self.config
        names = (cfg.odom_stream_name, cfg.lidar_stream_name)
        existing = sorted(set(self.store.list_streams()) & set(names))
        if existing and not cfg.force:
            raise RuntimeError(
                f"FastLio2Recorder: {Path(cfg.db_path).name} already has {existing}; "
                "set force=True to overwrite"
            )
        for name in existing:
            self.store.delete_stream(name)

    @pose_setter_for("odometry")
    def _odom_pose(self, msg: Odometry) -> Pose | None:
        pose = getattr(msg, "pose", None)
        self._last_odom_pose = getattr(pose, "pose", None) if pose is not None else None
        return self._last_odom_pose

    @pose_setter_for("lidar")
    def _lidar_pose(self, msg: PointCloud2) -> Pose | None:
        # Most-recent odometry pose, stamped directly (no tf). None before the
        # first odometry -> frame stored unposed, map-skipped.
        return self._last_odom_pose
