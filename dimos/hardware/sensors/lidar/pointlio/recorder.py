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

"""Record Point-LIO odometry + lidar into a memory2 SQLite db.

A ``TfRecorder`` whose ``odometry`` / ``lidar`` In ports auto-connect to a
PointLio's same-named outputs. Beyond the base recorder it: records under
configurable stream names, re-bases timestamps onto the db's existing clock so a
run can be appended (e.g. onto a fastlio replay) and compared on one timeline,
and replaces only its own streams when appending (``force``). Poses come straight
from the odometry stream (``@pose_setter_for``): each lidar frame is stamped with
the latest odometry pose so ``pointlio_lidar`` carries the trajectory and ``dimos
map global`` can register the body-frame cloud directly (no ``pose-fill`` pass).
"""

from __future__ import annotations

import math
from pathlib import Path
import sqlite3
import time
from typing import Any

from dimos.core.stream import In
from dimos.memory2.module import OnExisting
from dimos.memory2.tf_recorder import TfRecorder, TfRecorderConfig, pose_setter_for
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

# Below this the db's existing timestamps are sensor-boot seconds, not unix time.
_SENSOR_CLOCK_MAX = 1e8
# Strictly-increasing tie-breaker so two samples never collide on ts.
_EPS = 1e-9


def _existing_min_ts(db_path: Path) -> float:
    """Min ts across the db's existing streams, or -1.0 if none/absent."""
    if not db_path.exists():
        return -1.0
    con = sqlite3.connect(f"file:{db_path}?mode=ro", uri=True, timeout=2.0)
    try:
        tables = [
            row[0]
            for row in con.execute("SELECT name FROM sqlite_master WHERE type='table'").fetchall()
        ]
        best: float | None = None
        for table in tables:
            if table.startswith("_") or table.startswith("sqlite_"):
                continue
            try:
                cols = [col[1] for col in con.execute(f"PRAGMA table_info('{table}')").fetchall()]
                if "ts" not in cols:
                    continue
                row = con.execute(f"SELECT MIN(ts) FROM '{table}'").fetchone()
            except sqlite3.OperationalError:
                continue
            if row and row[0] is not None:
                best = row[0] if best is None else min(best, row[0])
        return best if best is not None else -1.0
    finally:
        con.close()


class PointlioRecorderConfig(TfRecorderConfig):
    """Target db + timing conversion for the Point-LIO recorder."""

    # db stream/table names the Point-LIO outputs are recorded under.
    odom_stream_name: str = "pointlio_odometry"
    lidar_stream_name: str = "pointlio_lidar"
    # Explicit offset override; NaN means auto-derive from the db's earliest ts.
    time_offset: float = float("nan")
    # Drop pre-existing odom/lidar streams instead of refusing to overwrite.
    force: bool = False
    # Append into a populated db (keep other streams); replace only our two.
    on_existing: OnExisting = OnExisting.APPEND


class PointlioRecorder(TfRecorder):
    config: PointlioRecorderConfig

    odometry: In[Odometry]
    lidar: In[PointCloud2]

    _offset: float | None = None
    _ref_start_ts: float = -1.0
    _last_odom_ts: float = 0.0
    _last_lidar_ts: float = 0.0
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
                f"PointlioRecorder: {Path(cfg.db_path).name} already has {existing}; "
                "set force=True to overwrite"
            )
        for name in existing:
            self.store.delete_stream(name)
        # Reference is the db's earliest ts *after* dropping our own old streams,
        # so only the data we're aligning against (e.g. a fastlio replay) counts.
        self._ref_start_ts = _existing_min_ts(Path(cfg.db_path))

    def _resolve_offset(self, first_ts: float) -> float:
        override = self.config.time_offset
        if not math.isnan(override):
            return override
        ref = self._ref_start_ts
        if ref < 0.0 or ref < _SENSOR_CLOCK_MAX:
            # Empty db, or db already on the sensor clock -> exact alignment.
            return 0.0
        # db on wall-clock time -> start-align Point-LIO onto the db's earliest ts.
        return ref - first_ts

    def _resolve_ts(self, name: str, msg: Any) -> float:
        # `is not None`, not `or`: a real sensor ts of 0.0 must not fall back to
        # wall time (would misclassify the stream's clock in _resolve_offset).
        raw = getattr(msg, "ts", None)
        raw = raw if raw is not None else time.time()
        if self._offset is None:
            self._offset = self._resolve_offset(raw)
        last = self._last_odom_ts if name == "odometry" else self._last_lidar_ts
        ts = max(raw + self._offset, last + _EPS)
        if name == "odometry":
            self._last_odom_ts = ts
        else:
            self._last_lidar_ts = ts
        return ts

    @pose_setter_for("odometry")
    def _odom_pose(self, msg: Odometry) -> Pose | None:
        pose = getattr(msg, "pose", None)
        self._last_odom_pose = getattr(pose, "pose", None) if pose is not None else None
        return self._last_odom_pose

    @pose_setter_for("lidar")
    def _lidar_pose(self, msg: PointCloud2) -> Pose | None:
        # Stamp each (sensor-frame) cloud with the most-recent odometry pose
        # directly — no tf / static-transform composition. None before the first
        # odometry, in which case the frame is stored unposed and map-skipped.
        return self._last_odom_pose
