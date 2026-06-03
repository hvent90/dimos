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

"""Record the Rust FAST-LIO2 outputs plus the raw Livox inputs to a ``.db``.

Wire it to :class:`Rustlio2` and :class:`Mid360` via ``autoconnect`` — the
matching ``odometry`` / ``global_map`` (FAST-LIO2 outputs) and ``lidar`` /
``imu`` (Livox outputs) ports connect automatically — to capture a SLAM run
for later replay or analysis.
"""

from __future__ import annotations

from datetime import datetime
from pathlib import Path

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.core.core import rpc
from dimos.core.stream import In
from dimos.memory2.module import Recorder, RecorderConfig
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


class Rustlio2RecorderConfig(RecorderConfig):
    db_path: str | Path = (
        DIMOS_PROJECT_ROOT / "recordings" / f"fastlio_rs_{datetime.now():%Y-%m-%d_%H-%M-%S}.db"
    )


class Rustlio2Recorder(Recorder):
    """Record FAST-LIO2 ``odometry`` / ``global_map`` plus Livox ``lidar`` / ``imu`` to a ``.db``.

    Defaults to a fresh, timestamped ``recordings/fastlio_rs_<date_time>.db`` per run.
    """

    config: Rustlio2RecorderConfig

    odometry: In[Odometry]
    global_map: In[PointCloud2]
    lidar: In[PointCloud2]
    imu: In[Imu]

    @rpc
    def start(self) -> None:
        if not self.config.g.replay:
            Path(self.config.db_path).parent.mkdir(parents=True, exist_ok=True)
        super().start()
