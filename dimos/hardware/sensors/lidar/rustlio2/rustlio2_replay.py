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

"""Replay a recorded ``lidar`` / ``imu`` memory2 ``.db`` for the Rust FAST-LIO2 module.

Reads a SQLite recording produced by :class:`LivoxRecorder` and republishes its
``lidar`` (:class:`PointCloud2`, carrying per-point ``time``) and ``imu``
(:class:`Imu`) streams at the recording's original wall-clock timing. This is
the dependency-free replacement for ``mcap_replay`` — the demo no longer needs
the ``mcap`` library or a Livox bag, only the ``.db``.
"""

from __future__ import annotations

from pathlib import Path

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.memory2.replay import resolve_db_path
from dimos.memory2.store.sqlite import SqliteStore
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.data import LfsPath
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class LivoxDbReplayConfig(ModuleConfig):
    dataset: str | Path | LfsPath | None = None
    speed: float = 1.0
    seek: float | None = None
    duration: float | None = None
    loop: bool = False


class LivoxDbReplay(Module):
    """Republish a recorded Livox ``.db`` onto the ``lidar`` / ``imu`` streams.

    Ports:
        lidar (Out[PointCloud2]): Per-scan clouds with a per-point ``time`` field.
        imu (Out[Imu]): IMU samples (m/s^2 linear accel, rad/s angular vel).
    """

    config: LivoxDbReplayConfig
    lidar: Out[PointCloud2]
    imu: Out[Imu]

    @rpc
    def start(self) -> None:
        super().start()

        if self.config.dataset is None:
            logger.error("LivoxDbReplay has no dataset configured; nothing to replay")
            return

        db_path = resolve_db_path(self.config.dataset)
        if not db_path.exists():
            logger.error(
                "Livox recording not found: %s — generate it with demo_record_livox_db.py",
                db_path,
            )
            return

        store = self.register_disposable(SqliteStore(path=str(db_path), must_exist=True))
        store.start()
        replay = store.replay(
            speed=self.config.speed,
            seek=self.config.seek,
            duration=self.config.duration,
            loop=self.config.loop,
        )

        self.register_disposable(
            replay.streams.lidar.observable().subscribe(
                on_next=self.lidar.publish,
                on_error=lambda e: logger.error("lidar replay error: %s", e, exc_info=True),
            )
        )
        self.register_disposable(
            replay.streams.imu.observable().subscribe(
                on_next=self.imu.publish,
                on_error=lambda e: logger.error("imu replay error: %s", e, exc_info=True),
            )
        )
