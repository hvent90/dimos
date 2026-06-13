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

"""Run Point-LIO over a .pcap and write its outputs into a .db.

Given a Livox Mid-360 pcap capture, this streams the pcap through the Point-LIO
native module (deterministic clock, single feeder -> never loads the whole pcap
into memory) and writes two streams into a memory2 SQLite database:

* ``pointlio_odometry`` -- the IESKF pose at the native odom rate (~30 Hz).
* ``pointlio_lidar`` -- the registered (deskewed, odom-frame) point cloud at the
  native pointcloud rate (~10 Hz).

The ``--db`` is optional. With no existing db the tool builds one **from
scratch** (omit ``--db`` and it defaults to ``<pcap>.db`` next to the pcap).
With an existing db the two streams are appended and time-aligned onto the db's
clock, so Point-LIO output can be compared against whatever it already holds
(e.g. a fastlio replay).

If either stream already exists in the db the run aborts, unless ``--force`` is
given, in which case the existing ``pointlio_odometry`` and ``pointlio_lidar``
streams are dropped before the new ones are written.

Timing conversion
-----------------
Point-LIO's deterministic output timestamps are in *sensor-boot seconds* (the
Livox packet clock, small values like 1588.x). The target db may use a different
clock for its existing streams:

* db already on the sensor clock (e.g. a fastlio replay db, min ts < 1e8):
  offset 0 -- both replay the same pcap packet clock, so they line up exactly.
* db on wall-clock unix time (min ts > 1e9): start-align by shifting Point-LIO's
  first odom ts onto the db's earliest existing ts.
* db has no existing timestamped rows: offset 0.

Pass ``--time-offset`` to override the auto choice.

Usage (from the dimos6 venv)::

    source .venv/bin/activate

    # Build a fresh db from scratch (no existing db needed). The ruwik2_part3
    # sample pcap is available via LFS:
    PCAP=$(python -c "from dimos.utils.data import get_data; \
        print(get_data('ruwik2_part3') / 'ruwik2_part3.pcap')")
    python -m dimos.hardware.sensors.lidar.pointlio.tools.pcap_to_db --pcap "$PCAP"
    # -> writes ruwik2_part3.db next to the sample with pointlio_odometry
    #    + pointlio_lidar.

    # Or append into an existing recording db for comparison:
    python -m dimos.hardware.sensors.lidar.pointlio.tools.pcap_to_db \
        --pcap /path/to/capture.pcap --db /path/to/memory.db
"""

from __future__ import annotations

import argparse
from collections.abc import AsyncIterator
import math
from pathlib import Path
import sqlite3
import sys
import time

from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

# Below this the db's existing timestamps are sensor-boot seconds, not unix time.
_SENSOR_CLOCK_MAX = 1e8
# Strictly-increasing tie-breaker so two odom samples never collide on ts.
_EPS = 1e-9
# Poll the db on this cadence while the replay drains the pcap.
_POLL_SEC = 1.0
# Stop after the odom stream has been stagnant this long (pcap fully drained).
_STAGNANT_SEC = 4.0


class _RecConfig(ModuleConfig):
    """Configures the recorder with the target db and timing conversion."""

    db_path: str = ""
    # Earliest existing ts in the db, or -1.0 if the db has no timestamped rows.
    ref_start_ts: float = -1.0
    # Explicit offset override; NaN means auto-derive from ref_start_ts.
    time_offset: float = float("nan")


class _Rec(Module):
    """Append Point-LIO odometry + lidar into a SQLite db with ts conversion.

    Underscore-prefixed so the blueprint registry generator skips it — this is
    an internal helper for the tool, not a public robot module.
    """

    config: _RecConfig
    pointlio_odometry: In[Odometry]
    pointlio_lidar: In[PointCloud2]
    _offset: float | None = None
    _last_odom_ts: float = 0.0
    _last_lidar_ts: float = 0.0
    _odom_count: int = 0
    _lidar_count: int = 0

    async def main(self) -> AsyncIterator[None]:
        from dimos.memory2.store.sqlite import SqliteStore

        self._store = SqliteStore(path=self.config.db_path)
        self._os = self._store.stream("pointlio_odometry", Odometry)
        self._ls = self._store.stream("pointlio_lidar", PointCloud2)
        yield
        self._store.stop()

    def _resolve_offset(self, first_ts: float) -> float:
        override = self.config.time_offset
        if not math.isnan(override):
            return override
        ref = self.config.ref_start_ts
        if ref < 0.0 or ref < _SENSOR_CLOCK_MAX:
            # Empty db, or db already on the sensor clock -> exact alignment.
            return 0.0
        # db on wall-clock time -> start-align Point-LIO onto the db's earliest ts.
        return ref - first_ts

    def _aligned_ts(self, raw_ts: float, last_ts: float) -> float:
        """Convert a sensor ts onto the db clock, kept strictly above last_ts."""
        if self._offset is None:
            self._offset = self._resolve_offset(raw_ts)
        return max(raw_ts + self._offset, last_ts + _EPS)

    async def handle_pointlio_odometry(self, v: Odometry) -> None:
        raw_ts = getattr(v, "ts", None) or time.time()
        ts = self._aligned_ts(raw_ts, self._last_odom_ts)
        self._last_odom_ts = ts
        pose = getattr(v, "pose", None)
        pose_inner = getattr(pose, "pose", None) if pose is not None else None
        self._os.append(v, ts=ts, pose=pose_inner)
        self._odom_count += 1

    async def handle_pointlio_lidar(self, v: PointCloud2) -> None:
        raw_ts = getattr(v, "ts", None) or time.time()
        ts = self._aligned_ts(raw_ts, self._last_lidar_ts)
        self._last_lidar_ts = ts
        self._ls.append(v, ts=ts)
        self._lidar_count += 1


def _db_ref_start_ts(db_path: Path) -> float:
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
                # vec0/rtree virtual tables (sqlite-vec etc.) raise "no such
                # module" here when the extension isn't loaded -- skip them.
                cols = [c[1] for c in con.execute(f"PRAGMA table_info('{table}')").fetchall()]
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


def _table_stats(db_path: Path, table: str) -> tuple[int, float, float]:
    """(count, min_ts, max_ts) for a stream table; zeros if absent."""
    if not db_path.exists():
        return 0, 0.0, 0.0
    con = sqlite3.connect(f"file:{db_path}?mode=ro", uri=True, timeout=2.0)
    try:
        try:
            row = con.execute(f"SELECT COUNT(*), MIN(ts), MAX(ts) FROM '{table}'").fetchone()
        except sqlite3.OperationalError:
            return 0, 0.0, 0.0
        cnt = row[0] or 0
        return cnt, row[1] or 0.0, row[2] or 0.0
    finally:
        con.close()


def _run(args: argparse.Namespace) -> int:
    pcap_path = Path(args.pcap).expanduser().resolve()
    if not pcap_path.exists():
        print(f"[pcap_to_db] missing pcap: {pcap_path}", file=sys.stderr)
        return 2
    if args.max_sensor_sec < 0:
        print("[pcap_to_db] --max-sensor-sec must be >= 0", file=sys.stderr)
        return 2
    # --db is optional: with no existing db, build one from scratch. When
    # omitted the output defaults to <pcap>.db next to the pcap, so a fresh
    # db can be generated with just --pcap.
    db_path = Path(args.db).expanduser().resolve() if args.db else pcap_path.with_suffix(".db")
    db_existed = db_path.exists()
    db_path.parent.mkdir(parents=True, exist_ok=True)

    from dimos.core.coordination.blueprints import autoconnect
    from dimos.core.coordination.module_coordinator import ModuleCoordinator
    from dimos.hardware.sensors.lidar.pointlio.module import PointLio
    from dimos.memory2.store.sqlite import SqliteStore

    pointlio_streams = ("pointlio_odometry", "pointlio_lidar")
    store = SqliteStore(path=str(db_path))
    try:
        existing = sorted(set(store.list_streams()) & set(pointlio_streams))
        if existing and not args.force:
            print(
                f"[pcap_to_db] {db_path.name} already has {existing}; pass --force to overwrite",
                file=sys.stderr,
            )
            return 2
        for name in existing:
            store.delete_stream(name)
        if existing:
            print(f"[pcap_to_db] --force: dropped existing {existing}", flush=True)
    finally:
        store.stop()

    ref_start_ts = _db_ref_start_ts(db_path)
    time_offset = float("nan") if args.time_offset is None else args.time_offset
    if not math.isnan(time_offset):
        offset_desc = f"explicit {time_offset:+.3f}s"
    elif ref_start_ts < 0.0:
        offset_desc = "auto: db empty -> 0"
    elif ref_start_ts < _SENSOR_CLOCK_MAX:
        offset_desc = f"auto: db sensor-clock (R0={ref_start_ts:.2f}) -> 0"
    else:
        offset_desc = f"auto: db wall-clock (R0={ref_start_ts:.2f}) -> start-align"
    print(
        f"[pcap_to_db] pcap={pcap_path.name} db={db_path.name} "
        f"({'append' if db_existed else 'new'}) "
        f"odom_freq={args.odom_freq}Hz offset={offset_desc}",
        flush=True,
    )

    blueprint = autoconnect(
        PointLio.blueprint(
            frame_id="world",
            map_freq=-1,
            odom_freq=args.odom_freq,
            replay_pcap=pcap_path,
            deterministic_clock=True,
            replay_dual_thread=False,
            debug=False,
        ).remappings(
            [
                (PointLio, "odometry", "pointlio_odometry"),
                (PointLio, "lidar", "pointlio_lidar"),
            ]
        ),
        _Rec.blueprint(
            db_path=str(db_path),
            ref_start_ts=ref_start_ts,
            time_offset=time_offset,
        ),
    ).global_config(n_workers=4, robot_model="mid360_pointlio_pcap_to_db")
    coord = ModuleCoordinator.build(blueprint)

    t0 = time.time()
    last_max = 0.0
    first_max: float | None = None
    stagnant_since: float | None = None
    try:
        while True:
            time.sleep(_POLL_SEC)
            cnt, min_ts, max_ts = _table_stats(db_path, "pointlio_odometry")
            if cnt == 0:
                continue
            if first_max is None:
                first_max = min_ts
            if args.max_sensor_sec > 0 and (max_ts - first_max) >= args.max_sensor_sec:
                print(
                    f"[pcap_to_db] reached --max-sensor-sec={args.max_sensor_sec:.1f}s",
                    flush=True,
                )
                break
            if max_ts == last_max:
                if stagnant_since is None:
                    stagnant_since = time.time()
                elif time.time() - stagnant_since > _STAGNANT_SEC:
                    break
            else:
                last_max = max_ts
                stagnant_since = None
    finally:
        coord.stop()

    o_cnt, o_min, o_max = _table_stats(db_path, "pointlio_odometry")
    l_cnt = _table_stats(db_path, "pointlio_lidar")[0]
    span = o_max - o_min
    print(
        f"[pcap_to_db] done odom={o_cnt} lidar={l_cnt} "
        f"ts=[{o_min:.3f}, {o_max:.3f}] span={span:.1f}s "
        f"wall={time.time() - t0:.1f}s",
        flush=True,
    )
    return 0


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pcap", required=True, help="Livox Mid-360 pcap capture")
    parser.add_argument(
        "--db",
        default=None,
        help="target memory2 SQLite db. If it exists, pointlio streams are appended/aligned "
        "onto its clock; if it doesn't, a fresh db is built from scratch. "
        "Omit to default to <pcap>.db next to the pcap.",
    )
    parser.add_argument(
        "--odom-freq",
        type=float,
        default=30.0,
        help="Point-LIO odometry publish rate in Hz (default 30)",
    )
    parser.add_argument(
        "--max-sensor-sec",
        type=float,
        default=0.0,
        help="stop after this many seconds of sensor time (0 = whole pcap)",
    )
    parser.add_argument(
        "--time-offset",
        type=float,
        default=None,
        help="seconds added to every output ts; omit to auto-derive from the db clock",
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="overwrite existing pointlio_odometry/pointlio_lidar streams in the db",
    )
    return _run(parser.parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
