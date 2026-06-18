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

"""Telemetry recorder for the Go2 characterization / benchmark blueprints.

Captures the live streams a session produces — commanded twist,
coord-aggregated joint_state, raw odom, and operator gate events —
into a per-session SQLite DB so post-process tools can re-fit, dissect
spikes, or compare runs without re-running on hardware.

Per-session DB filenames have a ``tag`` to differentiate use cases
(characterization runs vs benchmark runs land in separate DBs even on
the same date+sha): ``<robot_id>_<tag>_<date>_<sha>.db``. By default
files land in ``<repo>/data/characterization/<robot_id>/`` — both the
``out_dir`` and ``tag`` are config-settable so the benchmark blueprint
can route to ``<repo>/data/benchmark/<robot_id>/`` with ``tag="benchmark"``.

Read back with::

    from dimos.memory2.store.sqlite import SqliteStore
    store = SqliteStore(path="<the .db file>")
    store.start()
    for obs in store.stream("joint_state", JointState).iterate_ts():
        ts, msg = obs.ts, obs.data
        # re-fit, plot, etc.
"""

from __future__ import annotations

from datetime import date
from pathlib import Path
import time
from typing import Any

from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.stream import In, Stream
from dimos.memory2.module import Recorder, RecorderConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.std_msgs.Int8 import Int8
from dimos.utils.benchmarking.tuning import git_sha
from dimos.utils.path_utils import get_project_root

DEFAULT_OUT_DIR = get_project_root() / "data" / "characterization"


def _next_free_db_path(out_dir: Path, stem: str) -> Path:
    """First non-colliding ``<stem>.db`` (then ``<stem>_01.db``, ``_02`` ...).

    The date+sha stem isn't unique within a day, so a repeated run would
    otherwise overwrite or fail on an existing recording. This keeps every
    run's data under its own readable, ordered name.
    """
    candidate = out_dir / f"{stem}.db"
    index = 1
    while candidate.exists():
        candidate = out_dir / f"{stem}_{index:02d}.db"
        index += 1
    return candidate


class CharacterizationRecorderConfig(RecorderConfig):
    """Same as :class:`RecorderConfig` but with per-session db_path
    resolution from ``out_dir`` + ``robot_id`` + ``tag``. Set ``db_path``
    explicitly to bypass the default naming convention."""

    out_dir: str | None = None  # None -> DEFAULT_OUT_DIR / <robot_id>
    robot_id: str = "go2"
    tag: str = "recording"  # filename discriminator: "recording" (char), "benchmark" (bench)
    # Timestamped filenames make rerun-safe defaults; never silently
    # clobber prior recordings.
    overwrite: bool = False


class CharacterizationRecorder(Recorder):
    """Records the streams a characterization session emits.

    Composed alongside :class:`~dimos.utils.benchmarking.characterization.Characterizer`
    in the ``unitree-go2-characterization`` blueprint; ports are wired
    to the same LCM topics the rest of the stack already uses (LCM is
    multicast — additional subscribers are free).
    """

    config: CharacterizationRecorderConfig

    cmd_vel: In[Twist]  # commanded /cmd_vel during each SI step
    joint_state: In[JointState]  # /coordinator/joint_state (x, y, yaw)
    odom: In[PoseStamped]  # raw /go2/odom from GO2Connection
    gate: In[Int8]  # operator gate events (advance/skip/quit)

    @rpc
    def start(self) -> None:
        # Resolve a per-session db_path before super().start() opens the
        # SqliteStore. Mirrors the JSON+PNG artifact naming so a
        # session's three files (config.json, fits.png, recording.db)
        # land in the same dir with the same date+sha suffix.
        out_dir = (
            Path(self.config.out_dir).expanduser()
            if self.config.out_dir
            else DEFAULT_OUT_DIR / self.config.robot_id
        )
        out_dir.mkdir(parents=True, exist_ok=True)
        stem = f"{self.config.robot_id}_{self.config.tag}_{date.today().isoformat()}_{git_sha()}"
        self.config.db_path = _next_free_db_path(out_dir, stem)
        super().start()

    def _port_to_stream(self, name: str, input_topic: In[Any], stream: Stream[Any]) -> None:
        """Override the parent to skip the per-message TF lookup. These
        blueprints don't publish a TF graph, so the parent's "No tf
        available" warning fires once per message and floods the terminal.
        Pose is stored as ``None``; rerun-time analysis tools either don't
        need a pose anchor or pull it from the recorded ``odom`` stream."""
        import sqlite3
        from threading import Lock

        last_ts = [0.0]  # mutable single-cell for the closure
        lock = Lock()
        _EPS = 1e-6  # 1 microsecond

        def on_msg(msg: Any) -> None:
            raw_ts = getattr(msg, "ts", None) or time.time()
            with lock:
                # Strictly monotonic: bump duplicates / regressions to
                # last + EPS so the primary key never collides.
                ts = raw_ts if raw_ts > last_ts[0] else last_ts[0] + _EPS
                last_ts[0] = ts
            try:
                stream.append(msg, ts=ts, pose=None)
            except sqlite3.IntegrityError:
                # Defense in depth — never let the LCM thread die.
                pass

        self.register_disposable(Disposable(input_topic.subscribe(on_msg)))


__all__ = ["CharacterizationRecorder", "CharacterizationRecorderConfig"]
