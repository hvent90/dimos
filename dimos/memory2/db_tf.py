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

"""Transform lookups over the transforms recorded in a store.

Instead of loading every recorded transform into RAM, each tf row carries a
small **tf tree**: a snapshot, as of that row, of the last two recorded rows per
child frame. The tree is stored in a sibling ``<stream>_tree`` table and its
values are tf-row ids — *keys into the tf table* — so editing a transform's pose
in place changes what the tree resolves to.

``store.tf.get(target, source, ts)`` then: finds the tf row nearest the query
time, reads its tree, fetches only the (<= 2 per frame) referenced rows, and
composes/interpolates the chain through a small :class:`MultiTBuffer`. A FIFO of
the last few lookups avoids re-querying the db for repeated identical lookups.

NOTE — retro-correction caveat: because a tree's edges are keys to *existing*
rows, the right way to correct history (e.g. after a loop closure) is to EDIT the
existing tf rows' poses in place — already-built trees pick that up for free.
Inserting NEW corrective tf messages does NOT work: trees built before the
insertion still point at the old rows and never see the new ones. (Re-running the
recompute below would rebuild trees to include them, but in-place edits are the
intended path.)

``write_tf_tree`` populates the tf stream for a recording that lacks one.
"""

from __future__ import annotations

from collections import OrderedDict
import json
import re
import sqlite3
import threading
from typing import TYPE_CHECKING, Any

import numpy as np

from dimos.memory2.store.sqlite import SqliteStoreConfig
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.protocol.tf.tf import MultiTBuffer
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.memory2.store.base import Store

logger = setup_logger()

# The unified tf stream the tree is built over. The recorder folds both the
# dynamic (`tf`) and static (`static_tf`) topics into this one stream/table so the
# tree's row-id keys are unambiguous (static is treated the same as dynamic).
DEFAULT_TF_STREAM = "tf"
# Streams the RAM fallback (non-sqlite stores) reads.
TF_STREAMS = ("tf", "tf_static")
# Larger than any single recording's span so the fallback buffer never prunes.
_NO_PRUNE = 1.0e15
# Last-N identical lookups cached to avoid re-querying the db.
_LOOKUP_CACHE_SIZE = 20
# SQLite can't parameterize table names, so caller-supplied stream names are
# interpolated; allow only safe identifiers to keep that injection-free.
_SAFE_TABLE = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")


def _safe_table(name: str) -> str:
    if not _SAFE_TABLE.match(name):
        raise ValueError(f"unsafe stream/table name: {name!r}")
    return name


def _connect(db_path: str) -> sqlite3.Connection:
    """A connection that waits on the WAL write-lock instead of erroring — the
    store keeps its own connection to the same db open while we read/write trees."""
    conn = sqlite3.connect(db_path, check_same_thread=False)
    conn.execute("PRAGMA busy_timeout=5000")
    return conn


def _advance(latest: dict[str, list[int]], child_frame: str, tf_id: int) -> None:
    """Push ``tf_id`` as the newest reference for ``child_frame``, keeping <= 2."""
    refs = latest.get(child_frame)
    if refs is None:
        latest[child_frame] = [tf_id]
    else:
        refs.append(tf_id)
        if len(refs) > 2:
            del refs[0]


class TfTreeWriter:
    """Builds tf trees incrementally as rows are appended (used by the recorder).

    Holds the running ``{child_frame: [older_id, newer_id]}`` state and, after each
    tf row is appended, writes that row's full tree snapshot to ``<stream>_tree``.
    """

    def __init__(self, db_path: str, stream: str = DEFAULT_TF_STREAM) -> None:
        self._stream = _safe_table(stream)
        self._conn = _connect(db_path)
        self._lock = threading.Lock()
        self._latest: dict[str, list[int]] = {}
        ensure_tree_table(self._conn, self._stream)

    def record(self, child_frame: str, tf_id: int) -> None:
        """Record that tf row ``tf_id`` holds the latest ``child_frame`` edge."""
        with self._lock:
            _advance(self._latest, child_frame, tf_id)
            snapshot = json.dumps(self._latest)
            self._conn.execute(
                f'INSERT OR REPLACE INTO "{self._stream}_tree" (tf_id, tree) VALUES (?, ?)',
                (tf_id, snapshot),
            )
            self._conn.commit()

    def close(self) -> None:
        with self._lock:
            self._conn.close()


def ensure_tree_table(conn: sqlite3.Connection, stream: str) -> None:
    conn.execute(
        f'CREATE TABLE IF NOT EXISTS "{_safe_table(stream)}_tree" '
        "(tf_id INTEGER PRIMARY KEY, tree TEXT NOT NULL)"
    )
    # Speeds up the nearest-row lookup in get(); harmless if it already exists.
    conn.execute(
        f'CREATE INDEX IF NOT EXISTS "{_safe_table(stream)}_ts_idx" ON "{_safe_table(stream)}"(ts)'
    )
    conn.commit()


def recompute_trees(store: Store, stream: str = DEFAULT_TF_STREAM) -> int:
    """(Re)build every tf row's tree in chronological order, writing it back.

    Iterates the whole stream once — the expensive, one-time cost of adopting a
    recording that predates trees. Returns the number of tf rows given a tree.
    """
    config = store.config
    if not isinstance(config, SqliteStoreConfig):
        raise TypeError("recompute_trees needs a SqliteStore")
    conn = _connect(config.path)
    try:
        ensure_tree_table(conn, stream)
        conn.execute(f'DELETE FROM "{_safe_table(stream)}_tree"')
        latest: dict[str, list[int]] = {}
        rows: list[tuple[int, str]] = []
        for observation in store.stream(stream, TFMessage).order_by("ts"):
            transforms = getattr(observation.data, "transforms", None) or [observation.data]
            for transform in transforms:
                _advance(latest, transform.child_frame_id, observation.id)
            rows.append((observation.id, json.dumps(latest)))
        conn.executemany(
            f'INSERT OR REPLACE INTO "{_safe_table(stream)}_tree" (tf_id, tree) VALUES (?, ?)',
            rows,
        )
        conn.commit()
        return len(rows)
    finally:
        conn.close()


class DbTf:
    """Transform lookups backed by a store's tf stream + per-row tf trees."""

    def __init__(
        self,
        store: Store,
        stream: str = DEFAULT_TF_STREAM,
        stream_names: tuple[str, ...] = TF_STREAMS,
        cache_size: int = _LOOKUP_CACHE_SIZE,
    ) -> None:
        self._store = store
        self._stream = _safe_table(stream)
        self._stream_names = stream_names  # RAM fallback only
        self._cache_size = cache_size
        self._lock = threading.Lock()
        self._conn: sqlite3.Connection | None = None
        self._trees_ready = False
        self._cache: OrderedDict[tuple[str, str, float | None, float | None], Transform | None] = (
            OrderedDict()
        )
        # Instrumentation: number of tf rows decoded out of the db by get() (for
        # the "no full load" test). The whole-table recompute is not counted.
        self.rows_fetched = 0
        # RAM fallback for non-sqlite stores.
        self._buffer: MultiTBuffer | None = None

    # --- sqlite path -----------------------------------------------------------

    @property
    def _is_sqlite(self) -> bool:
        return isinstance(self._store.config, SqliteStoreConfig)

    def _connection(self) -> sqlite3.Connection:
        if self._conn is None:
            config = self._store.config
            assert isinstance(config, SqliteStoreConfig)  # guarded by _is_sqlite
            self._conn = _connect(config.path)
        return self._conn

    def _ensure_trees(self) -> None:
        """Make sure every tf row has a tree; recompute (loudly) if not."""
        if self._trees_ready:
            return
        conn = self._connection()
        ensure_tree_table(conn, self._stream)
        (n_rows,) = conn.execute(f'SELECT count(*) FROM "{self._stream}"').fetchone()
        (n_trees,) = conn.execute(f'SELECT count(*) FROM "{self._stream}_tree"').fetchone()
        if n_rows and n_trees < n_rows:
            logger.warning(
                "\n"
                "========================================================================\n"
                "  tf tree MISSING for stream %r (%d/%d rows have a tree).\n"
                "  Computing tf trees over the whole recording (one-time, chronological,\n"
                "  written back to the db). This can take a moment on large recordings.\n"
                "========================================================================",
                self._stream,
                n_trees,
                n_rows,
            )
            built = recompute_trees(self._store, self._stream)
            logger.warning("tf tree computed for %d rows of stream %r.", built, self._stream)
        self._trees_ready = True

    def _fetch_transforms(self, ids: list[int]) -> list[Transform]:
        """Decode the given tf-row ids into transforms (frames live in the blob)."""
        if not ids:
            return []
        backend: Any = self._store.stream(self._stream, TFMessage)._source
        transforms: list[Transform] = []
        for tf_id in ids:
            # _make_loader is the same blob-load + codec-decode the stream uses.
            message = backend._make_loader(tf_id)()
            transforms.extend(getattr(message, "transforms", None) or [message])
        self.rows_fetched += len(ids)
        return transforms

    def _anchor_id(self, conn: sqlite3.Connection, time_point: float | None) -> int | None:
        """The tf row whose tree to use: the first row with ts >= query (so its
        own edge brackets the query for interpolation); else the last row."""
        if time_point is None:
            row = conn.execute(
                f'SELECT id FROM "{self._stream}" ORDER BY ts DESC LIMIT 1'
            ).fetchone()
            return int(row[0]) if row else None
        row = conn.execute(
            f'SELECT id FROM "{self._stream}" WHERE ts >= ? ORDER BY ts ASC LIMIT 1', (time_point,)
        ).fetchone()
        if row is not None:
            return int(row[0])
        row = conn.execute(f'SELECT id FROM "{self._stream}" ORDER BY ts DESC LIMIT 1').fetchone()
        return int(row[0]) if row else None

    def _get_sqlite(
        self,
        target_frame: str,
        source_frame: str,
        time_point: float | None,
        time_tolerance: float | None,
    ) -> Transform | None:
        key = (target_frame, source_frame, time_point, time_tolerance)
        with self._lock:
            if key in self._cache:
                self._cache.move_to_end(key)
                return self._cache[key]
        self._ensure_trees()
        conn = self._connection()
        anchor = self._anchor_id(conn, time_point)
        result: Transform | None = None
        if anchor is not None:
            row = conn.execute(
                f'SELECT tree FROM "{self._stream}_tree" WHERE tf_id = ?', (anchor,)
            ).fetchone()
            if row is not None:
                tree: dict[str, list[int]] = json.loads(row[0])
                ids = sorted({tf_id for refs in tree.values() for tf_id in refs})
                # Only the <=2-per-frame referenced rows are read — never the
                # whole table. MultiTBuffer composes the chain + interpolates.
                buffer = MultiTBuffer(buffer_size=_NO_PRUNE)
                transforms = self._fetch_transforms(ids)
                if transforms:
                    buffer.receive_transform(*transforms)
                    result = buffer.lookup(target_frame, source_frame, time_point, time_tolerance)
        with self._lock:
            self._cache[key] = result
            self._cache.move_to_end(key)
            while len(self._cache) > self._cache_size:
                self._cache.popitem(last=False)
        return result

    # --- RAM fallback (non-sqlite stores) --------------------------------------

    def _ensure_loaded(self) -> MultiTBuffer:
        if self._buffer is not None:
            return self._buffer
        with self._lock:
            if self._buffer is not None:
                return self._buffer
            buffer = MultiTBuffer(buffer_size=_NO_PRUNE)
            available = set(self._store.list_streams())
            for name in self._stream_names:
                if name not in available:
                    continue
                for observation in self._store.stream(name, TFMessage):
                    transforms = getattr(observation.data, "transforms", None) or [observation.data]
                    buffer.receive_transform(*transforms)
            self._buffer = buffer
            return buffer

    # --- public API ------------------------------------------------------------

    def has_transforms(self) -> bool:
        if not self._is_sqlite:
            return bool(self._ensure_loaded().buffers)
        conn = self._connection()
        if self._stream not in set(self._store.list_streams()):
            return False
        (n_rows,) = conn.execute(f'SELECT count(*) FROM "{self._stream}"').fetchone()
        return bool(n_rows)

    def get(
        self,
        target_frame: str,
        source_frame: str,
        time_point: float | None = None,
        time_tolerance: float | None = None,
    ) -> Transform | None:
        """Transform that maps a point in ``source_frame`` into ``target_frame``.

        Returns ``None`` if no chain connects the two frames at the requested
        time. Resolves through the per-row tf tree (O(tree size) rows read), with
        a small FIFO cache over identical lookups.
        """
        if not self._is_sqlite:
            return self._ensure_loaded().lookup(
                target_frame, source_frame, time_point, time_tolerance
            )
        return self._get_sqlite(target_frame, source_frame, time_point, time_tolerance)


def transform_matrix(transform: Transform) -> tuple[np.ndarray, np.ndarray]:
    """Return ``(R, t)`` (3x3, 3) for ``transform`` so ``p_target = p_source @ R.T + t``."""
    rotation = transform.rotation
    rotation_matrix = np.asarray(rotation.to_rotation_matrix(), float).reshape(3, 3)
    translation = np.array(
        [transform.translation.x, transform.translation.y, transform.translation.z], float
    )
    return rotation_matrix, translation


def write_tf_tree(
    store: Store,
    *,
    odom_stream: str,
    odom_parent: str = "odom",
    odom_child: str = "base_link",
    root_links: tuple[tuple[str, str], ...] = (("world", "map"), ("map", "odom")),
    sensor_child: str = "mid360_link",
    sensor_translation: tuple[float, float, float] = (0.0, 0.0, 0.0),
    sensor_rotation: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0),
    static_period: float = 0.45,
    stream_name: str = "tf",
) -> int:
    """Populate ``store``'s tf stream from an odometry stream.

    - ``root_links`` and ``odom_child -> sensor_child`` are emitted as identity /
      fixed transforms every ``static_period`` seconds across the recording span.
    - ``odom_parent -> odom_child`` is emitted once per odometry sample, taken
      from each observation's pose.

    Returns the number of tf observations written.
    """
    config = store.config
    if not isinstance(config, SqliteStoreConfig):
        raise TypeError("write_tf_tree reads the db directly and needs a SqliteStore")
    db_path = config.path
    connection = sqlite3.connect(f"file:{db_path}?mode=ro&immutable=1", uri=True)
    odom = np.array(
        list(
            connection.execute(
                "select ts,pose_x,pose_y,pose_z,pose_qx,pose_qy,pose_qz,pose_qw "
                f"from {_safe_table(odom_stream)} order by ts"
            )
        ),
        float,
    )
    connection.close()
    if not len(odom):
        raise ValueError(f"odom stream {odom_stream!r} is empty; cannot build tf tree")

    tf_stream = store.stream(stream_name, TFMessage)
    written = 0

    # dynamic: odom_parent -> odom_child, one per odometry sample
    for row in odom:
        ts = float(row[0])
        transform = Transform(
            translation=Vector3(row[1], row[2], row[3]),
            rotation=Quaternion(row[4], row[5], row[6], row[7]),
            frame_id=odom_parent,
            child_frame_id=odom_child,
            ts=ts,
        )
        tf_stream.append(TFMessage(transform), ts=ts)
        written += 1

    # static: root links + sensor mount, resampled every static_period
    t0 = float(odom[0, 0])
    t1 = float(odom[-1, 0])

    def statics_at(ts: float) -> list[Transform]:
        links = [
            Transform(frame_id=parent, child_frame_id=child, ts=ts) for parent, child in root_links
        ]
        links.append(
            Transform(
                translation=Vector3(*sensor_translation),
                rotation=Quaternion(*sensor_rotation),
                frame_id=odom_child,
                child_frame_id=sensor_child,
                ts=ts,
            )
        )
        return links

    for static_ts in np.arange(t0, t1 + static_period, static_period):
        tf_stream.append(TFMessage(*statics_at(float(static_ts))), ts=float(static_ts))
        written += 1

    return written
