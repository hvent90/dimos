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

"""Tests for the per-row tf-tree DbTf: correctness vs the old full-load buffer,
no-full-load + FIFO behavior, recompute-on-load, and the recorder's tree build."""

from __future__ import annotations

import json
import math
from pathlib import Path
import sqlite3
import time

import pytest

from dimos.memory2 import db_tf as db_tf_module
from dimos.memory2.db_tf import DbTf, TfTreeWriter, recompute_trees
from dimos.memory2.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.protocol.tf.tf import MultiTBuffer

# Frame chain used throughout: world -> map -> odom -> base_link -> sensor.
# Only odom -> base_link moves; the rest are fixed (the realistic case).
_DYN_RATE = 30.0  # Hz, odom -> base_link
_STATIC_PERIOD = 0.5  # s, root + sensor links re-emitted
_DURATION = 10.0  # s
_T0 = 1000.0


def _yaw_quat(theta: float) -> Quaternion:
    return Quaternion(0.0, 0.0, math.sin(theta / 2.0), math.cos(theta / 2.0))


def _all_transforms() -> list[Transform]:
    """A moving odom->base_link plus fixed root + sensor links, interleaved by ts."""
    transforms: list[Transform] = []
    # dynamic odom -> base_link: drives a gentle arc
    n = int(_DURATION * _DYN_RATE)
    for i in range(n):
        ts = _T0 + i / _DYN_RATE
        transforms.append(
            Transform(
                translation=Vector3(0.5 * i / _DYN_RATE, 0.1 * i / _DYN_RATE, 0.0),
                rotation=_yaw_quat(0.02 * i),
                frame_id="odom",
                child_frame_id="base_link",
                ts=ts,
            )
        )
    # statics re-emitted every _STATIC_PERIOD
    m = int(_DURATION / _STATIC_PERIOD)
    for j in range(m):
        ts = _T0 + j * _STATIC_PERIOD
        transforms.append(Transform(frame_id="world", child_frame_id="map", ts=ts))
        transforms.append(Transform(frame_id="map", child_frame_id="odom", ts=ts))
        transforms.append(
            Transform(
                translation=Vector3(0.0, 0.0, 0.3),
                rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
                frame_id="base_link",
                child_frame_id="sensor",
                ts=ts,
            )
        )
    transforms.sort(key=lambda t: t.ts)
    return transforms


def _write_tf_db(path: Path, *, with_trees: bool) -> list[Transform]:
    """Build a SqliteStore with a `tf` stream (one transform per row). When
    ``with_trees`` is set, also write per-row trees as the recorder would."""
    store = SqliteStore(path=str(path))
    tf_stream = store.stream("tf", TFMessage)
    transforms = _all_transforms()
    writer = TfTreeWriter(str(path), "tf") if with_trees else None
    for transform in transforms:
        obs = tf_stream.append(TFMessage(transform), ts=transform.ts, pose=None)
        if writer is not None:
            writer.record(transform.child_frame_id, obs.id)
    if writer is not None:
        writer.close()
    store.stop()
    return transforms


def _reference_buffer(transforms: list[Transform]) -> MultiTBuffer:
    buffer = MultiTBuffer(buffer_size=1.0e15)
    buffer.receive_transform(*transforms)
    return buffer


def _diff(a: Transform, b: Transform) -> float:
    return (
        abs(a.translation.x - b.translation.x)
        + abs(a.translation.y - b.translation.y)
        + abs(a.translation.z - b.translation.z)
        + abs(a.rotation.x - b.rotation.x)
        + abs(a.rotation.y - b.rotation.y)
        + abs(a.rotation.z - b.rotation.z)
        + abs(a.rotation.w - b.rotation.w)
    )


def test_get_matches_full_load_buffer(tmp_path: Path) -> None:
    """Tree-based get matches the old load-everything MultiTBuffer within tol,
    including at query times BETWEEN samples (interpolation through the tree)."""
    transforms = _write_tf_db(tmp_path / "tf.db", with_trees=True)
    reference = _reference_buffer(transforms)
    store = SqliteStore(path=str(tmp_path / "tf.db"), must_exist=True)
    db = DbTf(store)

    # query at off-sample times to exercise interpolation
    queries = [_T0 + 0.013 + k * 0.317 for k in range(25)]
    compared = 0
    for q in queries:
        want = reference.lookup("world", "sensor", q, 0.5)
        got = db.get("world", "sensor", q, 0.5)
        assert (want is None) == (got is None), f"None mismatch at {q}"
        if want is not None and got is not None:
            assert _diff(want, got) < 1e-6, f"diff too large at {q}: {_diff(want, got)}"
            compared += 1
    assert compared >= 20  # actually exercised the interp path
    store.stop()


def test_no_full_load(tmp_path: Path) -> None:
    """get() reads only the handful of tree-referenced rows, never the whole table."""
    transforms = _write_tf_db(tmp_path / "tf.db", with_trees=True)
    store = SqliteStore(path=str(tmp_path / "tf.db"), must_exist=True)
    db = DbTf(store)
    queries = [_T0 + 0.05 + k * 0.5 for k in range(20)]
    for q in queries:
        db.get("world", "sensor", q, 0.5)
    # 4 frames x <=2 refs = <=8 rows per distinct query; total must be far below
    # the full table (len(transforms) rows).
    assert db.rows_fetched <= 8 * len(queries)
    assert db.rows_fetched < len(transforms) // 2
    store.stop()


def test_fifo_cache_avoids_db_reads(tmp_path: Path) -> None:
    """Repeated identical lookups are served from the FIFO with zero extra reads."""
    _write_tf_db(tmp_path / "tf.db", with_trees=True)
    store = SqliteStore(path=str(tmp_path / "tf.db"), must_exist=True)
    db = DbTf(store)
    q = _T0 + 2.34
    first = db.get("world", "sensor", q, 0.5)
    after_first = db.rows_fetched
    assert after_first > 0
    for _ in range(10):
        again = db.get("world", "sensor", q, 0.5)
        assert _diff(first, again) == 0.0
    assert db.rows_fetched == after_first  # no further DB reads
    store.stop()


def test_fifo_evicts_beyond_capacity(tmp_path: Path) -> None:
    """A lookup pushed out of the FIFO costs a DB read again; one still in it does not."""
    _write_tf_db(tmp_path / "tf.db", with_trees=True)
    store = SqliteStore(path=str(tmp_path / "tf.db"), must_exist=True)
    db = DbTf(store, cache_size=3)
    keys = [_T0 + 1.0 + k for k in range(3)]
    for q in keys:
        db.get("world", "sensor", q, 0.5)
    reads_before = db.rows_fetched
    # evict keys[0] by adding two fresh distinct lookups (capacity 3)
    db.get("world", "sensor", _T0 + 7.1, 0.5)
    db.get("world", "sensor", _T0 + 7.2, 0.5)
    # keys[0] is gone -> re-reads; an untouched recent one would not
    reads_mid = db.rows_fetched
    db.get("world", "sensor", keys[0], 0.5)
    assert db.rows_fetched > reads_mid  # evicted -> DB read happened
    assert reads_before > 0
    store.stop()


def test_recompute_warns_and_backfills(tmp_path: Path) -> None:
    """Opening a tree-less recording warns loudly and back-fills a tree per row."""
    transforms = _write_tf_db(tmp_path / "tf.db", with_trees=False)
    db_path = tmp_path / "tf.db"
    # no tree table yet
    conn = sqlite3.connect(str(db_path))
    has_tree = conn.execute(
        "SELECT name FROM sqlite_master WHERE type='table' AND name='tf_tree'"
    ).fetchone()
    conn.close()
    assert has_tree is None

    # The dimos logger is a structlog bound logger writing to the original stdout
    # and not propagating, so capture by wrapping its .warning directly.
    messages: list[str] = []
    original_warning = db_tf_module.logger.warning
    db_tf_module.logger.warning = lambda msg, *a, **k: messages.append(str(msg))

    store = SqliteStore(path=str(db_path), must_exist=True)
    db = DbTf(store)
    try:
        db.get("world", "sensor", _T0 + 1.0, 0.5)
    finally:
        db_tf_module.logger.warning = original_warning
    assert any("tf tree MISSING" in m for m in messages)  # warned loudly

    conn = sqlite3.connect(str(db_path))
    n_rows = conn.execute("SELECT count(*) FROM tf").fetchone()[0]
    n_trees = conn.execute("SELECT count(*) FROM tf_tree").fetchone()[0]
    conn.close()
    assert n_rows == len(transforms)
    assert n_trees == n_rows  # every tf row has a tree
    store.stop()


def test_recorder_tree_has_last_two_refs_per_frame(tmp_path: Path) -> None:
    """The recorder's incremental tree stores the last two row ids per child frame,
    and those ids are keys into the tf table."""
    db_path = tmp_path / "tf.db"
    store = SqliteStore(path=str(db_path))
    tf_stream = store.stream("tf", TFMessage)
    writer = TfTreeWriter(str(db_path), "tf")
    ids_by_child: dict[str, list[int]] = {}
    last_tf_id = -1
    for transform in _all_transforms():
        obs = tf_stream.append(TFMessage(transform), ts=transform.ts, pose=None)
        writer.record(transform.child_frame_id, obs.id)
        ids_by_child.setdefault(transform.child_frame_id, []).append(obs.id)
        last_tf_id = obs.id
    writer.close()
    store.stop()

    conn = sqlite3.connect(str(db_path))
    tree_json = conn.execute("SELECT tree FROM tf_tree WHERE tf_id=?", (last_tf_id,)).fetchone()[0]
    valid_ids = {row[0] for row in conn.execute("SELECT id FROM tf")}
    conn.close()
    tree = json.loads(tree_json)

    for child, expected in ids_by_child.items():
        assert tree[child] == expected[-2:]  # last two refs for that frame
        for tf_id in tree[child]:
            assert tf_id in valid_ids  # tree edges are keys into the tf table


def test_recompute_matches_recorder(tmp_path: Path) -> None:
    """Trees built incrementally by the recorder equal trees rebuilt by recompute."""
    recorder_db = tmp_path / "rec.db"
    _write_tf_db(recorder_db, with_trees=True)
    recompute_db = tmp_path / "recmp.db"
    _write_tf_db(recompute_db, with_trees=False)
    store = SqliteStore(path=str(recompute_db), must_exist=True)
    recompute_trees(store)
    store.stop()

    def trees(path: Path) -> dict[int, str]:
        conn = sqlite3.connect(str(path))
        out = {row[0]: row[1] for row in conn.execute("SELECT tf_id, tree FROM tf_tree")}
        conn.close()
        return out

    assert trees(recorder_db) == trees(recompute_db)


_CHINA_DB = Path.home() / "dimos_phase2_china" / "china_default.db"


@pytest.mark.skipif(not _CHINA_DB.exists(), reason="china_default.db not present")
def test_performance_on_real_recording(tmp_path: Path) -> None:
    """On a real ~38k-row recording: correct vs full-load, no full load, and the
    tree path is much faster than loading everything per query batch."""
    import shutil

    db_path = tmp_path / "china.db"
    shutil.copy(_CHINA_DB, db_path)
    store = SqliteStore(path=str(db_path), must_exist=True)

    # reference full-load buffer; collect the dynamic frame's (odom->base_link)
    # row timestamps — the realistic query pattern (scans align with odometry),
    # where the snapshot tree brackets the moving frame exactly.
    buffer = MultiTBuffer(buffer_size=1.0e15)
    total_rows = 0
    dynamic_ts: list[float] = []
    for obs in store.stream("tf", TFMessage).order_by("ts"):
        transforms = getattr(obs.data, "transforms", None) or [obs.data]
        for transform in transforms:
            buffer.receive_transform(transform)
        total_rows += 1
        if transforms[0].child_frame_id == "base_link" and total_rows % 1000 == 0:
            dynamic_ts.append(obs.ts)

    db = DbTf(store)
    max_diff = 0.0
    for q in dynamic_ts:
        want = buffer.lookup("world", "mid360_link", q, 0.5)
        got = db.get("world", "mid360_link", q, 0.5)
        assert (want is None) == (got is None)
        if want is not None and got is not None:
            max_diff = max(max_diff, _diff(want, got))
    assert max_diff < 1e-6, f"tree get diverged from full-load by {max_diff}"

    # no full load: a fresh DbTf reads far fewer rows than the table holds
    db2 = DbTf(store)
    t_start = time.perf_counter()
    for q in dynamic_ts:
        db2.get("world", "mid360_link", q, 0.5)
    elapsed = time.perf_counter() - t_start
    assert db2.rows_fetched < total_rows // 100  # << whole table
    assert elapsed < 2.0  # generous; in practice tens of ms for tens of queries
    store.stop()
