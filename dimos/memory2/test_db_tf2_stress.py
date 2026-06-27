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

"""Adversarial + scale tests for DbTf2 — deliberately built to break it:

* topology that *changes* mid-recording (relocalization re-parents a frame): a
  lookup must resolve against the graph as-of its own time, not the latest one.
* many topology changes (multi-robot churn) forcing the query-per-lookup fallback
  — correctness must survive the fallback path, not just the in-RAM path.
* a really large / deep tf tree and a long recording — correctness vs a full-load
  buffer, and per-lookup cost bounded by chain depth, not by row count.
* edge cases: query before the first / after the last sample, a dynamic frame with
  a single sample, tolerance rejection, and frame-name collision across robots.
"""

from __future__ import annotations

from pathlib import Path
import time

from dimos.memory2.db_tf2 import DbTf2, TfGraphWriter
from dimos.memory2.store.sqlite import SqliteStore
from dimos.memory2.test_db_tf2 import _append, _diff, _yaw
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.protocol.tf.tf import MultiTBuffer

_T0 = 1000.0
_NO_PRUNE = 1.0e15


def _dyn(parent: str, child: str, x: float, ts: float) -> Transform:
    return Transform(
        translation=Vector3(x, 0.0, 0.0),
        rotation=_yaw(0.01 * x),
        frame_id=parent,
        child_frame_id=child,
        ts=ts,
    )


def _stat(parent: str, child: str, x: float, ts: float) -> Transform:
    return Transform(
        translation=Vector3(x, 0.0, 0.0),
        rotation=Quaternion(0, 0, 0, 1),
        frame_id=parent,
        child_frame_id=child,
        ts=ts,
    )


def _ref(transforms: list[Transform]) -> MultiTBuffer:
    buffer = MultiTBuffer(buffer_size=_NO_PRUNE)
    buffer.receive_transform(*transforms)
    return buffer


# --------------------------------------------------------------------------- #
# 1. Topology that CHANGES mid-run (relocalization re-parents base_link).
# --------------------------------------------------------------------------- #
def test_reparent_midrun_uses_graph_as_of_query_time(tmp_path: Path) -> None:
    """world->map(+10) and map->odom(+100) are non-identity statics. base_link is
    dynamic; at t0+5 a 'relocalization' re-parents base_link from odom to map. A
    lookup at an early time must compose the odom branch (+100); a lookup after the
    switch must NOT (+10 only). Picking the wrong era is off by ~100 -> caught."""
    path = tmp_path / "reparent.db"
    store = SqliteStore(path=str(path))
    graph = TfGraphWriter(str(path), "tf")

    _append(store, graph, _stat("world", "map", 10.0, _T0), is_static=True)
    _append(store, graph, _stat("map", "odom", 100.0, _T0), is_static=True)

    era1: list[Transform] = []
    for i in range(150):  # [t0, t0+5) at 30Hz
        ts = _T0 + i / 30.0
        t = _dyn("odom", "base_link", 0.5 * i / 30.0, ts)
        _append(store, graph, t, is_static=False)
        era1.append(t)

    switch = _T0 + 5.0
    era2: list[Transform] = []
    for i in range(150):  # [t0+5, t0+10): base_link now hangs off map directly
        ts = switch + i / 30.0
        t = _dyn("map", "base_link", 7.0 + 0.3 * i / 30.0, ts)
        _append(store, graph, t, is_static=False)
        era2.append(t)
    graph.close()
    store.stop()

    store = SqliteStore(path=str(path), must_exist=True)
    db = DbTf2(store)

    # early lookup: ground truth = statics(+10,+100) ∘ era1 dynamic
    q1 = _T0 + 2.013
    ref1 = _ref([_stat("world", "map", 10.0, q1), _stat("map", "odom", 100.0, q1), *era1])
    want1 = ref1.lookup("world", "base_link", q1, 0.5)
    got1 = db.get("world", "base_link", q1, 0.5)
    assert want1 is not None and got1 is not None
    assert _diff(want1, got1) < 1e-6
    assert got1.translation.x > 100.0  # odom branch present

    # late lookup: ground truth = world->map(+10) ∘ era2 dynamic (no odom)
    q2 = switch + 2.013
    ref2 = _ref([_stat("world", "map", 10.0, q2), *era2])
    want2 = ref2.lookup("world", "base_link", q2, 0.5)
    got2 = db.get("world", "base_link", q2, 0.5)
    assert want2 is not None and got2 is not None
    assert _diff(want2, got2) < 1e-6
    assert got2.translation.x < 20.0  # odom branch gone

    store.stop()


# --------------------------------------------------------------------------- #
# 2. Too many topology updates -> query-per-lookup fallback path.
# --------------------------------------------------------------------------- #
def test_many_topology_changes_force_fallback_and_stay_correct(tmp_path: Path) -> None:
    """N robots each join with their own frames -> many tf_graph rows. With the
    threshold below N, the graph is NOT held in RAM (graph_queries fire). Correctness
    must hold on the fallback path, including resolving the LATEST topology."""
    path = tmp_path / "churn.db"
    store = SqliteStore(path=str(path))
    graph = TfGraphWriter(str(path), "tf")

    robot_count = 40
    last_per_robot: dict[int, Transform] = {}
    for robot in range(robot_count):
        # each robot adds a new child frame at a distinct time -> a topology change
        for i in range(5):
            ts = _T0 + robot * 1.0 + i / 30.0
            t = _dyn(f"world_{robot}", f"base_{robot}", 0.5 * i + robot, ts)
            _append(store, graph, t, is_static=False)
            last_per_robot[robot] = t
    graph.close()
    store.stop()

    store = SqliteStore(path=str(path), must_exist=True)
    db = DbTf2(store, max_graph_changes_in_ram=10)  # 40 changes >= 10 -> fallback

    lookups = 0
    for robot in (0, 7, 20, 39):
        q = _T0 + robot * 1.0 + 4 / 30.0
        ref = _ref([last_per_robot[robot]])
        want = ref.lookup(f"world_{robot}", f"base_{robot}", q, 0.5)
        got = db.get(f"world_{robot}", f"base_{robot}", q, 0.5)
        assert want is not None and got is not None, f"robot {robot}"
        assert _diff(want, got) < 1e-6, f"robot {robot}: {_diff(want, got)}"
        lookups += 1

    assert db._graph_in_ram is None  # confirmed NOT cached
    assert db.graph_queries == lookups  # one graph query per lookup


# --------------------------------------------------------------------------- #
# 3. Really large / deep tf tree + long recording.
# --------------------------------------------------------------------------- #
def test_large_deep_tree_correct_and_bounded(tmp_path: Path) -> None:
    """A 30-link deep chain (world->link_0->...->link_29), each link dynamic at 30Hz
    for 60s = ~54k rows. DbTf2 must (a) match a full-load buffer and (b) keep
    per-lookup latency bounded by chain depth, not the 54k row count."""
    path = tmp_path / "big.db"
    store = SqliteStore(path=str(path))
    graph = TfGraphWriter(str(path), "tf")

    depth = 30
    duration_s = 60
    rate_hz = 30
    all_transforms: list[Transform] = []
    parents = ["world"] + [f"link_{d}" for d in range(depth)]
    for step in range(duration_s * rate_hz):
        ts = _T0 + step / rate_hz
        for d in range(depth):
            t = _dyn(parents[d], f"link_{d}", 0.1 * d + 0.01 * step, ts)
            _append(store, graph, t, is_static=False)
            all_transforms.append(t)
    graph.close()
    store.stop()

    row_count = depth * duration_s * rate_hz

    store = SqliteStore(path=str(path), must_exist=True)
    db = DbTf2(store)
    reference = _ref(all_transforms)

    target = f"link_{depth - 1}"
    compared = 0
    for k in range(25):
        q = _T0 + 0.017 + k * (duration_s / 26.0)
        want = reference.lookup("world", target, q, 0.5)
        got = db.get("world", target, q, 0.5)
        assert (want is None) == (got is None), f"None mismatch at {q}"
        if want is not None and got is not None:
            assert _diff(want, got) < 1e-6, f"diff at {q}: {_diff(want, got)}"
            compared += 1
    assert compared >= 20

    # bounded latency: warm, then time. Should be sub-millisecond despite 54k rows.
    for k in range(50):
        db.get("world", target, _T0 + 5.0 + k * 0.1, 0.5)
    n = 300
    start = time.perf_counter()
    for k in range(n):
        db.get("world", target, _T0 + 10.0 + (k % 400) * 0.1, 0.5)
    per_lookup_us = (time.perf_counter() - start) / n * 1e6
    assert per_lookup_us < 3000.0, f"per-lookup {per_lookup_us:.0f}us — not row-count bounded"
    print(f"\nlarge tree: {row_count} rows, depth {depth}, per-lookup {per_lookup_us:.0f}us")

    store.stop()


# --------------------------------------------------------------------------- #
# 4. Edge cases meant to break it.
# --------------------------------------------------------------------------- #
def _single_chain(path: Path) -> list[Transform]:
    store = SqliteStore(path=str(path))
    graph = TfGraphWriter(str(path), "tf")
    written: list[Transform] = []
    written.append(_stat("world", "odom", 1.0, _T0))
    _append(store, graph, written[-1], is_static=True)
    for i in range(100):  # [t0, t0+~3.3s)
        ts = _T0 + i / 30.0
        t = _dyn("odom", "base_link", 0.5 * i / 30.0, ts)
        _append(store, graph, t, is_static=False)
        written.append(t)
    graph.close()
    store.stop()
    return written


def test_query_before_topology_returns_none_cleanly(tmp_path: Path) -> None:
    """BEHAVIORAL DIFFERENCE vs the old full-load DbTf, pinned here on purpose:
    a query *before a frame ever appears* in the topology log resolves to None (not
    a crash). base_link's first topology entry coincides with its first sample, so a
    time earlier than the recording start has no base_link in the graph-as-of-then.
    This is defensible (the frame did not exist yet) but the full-load buffer would
    still answer it — flag on the DbTf->DbTf2 swap."""
    _single_chain(tmp_path / "r.db")
    store = SqliteStore(path=str(tmp_path / "r.db"), must_exist=True)
    db = DbTf2(store)
    assert db.get("world", "base_link", _T0 - 100.0, 1e9) is None  # clean, no crash
    # but a query at the very first sample (frame now exists) DOES resolve:
    assert db.get("world", "base_link", _T0, 1e9) is not None
    store.stop()


def test_query_after_last_sample(tmp_path: Path) -> None:
    """A time after the last sample: no 'hi' bracket; should resolve from the latest
    sample (lo) when tolerance allows."""
    _single_chain(tmp_path / "r.db")
    store = SqliteStore(path=str(tmp_path / "r.db"), must_exist=True)
    db = DbTf2(store)
    got = db.get("world", "base_link", _T0 + 100.0, 1e9)
    assert got is not None
    store.stop()


def test_tolerance_rejects_far_dynamic(tmp_path: Path) -> None:
    """A dynamic frame whose nearest sample is far outside tolerance must be rejected
    (None), not silently snapped to a stale value."""
    _single_chain(tmp_path / "r.db")
    store = SqliteStore(path=str(tmp_path / "r.db"), must_exist=True)
    db = DbTf2(store)
    got = db.get("world", "base_link", _T0 + 100.0, 0.01)  # 100s away, 10ms tol
    assert got is None
    store.stop()


def test_single_dynamic_sample(tmp_path: Path) -> None:
    """A dynamic frame with exactly one sample (no second bracket) must still resolve
    at that sample's time."""
    path = tmp_path / "one.db"
    store = SqliteStore(path=str(path))
    graph = TfGraphWriter(str(path), "tf")
    only = _dyn("world", "base_link", 3.0, _T0)
    _append(store, graph, only, is_static=False)
    graph.close()
    store.stop()
    store = SqliteStore(path=str(path), must_exist=True)
    db = DbTf2(store)
    got = db.get("world", "base_link", _T0, 0.5)
    assert got is not None and abs(got.translation.x - 3.0) < 1e-6
    store.stop()


def test_frame_name_collision_across_robots_is_documented(tmp_path: Path) -> None:
    """KNOWN LIMITATION probe: two robots that reuse the SAME frame names share rows
    in the tf table (child_frame is the only key). This test pins the CURRENT
    behavior so a future fix is a deliberate, visible change — not a silent one."""
    path = tmp_path / "collide.db"
    store = SqliteStore(path=str(path))
    graph = TfGraphWriter(str(path), "tf")
    # both robots publish odom->base_link with DIFFERENT motion at the same times
    for i in range(30):
        ts = _T0 + i / 30.0
        _append(store, graph, _dyn("odom", "base_link", 0.5 * i, ts), is_static=False)
        _append(store, graph, _dyn("odom", "base_link", -0.5 * i, ts + 1e-4), is_static=False)
    graph.close()
    store.stop()
    store = SqliteStore(path=str(path), must_exist=True)
    db = DbTf2(store)
    got = db.get("odom", "base_link", _T0 + 0.5, 0.5)
    assert got is not None  # resolves to *a* sample; which one is undefined-by-design
    store.stop()
