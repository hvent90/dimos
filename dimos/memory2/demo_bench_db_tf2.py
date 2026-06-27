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

"""Benchmark: DbTf (per-row tf-tree) vs DbTf2 (graph-stream + in-RAM graph).

Builds a synthetic single-robot recording (one transform per row, child_frame
tagged, topology change-log), then measures per-lookup latency and SQL query
counts for both implementations. Run: `python -m dimos.memory2.demo_bench_db_tf2`.
"""

from __future__ import annotations

from pathlib import Path
import tempfile
import time

from dimos.memory2.db_tf import DbTf
from dimos.memory2.db_tf2 import DbTf2
from dimos.memory2.store.sqlite import SqliteStore
from dimos.memory2.test_db_tf2 import _T0, _record_single_robot

_WARMUP = 200
_N = 2000


def _count_queries(db: object, query_time: float) -> int:
    conn = db._connection()  # type: ignore[attr-defined]
    count = {"n": 0}

    def trace(_sql: str) -> None:
        count["n"] += 1

    conn.set_trace_callback(trace)
    db.get("world", "sensor", query_time, 0.5)  # type: ignore[attr-defined]
    conn.set_trace_callback(None)
    return count["n"]


def _time_lookups(db: object, n: int, offset: int) -> float:
    start = time.perf_counter()
    for k in range(n):
        q = _T0 + 0.05 + ((offset + k) % 290) * 0.033
        db.get("world", "sensor", q, 0.5)  # type: ignore[attr-defined]
    return time.perf_counter() - start


def main() -> None:
    tmp = Path(tempfile.mkdtemp())
    path = tmp / "bench.db"
    _record_single_robot(path, static_repeat=True)

    results = {}
    for name, make in (("DbTf ", lambda s: DbTf(s)), ("DbTf2", lambda s: DbTf2(s))):
        store = SqliteStore(path=str(path), must_exist=True)
        db = make(store)

        # one-time build cost (tree / graph migration) folded into a cold first call
        cold_start = time.perf_counter()
        db.get("world", "sensor", _T0 + 0.05, 0.5)
        cold = time.perf_counter() - cold_start

        first_q = _count_queries(db, _T0 + 7.123)
        warm_q = _count_queries(db, _T0 + 3.777)

        _time_lookups(db, _WARMUP, offset=0)  # warm caches/pages
        warm_total = _time_lookups(db, _N, offset=_WARMUP)

        graph_q = getattr(db, "graph_queries", None)
        results[name] = {
            "cold_ms": cold * 1e3,
            "warm_us": warm_total / _N * 1e6,
            "first_q": first_q,
            "warm_q": warm_q,
            "graph_q": graph_q,
        }
        store.stop()

    print(f"\nrecording: single robot, 5 frames, 30Hz dynamic, 10s  |  lookups timed: {_N}\n")
    header = f"{'impl':6} {'cold(ms)':>10} {'warm/lookup(us)':>17} {'1st-call q':>11} {'warm q':>8} {'graph q':>8}"
    print(header)
    print("-" * len(header))
    for name, r in results.items():
        gq = "-" if r["graph_q"] is None else r["graph_q"]
        print(
            f"{name:6} {r['cold_ms']:10.1f} {r['warm_us']:17.1f} "
            f"{r['first_q']:11d} {r['warm_q']:8d} {gq!s:>8}"
        )
    print()


if __name__ == "__main__":
    main()
