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

"""Make short.db + short.rrd: the first N seconds of a recording.

Given a recording dir, copies only the rows within [first_timestamp,
first_timestamp + seconds] into a fresh `short.db` (schema-faithful: data,
`_blob` and `_rtree` tables, so poses/tags survive), then rebuilds `short.rrd`
from it. Copies only the in-window rows (never the whole db), so it stays small
and works even when the source is huge / the disk is nearly full.

    uv run python dimos/mapping/recording/utils/short.py REC_DIR [--seconds 30]
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sqlite3

from dimos.mapping.recording.utils import stream_names
from dimos.mapping.recording.utils.trunc import rebuild_rrd

DB_NAME = "mem2.db"
SHORT_DB = "short.db"
SHORT_RRD = "short.rrd"
DEFAULT_SECONDS = 30.0
# rtree shadow tables are created automatically by the virtual table, not by us.
_RTREE_SHADOWS = ("_rtree_node", "_rtree_parent", "_rtree_rowid")


def _first_ts(conn: sqlite3.Connection) -> float | None:
    """Earliest timestamp across all streams."""
    mins: list[float] = []
    for (name,) in conn.execute("SELECT name FROM _streams"):
        try:
            value = conn.execute(f'SELECT MIN(ts) FROM "{name}"').fetchone()[0]
        except sqlite3.OperationalError:
            continue
        if value is not None:
            mins.append(value)
    return min(mins) if mins else None


def make_short_db(src_db: str, short_db: str, seconds: float) -> tuple[float, float, int]:
    """Build `short_db` from the first `seconds` of `src_db`. Returns
    (t_start, cutoff, rows_copied)."""
    for suffix in ("", "-wal", "-shm"):
        stale = Path(short_db + suffix)
        if stale.exists():
            stale.unlink()

    probe = sqlite3.connect(src_db)
    t_start = _first_ts(probe)
    probe.close()
    if t_start is None:
        raise SystemExit("no data in source db")
    cutoff = t_start + seconds

    conn = sqlite3.connect(short_db)
    try:
        conn.execute("ATTACH DATABASE ? AS src", (src_db,))
        # Recreate every schema object verbatim (skip rtree shadow tables — the
        # `CREATE VIRTUAL TABLE ... rtree` recreates those itself).
        for _type, name, sql in conn.execute(
            "SELECT type, name, sql FROM src.sqlite_master "
            "WHERE sql IS NOT NULL AND name NOT LIKE 'sqlite_%'"
        ).fetchall():
            if name.endswith(_RTREE_SHADOWS):
                continue
            conn.execute(sql)

        tables = {
            row[0] for row in conn.execute("SELECT name FROM src.sqlite_master WHERE type='table'")
        }
        conn.execute("INSERT INTO _streams SELECT * FROM src._streams")
        rows_copied = 0
        for (name,) in conn.execute("SELECT name FROM src._streams").fetchall():
            in_window_ids = f'SELECT id FROM src."{name}" WHERE ts <= ?'
            cursor = conn.execute(
                f'INSERT INTO "{name}" SELECT * FROM src."{name}" WHERE ts <= ?', (cutoff,)
            )
            rows_copied += cursor.rowcount
            if f"{name}_blob" in tables:
                conn.execute(
                    f'INSERT INTO "{name}_blob" SELECT * FROM src."{name}_blob" '
                    f"WHERE id IN ({in_window_ids})",
                    (cutoff,),
                )
            if f"{name}_rtree" in tables:
                conn.execute(
                    f'INSERT INTO "{name}_rtree" SELECT * FROM src."{name}_rtree" '
                    f"WHERE id IN ({in_window_ids})",
                    (cutoff,),
                )
        conn.commit()
        return t_start, cutoff, rows_copied
    finally:
        conn.close()


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("recording", help="recording dir (or its mem2.db)")
    parser.add_argument(
        "--seconds", type=float, default=DEFAULT_SECONDS, help="window length (default: 30)"
    )
    args = parser.parse_args()

    target = Path(args.recording)
    src_db = target if target.name == DB_NAME else target / DB_NAME
    if not src_db.exists():
        raise SystemExit(f"no {DB_NAME} at {target}")
    recording_dir = src_db.parent
    short_db = recording_dir / SHORT_DB
    short_rrd = recording_dir / SHORT_RRD

    print(f">> first {args.seconds:g}s of {src_db}")
    *_, rows = make_short_db(str(src_db), str(short_db), args.seconds)
    span = (
        sqlite3.connect(str(short_db))
        .execute(f'SELECT MAX(ts) - MIN(ts) FROM "{stream_names.FASTLIO_ODOM}"')
        .fetchone()[0]
    )
    print(f"   db: {short_db.name} ({rows} rows, ~{span:.1f}s of fastlio span)")
    print(f"   rrd: building -> {short_rrd.name}")
    rebuild_rrd(str(short_db), short_rrd)
    print("done")


if __name__ == "__main__":
    main()
