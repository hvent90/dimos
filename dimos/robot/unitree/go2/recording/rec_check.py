#!/usr/bin/env python3
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

"""Sanity-check a go2 recording dir: pcap + mem2.db sizes, stream rates, pose travel."""

from __future__ import annotations

from datetime import datetime
import math
from pathlib import Path
import sqlite3
import subprocess
import sys

STREAMS = (
    "livox_imu",
    "livox_lidar",
    "lidar",
    "fastlio_lidar",
    "fastlio_odometry",
    "odom",
    "color_image",
)
RECORDINGS_DIR = Path("go2_recordings")


def find_dir(argv: list[str]) -> Path:
    if len(argv) > 1:
        directory = Path(argv[1])
        if not directory.exists():
            sys.exit(f"not found: {directory}")
        return directory
    candidates = sorted(
        (p for p in RECORDINGS_DIR.glob("2*") if p.is_dir()),
        key=lambda p: p.stat().st_mtime,
    )
    if not candidates:
        sys.exit(f"no recordings under {RECORDINGS_DIR}/")
    return candidates[-1]


def human_size(n: int) -> str:
    for unit in ("B", "KB", "MB", "GB", "TB"):
        if n < 1024:
            return f"{n:.1f}{unit}"
        n /= 1024  # type: ignore[assignment]
    return f"{n:.1f}PB"


def pcap_stats(pcap: Path) -> tuple[int, float, float] | None:
    try:
        result = subprocess.run(
            ["capinfos", "-Mra", str(pcap)],
            capture_output=True,
            text=True,
            check=True,
        )
    except (FileNotFoundError, subprocess.CalledProcessError):
        return _pcap_stats_via_tcpdump(pcap)
    packets = first = last = None
    for line in result.stdout.splitlines():
        if "Number of packets" in line:
            packets = int(line.split(":", 1)[1].strip().replace(",", ""))
        elif "First packet time" in line:
            first = _parse_capinfos_time(line.split(":", 1)[1].strip())
        elif "Last packet time" in line:
            last = _parse_capinfos_time(line.split(":", 1)[1].strip())
    if packets is None or first is None or last is None:
        return None
    return packets, first, last


def _parse_capinfos_time(value: str) -> float | None:
    for fmt in ("%Y-%m-%d %H:%M:%S.%f", "%Y-%m-%d %H:%M:%S"):
        try:
            return datetime.strptime(value.split(" UTC")[0], fmt).timestamp()
        except ValueError:
            continue
    return None


def _pcap_stats_via_tcpdump(pcap: Path) -> tuple[int, float, float] | None:
    try:
        result = subprocess.run(
            ["tcpdump", "-r", str(pcap), "-tt", "-nn"],
            capture_output=True,
            text=True,
            check=True,
        )
    except (FileNotFoundError, subprocess.CalledProcessError):
        return None
    timestamps = []
    for line in result.stdout.splitlines():
        head = line.split(" ", 1)[0]
        try:
            timestamps.append(float(head))
        except ValueError:
            continue
    if not timestamps:
        return None
    return len(timestamps), timestamps[0], timestamps[-1]


def stream_rows(cur: sqlite3.Cursor, name: str) -> tuple[int, float | None, float | None, int]:
    tables = {row[0] for row in cur.execute("SELECT name FROM sqlite_master WHERE type='table'")}
    if name not in tables:
        return 0, None, None, 0
    n, t0, t1 = cur.execute(f'SELECT COUNT(*), MIN(ts), MAX(ts) FROM "{name}"').fetchone()
    pose_non_null = cur.execute(f'SELECT COUNT(pose_x) FROM "{name}"').fetchone()[0]
    return n, t0, t1, pose_non_null


def odometry_travel(cur: sqlite3.Cursor) -> dict | None:
    rows = cur.execute(
        "SELECT pose_x, pose_y, pose_z FROM fastlio_odometry WHERE pose_x IS NOT NULL ORDER BY ts"
    ).fetchall()
    if not rows:
        return None
    xs, ys, zs = zip(*rows, strict=False)
    path_length = sum(math.dist(rows[i - 1], rows[i]) for i in range(1, len(rows)))
    return {
        "rows": len(rows),
        "start": rows[0],
        "end": rows[-1],
        "path_length": path_length,
        "straight_line": math.dist(rows[0], rows[-1]),
        "bbox_x": (min(xs), max(xs)),
        "bbox_y": (min(ys), max(ys)),
        "bbox_z": (min(zs), max(zs)),
    }


def format_clock(seconds: float | None) -> str:
    if seconds is None:
        return "-"
    return datetime.fromtimestamp(seconds).strftime("%H:%M:%S")


def main() -> int:
    return report(find_dir(sys.argv))


def report(directory: Path) -> int:
    print(f"=== {directory} ===")
    print()

    pcap = directory / "raw_mid360.pcap"
    db = directory / "mem2.db"
    print("files:")
    for path in (pcap, db, directory / "mem2.db-wal", directory / "mem2.db-shm"):
        if path.exists():
            print(f"  {path.name:<20} {human_size(path.stat().st_size):>10}")
        else:
            print(f"  {path.name:<20} (missing)")
    print()

    if pcap.exists() and pcap.stat().st_size > 24:
        stats = pcap_stats(pcap)
        if stats is None:
            print("pcap: present (capinfos/tcpdump unavailable to inspect)")
        else:
            packets, first, last = stats
            span = last - first
            rate = packets / span if span > 0 else 0
            print(
                f"pcap: {packets:,} packets  {format_clock(first)} -> {format_clock(last)}  "
                f"span={span:.1f}s  rate={rate:.0f}pkt/s"
            )
    elif pcap.exists():
        print(f"pcap: empty (only {pcap.stat().st_size}B — header only)")
    else:
        print("pcap: missing")
    print()

    if not db.exists():
        print("mem2.db missing — cannot check streams.")
        return 1

    connection = sqlite3.connect(db)
    cur = connection.cursor()
    header = f"{'stream':<18} {'rows':>9} {'span_s':>8} {'hz':>7} {'pose%':>7}  blob"
    print(header)
    print("-" * len(header))
    for name in STREAMS:
        n, t0, t1, pose_n = stream_rows(cur, name)
        if n == 0:
            print(f"  {name:<16} {'-':>9}  (no rows)")
            continue
        span = (t1 - t0) if (t0 and t1) else 0
        rate = (n - 1) / span if span > 0 else 0
        pose_pct = 100 * pose_n / n if n else 0
        blob = cur.execute(
            f'SELECT LENGTH(b.data) FROM "{name}" t JOIN "{name}_blob" b ON t.id=b.id LIMIT 1'
        ).fetchone()
        blob_label = human_size(blob[0]) if blob else "-"
        print(f"  {name:<16} {n:>9,} {span:>8.1f} {rate:>7.1f} {pose_pct:>6.0f}%  {blob_label}")

    travel = odometry_travel(cur)
    print()
    if travel:
        sx, sy, sz = travel["start"]
        ex, ey, ez = travel["end"]
        bx, by, bz = travel["bbox_x"], travel["bbox_y"], travel["bbox_z"]
        print("fastlio_odometry travel:")
        print(f"  start          x={sx:.2f}  y={sy:.2f}  z={sz:.2f}")
        print(f"  end            x={ex:.2f}  y={ey:.2f}  z={ez:.2f}")
        print(f"  path_length    {travel['path_length']:.2f} m")
        print(f"  straight_line  {travel['straight_line']:.2f} m")
        print(
            f"  bbox           x=[{bx[0]:.1f},{bx[1]:.1f}]  "
            f"y=[{by[0]:.1f},{by[1]:.1f}]  z=[{bz[0]:.1f},{bz[1]:.1f}]"
        )
    else:
        print("fastlio_odometry travel: no pose-stamped rows")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
