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

"""Sanity-check a recording dir: pcap + mem2.db sizes, stream rates, pose travel."""

from __future__ import annotations

from datetime import datetime
import json
import math
from pathlib import Path
import re
import sqlite3
import subprocess
import sys
from typing import Any

from dimos.mapping.recording.utils import stream_names

APRIL_TAG_STREAM = stream_names.APRIL_TAGS
MIN_DISTINCT_TAGS = 3  # groundtruth/anchoring needs >=3 non-collinear static tags
_MARKER_ID_RE = re.compile(rb"marker_id[#:= ]*(-?\d+)")

RECORDINGS_DIR = Path("recordings")
TRAVEL_STREAM = stream_names.FASTLIO_ODOM
POSE_PCT_MIN = 99.0  # a "pose-bearing" stream should have nearly all rows posed
COVERAGE_MIN = 0.9  # required streams must span >=90% of the longest stream
# A pcap with only its global header (no packets) is exactly this many bytes.
PCAP_HEADER_BYTES = 24


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


def list_stream_names(cur: sqlite3.Cursor) -> list[str]:
    """The recording's actual stream names, in creation order. Reads the mem2
    `_streams` metadata table; falls back to data tables that have a `_blob` twin."""
    try:
        rows = cur.execute("SELECT name FROM _streams ORDER BY rowid").fetchall()
        if rows:
            return [row[0] for row in rows]
    except sqlite3.OperationalError:
        pass
    tables = {row[0] for row in cur.execute("SELECT name FROM sqlite_master WHERE type='table'")}
    return sorted(name for name in tables if f"{name}_blob" in tables)


def stream_rows(cur: sqlite3.Cursor, name: str) -> tuple[int, float | None, float | None, int]:
    tables = {row[0] for row in cur.execute("SELECT name FROM sqlite_master WHERE type='table'")}
    if name not in tables:
        return 0, None, None, 0
    n, t0, t1 = cur.execute(f'SELECT COUNT(*), MIN(ts), MAX(ts) FROM "{name}"').fetchone()
    pose_non_null = cur.execute(f'SELECT COUNT(pose_x) FROM "{name}"').fetchone()[0]
    return n, t0, t1, pose_non_null


def april_tag_markers(cur: sqlite3.Cursor) -> list[int] | None:
    """Distinct AprilTag marker ids in the `april_tags` stream, or None if the
    stream doesn't exist yet (detection hasn't run). The per-row `tags` blob
    embeds a literal `marker_id#<n>`, so a regex reads the ids without decoding."""
    try:
        rows = cur.execute(f'SELECT tags FROM "{APRIL_TAG_STREAM}"').fetchall()
    except sqlite3.OperationalError:
        return None
    markers: set[int] = set()
    for (blob,) in rows:
        if blob is None:
            continue
        raw = blob if isinstance(blob, (bytes, bytearray)) else str(blob).encode()
        markers.update(int(match) for match in _MARKER_ID_RE.findall(raw))
    return sorted(markers)


def odometry_travel(cur: sqlite3.Cursor) -> dict | None:
    rows = cur.execute(
        f'SELECT pose_x, pose_y, pose_z FROM "{TRAVEL_STREAM}" WHERE pose_x IS NOT NULL ORDER BY ts'
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


def summarize(directory: Path) -> dict[str, Any]:
    """The same stats report() prints, as a JSON-able dict."""
    pcap = directory / "raw_mid360.pcap"
    db = directory / "mem2.db"
    summary: dict[str, Any] = {
        "directory": str(directory),
        "files": {},
        "pcap": None,
        "streams": {},
        "fastlio_odometry_travel": None,
    }
    for path in (pcap, db, directory / "mem2.db-wal", directory / "mem2.db-shm"):
        summary["files"][path.name] = path.stat().st_size if path.exists() else None

    if pcap.exists() and pcap.stat().st_size > PCAP_HEADER_BYTES:
        stats = pcap_stats(pcap)
        if stats is not None:
            packets, first, last = stats
            span = last - first
            summary["pcap"] = {
                "packets": packets,
                "first": first,
                "last": last,
                "span_s": span,
                "rate_pkt_s": packets / span if span > 0 else 0,
            }

    if not db.exists():
        summary["error"] = "mem2.db missing"
        return summary

    connection = sqlite3.connect(db)
    cur = connection.cursor()
    for name in list_stream_names(cur):
        n, t0, t1, pose_n = stream_rows(cur, name)
        if n == 0:
            summary["streams"][name] = {"rows": 0}
            continue
        span = (t1 - t0) if (t0 and t1) else 0
        summary["streams"][name] = {
            "rows": n,
            "span_s": span,
            "hz": (n - 1) / span if span > 0 else 0,
            "pose_pct": 100 * pose_n / n if n else 0,
            "t0": t0,
            "t1": t1,
        }
    summary["april_markers"] = april_tag_markers(cur)
    summary["fastlio_odometry_travel"] = odometry_travel(cur)
    connection.close()
    return summary


def write_summary(directory: Path) -> Path:
    """Write summarize() to <directory>/summary.json and return its path."""
    path = directory / "summary.json"
    path.write_text(json.dumps(summarize(directory), indent=2))
    return path


def main() -> int:
    return report(find_dir(sys.argv))


def evaluate_checks(summary: dict[str, Any]) -> list[tuple[str, str, str]]:
    """Turn a summarize() dict into (status, label, detail) rows, status in
    {PASS, FAIL, N/A}. Validates the streams a go2+mid360 recording needs and
    what each must do (poses populated, overlap, full coverage)."""
    streams = summary.get("streams", {})

    def info(name: str) -> dict[str, Any]:
        return streams.get(name) or {}

    def present(name: str) -> bool:
        return info(name).get("rows", 0) > 0

    def has_poses(name: str) -> bool:
        return present(name) and info(name).get("pose_pct", 0) >= POSE_PCT_MIN

    checks: list[tuple[str, str, str]] = [
        (
            "PASS" if has_poses(stream_names.FASTLIO_ODOM) else "FAIL",
            stream_names.FASTLIO_ODOM,
            "6-DoF poses",
        ),
        (
            "PASS" if has_poses(stream_names.FASTLIO_LIDAR) else "FAIL",
            stream_names.FASTLIO_LIDAR,
            "clouds carry poses",
        ),
        (
            "PASS" if present(stream_names.COLOR_IMAGE) else "FAIL",
            stream_names.COLOR_IMAGE,
            "present",
        ),
    ]

    markers = summary.get("april_markers")
    if markers is None:
        checks.append(("N/A", "april_tags", "stream absent — detection not run"))
    elif len(markers) >= MIN_DISTINCT_TAGS:
        checks.append(("PASS", "april_tags", f"{len(markers)} distinct markers {markers}"))
    else:
        checks.append(
            ("FAIL", "april_tags", f"only {len(markers)} distinct markers {markers} (need >=3)")
        )

    is_go2 = present(stream_names.ODOM) or present(stream_names.LIDAR)
    if is_go2:
        checks.append(
            (
                "PASS" if has_poses(stream_names.ODOM) else "FAIL",
                stream_names.ODOM,
                "poses (for go2-align)",
            )
        )
        checks.append(
            ("PASS" if present(stream_names.LIDAR) else "FAIL", stream_names.LIDAR, "present")
        )
        odom, fastlio = info(stream_names.ODOM), info(stream_names.FASTLIO_ODOM)
        label = f"{stream_names.ODOM} ∩ fastlio"
        if odom.get("t0") and fastlio.get("t0"):
            overlap = max(odom["t0"], fastlio["t0"]) < min(odom["t1"], fastlio["t1"])
            checks.append(("PASS" if overlap else "FAIL", label, "time ranges overlap"))
        else:
            checks.append(("N/A", label, "no timestamps to compare"))

    required = [stream_names.FASTLIO_ODOM, stream_names.FASTLIO_LIDAR, stream_names.COLOR_IMAGE]
    if is_go2:
        required += [stream_names.ODOM, stream_names.LIDAR]
    max_span = max((info(name).get("span_s", 0) for name in streams), default=0)
    short = [
        name
        for name in required
        if present(name) and max_span > 0 and info(name).get("span_s", 0) < COVERAGE_MIN * max_span
    ]
    coverage_detail = f"all span >= {COVERAGE_MIN:.0%} of {max_span:.0f}s"
    if short:
        coverage_detail += f" — short: {', '.join(short)}"
    checks.append(("PASS" if not short else "FAIL", "stream coverage", coverage_detail))
    return checks


def print_checks(summary: dict[str, Any]) -> int:
    """Print the ✓/✗ checklist; return the number of failed checks."""
    glyph = {"PASS": "✓", "FAIL": "✗", "N/A": "-"}
    checks = evaluate_checks(summary)
    label_width = max(len(label) for _status, label, _detail in checks)
    print("checks:")
    failed = 0
    for status, label, detail in checks:
        print(f"  {glyph[status]} {label:<{label_width}}  {detail}")
        failed += status == "FAIL"
    print()
    print(f"  {'PASS — all checks ok' if not failed else f'FAIL — {failed} check(s) failed'}")
    return failed


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

    if pcap.exists() and pcap.stat().st_size > PCAP_HEADER_BYTES:
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
    names = list_stream_names(cur)
    indent = "  "
    name_width = max(len("stream"), max((len(name) for name in names), default=0))
    header = (
        f"{indent}{'stream':<{name_width}}  {'rows':>9} {'span_s':>8} {'hz':>7} {'pose%':>7}  blob"
    )
    print(header)
    print("-" * len(header))
    for name in names:
        n, t0, t1, pose_n = stream_rows(cur, name)
        if n == 0:
            print(f"{indent}{name:<{name_width}}  {'-':>9}  (no rows)")
            continue
        span = (t1 - t0) if (t0 and t1) else 0
        rate = (n - 1) / span if span > 0 else 0
        pose_pct = 100 * pose_n / n if n else 0
        blob = cur.execute(
            f'SELECT LENGTH(b.data) FROM "{name}" t JOIN "{name}_blob" b ON t.id=b.id LIMIT 1'
        ).fetchone()
        blob_label = human_size(blob[0]) if blob else "-"
        print(
            f"{indent}{name:<{name_width}}  {n:>9,} {span:>8.1f} {rate:>7.1f} "
            f"{pose_pct:>6.0f}%  {blob_label}"
        )

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
    connection.close()

    print()
    failed = print_checks(summarize(directory))
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
