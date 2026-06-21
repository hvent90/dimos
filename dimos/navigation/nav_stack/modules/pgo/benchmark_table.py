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

"""Resumable PGO benchmark over (environment x config).

Fills one table: every environment (go2 recordings) scored against every PGO
column (the nav-stack PGO in one or more configs). Each cell is one eval.py
subprocess (sequential — they share the isolated LCM bus).

CHECKPOINTED: a cell is skipped when its summary.json already exists and its
fingerprint still matches the db (size+mtime) and EVAL_VERSION. Kill it any
time and rerun — only missing/stale cells recompute. `--force` recomputes all.

The universal score is **voxel agreement** (re-anchoring scans onto the
corrected trajectory should collapse double walls — ground-truth-free and
needs no camera). April-tag agreement is reported additionally wherever a
camera + intrinsics sidecar exists.

Usage:
    uv run python dimos/navigation/nav_stack/modules/pgo/benchmark_table.py
    ... [--go2-root ~/datasets/go2_recordings] [--force]
    ... [--only-env NAME] [--only-col NAME] [--table-only]
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
import json
from pathlib import Path
import subprocess
import sys
import time
from typing import Any

PGO_DIR = Path(__file__).resolve().parent
EVAL_PY = PGO_DIR / "eval.py"
MODULE_PY = PGO_DIR / "pgo.py"
RESULTS_DIR = PGO_DIR / "eval_results"
TABLE_PATH = RESULTS_DIR / "benchmark_table.md"

DEFAULT_GO2_ROOT = Path("~/datasets/go2_recordings").expanduser()

# Loop-closure thresholds shared across columns (eval.py applies these as
# DEFAULT_PGO_CONFIG too; kept explicit here so columns can diverge from them).
_PGO_BASE: dict[str, Any] = {
    "loop_search_radius": 3.0,
    "loop_time_thresh": 5.0,
    "min_loop_detect_duration": 2.0,
    "key_pose_delta_trans": 0.5,
    "use_scan_context": True,
    # The global map is a big cloud nothing in the eval consumes; its publishes
    # congest the corrected_odometry ack channel the lockstep replay waits on.
    "global_map_publish_rate": 0.0,
    # Lockstep waits for one corrected_odometry ack per scan; if PGO drops the
    # scan as stale it never acks and the replay stalls out on ack timeouts.
    "drain_stale_scans": False,
}


@dataclass(frozen=True)
class Column:
    """One implementation+config = one table column."""

    name: str  # column label + results-suffix (disambiguates configs)
    overrides: dict[str, Any] = field(default_factory=dict)


COLUMNS: list[Column] = [
    Column("scan_context", {**_PGO_BASE, "use_scan_context": True}),
    Column("radius", {**_PGO_BASE, "use_scan_context": False}),
]


@dataclass(frozen=True)
class Environment:
    """One dataset = one table row."""

    name: str  # results-dir recording key
    db_path: Path
    odom_stream: str
    lidar_stream: str
    camera_stream: str | None = None
    intrinsics_json: Path | None = None
    ignore_tags: str = ""  # comma-separated dynamic/unreliable tag ids


# Per-recording dynamic/unreliable April tags to drop from scoring (their motion
# would otherwise look like trajectory drift). #17 in huge_loop_realsense rode a
# moving object.
IGNORE_TAGS_BY_RECORDING: dict[str, str] = {
    "2026-06-04_12-57pm-PST__huge_loop_realsense": "17",
}


def discover_go2(root: Path) -> list[Environment]:
    environments = []
    for db_path in sorted(root.glob("*/mem2.db")):
        recording = db_path.parent
        sidecar = recording / "camera_intrinsics.json"
        environments.append(
            Environment(
                name=recording.name,
                db_path=db_path,
                odom_stream="fastlio_odometry",
                lidar_stream="fastlio_lidar",
                camera_stream="color_image",
                intrinsics_json=sidecar if sidecar.exists() else None,
                ignore_tags=IGNORE_TAGS_BY_RECORDING.get(recording.name, ""),
            )
        )
    return environments


def cell_dir(environment: Environment, column: Column) -> Path:
    # Mirrors eval.py's out_dir formula: <recording>__<package>.PGO[.<suffix>].
    return RESULTS_DIR / f"{environment.name}__pgo.PGO.{column.name}"


def cell_is_fresh(environment: Environment, column: Column) -> bool:
    summary_path = cell_dir(environment, column) / "summary.json"
    if not summary_path.exists():
        return False
    try:
        summary = json.loads(summary_path.read_text())
    except (json.JSONDecodeError, OSError):
        return False
    fingerprint = summary.get("fingerprint", {})
    stat = environment.db_path.stat()
    return (
        fingerprint.get("db_bytes") == stat.st_size
        and fingerprint.get("db_mtime") == int(stat.st_mtime)
        and fingerprint.get("version") is not None
    )


def _kill_zombies() -> None:
    """Clear leftover native processes / workers that can wedge the next cell."""
    subprocess.run(
        "lsof -ti tcp:7766 2>/dev/null | xargs kill -9 2>/dev/null;"
        ' pkill -9 -f "bin/pgo" 2>/dev/null',
        shell=True,
        check=False,
    )


def run_cell(environment: Environment, column: Column) -> bool:
    command = [
        sys.executable,
        "-u",
        str(EVAL_PY),
        "--db-path",
        str(environment.db_path),
        "--odom-stream",
        environment.odom_stream,
        "--lidar-stream",
        environment.lidar_stream,
        "--module-path",
        str(MODULE_PY),
        "--module-name",
        "PGO",
        "--recording-name",
        environment.name,
        "--results-suffix",
        column.name,
        "--with-rrd",
        "false",
        "--lockstep",
        "true",
    ]
    if environment.camera_stream is not None:
        command += ["--camera-stream", environment.camera_stream]
    if environment.intrinsics_json is not None:
        command += ["--camera-intrinsics-json-path", str(environment.intrinsics_json)]
    if environment.ignore_tags:
        command += ["--ignore-tags", environment.ignore_tags]
    if column.overrides:
        command += ["--pgo-config-json", json.dumps(column.overrides)]
    print(f"\n=== {environment.name} x {column.name} ===", flush=True)
    result = subprocess.run(command, check=False)
    print(f"=== {environment.name} x {column.name} exit: {result.returncode} ===", flush=True)
    return result.returncode == 0


def _fmt(value: float | None, places: int = 3, signed: bool = True) -> str:
    if value is None:
        return "—"
    return f"{value:+.{places}f}" if signed else f"{value:.{places}f}"


def render_table(environments: list[Environment]) -> Path:
    cells: dict[tuple[str, str], dict[str, Any]] = {}
    for summary_path in RESULTS_DIR.glob("*/summary.json"):
        recording, _, module_key = summary_path.parent.name.rpartition("__")
        try:
            summary = json.loads(summary_path.read_text())
        except (json.JSONDecodeError, OSError):
            continue
        cells[(recording, module_key)] = summary.get("scores", {})

    column_keys = [f"pgo.PGO.{column.name}" for column in COLUMNS]
    header = "| environment | " + " | ".join(column.name for column in COLUMNS) + " |"
    sep = "|" + "---|" * (len(COLUMNS) + 1)
    lines = [
        "# PGO benchmark — environments x configs",
        "",
        "Each cell: **voxel improvement** (fractional drop in occupied 0.2 m voxels "
        "after re-anchoring scans onto the corrected trajectory; the universal, "
        "ground-truth-free score) — then `tag:<april-tag improvement>` where a camera "
        "exists, and `cl<closures>`. Higher is better; `—` = not yet run / N/A.",
        "",
        header,
        sep,
    ]
    for environment in environments:
        row_cells = []
        for column_key in column_keys:
            scores = cells.get((environment.name, column_key))
            if scores is None:
                row_cells.append("—")
                continue
            voxel = _fmt(scores.get("voxel_improvement"))
            tag = scores.get("tag_improvement")
            closures = scores.get("closures")
            text = voxel
            if tag is not None:
                text += f" tag:{tag:+.2f}"
            if closures is not None:
                text += f" cl{closures}"
            row_cells.append(text)
        lines.append(f"| {environment.name} | " + " | ".join(row_cells) + " |")

    # Per-column mean voxel improvement (over environments that have a number).
    lines += ["", "## Mean voxel improvement per column", ""]
    lines.append("| " + " | ".join(column.name for column in COLUMNS) + " |")
    lines.append("|" + "---|" * len(COLUMNS))
    means = []
    for column_key in column_keys:
        values = [
            cells[(environment.name, column_key)]["voxel_improvement"]
            for environment in environments
            if (environment.name, column_key) in cells
            and cells[(environment.name, column_key)].get("voxel_improvement") is not None
        ]
        means.append(f"{sum(values) / len(values):+.3f}" if values else "—")
    lines.append("| " + " | ".join(means) + " |")
    lines.append("")

    RESULTS_DIR.mkdir(exist_ok=True)
    TABLE_PATH.write_text("\n".join(lines) + "\n")
    return TABLE_PATH


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--go2-root", type=Path, default=DEFAULT_GO2_ROOT)
    parser.add_argument("--only-env", help="comma-separated environment names")
    parser.add_argument("--only-col", help="comma-separated column names")
    parser.add_argument("--force", action="store_true", help="recompute fresh cells too")
    parser.add_argument(
        "--attempts", type=int, default=2, help="retries per cell on transient RPC timeouts"
    )
    parser.add_argument("--table-only", action="store_true", help="render from cache, run nothing")
    args = parser.parse_args()

    environments = discover_go2(args.go2_root.expanduser())
    if args.only_env:
        wanted = {name.strip() for name in args.only_env.split(",")}
        environments = [environment for environment in environments if environment.name in wanted]

    columns = COLUMNS
    if args.only_col:
        wanted = {name.strip() for name in args.only_col.split(",")}
        columns = [column for column in COLUMNS if column.name in wanted]

    if args.table_only:
        print(f"table -> {render_table(environments)}")
        return

    total = len(environments) * len(columns)
    print(f"benchmark: {len(environments)} environments x {len(columns)} columns = {total} cells")
    done = skipped = failed = 0
    for environment in environments:
        for column in columns:
            if not args.force and cell_is_fresh(environment, column):
                skipped += 1
                print(f"skip (fresh): {environment.name} x {column.name}", flush=True)
                continue
            # Retry transient LCM startup-RPC timeouts; a fresh process almost
            # always gets past them. Kill zombies between attempts.
            ok = False
            for attempt in range(1, args.attempts + 1):
                ok = run_cell(environment, column)
                if ok:
                    break
                _kill_zombies()
                if attempt < args.attempts:
                    print(
                        f"retry {attempt + 1}/{args.attempts}: {environment.name} x {column.name}"
                    )
                    time.sleep(5)
            done += 1 if ok else 0
            failed += 0 if ok else 1
            render_table(environments)  # refresh after every cell — live + crash-safe

    table = render_table(environments)
    print(f"\ncells: {done} ran, {skipped} cached, {failed} failed")
    print(f"table -> {table}")


if __name__ == "__main__":
    main()
