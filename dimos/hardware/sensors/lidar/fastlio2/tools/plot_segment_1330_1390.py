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

"""Plot speed (|v| from pose deltas) for every attempt under the segment runs root.

One Plotly HTML output, one line per attempt_NNN/fastlio.db. X-axis is
seconds-into-recording (rec_t), Y-axis is |v| in m/s on a symlog scale so
both the bounded ~m/s regime and any divergent excursions to 100+ m/s are
visible in one chart.

Hardcoded config — bump the constants and recommit when you want a
different segment / different runs root.

Run from the dimos venv:

    cd ~/repos/dimos
    source .venv/bin/activate
    python -m dimos.hardware.sensors.lidar.fastlio2.tools.plot_segment_1330_1390
"""

from __future__ import annotations

import math
from pathlib import Path
import sqlite3
import sys

import plotly.graph_objects as go

# ---------------- Configuration (must match replay_segment_1330_1390.py) ----

RUNS_ROOT = Path("/media/dimos/USB/fastlio_recordings/segment_replays_1330_1390")
OUT_HTML = RUNS_ROOT / "segment_replays.html"

# Recording reference. The fastlio binary publishes ts in sensor-boot
# seconds when deterministic_clock=True. Add the offset to get epoch, then
# subtract REC_START_EPOCH to get rec_t.
REC_START_EPOCH = 1780020531.706
SENSOR_BOOT_EPOCH_OFFSET = 1780018948.01

T_LO_REC_SEC = 1330.0
T_HI_REC_SEC = 1390.0


# ---------------- Speed derivation ------------------------------------------

# Centered rolling-mean window for the smoothed |v| trace. At ~30 Hz odom
# publish rate, a 9-sample window is ~300 ms of context — wide enough to
# kill per-sample noise but narrow enough to keep the post-gap ramp shape.
SMOOTH_WINDOW = 9


def _load_attempt(db_path: Path) -> tuple[list[float], list[float]]:
    """Return (rec_t_sec, abs_v_mps) for the attempt's fastlio_odometry rows.

    Rows with NULL pose_x/y/z (the first few while IESKF is initialising)
    are skipped. Speed is the L2 norm of the per-step pose delta divided
    by the per-step ts delta. ts is sensor-boot seconds (deterministic
    clock mode), converted to rec_t for the plot.
    """
    con = sqlite3.connect(f"file:{db_path}?mode=ro", uri=True)
    try:
        rows = con.execute(
            "SELECT ts, pose_x, pose_y, pose_z FROM fastlio_odometry "
            "WHERE pose_x IS NOT NULL ORDER BY ts"
        ).fetchall()
    finally:
        con.close()

    if len(rows) < 2:
        return [], []

    rec_ts: list[float] = []
    speeds: list[float] = []
    prev_ts, prev_x, prev_y, prev_z = rows[0]
    for ts, x, y, z in rows[1:]:
        dt = ts - prev_ts
        if dt <= 0:
            prev_ts, prev_x, prev_y, prev_z = ts, x, y, z
            continue
        dx, dy, dz = x - prev_x, y - prev_y, z - prev_z
        v = math.sqrt(dx * dx + dy * dy + dz * dz) / dt
        epoch = ts + SENSOR_BOOT_EPOCH_OFFSET
        rec_t = epoch - REC_START_EPOCH
        rec_ts.append(rec_t)
        speeds.append(v)
        prev_ts, prev_x, prev_y, prev_z = ts, x, y, z
    return rec_ts, speeds


def _smooth(values: list[float], window: int) -> list[float]:
    """Centered rolling-mean smoothing on a list of floats.

    Window is the full span (odd works best — centers cleanly). Endpoints
    use whatever samples are available within the window, so the smoothed
    array has the same length as the input and aligns to the same x-axis.
    """
    if window <= 1 or len(values) <= 1:
        return list(values)
    half = window // 2
    n = len(values)
    out = [0.0] * n
    for i in range(n):
        lo = max(0, i - half)
        hi = min(n, i + half + 1)
        seg = values[lo:hi]
        out[i] = sum(seg) / len(seg)
    return out


def _list_attempts() -> list[Path]:
    return sorted(
        (p for p in RUNS_ROOT.iterdir() if p.is_dir() and p.name.startswith("attempt_")),
        key=lambda p: p.name,
    )


# ---------------- Plot ------------------------------------------------------


def _build_figure(attempts: list[Path]) -> go.Figure:
    fig = go.Figure()
    n_traces = 0
    # One colour per attempt so the raw + smoothed traces for a single
    # attempt share the same hue (smoothed bright, raw faded behind).
    palette = [
        "#1f77b4",
        "#ff7f0e",
        "#2ca02c",
        "#d62728",
        "#9467bd",
        "#8c564b",
        "#e377c2",
        "#7f7f7f",
        "#bcbd22",
        "#17becf",
    ]
    for idx, attempt_dir in enumerate(attempts):
        db_path = attempt_dir / "fastlio.db"
        if not db_path.exists():
            print(f"[plot_segment] skip {attempt_dir.name}: no fastlio.db", flush=True)
            continue
        rec_ts, speeds = _load_attempt(db_path)
        if not rec_ts:
            print(f"[plot_segment] skip {attempt_dir.name}: no valid odom rows", flush=True)
            continue
        smoothed = _smooth(speeds, SMOOTH_WINDOW)
        raw_peak = max(speeds)
        smoothed_peak = max(smoothed)
        colour = palette[idx % len(palette)]
        # Raw trace: thin, faded, behind the smoothed line. legendgroup
        # ties it to the smoothed trace so clicking the legend toggles both.
        fig.add_trace(
            go.Scatter(
                x=rec_ts,
                y=speeds,
                mode="lines",
                name=f"{attempt_dir.name} raw",
                legendgroup=attempt_dir.name,
                showlegend=False,
                line={"width": 0.7, "color": colour},
                opacity=0.25,
                hovertemplate="rec_t=%{x:.2f}s |v|=%{y:.3f} m/s (raw)<extra></extra>",
            )
        )
        # Smoothed trace: bolder, in front, this is what the legend names.
        fig.add_trace(
            go.Scatter(
                x=rec_ts,
                y=smoothed,
                mode="lines",
                name=(
                    f"{attempt_dir.name} "
                    f"(smoothed peak {smoothed_peak:.2f} m/s, raw peak {raw_peak:.2f} m/s)"
                ),
                legendgroup=attempt_dir.name,
                line={"width": 2.0, "color": colour},
                hovertemplate="rec_t=%{x:.2f}s |v|=%{y:.3f} m/s (smoothed)<extra></extra>",
            )
        )
        n_traces += 1

    fig.update_layout(
        title=(
            f"FAST-LIO replay speed — segment [{T_LO_REC_SEC:.0f}, {T_HI_REC_SEC:.0f}] s "
            f"({n_traces} attempt{'' if n_traces == 1 else 's'}, "
            f"smoothed bold + raw faded behind, window={SMOOTH_WINDOW})"
        ),
        xaxis_title="seconds into recording",
        yaxis_title="|v| (m/s, log scale)",
        yaxis={"type": "log"},
        hovermode="closest",
        template="plotly_white",
    )
    return fig


def main() -> int:
    if not RUNS_ROOT.exists():
        print(f"[plot_segment] runs root does not exist: {RUNS_ROOT}", file=sys.stderr)
        return 2
    attempts = _list_attempts()
    if not attempts:
        print(f"[plot_segment] no attempt_*/ dirs under {RUNS_ROOT}", file=sys.stderr)
        return 2
    fig = _build_figure(attempts)
    OUT_HTML.parent.mkdir(parents=True, exist_ok=True)
    fig.write_html(str(OUT_HTML), include_plotlyjs="cdn")
    print(f"[plot_segment] wrote {OUT_HTML}  attempts={len(attempts)}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
