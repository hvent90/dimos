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

"""Load a characterization recording and segment it into per-axis steps.

Reads through the memory2 ``SqliteStore`` interface (no raw SQL), materializing
each payload while the connection is open. Works on both sim recordings from
``sim_ground_truth`` and real Go2 sessions -- the command/odom streams are the
same. Segmentation detects intervals where exactly one axis is commanded at a
constant amplitude, which is how the sweep excites the plant one axis at a time.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from itertools import pairwise
from pathlib import Path

import numpy as np

from dimos.memory2.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.std_msgs.Int8 import Int8

_CMD_EPS = 1e-6  # command magnitude below this is treated as zero
_MIN_STEP_SAMPLES = 3  # commanded intervals shorter than this are ignored
# A real SI step's command starts within this long after the operator's gate
# (ENTER) event; anything not following a gate is repositioning teleop, not a step.
_GATE_WINDOW_S = 3.0


@dataclass
class Recording:
    """Time-aligned command and odom traces from one recording."""

    cmd_t: np.ndarray
    cmd: np.ndarray  # (n, 3) = vx, vy, wz
    odom_t: np.ndarray
    odom: np.ndarray  # (m, 3) = x, y, yaw
    gate_t: np.ndarray = field(default_factory=lambda: np.empty(0))  # operator gate event times


@dataclass(frozen=True)
class StepSpan:
    """One single-axis commanded step: ``axis`` held at ``amplitude``."""

    axis: str  # "vx" | "vy" | "wz"
    amplitude: float
    t_start: float
    t_end: float


_AXIS_NAMES = ("vx", "vy", "wz")


def load_recording(db_path: str | Path) -> Recording:
    """Load cmd_vel + odom into aligned numpy traces (times are epoch seconds)."""
    store = SqliteStore(path=str(db_path))
    store.start()
    try:
        cmd_rows = [
            (obs.ts, obs.data.linear.x, obs.data.linear.y, obs.data.angular.z)
            for obs in store.stream("cmd_vel", Twist)
        ]
        odom_rows = [
            (obs.ts, obs.data.x, obs.data.y, obs.data.yaw)
            for obs in store.stream("odom", PoseStamped)
        ]
        # Gate (operator advance) events — read only if the stream exists, so we
        # don't create an empty table on recordings that never had one.
        gate_rows = (
            [obs.ts for obs in store.stream("gate", Int8)] if "gate" in store.list_streams() else []
        )
    finally:
        store.stop()
    if not cmd_rows or not odom_rows:
        raise ValueError(f"{db_path}: recording needs both cmd_vel and odom streams")

    cmd_arr = np.asarray(cmd_rows, dtype=float)
    odom_arr = np.asarray(odom_rows, dtype=float)
    return Recording(
        cmd_t=cmd_arr[:, 0],
        cmd=cmd_arr[:, 1:4],
        odom_t=odom_arr[:, 0],
        odom=odom_arr[:, 1:4],
        gate_t=np.asarray(gate_rows, dtype=float),
    )


def segment_steps(
    recording: Recording,
    *,
    min_samples: int = _MIN_STEP_SAMPLES,
    gated_only: bool = True,
    gate_window_s: float = _GATE_WINDOW_S,
) -> list[StepSpan]:
    """Split the command stream into single-axis constant-amplitude SI steps.

    A step is a maximal interval over which the command vector is constant and
    exactly one axis is non-zero. Constant-input intervals are found from command
    change points, so this is robust to variable command rates and back-to-back
    amplitudes (an amplitude change starts a new step).

    When the recording has a ``gate`` stream and ``gated_only`` is set (default),
    only steps that START within ``gate_window_s`` after an operator gate (ENTER)
    event are kept -- this drops operator repositioning teleop and any other
    ``/cmd_vel`` traffic that is not an actual SI step. Recordings with no gate
    stream return every single-axis interval (legacy behavior).
    """
    cmd_t = recording.cmd_t
    cmd = recording.cmd
    n = cmd.shape[0]
    if n == 0:
        return []

    changed = np.any(np.abs(np.diff(cmd, axis=0)) > _CMD_EPS, axis=1)
    boundaries = [0, *(np.flatnonzero(changed) + 1).tolist(), n]
    use_gate = gated_only and recording.gate_t.size > 0

    spans: list[StepSpan] = []
    for start, end in pairwise(boundaries):
        if end - start < min_samples:
            continue
        level = np.median(cmd[start:end], axis=0)
        active = np.flatnonzero(np.abs(level) > _CMD_EPS)
        if active.size != 1:
            continue  # zero (rest) or multi-axis -- not a clean single-axis step
        t_start = float(cmd_t[start])
        if use_gate and not _follows_gate(t_start, recording.gate_t, gate_window_s):
            continue  # not a gated SI step -> operator repositioning, skip
        axis_idx = int(active[0])
        spans.append(
            StepSpan(
                axis=_AXIS_NAMES[axis_idx],
                amplitude=float(level[axis_idx]),
                t_start=t_start,
                t_end=float(cmd_t[end - 1]),
            )
        )
    return spans


def _follows_gate(t_start: float, gate_t: np.ndarray, window_s: float) -> bool:
    """True if a gate event occurred just before ``t_start`` (step starts after gate)."""
    dt = t_start - gate_t
    return bool(np.any((dt >= -0.5) & (dt <= window_s)))


def step_pose_channel(recording: Recording, span: StepSpan) -> tuple[np.ndarray, np.ndarray]:
    """Odom (t_rel, pose-channel) for ``span``, projected into the start body frame.

    vx -> body-x displacement, vy -> body-y displacement, wz -> unwrapped yaw
    change. ``t_rel`` is relative to the command edge so deadtime is measured
    from when the command was issued. Includes the hold window only (the step
    response model assumes the command stays at ``amplitude``).
    """
    odom_t = recording.odom_t
    in_window = (odom_t >= span.t_start) & (odom_t <= span.t_end)
    t_rel = odom_t[in_window] - span.t_start
    x = recording.odom[in_window, 0]
    y = recording.odom[in_window, 1]
    yaw = recording.odom[in_window, 2]
    if t_rel.size == 0:
        return t_rel, np.empty(0)

    x0, y0, yaw0 = x[0], y[0], yaw[0]
    if span.axis == "wz":
        channel = np.unwrap(yaw) - yaw0
    else:
        cos_y, sin_y = np.cos(yaw0), np.sin(yaw0)
        dx, dy = x - x0, y - y0
        if span.axis == "vx":
            channel = cos_y * dx + sin_y * dy
        else:  # vy
            channel = -sin_y * dx + cos_y * dy
    return t_rel, channel


__all__ = [
    "Recording",
    "StepSpan",
    "load_recording",
    "segment_steps",
    "step_pose_channel",
]
