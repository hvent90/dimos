# Copyright 2025-2026 Dimensional Inc.
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

"""Self-contained sim A/B for the ESO/ADRC disturbance-rejection layer.

This is the evidence the ESO has to earn before any hardware trial: a single
process, no LCM, no second coordinator (which has raced ``/go2/cmd_vel`` in the
past). It drives the *real* :class:`TrajectoryTrackingTask` control law against
a FOPDT plant, the only variable between the two arms being ``eso`` on/off.

Two plants:

* **go2-hard** — the realistically nasty Go2: FOPDT (K, tau, deadtime L) plus a
  speed-dependent gain sag (the high-speed K-droop), band-limited "gait" process
  noise, and odometry delivered at ~16 Hz with measurement noise and slow drift.
  This is where the bare tracker blows up on curves at speed and where the ESO
  has to help.
* **flowbase** — the clean wheeled base (near-ideal plant, low odom noise). This
  is the control: the ESO must be ~no-op here (no regression on an already-good
  loop).

Single-variable A/B: for each (plant, path, speed) the baseline and ESO arms see
the *same* process-noise and measurement-noise realizations (same seeds), so any
difference in cross-track error is the ESO and nothing else. We score geometric
cross-track error (cte) against the reference path with the same
:func:`~dimos.utils.benchmarking.scoring.score_run` the hardware benchmark uses,
against the **ground-truth** pose (not the noisy odom the controller saw).

    python -m dimos.utils.benchmarking.eso_sim_ab            # full sweep + plots
    python -m dimos.utils.benchmarking.eso_sim_ab --dial-sweep  # tune the dial
    python -m dimos.utils.benchmarking.eso_sim_ab --quick     # fast subset
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass, field
import json
import math
from pathlib import Path as FsPath
from typing import Literal
import zlib

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from dimos.control.task import CoordinatorState, JointStateSnapshot
from dimos.control.tasks.trajectory_tracking_task.config import TrackingConfig
from dimos.control.tasks.trajectory_tracking_task.constants import FLOWBASE_TRACKING
from dimos.control.tasks.trajectory_tracking_task.trajectory_tracking_task import (
    TrajectoryTrackingTask,
    TrajectoryTrackingTaskConfig,
)
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Path import Path as NavPath
from dimos.utils.benchmarking.paths import circle, smooth_corner, square, straight_line
from dimos.utils.benchmarking.plant import (
    FLOWBASE_PLANT_FITTED,
    GO2_PLANT_FITTED,
    FOPDTChannel,
    TwistBasePlantParams,
)
from dimos.utils.benchmarking.scoring import (
    ExecutedTrajectory,
    TrajectoryTick,
    score_run,
    score_run_with_trajectory,
)
from dimos.utils.benchmarking.tuning import Provenance, derive_config
from dimos.utils.path_utils import get_project_root

_JOINTS = ["go2/vx", "go2/vy", "go2/wz"]
_T0 = 1000.0
# Plant integrates this many sub-steps per control tick (keeps the FOPDT lag +
# deadtime well-resolved while the controller runs at the odom rate).
_SUBSTEPS = 8
DEFAULT_OUT_DIR = get_project_root() / "data" / "benchmark" / "eso_sim_ab"


def _run_seed(base_seed: int, plant: str, path: str, speed: float) -> int:
    """Deterministic per-(plant,path,speed) seed. NOT Python ``hash`` (which is
    randomized per process) — the two arms must replay identical noise across
    runs for the A/B to be reproducible."""
    key = f"{base_seed}:{plant}:{path}:{speed:.4f}".encode()
    return (base_seed + zlib.crc32(key)) % (2**31)


# --------------------------------------------------------------------------
# Hard sim plant
# --------------------------------------------------------------------------


@dataclass
class HardPlantConfig:
    """Knobs that make a FOPDT plant realistically hard (or clean).

    All default to a non-trivial Go2; the FlowBase 'control' plant zeroes the
    nonlinearity/gait and lowers the odom noise. Calibrated so the bare-baseline
    symptoms match the documented Go2: straight ~6-9 cm cross-track, circle
    R=1.0 @ 1.0 m/s ~20 cm, smooth corner ~25 cm.
    """

    # Constant per-axis gain error: the true gain is K * gain_factor, so the
    # single-K gain inversion is mis-calibrated at the operating point. This is
    # the Go2's documented reality — its velocity gain varies 2-3x across the
    # speed range, so one fitted K is wrong almost everywhere. It is the exact
    # systematic disturbance an ESO is built to learn and cancel. wz (the
    # problem axis) is the most off. (vx, vy, wz).
    gain_factor: tuple[float, float, float] = (0.85, 0.85, 0.70)
    # Speed-dependent gain sag on top of the constant error: effective command =
    # gain_factor * cmd / (1 + k_sag*|cmd|) (nonlinearity). Per axis.
    k_sag: tuple[float, float, float] = (0.10, 0.10, 0.18)
    # Band-limited "gait" PROCESS noise added to the actual velocity (the
    # disturbance the ESO must reject). Per-axis stationary std + correlation
    # time (s).
    gait_sigma: tuple[float, float, float] = (0.035, 0.05, 0.07)
    gait_tau: float = 0.35
    # Odometry MEASUREMENT model: white std on (x, y) [m] and yaw [rad], plus a
    # slow random-walk drift on (x, y) [m/sqrt(tick)] clipped to drift_clip [m].
    odom_pos_sigma: float = 0.015
    odom_yaw_sigma: float = 0.010
    odom_drift_sigma: float = 0.0020
    odom_drift_clip: float = 0.07
    # Odometry arrives late relative to the commands that caused the motion.
    # Phase 2's feedback dead-time compensation should be evaluated against
    # this delayed-feedback path, not only against noisy current pose.
    odom_delay_s: float = 0.10

    @staticmethod
    def clean() -> HardPlantConfig:
        """FlowBase 'control' plant: ideal gain, no nonlinearity/gait, low odom
        noise. The ESO must be ~no-op here (no regression)."""
        return HardPlantConfig(
            gain_factor=(1.0, 1.0, 1.0),
            k_sag=(0.0, 0.0, 0.0),
            gait_sigma=(0.0, 0.0, 0.0),
            odom_pos_sigma=0.004,
            odom_yaw_sigma=0.003,
            odom_drift_sigma=0.0004,
            odom_drift_clip=0.02,
            odom_delay_s=0.02,
        )


class HardTwistPlant:
    """Unicycle FOPDT plant with gain sag + gait process noise.

    Reuses :class:`FOPDTChannel` for the per-axis first-order-lag + deadtime,
    then injects a multiplicative gain sag on the command and an additive
    Ornstein-Uhlenbeck gait perturbation on the actual velocity. The gait RNG
    is supplied by the caller and reset per arm so both arms replay the same
    process disturbance.
    """

    def __init__(
        self, params: TwistBasePlantParams, cfg: HardPlantConfig, rng: np.random.RandomState
    ) -> None:
        self._ch = [FOPDTChannel(params.vx), FOPDTChannel(params.vy), FOPDTChannel(params.wz)]
        self._cfg = cfg
        self._rng = rng
        self._gait = [0.0, 0.0, 0.0]
        self.x = self.y = self.yaw = 0.0
        self.vx = self.vy = self.wz = 0.0

    def reset(self, x: float, y: float, yaw: float, dt: float) -> None:
        self.x, self.y, self.yaw = x, y, yaw
        self.vx = self.vy = self.wz = 0.0
        self._gait = [0.0, 0.0, 0.0]
        for ch in self._ch:
            ch.reset(dt)

    def step(self, cvx: float, cvy: float, cwz: float, dt: float) -> None:
        cmds = (cvx, cvy, cwz)
        vels = []
        for i, ch in enumerate(self._ch):
            ks = self._cfg.k_sag[i]
            gf = self._cfg.gain_factor[i]
            # Constant gain error * speed-dependent sag, folded into the command
            # (the FOPDT channel still applies the nominal K, so the effective
            # steady gain is K * gf / (1 + ks|cmd|)).
            ceff = gf * cmds[i] / (1.0 + ks * abs(cmds[i])) if ks > 0.0 else gf * cmds[i]
            v = ch.step(ceff, dt)
            sigma = self._cfg.gait_sigma[i]
            if sigma > 0.0:
                tau = self._cfg.gait_tau
                g = self._gait[i]
                g += -g / tau * dt + sigma * math.sqrt(2.0 * dt / tau) * self._rng.standard_normal()
                self._gait[i] = g
                v += g
            vels.append(v)
        self.vx, self.vy, self.wz = vels
        self.x += (self.vx * math.cos(self.yaw) - self.vy * math.sin(self.yaw)) * dt
        self.y += (self.vx * math.sin(self.yaw) + self.vy * math.cos(self.yaw)) * dt
        self.yaw = (self.yaw + self.wz * dt + math.pi) % (2 * math.pi) - math.pi


# --------------------------------------------------------------------------
# Closed-loop run of the real control task
# --------------------------------------------------------------------------


def _pose(x: float, y: float, yaw: float) -> PoseStamped:
    return PoseStamped(
        position=Vector3(x, y, 0.0),
        orientation=Quaternion.from_euler(Vector3(0.0, 0.0, yaw)),
    )


def _sample_pose_history(
    history: list[tuple[float, float, float, float]], t: float
) -> tuple[float, float, float]:
    if t <= history[0][0]:
        return history[0][1], history[0][2], history[0][3]
    if t >= history[-1][0]:
        return history[-1][1], history[-1][2], history[-1][3]
    lo = 0
    hi = len(history) - 1
    while lo <= hi:
        mid = (lo + hi) // 2
        if history[mid][0] <= t:
            lo = mid + 1
        else:
            hi = mid - 1
    t0, x0, y0, yaw0 = history[hi]
    t1, x1, y1, yaw1 = history[hi + 1]
    frac = (t - t0) / max(t1 - t0, 1e-9)
    dyaw = (yaw1 - yaw0 + math.pi) % (2.0 * math.pi) - math.pi
    return x0 + frac * (x1 - x0), y0 + frac * (y1 - y0), yaw0 + frac * dyaw


# Both arms run for this multiple of the planned trajectory duration and are
# scored over that whole fixed window. Equal time + equal noise => the only
# difference is the ESO. This is what removes the confound the review caught:
# scoring "to arrival" let the ESO arm orbit a closed path longer and pad its
# cross-track RMS with extra low-error laps. On the hard plant neither arm
# actually completes within 1.5x (the plant is ~2x too slow for the plan), so
# there is no park-at-goal asymmetry; on the clean plant both arrive and park
# the same way (the ESO is a no-op there).
_HORIZON_FACTOR = 1.5


@dataclass
class RunResult:
    cte_rms: float  # geometric cross-track RMS vs the path (over the fixed window)
    cte_max: float
    # Time-indexed errors vs r(t): cross = perpendicular deviation from where the
    # robot SHOULD be at time t (on-path-ness); lag = how far BEHIND it is
    # (keeping-up). A cross-track "win" that is really a slowdown shows up as a
    # worse lag, so both are reported and the verdict checks both.
    cross_traj_rms: float
    along_lag_rms: float
    heading_rms: float
    cmd_rate: float  # smoothness (sum |dcmd|); chatter detector
    arrived: bool
    n_ticks: int
    xy: list[tuple[float, float]]  # ground-truth path (for overlays)
    ref: list[tuple[float, float]]


def run_arm(
    tracking: TrackingConfig,
    plant_params: TwistBasePlantParams,
    plant_cfg: HardPlantConfig,
    path: NavPath,
    speed: float,
    *,
    eso: bool,
    dial: float,
    tick_hz: float,
    seed: int,
    deadtime_compensation: bool = False,
    deadtime_feedback_lag_s: float | None = None,
    deadtime_prediction_blend: float = 1.0,
    deadtime_prediction_mode: Literal["full", "yaw_only"] = "full",
    yaw_feedforward_mode: Literal["planned", "measured_speed"] = "planned",
    command_rate_hz: float | None = None,
) -> RunResult:
    """Drive the real TrajectoryTrackingTask for a FIXED time window. ``seed``
    seeds BOTH the gait process noise and the odom measurement noise so the two
    arms (eso on/off) replay identical disturbances — the single-variable
    guarantee. Scoring over a fixed window (not "to arrival") is what makes the
    A/B honest: see ``_HORIZON_FACTOR``."""
    proc_rng = np.random.RandomState(seed)
    odom_rng = np.random.RandomState(seed + 10_000)
    plant = HardTwistPlant(plant_params, plant_cfg, proc_rng)

    dt = 1.0 / tick_hz
    plant_dt = dt / _SUBSTEPS
    start = path.poses[0]
    yaw0 = start.orientation.euler[2]
    plant.reset(start.position.x, start.position.y, yaw0, plant_dt)

    task = TrajectoryTrackingTask(
        "eso_ab",
        TrajectoryTrackingTaskConfig(
            joint_names=list(_JOINTS),
            tracking=tracking,
            max_speed=speed,
            eso=eso,
            eso_bandwidth=dial,
            deadtime_compensation=deadtime_compensation,
            deadtime_feedback_lag_s=(
                plant_cfg.odom_delay_s
                if deadtime_feedback_lag_s is None
                else deadtime_feedback_lag_s
            ),
            deadtime_prediction_blend=deadtime_prediction_blend,
            deadtime_prediction_mode=deadtime_prediction_mode,
            yaw_feedforward_mode=yaw_feedforward_mode,
            command_rate_hz=command_rate_hz,
        ),
    )
    assert task.start_path(path, _pose(start.position.x, start.position.y, yaw0))
    trajectory = task._trajectory
    assert trajectory is not None
    horizon_ticks = int(_HORIZON_FACTOR * trajectory.duration / dt) + 1

    drift = np.zeros(2)
    gt_history: list[tuple[float, float, float, float]] = [(0.0, plant.x, plant.y, plant.yaw)]
    ticks: list[TrajectoryTick] = []
    arrived = False
    for k in range(horizon_ticks):
        t_now = _T0 + k * dt
        # Noisy, drifting odom from a delayed ground-truth pose. The same
        # delayed samples and noise seed are used by both arms.
        odom_x, odom_y, odom_yaw = _sample_pose_history(gt_history, k * dt - plant_cfg.odom_delay_s)
        drift += odom_rng.normal(0.0, plant_cfg.odom_drift_sigma, size=2)
        np.clip(drift, -plant_cfg.odom_drift_clip, plant_cfg.odom_drift_clip, out=drift)
        nx = odom_x + drift[0] + odom_rng.normal(0.0, plant_cfg.odom_pos_sigma)
        ny = odom_y + drift[1] + odom_rng.normal(0.0, plant_cfg.odom_pos_sigma)
        nyaw = odom_yaw + odom_rng.normal(0.0, plant_cfg.odom_yaw_sigma)

        state = CoordinatorState(
            joints=JointStateSnapshot(
                joint_positions={_JOINTS[0]: nx, _JOINTS[1]: ny, _JOINTS[2]: nyaw},
                joint_velocities={j: 0.0 for j in _JOINTS},
            ),
            t_now=t_now,
            dt=dt,
        )
        out = task.compute(state)
        # Keep ticking for the full window even after arrival (command zero so
        # the robot parks at the goal) so both arms are scored over equal time.
        if not task.is_active():
            arrived = arrived or task.get_state() == "arrived"
            cvx, cvy, cwz = 0.0, 0.0, 0.0
        else:
            assert out is not None and out.velocities is not None
            cvx, cvy, cwz = out.velocities
        for _ in range(_SUBSTEPS):
            plant.step(cvx, cvy, cwz, plant_dt)
        gt_history.append((k * dt + dt, plant.x, plant.y, plant.yaw))
        ticks.append(
            TrajectoryTick(
                t=k * dt,
                pose=_pose(plant.x, plant.y, plant.yaw),  # GROUND TRUTH for scoring
                cmd_twist=Twist(
                    linear=Vector3(cvx, cvy, 0.0), angular=Vector3(0.0, 0.0, cwz)
                ),
                actual_twist=Twist(
                    linear=Vector3(plant.vx, plant.vy, 0.0),
                    angular=Vector3(0.0, 0.0, plant.wz),
                ),
            )
        )

    executed = ExecutedTrajectory(ticks=ticks, arrived=arrived)
    s = score_run(path, executed)  # geometric perpendicular distance to the path
    # Time-indexed score against r(t): the reference is the same TimedTrajectory
    # the controller tracks, sampled at each tick's wall-clock time. cross =
    # on-path-ness at time t; lag = keeping-up.
    st = score_run_with_trajectory(executed, trajectory.sample, duration_s=trajectory.duration)
    return RunResult(
        cte_rms=s.cte_rms,
        cte_max=s.cte_max,
        cross_traj_rms=st.cross_track_traj_rms,
        along_lag_rms=st.along_track_lag_rms,
        heading_rms=s.heading_err_rms,
        cmd_rate=s.cmd_rate_integral,
        arrived=arrived,
        n_ticks=len(ticks),
        xy=[(t.pose.position.x, t.pose.position.y) for t in ticks],
        ref=[(p.position.x, p.position.y) for p in path.poses],
    )


# --------------------------------------------------------------------------
# Plant / config / path registries
# --------------------------------------------------------------------------


def _go2_tracking() -> TrackingConfig:
    """Go2 TrackingConfig from the vendored fit (same path the task takes from
    a characterization artifact)."""
    artifact = derive_config(
        GO2_PLANT_FITTED,
        Provenance(robot_id="go2", surface="concrete", mode="default", sim_or_hw="sim"),
    )
    return TrackingConfig.from_artifact(artifact)


def _plants() -> dict[str, tuple[TrackingConfig, TwistBasePlantParams, HardPlantConfig]]:
    return {
        "go2-hard": (_go2_tracking(), GO2_PLANT_FITTED, HardPlantConfig()),
        "flowbase": (FLOWBASE_TRACKING, FLOWBASE_PLANT_FITTED, HardPlantConfig.clean()),
    }


def _paths() -> dict[str, NavPath]:
    return {
        "straight": straight_line(length=3.0),
        "circle_r1.0": circle(radius=1.0),
        "smooth_corner": smooth_corner(leg_length=2.0, angle_deg=90.0, arc_radius=0.5),
        "square": square(side=2.0),
    }


# --------------------------------------------------------------------------
# Sweeps
# --------------------------------------------------------------------------


@dataclass
class Cell:
    plant: str
    path: str
    speed: float
    dial: float
    base: RunResult
    eso: RunResult

    @property
    def rms_delta_pct(self) -> float:
        if self.base.cte_rms <= 1e-9:
            return 0.0
        return 100.0 * (self.eso.cte_rms - self.base.cte_rms) / self.base.cte_rms


def ab_sweep(
    plants: list[str],
    paths: list[str],
    speeds: list[float],
    dial: float,
    tick_hz: float,
    seed: int,
    *,
    treatment_eso: bool = True,
    deadtime_compensation: bool = False,
    deadtime_prediction_blend: float = 1.0,
    deadtime_prediction_mode: Literal["full", "yaw_only"] = "full",
    yaw_feedforward_mode: Literal["planned", "measured_speed"] = "planned",
    command_rate_hz: float | None = None,
) -> list[Cell]:
    registry = _plants()
    path_reg = _paths()
    cells: list[Cell] = []
    for pl in plants:
        tracking, params, pcfg = registry[pl]
        for pa in paths:
            path = path_reg[pa]
            for sp in speeds:
                # Same seed for both arms => identical noise realization.
                run_seed = _run_seed(seed, pl, pa, sp)
                base = run_arm(
                    tracking, params, pcfg, path, sp,
                    eso=False, dial=dial, tick_hz=tick_hz, seed=run_seed,
                )
                eso = run_arm(
                    tracking, params, pcfg, path, sp,
                    eso=treatment_eso,
                    dial=dial,
                    tick_hz=tick_hz,
                    seed=run_seed,
                    deadtime_compensation=deadtime_compensation,
                    deadtime_prediction_blend=deadtime_prediction_blend,
                    deadtime_prediction_mode=deadtime_prediction_mode,
                    yaw_feedforward_mode=yaw_feedforward_mode,
                    command_rate_hz=command_rate_hz,
                )
                cells.append(Cell(pl, pa, sp, dial, base, eso))
    return cells


# --------------------------------------------------------------------------
# Reporting
# --------------------------------------------------------------------------


# A cross-track change counts as a real win only if the along-track lag did not
# get materially worse (so the cte gain is not bought by falling behind).
_LAG_TOL_M = 0.02


def _fair(c: Cell) -> str:
    laggier = c.eso.along_lag_rms > c.base.along_lag_rms + _LAG_TOL_M
    if c.rms_delta_pct < -5:
        return "LAGGED" if laggier else "win"
    if c.rms_delta_pct > 5:
        return "regress"
    return "flat"


def _fmt_cell(c: Cell) -> str:
    return (
        f"  {c.plant:9} {c.path:14} v={c.speed:.2f}  "
        f"cte {c.base.cte_rms * 100:5.1f}->{c.eso.cte_rms * 100:5.1f}cm "
        f"({c.rms_delta_pct:+4.0f}%)  "
        f"lag {c.base.along_lag_rms * 100:5.1f}->{c.eso.along_lag_rms * 100:5.1f}cm  "
        f"[{_fair(c)}]"
    )


def print_table(cells: list[Cell]) -> None:
    print(
        "\n=== Trajectory-tracker A/B (fixed-time window; base->treatment; cte=geometric cross-track,"
        " lag=time-indexed along-track; cm) ==="
    )
    for c in cells:
        print(_fmt_cell(c))
    wins = sum(1 for c in cells if _fair(c) == "win")
    lagged = sum(1 for c in cells if _fair(c) == "LAGGED")
    regress = sum(1 for c in cells if _fair(c) == "regress")
    flat = sum(1 for c in cells if _fair(c) == "flat")
    print(
        f"\nsummary: {wins} clean wins, {lagged} cte-down-but-laggier, "
        f"{regress} regressions, {flat} flat (of {len(cells)} cells)"
    )


def plot_cte_vs_speed(cells: list[Cell], out: FsPath) -> None:
    plants = sorted({c.plant for c in cells})
    paths = sorted({c.path for c in cells})
    fig, axes = plt.subplots(
        len(plants), len(paths), figsize=(4.2 * len(paths), 3.4 * len(plants)), squeeze=False
    )
    for r, pl in enumerate(plants):
        for col, pa in enumerate(paths):
            ax = axes[r][col]
            sel = sorted((c for c in cells if c.plant == pl and c.path == pa), key=lambda c: c.speed)
            if sel:
                xs = [c.speed for c in sel]
                ax.plot(xs, [c.base.cte_rms * 100 for c in sel], "o-", label="baseline")
                ax.plot(xs, [c.eso.cte_rms * 100 for c in sel], "s-", label="treatment")
            ax.set_title(f"{pl} / {pa}", fontsize=9)
            ax.set_xlabel("speed (m/s)")
            ax.set_ylabel("cte_rms (cm)")
            ax.grid(True, alpha=0.3)
            ax.legend(fontsize=7)
    fig.suptitle("ESO A/B: cross-track RMS vs speed (baseline vs ESO)")
    fig.tight_layout()
    fig.savefig(out, dpi=120)
    plt.close(fig)


def plot_xy(cells: list[Cell], out: FsPath, plant: str, speed: float) -> None:
    sel = [c for c in cells if c.plant == plant and abs(c.speed - speed) < 1e-6]
    if not sel:
        return
    n = len(sel)
    cols = min(n, 2)
    rows = (n + cols - 1) // cols
    fig, axes = plt.subplots(rows, cols, figsize=(5.5 * cols, 5.0 * rows), squeeze=False)
    flat = [a for row in axes for a in row]
    for ax, c in zip(flat, sel, strict=False):
        rx = [p[0] for p in c.base.ref]
        ry = [p[1] for p in c.base.ref]
        ax.plot(rx, ry, "k-", lw=2.0, label="reference")
        ax.plot([p[0] for p in c.base.xy], [p[1] for p in c.base.xy], "C0", lw=1.2,
                label=f"baseline (max {c.base.cte_max * 100:.0f}cm)")
        ax.plot([p[0] for p in c.eso.xy], [p[1] for p in c.eso.xy], "C1", lw=1.2,
                label=f"treatment (max {c.eso.cte_max * 100:.0f}cm)")
        ax.set_title(f"{c.plant} / {c.path} @ {c.speed:.2f} m/s", fontsize=9)
        ax.set_aspect("equal", adjustable="datalim")
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=7)
    for ax in flat[n:]:
        ax.set_visible(False)
    fig.tight_layout()
    fig.savefig(out, dpi=120)
    plt.close(fig)


def _result_dict(r: RunResult) -> dict:
    d = asdict(r)
    d.pop("xy")
    d.pop("ref")
    return d


def write_json(
    cells: list[Cell],
    dial: float,
    tick_hz: float,
    out: FsPath,
    *,
    treatment_eso: bool = True,
    deadtime_compensation: bool = False,
    deadtime_prediction_blend: float = 1.0,
    deadtime_prediction_mode: str = "full",
    yaw_feedforward_mode: str = "planned",
    command_rate_hz: float | None = None,
) -> None:
    payload = {
        "dial": dial,
        "tick_hz": tick_hz,
        "treatment_eso": treatment_eso,
        "deadtime_compensation": deadtime_compensation,
        "deadtime_prediction_blend": deadtime_prediction_blend,
        "deadtime_prediction_mode": deadtime_prediction_mode,
        "yaw_feedforward_mode": yaw_feedforward_mode,
        "command_rate_hz": command_rate_hz,
        "cells": [
            {
                "plant": c.plant,
                "path": c.path,
                "speed": c.speed,
                "baseline": _result_dict(c.base),
                "eso": _result_dict(c.eso),
                "rms_delta_pct": c.rms_delta_pct,
            }
            for c in cells
        ],
    }
    out.write_text(json.dumps(payload, indent=2))


def dial_sweep(
    plant: str, path: str, speed: float, dials: list[float], tick_hz: float, seed: int
) -> list[tuple[float, RunResult]]:
    registry = _plants()
    tracking, params, pcfg = registry[plant]
    pobj = _paths()[path]
    run_seed = _run_seed(seed, plant, path, speed)
    base = run_arm(tracking, params, pcfg, pobj, speed, eso=False, dial=1.0, tick_hz=tick_hz, seed=run_seed)
    out = [(0.0, base)]
    for d in dials:
        out.append(
            (d, run_arm(tracking, params, pcfg, pobj, speed, eso=True, dial=d, tick_hz=tick_hz, seed=run_seed))
        )
    return out


def dial_grid(
    cases: list[tuple[str, str, float]], dials: list[float], tick_hz: float, seed: int
) -> None:
    """Sweep the ESO dial across several (plant, path, speed) cases at once, so
    a single dial that is a NET win (curves down, straights not up) can be
    picked. Prints baseline + each dial's cte_rms per case."""
    registry = _plants()
    path_reg = _paths()
    print(f"=== ESO dial grid (cte_rms cm; dials {dials}) ===")
    header = "  case".ljust(34) + "base  " + "  ".join(f"d{d:g}".rjust(6) for d in dials)
    print(header)
    for pl, pa, sp in cases:
        tracking, params, pcfg = registry[pl]
        pobj = path_reg[pa]
        rs = _run_seed(seed, pl, pa, sp)
        base = run_arm(tracking, params, pcfg, pobj, sp, eso=False, dial=1.0, tick_hz=tick_hz, seed=rs)
        cells = []
        for d in dials:
            r = run_arm(tracking, params, pcfg, pobj, sp, eso=True, dial=d, tick_hz=tick_hz, seed=rs)
            # cte%, with 'L' marker when the along-track lag got materially worse
            mark = "L" if r.along_lag_rms > base.along_lag_rms + _LAG_TOL_M else ""
            dpct = 100 * (r.cte_rms - base.cte_rms) / base.cte_rms if base.cte_rms > 1e-9 else 0
            cells.append(f"{dpct:+4.0f}{mark}")
        label = f"  {pl}/{pa}@{sp:g}".ljust(30)
        print(label + f"base {base.cte_rms * 100:4.1f}cm  " + " ".join(c.rjust(6) for c in cells))


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------


def main() -> None:
    ap = argparse.ArgumentParser(description="ESO/ADRC sim A/B harness")
    ap.add_argument("--plants", default="go2-hard,flowbase")
    ap.add_argument("--paths", default="straight,circle_r1.0,smooth_corner,square")
    ap.add_argument("--speeds", default="0.3,0.5,0.7,1.0")
    ap.add_argument("--dial", type=float, default=1.0, help="ESO bandwidth dial")
    ap.add_argument("--tick-hz", type=float, default=16.0, help="control = odom rate")
    ap.add_argument(
        "--command-rate-hz",
        type=float,
        default=None,
        help="optional task command recompute throttle for both arms",
    )
    ap.add_argument(
        "--deadtime-comp",
        action="store_true",
        help="add feedback dead-time compensation to the ESO treatment arm",
    )
    ap.add_argument(
        "--treatment-no-eso",
        action="store_true",
        help="disable ESO in the treatment arm, for isolated Phase 2 A/B",
    )
    ap.add_argument("--deadtime-prediction-blend", type=float, default=1.0)
    ap.add_argument("--deadtime-prediction-mode", choices=["full", "yaw_only"], default="full")
    ap.add_argument("--yaw-feedforward-mode", choices=["planned", "measured_speed"], default="planned")
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--out", default=None)
    ap.add_argument("--quick", action="store_true", help="go2-hard, circle+straight, 0.5/1.0 only")
    ap.add_argument(
        "--dial-sweep",
        action="store_true",
        help="sweep the ESO dial on go2-hard circle@1.0 instead of the A/B grid",
    )
    ap.add_argument(
        "--dial-grid",
        action="store_true",
        help="sweep the ESO dial across several go2-hard cases (pick a net-win dial)",
    )
    args = ap.parse_args()

    if args.dial_grid:
        dial_grid(
            [
                ("go2-hard", "straight", 0.5),
                ("go2-hard", "straight", 1.0),
                ("go2-hard", "circle_r1.0", 0.5),
                ("go2-hard", "circle_r1.0", 1.0),
                ("go2-hard", "smooth_corner", 0.7),
                ("go2-hard", "smooth_corner", 1.0),
                ("go2-hard", "square", 0.7),
                ("go2-hard", "square", 1.0),
            ],
            [0.15, 0.25, 0.4, 0.6],
            args.tick_hz,
            args.seed,
        )
        return

    out_dir = FsPath(args.out).expanduser() if args.out else DEFAULT_OUT_DIR
    out_dir.mkdir(parents=True, exist_ok=True)

    if args.dial_sweep:
        dials = [0.5, 0.75, 1.0, 1.5, 2.0, 3.0]
        print("=== ESO dial sweep: go2-hard / circle_r1.0 @ 1.0 m/s ===")
        rows = dial_sweep("go2-hard", "circle_r1.0", 1.0, dials, args.tick_hz, args.seed)
        for d, r in rows:
            tag = "baseline" if d == 0.0 else f"dial={d:>4.2f}"
            print(f"  {tag}: cte_rms={r.cte_rms * 100:6.1f}cm  cte_max={r.cte_max * 100:6.1f}cm  "
                  f"cmd_rate={r.cmd_rate:6.1f}  arrived={r.arrived}")
        return

    if args.quick:
        plants = ["go2-hard"]
        paths = ["straight", "circle_r1.0"]
        speeds = [0.5, 1.0]
    else:
        plants = args.plants.split(",")
        paths = args.paths.split(",")
        speeds = [float(s) for s in args.speeds.split(",")]

    cells = ab_sweep(
        plants,
        paths,
        speeds,
        args.dial,
        args.tick_hz,
        args.seed,
        treatment_eso=not args.treatment_no_eso,
        deadtime_compensation=args.deadtime_comp,
        deadtime_prediction_blend=args.deadtime_prediction_blend,
        deadtime_prediction_mode=args.deadtime_prediction_mode,
        yaw_feedforward_mode=args.yaw_feedforward_mode,
        command_rate_hz=args.command_rate_hz,
    )
    print_table(cells)

    write_json(
        cells,
        args.dial,
        args.tick_hz,
        out_dir / "eso_ab.json",
        treatment_eso=not args.treatment_no_eso,
        deadtime_compensation=args.deadtime_comp,
        deadtime_prediction_blend=args.deadtime_prediction_blend,
        deadtime_prediction_mode=args.deadtime_prediction_mode,
        yaw_feedforward_mode=args.yaw_feedforward_mode,
        command_rate_hz=args.command_rate_hz,
    )
    plot_cte_vs_speed(cells, out_dir / "eso_ab_cte_vs_speed.png")
    top = max(speeds)
    for pl in plants:
        plot_xy(cells, out_dir / f"eso_ab_xy_{pl}_v{top:g}.png", pl, top)
    print(f"\nartifacts -> {out_dir}")


if __name__ == "__main__":
    main()
