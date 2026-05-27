#!/usr/bin/env python
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

"""End-to-end ReferenceGovernor demo (in-process sim).

Wires the governor as the BaselinePathFollowerTask's `external_profile_cap`,
drives a manual tick loop against a TwistBasePlantSim, and prints the CTE
summary using the existing benchmark scoring.

Two modes:
  --mode static       fixed e_max throughout the run
  --mode square-wave  e_max alternates high/low in a background thread,
                      exercising the atomic-snapshot recompute path

The same coordinator + follower + plant code path runs against real
hardware when the operator coord brings up a DDS adapter instead of the
in-process FOPDT plant — this script is the simplest reproducible variant.

Usage examples (from repo root):
    uv run python examples/go2_reference_governor_demo.py --path single_corner --e-max 0.05
    uv run python examples/go2_reference_governor_demo.py --path circle --mode square-wave \\
        --e-max-high 0.10 --e-max-low 0.02 --period 4.0
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path as FsPath
import sys
import tempfile
import threading
import time

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from dimos.control.task import (
    CoordinatorState,
    JointStateSnapshot,
)
from dimos.control.tasks.baseline_path_follower_task import (
    BaselinePathFollowerTask,
    BaselinePathFollowerTaskConfig,
)
from dimos.core.global_config import global_config
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Path import Path as NavPath
from dimos.navigation.reference_governor import ReferenceGovernor
from dimos.utils.benchmarking import paths as path_battery
from dimos.utils.benchmarking.plant import (
    GO2_PLANT_FITTED,
    TwistBasePlantSim,
)
from dimos.utils.benchmarking.scoring import (
    ExecutedTrajectory,
    TrajectoryTick,
    score_run,
)
from dimos.utils.benchmarking.tuning import (
    FopdtChannelDC,
    PlantModelDC,
    Provenance,
    derive_config,
    git_sha,
)

# --- Joints used by the BaselinePathFollowerTask (twist-base trio) ---------
_JX, _JY, _JYAW = "base/vx", "base/vy", "base/wz"
_JOINTS = [_JX, _JY, _JYAW]


def _write_self_test_artifact(tmpdir: FsPath) -> FsPath:
    """Emit a minimal TuningConfig artifact at tmpdir/config.json built
    from the vendored Go2 plant. Avoids requiring a real characterization
    artifact on disk just to run the demo."""
    plant = GO2_PLANT_FITTED
    prov = Provenance(
        robot_id="go2-demo",
        surface="demo",
        mode="default",
        date="demo",
        git_sha=git_sha(),
        sim_or_hw="hw",  # hw so valid_for_tuning=True (we want the gates open)
        characterization_session_dir="self-test (demo script)",
    )
    cfg = derive_config(plant, prov)
    # Substitute the artifact's plant DC for explicit clarity in JSON.
    cfg.plant = PlantModelDC(
        vx=FopdtChannelDC(plant.vx.K, plant.vx.tau, plant.vx.L),
        vy=FopdtChannelDC(plant.vy.K, plant.vy.tau, plant.vy.L),
        wz=FopdtChannelDC(plant.wz.K, plant.wz.tau, plant.wz.L),
    )
    p = tmpdir / "go2_demo_config.json"
    cfg.to_json(p)
    return p


def _resolve_path(name: str) -> NavPath:
    table = {
        "straight_line": lambda: path_battery.straight_line(length=2.0),
        "single_corner": lambda: path_battery.single_corner(leg_length=2.0, angle_deg=90.0),
        "circle": lambda: path_battery.circle(radius=1.0, n_points=120),
        "square": lambda: path_battery.square(side=2.0),
        "figure_eight": lambda: path_battery.figure_eight(loop_radius=1.0, n_points=200),
        "slalom": lambda: path_battery.slalom(),
        "smooth_corner": lambda: path_battery.smooth_corner(
            leg_length=2.0, angle_deg=90.0, arc_radius=0.5
        ),
    }
    if name not in table:
        raise SystemExit(f"unknown --path {name!r}; pick one of {sorted(table)}")
    return table[name]()


def _state(plant: TwistBasePlantSim, t_now: float, dt: float) -> CoordinatorState:
    return CoordinatorState(
        joints=JointStateSnapshot(
            joint_positions={_JX: plant.x, _JY: plant.y, _JYAW: plant.yaw},
            joint_velocities={_JX: plant.vx, _JY: plant.vy, _JYAW: plant.wz},
            joint_efforts={_JX: 0.0, _JY: 0.0, _JYAW: 0.0},
            timestamp=t_now,
        ),
        t_now=t_now,
        dt=dt,
    )


def _start_pose(plant: TwistBasePlantSim, t_now: float) -> PoseStamped:
    return PoseStamped(
        ts=t_now,
        position=Vector3(plant.x, plant.y, 0.0),
        orientation=Quaternion.from_euler(Vector3(0.0, 0.0, plant.yaw)),
    )


def _square_wave_e_max(
    governor: ReferenceGovernor,
    e_high: float,
    e_low: float,
    period: float,
    stop: threading.Event,
    log: list[tuple[float, float]],
) -> None:
    t0 = time.perf_counter()
    half = period / 2.0
    while not stop.is_set():
        for value in (e_high, e_low):
            if stop.is_set():
                break
            governor.update_e_max(value)
            log.append((time.perf_counter() - t0, value))
            stop.wait(half)


@dataclass
class RunResult:
    traj: ExecutedTrajectory
    e_log: list[tuple[float, float]]
    alpha_log: list[tuple[float, float]]  # (t, alpha) — empty unless closed_loop
    cte_log: list[tuple[float, float]]  # (t, instantaneous CTE m) — empty unless closed_loop


def _run_once(
    path: NavPath,
    args: argparse.Namespace,
    artifact_path: FsPath,
    use_governor: bool,
    label: str,
) -> RunResult:
    """Single pass: build follower (+ optional governor), drive plant, return
    trajectory + telemetry. Used once normally, twice in --compare mode."""
    governor: ReferenceGovernor | None = None
    if use_governor:
        governor = ReferenceGovernor(
            plant_artifact_path=str(artifact_path),
            e_max_default=args.e_max,
            closed_loop=bool(args.closed_loop),
        )
        governor.set_path(path)

    follower_cfg = BaselinePathFollowerTaskConfig(
        joint_names=_JOINTS,
        priority=20,
        speed=args.speed,
        control_frequency=float(args.tick_hz),
        goal_tolerance=0.2,
        orientation_tolerance=0.35,
        k_angular=0.5,
    )
    follower = BaselinePathFollowerTask(
        name="baseline_follower",
        config=follower_cfg,
        global_config=global_config,
        external_profile_cap=governor,
    )

    dt = 1.0 / args.tick_hz
    plant = TwistBasePlantSim(GO2_PLANT_FITTED)
    plant.reset(0.0, 0.0, 0.0, dt)

    start_pose = _start_pose(plant, time.perf_counter())
    if not follower.start_path(path, start_pose):
        print(f"[{label}] start_path rejected", file=sys.stderr)
        if governor is not None:
            governor._close_module()
        return RunResult(
            traj=ExecutedTrajectory(ticks=[], arrived=False),
            e_log=[],
            alpha_log=[],
            cte_log=[],
        )

    e_log: list[tuple[float, float]] = []
    alpha_log: list[tuple[float, float]] = []
    cte_log: list[tuple[float, float]] = []
    stop_ev = threading.Event()
    sq_thread: threading.Thread | None = None
    if use_governor and args.mode == "square-wave" and governor is not None:
        sq_thread = threading.Thread(
            target=_square_wave_e_max,
            args=(
                governor,
                args.e_max_high,
                args.e_max_low,
                args.period,
                stop_ev,
                e_log,
            ),
            daemon=True,
        )
        sq_thread.start()

    ticks: list[TrajectoryTick] = []
    t0 = time.perf_counter()
    deadline = t0 + args.timeout
    prev_t = t0
    arrived = False

    while time.perf_counter() < deadline:
        now = time.perf_counter()
        dt_tick = max(dt, now - prev_t)
        state = _state(plant, now, dt_tick)
        output = follower.compute(state)
        if output is None or output.velocities is None:
            if follower.get_state() == "arrived":
                arrived = True
            break
        vx, vy, wz = output.velocities
        plant.step(vx, vy, wz, dt_tick)
        prev_t = now
        ticks.append(
            TrajectoryTick(
                t=now - t0,
                pose=_start_pose(plant, now),
                cmd_twist=Twist(
                    linear=Vector3(vx, vy, 0.0),
                    angular=Vector3(0.0, 0.0, wz),
                ),
                actual_twist=Twist(
                    linear=Vector3(plant.vx, plant.vy, 0.0),
                    angular=Vector3(0.0, 0.0, plant.wz),
                ),
            )
        )
        # Closed-loop telemetry: lock-safe snapshot of alpha + filtered CTE
        # (these are updated inside speed_limit_at on every follower tick).
        if governor is not None and args.closed_loop:
            with governor._state_lock:
                alpha = governor._alpha
                cte_filt = governor._cte_filtered
            alpha_log.append((now - t0, alpha))
            cte_log.append((now - t0, cte_filt))

        sleep_for = (prev_t + dt) - time.perf_counter()
        if sleep_for > 0:
            time.sleep(sleep_for)

    if sq_thread is not None:
        stop_ev.set()
        sq_thread.join(timeout=1.0)
    if governor is not None:
        governor._close_module()

    return RunResult(
        traj=ExecutedTrajectory(ticks=ticks, arrived=arrived),
        e_log=e_log,
        alpha_log=alpha_log,
        cte_log=cte_log,
    )


def _report(
    label: str,
    path: NavPath,
    result: RunResult,
    e_max: float,
) -> None:
    score = score_run(path, result.traj)
    duration = result.traj.ticks[-1].t if result.traj.ticks else 0.0
    lines = [
        f"\n=== {label} ===",
        f"  arrived       : {result.traj.arrived}",
        f"  cte_max       : {score.cte_max * 100:.1f} cm",
        f"  cte_rms       : {score.cte_rms * 100:.1f} cm",
        f"  duration      : {duration:.1f} s ({len(result.traj.ticks)} ticks)",
    ]
    if result.alpha_log:
        final_alpha = result.alpha_log[-1][1]
        within = "YES" if score.cte_max <= e_max else "NO"
        lines.extend(
            [
                f"  final alpha   : {final_alpha:.3f}",
                f"  converged?    : {within}  (cte_max {score.cte_max * 100:.1f}cm vs target {e_max * 100:.1f}cm)",
            ]
        )
    print("\n".join(lines))


def run(args: argparse.Namespace) -> int:
    with tempfile.TemporaryDirectory() as raw_tmp:
        tmpdir = FsPath(raw_tmp)
        artifact_path = FsPath(args.config) if args.config else _write_self_test_artifact(tmpdir)
        path = _resolve_path(args.path)
        out = FsPath(args.out)
        out.parent.mkdir(parents=True, exist_ok=True)

        if args.compare:
            r_g = _run_once(
                path,
                args,
                artifact_path,
                use_governor=True,
                label=f"{args.path} WITH governor",
            )
            r_n = _run_once(
                path,
                args,
                artifact_path,
                use_governor=False,
                label=f"{args.path} WITHOUT governor",
            )
            _report(f"{args.path} (mode={args.mode}) WITH governor", path, r_g, args.e_max)
            _report(f"{args.path} (mode={args.mode}) WITHOUT governor", path, r_n, args.e_max)
            _plot_compare(path, r_g, r_n, args, out)
        else:
            r = _run_once(
                path,
                args,
                artifact_path,
                use_governor=not args.no_governor,
                label=args.path,
            )
            if args.no_governor:
                tag = "WITHOUT governor"
            elif args.closed_loop:
                tag = f"CLOSED-LOOP governor e_max={args.e_max:g}m"
            else:
                tag = f"open-loop governor e_max={args.e_max:g}m"
            _report(f"{args.path} (mode={args.mode}) {tag}", path, r, args.e_max)
            _plot(path, r, args, out)

        print(f"plot           : {out.resolve()}")
        # Single-run mode returns 0 only if arrived; compare always returns 0
        # (it's diagnostic, not a pass/fail gate).
        return 0


def _plot_compare(
    path: NavPath,
    r_g: RunResult,
    r_n: RunResult,
    args: argparse.Namespace,
    out: FsPath,
) -> None:
    """Side-by-side: XY overlay (left) + |cmd vx|(t) overlay (right), with
    an optional third subplot showing alpha(t) + measured CTE(t) when the
    closed-loop variant ran. The closed-loop subplot is the diagnostic
    surface for "did alpha converge to a steady value that meets the
    corridor"."""
    have_cl = bool(r_g.alpha_log)
    ncols = 3 if have_cl else 2
    fig, axes = plt.subplots(1, ncols, figsize=(6.5 * ncols, 5.0))
    ax_xy = axes[0]
    ax_v = axes[1]

    ref_x = [p.position.x for p in path.poses]
    ref_y = [p.position.y for p in path.poses]
    ax_xy.plot(ref_x, ref_y, "k--", lw=2, label="reference")

    def _draw(ax, ticks, color, label):
        xs = [tk.pose.position.x for tk in ticks]
        ys = [tk.pose.position.y for tk in ticks]
        if not xs:
            return
        ax.plot(xs, ys, color=color, lw=1.4, label=label)
        ax.plot(xs[0], ys[0], "o", color=color, ms=5)
        ax.plot(xs[-1], ys[-1], "s", color=color, ms=5)

    sg = score_run(path, r_g.traj)
    sn = score_run(path, r_n.traj)
    _draw(ax_xy, r_g.traj.ticks, "tab:blue", f"WITH governor (cte_max={sg.cte_max * 100:.1f}cm)")
    _draw(
        ax_xy, r_n.traj.ticks, "tab:orange", f"WITHOUT governor (cte_max={sn.cte_max * 100:.1f}cm)"
    )
    ax_xy.set_aspect("equal", adjustable="datalim")
    ax_xy.set_xlabel("x (m)")
    ax_xy.set_ylabel("y (m)")
    ax_xy.grid(True, alpha=0.3)
    ax_xy.legend(fontsize=8)
    ax_xy.set_title(f"{args.path} — XY tracking")

    tg = [tk.t for tk in r_g.traj.ticks]
    vg = [abs(tk.cmd_twist.linear.x) for tk in r_g.traj.ticks]
    tn = [tk.t for tk in r_n.traj.ticks]
    vn = [abs(tk.cmd_twist.linear.x) for tk in r_n.traj.ticks]
    ax_v.plot(tg, vg, "tab:blue", lw=1.0, label="WITH governor")
    ax_v.plot(tn, vn, "tab:orange", lw=1.0, label="WITHOUT governor")
    if r_g.e_log:
        ax2 = ax_v.twinx()
        ts = [t for t, _ in r_g.e_log]
        es = [e for _, e in r_g.e_log]
        ax2.step(ts, es, "r-", where="post", lw=0.8, alpha=0.6, label="e_max")
        ax2.set_ylabel("e_max (m)", color="r")
    ax_v.set_xlabel("t (s)")
    ax_v.set_ylabel("|cmd vx| (m/s)")
    ax_v.set_title("Commanded forward speed")
    ax_v.grid(True, alpha=0.3)
    ax_v.legend(fontsize=8, loc="upper right")

    if have_cl:
        _plot_closed_loop_panel(axes[2], r_g, args.e_max)

    fig.tight_layout()
    fig.savefig(out, dpi=120)
    plt.close(fig)


def _plot(
    path: NavPath,
    r: RunResult,
    args: argparse.Namespace,
    out: FsPath,
) -> None:
    have_cl = bool(r.alpha_log)
    have_e = bool(r.e_log)
    ncols = 1 + int(have_e) + int(have_cl)
    fig, axes = plt.subplots(1, ncols, figsize=(6.0 * ncols, 5.0), squeeze=False)
    panel = list(axes[0])
    ax_xy = panel.pop(0)

    ref_x = [p.position.x for p in path.poses]
    ref_y = [p.position.y for p in path.poses]
    exec_x = [tk.pose.position.x for tk in r.traj.ticks]
    exec_y = [tk.pose.position.y for tk in r.traj.ticks]
    ax_xy.plot(ref_x, ref_y, "k--", lw=2, label="reference")
    ax_xy.plot(exec_x, exec_y, "b-", lw=1.3, label="executed")
    if exec_x:
        ax_xy.plot(exec_x[0], exec_y[0], "go", ms=6, label="start")
        ax_xy.plot(exec_x[-1], exec_y[-1], "rs", ms=6, label="end")
    ax_xy.set_aspect("equal", adjustable="datalim")
    ax_xy.set_xlabel("x (m)")
    ax_xy.set_ylabel("y (m)")
    ax_xy.grid(True, alpha=0.3)
    ax_xy.legend(fontsize=8)
    mode_tag = "CL" if args.closed_loop else "OL"
    ax_xy.set_title(f"{args.path} — {mode_tag} governor e_max={args.e_max:g}m")

    if have_e:
        ax_e = panel.pop(0)
        ts = [t for t, _ in r.e_log]
        es = [e for _, e in r.e_log]
        cmd_t = [tk.t for tk in r.traj.ticks]
        cmd_v = [abs(tk.cmd_twist.linear.x) for tk in r.traj.ticks]
        ax_e.step(ts, es, "r-", where="post", label="e_max (m)")
        ax2 = ax_e.twinx()
        ax2.plot(cmd_t, cmd_v, "b-", lw=0.8, alpha=0.7, label="|cmd vx|")
        ax_e.set_xlabel("t (s)")
        ax_e.set_ylabel("e_max (m)", color="r")
        ax2.set_ylabel("|cmd vx| (m/s)", color="b")
        ax_e.set_title("e_max vs commanded speed")
        ax_e.grid(True, alpha=0.3)

    if have_cl:
        _plot_closed_loop_panel(panel.pop(0), r, args.e_max)

    fig.tight_layout()
    fig.savefig(out, dpi=120)
    plt.close(fig)


def _plot_closed_loop_panel(ax, r: RunResult, e_max: float) -> None:
    """alpha(t) on the left axis, instantaneous CTE(t) on the right axis,
    with a horizontal line at e_max so convergence is visible at a glance."""
    if not r.alpha_log:
        return
    t_a = [t for t, _ in r.alpha_log]
    a = [v for _, v in r.alpha_log]
    t_c = [t for t, _ in r.cte_log]
    c = [v * 100 for _, v in r.cte_log]  # cm
    ax.plot(t_a, a, "tab:blue", lw=1.0, label="alpha(t)")
    ax.set_ylabel("alpha", color="tab:blue")
    ax.set_ylim(0.0, 1.05)
    ax2 = ax.twinx()
    ax2.plot(t_c, c, "tab:red", lw=0.9, alpha=0.7, label="|CTE| (cm)")
    ax2.axhline(
        e_max * 100, color="tab:red", ls=":", lw=0.8, alpha=0.6, label=f"e_max={e_max * 100:.0f}cm"
    )
    ax2.set_ylabel("|CTE| (cm)", color="tab:red")
    ax.set_xlabel("t (s)")
    ax.set_title("Closed-loop alpha vs measured CTE")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper left", fontsize=8)
    ax2.legend(loc="upper right", fontsize=8)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "--config",
        help="TuningConfig JSON. If omitted, a self-test artifact "
        "is generated from the vendored Go2 plant.",
    )
    ap.add_argument(
        "--path",
        default="single_corner",
        help="straight_line | single_corner | circle | square | figure_eight | "
        "slalom | smooth_corner",
    )
    ap.add_argument(
        "--speed",
        type=float,
        default=0.55,
        help="follower base speed (m/s). Governor caps it per-waypoint.",
    )
    ap.add_argument("--tick-hz", type=float, default=20.0)
    ap.add_argument("--timeout", type=float, default=60.0)
    ap.add_argument("--mode", choices=["static", "square-wave"], default="static")
    # static mode
    ap.add_argument("--e-max", type=float, default=0.05, help="static corridor half-width (m)")
    # square-wave mode
    ap.add_argument("--e-max-high", type=float, default=0.10)
    ap.add_argument("--e-max-low", type=float, default=0.02)
    ap.add_argument(
        "--period",
        type=float,
        default=4.0,
        help="square-wave period (s); half-period at each level",
    )
    ap.add_argument("--out", default="/tmp/reference_governor_demo.png")
    # --- comparison knobs ---
    ap.add_argument(
        "--no-governor",
        action="store_true",
        help="Run a single pass WITHOUT the governor (bare follower at "
        "--speed). Use as a baseline reference.",
    )
    ap.add_argument(
        "--compare",
        action="store_true",
        help="Run the same path TWICE: once with the governor, once "
        "without. Overlays both trajectories on the output plot.",
    )
    # --- closed-loop knob (only meaningful when the governor is in use) ---
    ap.add_argument(
        "--closed-loop",
        action="store_true",
        help="Enable the closed-loop alpha-feedback variant: the governor "
        "observes per-tick CTE and dynamically scales the open-loop "
        "bound to drive actual tracking error toward e_max. Default "
        "off (open-loop). Implies the governor is in use; ignored "
        "when --no-governor is set.",
    )
    args = ap.parse_args()
    if args.no_governor and args.compare:
        ap.error("--no-governor and --compare are mutually exclusive")
    if args.no_governor and args.closed_loop:
        ap.error("--closed-loop has no effect with --no-governor")
    return run(args)


if __name__ == "__main__":
    sys.exit(main())
