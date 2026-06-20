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

"""Offline re-fit of a stored recording into a plant model + tuning artifact.

No robot: load a recording, gate-segment it into SI steps, drop sub-floor probes,
and measure the steady-state GAIN K per axis directly (settled velocity / amp).
tau/L are reported as bounded nominals -- 16 Hz pose cannot identify them, and a
FOPDT fit that tries only ends up biasing K (the gain and deadtime trade off), so
we don't. Emits the standard ``TuningConfig`` artifact + a quality sidecar
(per-amplitude K, gain spread, tau/L-nominal flag) and gain-envelope/step PNGs.

The existing ``characterization --mode re-derive`` only re-applies the envelope;
it does NOT re-derive the gain from the recording. This does.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import date
import json
from pathlib import Path
import warnings

import numpy as np

from dimos.utils.benchmarking.plant import FopdtChannelParams, TwistBasePlantParams
from dimos.utils.benchmarking.tuning import Provenance, derive_config, git_sha
from dimos.utils.characterization.recording_io import (
    Recording,
    StepSpan,
    load_recording,
    segment_steps,
    step_pose_channel,
)

_AXES = ("vx", "vy", "wz")
# Nominal tau/L: 16 Hz pose cannot resolve them (documented), so we do NOT fit
# them -- we report these bounded nominals and flag them as un-identified.
_NOMINAL_TAU_S = 0.30
_NOMINAL_L_S = 0.15
# Fraction of a step treated as transient; the steady-state gain is measured on
# the remaining settled tail.
_SETTLE_FRAC = 0.4
# Minimum net step displacement (m for vx/vy, rad for wz) to count as real
# motion. Floor-probe steps (commanded below the floor) move ~0 and are excluded.
_MOTION_MIN = 0.3


def _net_motion(recording: Recording, span: StepSpan) -> float:
    """Net displacement magnitude of a step's pose channel (0 if no samples)."""
    _, p = step_pose_channel(recording, span)
    return abs(float(p[-1] - p[0])) if p.size else 0.0


def _moving_spans(recording: Recording, spans: list[StepSpan]) -> list[StepSpan]:
    """Steps that actually moved the robot (drops sub-floor / no-motion probes)."""
    return [s for s in spans if _net_motion(recording, s) >= _MOTION_MIN]


def _steady_gain(t_rel: np.ndarray, p_meas: np.ndarray, amp: float) -> tuple[float, float]:
    """Directly-measured steady-state gain ``K = v_ss / amp`` and the line r^2.

    At steady state the FOPDT output velocity is exactly ``K * amp``, so a linear
    fit of the SETTLED tail of the pose channel gives K with no dependence on
    tau/L (which 16 Hz pose can't resolve and which otherwise bias K). Returns
    ``(K, r2)`` of that line fit, or ``(nan, nan)`` if the step is too short.
    """
    if t_rel.size < 4 or abs(amp) < 1e-9:
        return float("nan"), float("nan")
    duration = float(t_rel[-1] - t_rel[0])
    settled = (t_rel - t_rel[0]) >= _SETTLE_FRAC * duration
    if int(np.count_nonzero(settled)) < 3:
        return float("nan"), float("nan")
    tt, pp = t_rel[settled], p_meas[settled]
    slope, intercept = np.polyfit(tt, pp, 1)
    pred = slope * tt + intercept
    ss_res = float(np.sum((pp - pred) ** 2))
    ss_tot = float(np.sum((pp - np.mean(pp)) ** 2))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else float("nan")
    return float(slope) / amp, r2


@dataclass
class AxisFit:
    """Per-axis result: directly-measured steady-state gain + nominal tau/L.

    ``K`` is the steady-state gain measured from each step's settled tail (no
    FOPDT fit, so no tau/L coupling). ``tau``/``L`` are NOMINAL -- 16 Hz pose
    cannot identify them, so ``tau_l_identified`` is always False and records the
    limit. ``r_squared`` is the median settled-line fit quality (how linear the
    settled region is), NOT a dynamics score.
    """

    axis: str
    K: float
    tau: float
    L: float
    k_by_amp: list[tuple[float, float]]  # (|amplitude|, measured K) -- the gain envelope
    k_spread: float  # std of K across amplitudes (trust indicator)
    r_squared: float  # median settled-line r^2
    tau_l_identified: bool
    valid: bool


@dataclass
class PoseDomainFit:
    """Whole-recording fit: per-axis results + assembled plant."""

    plant: TwistBasePlantParams
    axes: dict[str, AxisFit]
    per_amplitude: dict[str, list[dict[str, float]]]


def _fit_axis(
    recording: Recording,
    axis: str,
    spans: list[StepSpan],
    *,
    l_fixed: float | None = None,
) -> AxisFit:
    """Measure steady-state gain K per amplitude; tau/L are reported nominal.

    Only real-motion steps are used. K for each step = settled velocity / amp;
    repeats at one amplitude are medianed, and the axis K is the median across
    distinct amplitudes (robust to floor sag and high-amp saturation).
    """
    nominal_l = l_fixed if l_fixed is not None else _NOMINAL_L_S
    by_amp: dict[float, list[float]] = {}
    r2s: list[float] = []
    for span in _moving_spans(recording, spans):
        t_rel, p_meas = step_pose_channel(recording, span)
        gain, r2 = _steady_gain(t_rel, p_meas, span.amplitude)
        if np.isfinite(gain) and gain > 0.0:
            by_amp.setdefault(round(abs(span.amplitude), 3), []).append(gain)
            if np.isfinite(r2):
                r2s.append(r2)

    if not by_amp:
        return AxisFit(
            axis,
            float("nan"),
            _NOMINAL_TAU_S,
            nominal_l,
            [],
            float("nan"),
            float("nan"),
            False,
            False,
        )

    k_by_amp = sorted((amp, float(np.median(ks))) for amp, ks in by_amp.items())
    distinct_k = [k for _, k in k_by_amp]
    k_axis = float(np.median(distinct_k))
    return AxisFit(
        axis=axis,
        K=k_axis,
        tau=_NOMINAL_TAU_S,
        L=nominal_l,
        k_by_amp=k_by_amp,
        k_spread=float(np.std(distinct_k)),
        r_squared=float(np.median(r2s)) if r2s else float("nan"),
        tau_l_identified=False,
        valid=bool(0.0 < k_axis < 5.0),
    )


def fit_recording_pose_domain(
    recording: Recording,
    *,
    estimate_l: bool = True,
    l_by_axis: dict[str, float] | None = None,
    tau_bounds: tuple[float, float] = (0.03, 0.6),
    l_bounds: tuple[float, float] = (0.05, 0.30),
) -> PoseDomainFit:
    """Measure per-axis steady-state gain K (tau/L nominal) from the SI steps.

    ``estimate_l``/``tau_bounds``/``l_bounds`` are accepted for call-site
    compatibility but unused -- K is measured directly, tau/L are nominal.
    ``l_by_axis`` pins L per axis if supplied from a higher-rate source.
    """
    spans = segment_steps(recording)
    by_axis: dict[str, list[StepSpan]] = {a: [] for a in _AXES}
    for span in spans:
        by_axis[span.axis].append(span)

    axes: dict[str, AxisFit] = {}
    for axis in _AXES:
        l_fixed = None if l_by_axis is None else l_by_axis.get(axis)
        axes[axis] = _fit_axis(recording, axis, by_axis[axis], l_fixed=l_fixed)

    # vy often has no excitation on Go2 (no native strafe) -> fall back to vx.
    if np.isnan(axes["vy"].K) and not np.isnan(axes["vx"].K):
        axes["vy"] = AxisFit(
            axis="vy",
            K=axes["vx"].K,
            tau=axes["vx"].tau,
            L=axes["vx"].L,
            k_by_amp=[],
            k_spread=float("nan"),
            r_squared=float("nan"),
            tau_l_identified=False,
            valid=False,
        )

    plant = TwistBasePlantParams(
        vx=FopdtChannelParams(K=axes["vx"].K, tau=axes["vx"].tau, L=axes["vx"].L),
        vy=FopdtChannelParams(K=axes["vy"].K, tau=axes["vy"].tau, L=axes["vy"].L),
        wz=FopdtChannelParams(K=axes["wz"].K, tau=axes["wz"].tau, L=axes["wz"].L),
    )
    per_amplitude: dict[str, list[dict[str, float]]] = {}
    for axis in _AXES:
        rows = [{"amplitude": amp, "K": k} for amp, k in axes[axis].k_by_amp]
        if rows:
            per_amplitude[axis] = rows
    return PoseDomainFit(plant=plant, axes=axes, per_amplitude=per_amplitude)


_AXIS_UNIT = {"vx": "m", "vy": "m", "wz": "rad"}
_AXIS_CMD_UNIT = {"vx": "m/s", "vy": "m/s", "wz": "rad/s"}


def _write_pose_plots(
    recording: Recording, fit: PoseDomainFit, out_dir: Path, stem: str
) -> list[Path]:
    """Diagnostic PNGs: per-step measured pose + the measured steady-state gain
    line (``_steps.png``), and the gain envelope K vs amplitude (``_envelope.png``).
    Only real-motion steps; tau/L are not plotted (nominal, un-identifiable)."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    by_axis: dict[str, list[StepSpan]] = {
        a: _moving_spans(recording, [s for s in segment_steps(recording) if s.axis == a])
        for a in _AXES
    }
    written: list[Path] = []

    active = {a: spans for a, spans in by_axis.items() if spans}
    if active:
        ncols = max(len(spans) for spans in active.values())
        nrows = len(active)
        fig, axes = plt.subplots(nrows, ncols, figsize=(4 * ncols, 3 * nrows), squeeze=False)
        for row, (axis, spans) in enumerate(active.items()):
            for col in range(ncols):
                ax = axes[row][col]
                if col >= len(spans):
                    ax.axis("off")
                    continue
                span = spans[col]
                t_rel, p_meas = step_pose_channel(recording, span)
                if t_rel.size < 4:
                    ax.axis("off")
                    continue
                ax.plot(t_rel, p_meas, "k.", ms=4, label="measured")
                gain, _ = _steady_gain(t_rel, p_meas, span.amplitude)
                duration = float(t_rel[-1] - t_rel[0])
                settled = (t_rel - t_rel[0]) >= _SETTLE_FRAC * duration
                if int(np.count_nonzero(settled)) >= 3:
                    slope, intercept = np.polyfit(t_rel[settled], p_meas[settled], 1)
                    ax.plot(
                        t_rel,
                        slope * t_rel + intercept,
                        "-",
                        lw=2,
                        color="tab:green",
                        label="steady-state gain",
                    )
                ax.set_title(f"{axis} @ {span.amplitude:g}  (K={gain:.2f})", fontsize=9)
                if row == nrows - 1:
                    ax.set_xlabel("t since cmd (s)")
                if col == 0:
                    ax.set_ylabel(f"displacement ({_AXIS_UNIT[axis]})")
                if row == 0 and col == 0:
                    ax.legend(fontsize=7)
        fig.suptitle(f"{stem} — measured pose + steady-state gain (real-motion steps)")
        fig.tight_layout()
        steps_path = out_dir / f"{stem}_steps.png"
        fig.savefig(steps_path, dpi=110)
        plt.close(fig)
        written.append(steps_path)

    fig, axes = plt.subplots(1, len(_AXES), figsize=(4 * len(_AXES), 3.5), squeeze=False)
    for col, axis in enumerate(_AXES):
        axis_fit = fit.axes[axis]
        ax = axes[0][col]
        if axis_fit.k_by_amp:
            ax.scatter(
                [a for a, _ in axis_fit.k_by_amp],
                [k for _, k in axis_fit.k_by_amp],
                color="tab:blue",
                s=35,
            )
        if np.isfinite(axis_fit.K):
            ax.axhline(axis_fit.K, ls="--", color="gray", label=f"axis K={axis_fit.K:.2f}")
        ax.set_title(f"{axis}: gain K vs amplitude", fontsize=9)
        ax.set_xlabel(f"commanded amplitude ({_AXIS_CMD_UNIT[axis]})")
        ax.set_ylabel("K = v_ss / amp")
        ax.legend(fontsize=7)
    fig.suptitle(f"{stem} — gain envelope (tau/L nominal: not identifiable at 16 Hz)")
    fig.tight_layout()
    env_path = out_dir / f"{stem}_envelope.png"
    fig.savefig(env_path, dpi=110)
    plt.close(fig)
    written.append(env_path)
    return written


def reprocess(
    db_path: str | Path,
    *,
    robot_id: str = "go2",
    surface: str = "concrete",
    mode: str = "default",
    sim_or_hw: str = "hw",
    out_dir: str | Path | None = None,
    git_sha: str = "unknown",
    estimate_l: bool = True,
    l_by_axis: dict[str, float] | None = None,
    tau_bounds: tuple[float, float] = (0.03, 0.6),
    l_bounds: tuple[float, float] = (0.05, 0.30),
    plots: bool = True,
) -> Path:
    """Re-fit ``db_path`` and write a TuningConfig.

    K is the directly-measured steady-state gain per axis; tau/L are reported as
    bounded nominals (16 Hz pose cannot identify them). ``l_by_axis`` pins L per
    axis if you have it from a higher-rate source. Returns the artifact path and
    writes a ``*_quality.json`` sidecar (per-amplitude K, gain spread, and the
    tau/L = nominal flag). Warns if an axis has no usable (real-motion) steps.
    """
    db_path = Path(db_path)
    recording = load_recording(db_path)
    fit = fit_recording_pose_domain(recording, l_by_axis=l_by_axis)

    implausible = [a for a, f in fit.axes.items() if not f.valid]
    if implausible:
        warnings.warn(
            f"{db_path.name}: no usable steady-state gain on {implausible} "
            f"(no real-motion steps) -- artifact marked not-for-tuning",
            stacklevel=2,
        )

    provenance = Provenance(
        robot_id=robot_id,
        surface=surface,
        mode=mode,
        date=date.today().isoformat(),
        git_sha=git_sha,
        sim_or_hw=sim_or_hw if not implausible else "self-test",
        characterization_session_dir=str(db_path),
    )
    config = derive_config(fit.plant, provenance, per_amplitude=fit.per_amplitude or None)

    out_dir = Path(out_dir) if out_dir else db_path.parent
    out_dir.mkdir(parents=True, exist_ok=True)
    stem = f"{robot_id}_config_{mode}_{surface}_{date.today().isoformat()}_{git_sha}_posedomain"
    artifact_path = out_dir / f"{stem}.json"
    config.to_json(artifact_path)

    quality = {
        axis: {
            "K": f.K,
            "K_by_amplitude": [{"amplitude": a, "K": k} for a, k in f.k_by_amp],
            "K_spread": f.k_spread,
            "tau": f.tau,
            "L": f.L,
            "tau_L_identified": f.tau_l_identified,
            "settled_line_r2": f.r_squared,
            "valid": f.valid,
            "note": "K measured directly (steady-state); tau/L are nominal "
            "(not identifiable from 16 Hz pose).",
        }
        for axis, f in fit.axes.items()
    }
    (out_dir / f"{stem}_quality.json").write_text(json.dumps(quality, indent=2))

    if plots:
        _write_pose_plots(recording, fit, out_dir, stem)
    return artifact_path


def main() -> None:
    """CLI: pose-domain re-fit of a recording -> TuningConfig artifact.

    Example::

        python -m dimos.utils.characterization.reprocess \\
            data/characterization/go2/go2_recording_default_2026-06-19_<sha>.db \\
            --robot-id go2_u01 --surface concrete --sim-or-hw hw
    """
    parser = argparse.ArgumentParser(
        description="Re-fit a characterization recording with the pose-domain "
        "(output-error) FOPDT method and write a TuningConfig artifact."
    )
    parser.add_argument("db", help="path to the recording .db to re-fit")
    parser.add_argument("--robot-id", default="go2", help="per-unit id, e.g. go2_u01")
    parser.add_argument("--surface", default="concrete")
    parser.add_argument("--mode", default="default", help="gait mode")
    parser.add_argument(
        "--sim-or-hw",
        default="hw",
        choices=["hw", "sim", "self-test"],
        help="hw -> artifact is valid_for_tuning; sim/self-test -> not",
    )
    parser.add_argument("--out", default=None, help="output dir (default: alongside the .db)")
    parser.add_argument("--git-sha", default=git_sha(), help="provenance git sha")
    parser.add_argument(
        "--no-estimate-l",
        action="store_true",
        help="skip deadtime profiling; use --l-vx/--l-vy/--l-wz (or nominal) instead",
    )
    parser.add_argument("--no-plots", action="store_true", help="skip the _steps/_envelope PNGs")
    parser.add_argument("--l-vx", type=float, default=None, help="fixed deadtime L for vx (s)")
    parser.add_argument("--l-vy", type=float, default=None, help="fixed deadtime L for vy (s)")
    parser.add_argument("--l-wz", type=float, default=None, help="fixed deadtime L for wz (s)")
    parser.add_argument(
        "--l-min", type=float, default=0.05, help="plausibility lower bound on L (s)"
    )
    parser.add_argument(
        "--l-max", type=float, default=0.30, help="plausibility upper bound on L (s)"
    )
    parser.add_argument(
        "--tau-min", type=float, default=0.03, help="plausibility lower bound on tau (s)"
    )
    parser.add_argument(
        "--tau-max", type=float, default=0.6, help="plausibility upper bound on tau (s)"
    )
    args = parser.parse_args()

    l_by_axis: dict[str, float] | None = None
    fixed = {
        a: v for a, v in (("vx", args.l_vx), ("vy", args.l_vy), ("wz", args.l_wz)) if v is not None
    }
    if fixed:
        l_by_axis = fixed

    artifact = reprocess(
        args.db,
        robot_id=args.robot_id,
        surface=args.surface,
        mode=args.mode,
        sim_or_hw=args.sim_or_hw,
        out_dir=args.out,
        git_sha=args.git_sha,
        estimate_l=not args.no_estimate_l,
        l_by_axis=l_by_axis,
        tau_bounds=(args.tau_min, args.tau_max),
        l_bounds=(args.l_min, args.l_max),
        plots=not args.no_plots,
    )
    quality = json.loads((artifact.parent / f"{artifact.stem}_quality.json").read_text())
    print(f"\nartifact: {artifact}")
    print(f"quality:  {artifact.parent / (artifact.stem + '_quality.json')}\n")
    print("K = measured steady-state gain;  tau/L = NOMINAL (not identifiable at 16 Hz)\n")
    print(f"{'axis':5s} {'K':>8s} {'K_spread':>9s} {'tau*':>6s} {'L*':>6s}  valid")
    for axis in _AXES:
        q = quality[axis]
        print(
            f"{axis:5s} {q['K']:8.3f} {q['K_spread']:9.3f} {q['tau']:6.2f} {q['L']:6.2f}  {q['valid']}"
        )
    print("\n* tau/L are nominal placeholders, not measured. Trust K.")


__all__ = [
    "AxisFit",
    "PoseDomainFit",
    "fit_recording_pose_domain",
    "reprocess",
]


if __name__ == "__main__":
    main()
