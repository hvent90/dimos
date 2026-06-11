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

"""Standard reachability plots: green = reachable, red = not, gradient between.

Because the map quotients pelvis heading, position reachability depends
only on (radius from the pelvis axis, height) — so the native plot is an
r-z heatmap, and the familiar x-z / x-y workspace slices are painted from
the radial profile (they are rotationally symmetric by construction; the
robot can always turn in place).

CLI::

    python -m dimos.manipulation.reachability.plots \\
        --map data/reachability/g1_left_capability.npz --out-dir plots/
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from dimos.manipulation.reachability.capability_map import CapabilityMap
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Approach-direction bands worth a dedicated panel (radians).
_THETA_BANDS = [
    ("top-down (θ<30°)", 0.0, np.deg2rad(30)),
    ("horizontal (75°<θ<105°)", np.deg2rad(75), np.deg2rad(105)),
    ("bottom-up (θ>150°)", np.deg2rad(150), np.pi),
]


def _score_to_rgba(scores: np.ndarray, vmax: float | None = None):
    """Score grid → RdYlGn image with score-0 cells in red."""
    import matplotlib
    import matplotlib.colors as mcolors

    vmax = vmax or max(float(scores.max()), 1.0)
    norm = mcolors.Normalize(vmin=0.0, vmax=vmax)
    return matplotlib.colormaps["RdYlGn"](norm(scores.astype(float)))


def plot_radial(cap: CapabilityMap, out_dir: Path, vmax: float | None = None) -> list[Path]:
    """r-z heatmaps: overall max-over-orientation plus per-θ-band panels."""
    import matplotlib.pyplot as plt

    params = cap.params
    # Radial column i spans radius [i, i+1)·cell — the grid covers radii up
    # to n_xy·cell (the canonical square's diagonal), not r_xy.
    extent = (0.0, params.n_xy * params.cell, params.z_min, params.z_max)
    panels = [("all approach directions", cap.position_scores())]
    panels += [(label, cap.theta_band_position_scores(lo, hi)) for label, lo, hi in _THETA_BANDS]

    fig, axes = plt.subplots(1, len(panels), figsize=(4.2 * len(panels), 4.6), sharey=True)
    vmax = vmax or max(float(panels[0][1].max()), 1.0)
    for ax, (label, scores) in zip(np.atleast_1d(axes), panels, strict=True):
        ax.imshow(
            _score_to_rgba(scores, vmax),
            origin="lower",
            extent=extent,
            aspect="equal",
            interpolation="nearest",
        )
        ax.set_xlim(0.0, params.r_xy)
        ax.axhline(params.pelvis_height, color="k", linestyle=":", linewidth=0.8)
        ax.set_title(label, fontsize=10)
        ax.set_xlabel("radius from pelvis axis [m]")
    np.atleast_1d(axes)[0].set_ylabel("height above ground [m]")
    fig.suptitle(
        f"G1 {cap.side} arm reachability — score gradient (red = unreachable, "
        f"green = high sample density); dotted line = pelvis height",
        fontsize=11,
    )
    fig.tight_layout()
    path = out_dir / f"g1_{cap.side}_radial.png"
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return [path]


def plot_workspace_slices(
    cap: CapabilityMap, out_dir: Path, slice_heights: tuple[float, ...] = (0.5, 0.9, 1.3)
) -> list[Path]:
    """Classic x-z and x-y workspace slices, painted from the radial profile."""
    import matplotlib.pyplot as plt

    params = cap.params
    radial = cap.position_scores()  # (n_z, n_r)
    vmax = max(float(radial.max()), 1.0)
    half = params.r_xy
    axis = np.linspace(-half, half, 2 * params.n_xy)
    paths: list[Path] = []

    # x-z slice through the pelvis (y = 0).
    r_idx = np.minimum((np.abs(axis) / params.cell).astype(int), params.n_xy - 1)
    xz = radial[:, r_idx]
    fig, ax = plt.subplots(figsize=(7, 5))
    ax.imshow(
        _score_to_rgba(xz, vmax),
        origin="lower",
        extent=(-half, half, params.z_min, params.z_max),
        aspect="equal",
        interpolation="nearest",
    )
    ax.plot(0.0, params.pelvis_height, "k^", markersize=8, label="pelvis")
    ax.set_xlabel("x (forward) [m]")
    ax.set_ylabel("z (height) [m]")
    ax.set_title(f"G1 {cap.side} arm — x-z slice (heading-free, rotationally symmetric)")
    ax.legend(loc="upper right", fontsize=8)
    fig.tight_layout()
    path = out_dir / f"g1_{cap.side}_xz.png"
    fig.savefig(path, dpi=160)
    plt.close(fig)
    paths.append(path)

    # x-y slices at chosen heights.
    fig, axes = plt.subplots(1, len(slice_heights), figsize=(4.4 * len(slice_heights), 4.6))
    xx, yy = np.meshgrid(axis, axis)
    rr_idx = np.minimum((np.hypot(xx, yy) / params.cell).astype(int), params.n_xy - 1)
    for ax, z in zip(np.atleast_1d(axes), slice_heights, strict=True):
        iz = int(np.clip((z - params.z_min) / params.cell, 0, params.n_z - 1))
        ax.imshow(
            _score_to_rgba(radial[iz][rr_idx], vmax),
            origin="lower",
            extent=(-half, half, -half, half),
            aspect="equal",
            interpolation="nearest",
        )
        ax.plot(0.0, 0.0, "k^", markersize=8)
        ax.set_title(f"z = {z:.2f} m", fontsize=10)
        ax.set_xlabel("x [m]")
    np.atleast_1d(axes)[0].set_ylabel("y [m]")
    fig.suptitle(f"G1 {cap.side} arm — x-y slices (pelvis at origin)", fontsize=11)
    fig.tight_layout()
    path = out_dir / f"g1_{cap.side}_xy.png"
    fig.savefig(path, dpi=160)
    plt.close(fig)
    paths.append(path)
    return paths


def plot_theta_profile(cap: CapabilityMap, out_dir: Path) -> list[Path]:
    """Marked-cell fraction per approach angle — where the arm has options."""
    import matplotlib.pyplot as plt

    marked = (cap.counts > 0).any(axis=4).sum(axis=(0, 2, 3))
    theta_deg = (np.arange(cap.params.n_theta) + 0.5) * 180.0 / cap.params.n_theta
    fig, ax = plt.subplots(figsize=(7, 3.6))
    ax.bar(theta_deg, marked, width=180.0 / cap.params.n_theta * 0.9, color="seagreen")
    ax.set_xlabel("approach angle θ vs gravity [deg] (0 = pointing down)")
    ax.set_ylabel("reachable cells")
    ax.set_title(f"G1 {cap.side} arm — reachable cells per approach angle")
    fig.tight_layout()
    path = out_dir / f"g1_{cap.side}_theta_profile.png"
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return [path]


def render_all(cap: CapabilityMap, out_dir: str | Path) -> list[Path]:
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    paths = []
    paths += plot_radial(cap, out)
    paths += plot_workspace_slices(cap, out)
    paths += plot_theta_profile(cap, out)
    for p in paths:
        logger.info(f"wrote {p}")
    return paths


def cli_main() -> None:
    parser = argparse.ArgumentParser(description="Render capability-map reachability plots.")
    parser.add_argument("--map", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, default=Path("plots"))
    args = parser.parse_args()
    cap = CapabilityMap.load(args.map)
    for path in render_all(cap, args.out_dir):
        print(path)


if __name__ == "__main__":
    cli_main()


__all__ = ["plot_radial", "plot_theta_profile", "plot_workspace_slices", "render_all"]
