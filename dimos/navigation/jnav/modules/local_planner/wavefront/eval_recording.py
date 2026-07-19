# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

"""Score the planner's costmap against a hand-labeled map on a real recording.

Replays a mem2 recording (sensor-frame lidar tf-registered into odom) through the real
native code paths — the andrew_ray_tracing voxel clearing (pyo3
``VoxelRayMapper``) and the wavefront costmap (pyo3 ``build_costmap``) —
then scores the produced costmap against a hand-made 2D ground-truth grid
(floor / obstacle / stairs), cell for cell:

  * obstacle recall  — GT obstacle cells the costmap marks lethal (misses are
    the cherry-picker failure: real structure erased or never marked)
  * floor accuracy   — GT floor cells the costmap does NOT mark lethal
  * stairs open      — GT stairs cells not lethal (the robot must still climb)

Usage (defaults match the 2026-07-09 warehouse recording + its ground truth):

    python eval_recording.py --at 137.6
    python eval_recording.py --no-clearing            # costmap alone
    python eval_recording.py --max-health 8           # tuned clearing
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

LETHAL_THRESHOLD = 50


def iter_registered_scans(db_path: Path, world: str = "odom"):
    """Yield (t_rel, world_points, sensor_origin) per scan, tf-registered.

    The recording's clouds are sensor-frame (frame_id mid360_link); the tf
    stream's ``world <- frame_id`` transform at scan time lifts them into the
    world frame — the same registration ``dimos map global`` uses. The tf
    translation doubles as the sensor origin for raycast clearing.
    """
    from dimos.memory2.cli.dataset import open_store
    from dimos.memory2.tf import StreamTF
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

    store = open_store(db_path)
    tf_buf = StreamTF.from_store(store)
    ts0 = None
    for obs in store.stream("pointlio_lidar", PointCloud2):
        if ts0 is None:
            ts0 = obs.ts
        tf = tf_buf.get(world, obs.data.frame_id, time_point=obs.ts)
        if tf is None:
            continue
        pts = obs.data.transform(tf).points().numpy().astype(np.float32)
        if not len(pts):
            continue
        tr = tf.translation
        yield obs.ts - ts0, pts, (float(tr.x), float(tr.y), float(tr.z))


def replay_clearing(
    db_path: Path,
    t_end: float,
    t_start: float,
    *,
    voxel_size: float,
    max_range: float,
    ray_subsample: int,
    shadow_depth: float,
    grace_depth: float,
    min_health: int,
    max_health: int,
) -> np.ndarray:
    """Feed registered scans [t_start, t_end] through VoxelRayMapper."""
    from dimos_voxel_ray_tracing import VoxelRayMapper

    mapper = VoxelRayMapper(
        voxel_size=voxel_size,
        max_range=max_range,
        ray_subsample=ray_subsample,
        shadow_depth=shadow_depth,
        grace_depth=grace_depth,
        min_health=min_health,
        max_health=max_health,
    )
    for t, pts, origin in iter_registered_scans(db_path):
        if t < t_start or t > t_end:
            continue
        mapper.add_frame(np.ascontiguousarray(pts), origin)
    return mapper.global_map()


def accumulate_raw(db_path: Path, t_end: float, t_start: float) -> np.ndarray:
    """No-clearing baseline: registered scan points in the window, concatenated."""
    out = [
        pts for t, pts, _ in iter_registered_scans(db_path) if t_start <= t <= t_end
    ]
    return np.concatenate(out) if out else np.zeros((0, 3), np.float32)


def robot_pose_at(db_path: Path, t: float) -> tuple[float, float, float]:
    from dimos.navigation.jnav.utils.recording_db import iterate_stream

    ts0, best, bd = None, None, 1e18
    for ts, msg in iterate_stream(db_path, "pointlio_odometry"):
        if ts0 is None:
            ts0 = ts
        d = abs((ts - ts0) - t)
        if d < bd:
            p = msg.pose.pose.position
            bd, best = d, (float(p.x), float(p.y), float(p.z))
    return best


def score(
    cost: np.ndarray,
    origin: tuple[float, float],
    res: float,
    gt: dict,
) -> dict:
    """Cell-for-cell comparison on the GT grid, within the costmap extent."""
    label = gt["label"]
    gx0, gy0, gres = float(gt["x0"]), float(gt["y0"]), float(gt["res"])
    H, W = label.shape
    yy, xx = np.mgrid[0:H, 0:W]
    wx = gx0 + xx * gres
    wy = gy0 + yy * gres
    col = ((wx - origin[0]) / res + 0.5).astype(int)
    row = ((wy - origin[1]) / res + 0.5).astype(int)
    inside = (col >= 0) & (col < cost.shape[1]) & (row >= 0) & (row < cost.shape[0])
    if "observed" in gt:
        # Score only lidar-observed GT cells: the dense map's nearest-class fill
        # paints unobserved periphery as "floor", while the costmap deliberately
        # marks unobserved interior lethal — a definitional mismatch, not error.
        inside &= gt["observed"].astype(bool)
    c = np.full(label.shape, -128, np.int16)
    c[inside] = cost[row[inside], col[inside]]
    lethal = c >= LETHAL_THRESHOLD
    known = inside & (c != -1)  # -1 = costmap unknown

    out = {}
    for cls, name in ((2, "obstacle"), (1, "floor"), (3, "stairs")):
        sel = (label == cls) & inside
        n = int(sel.sum())
        n_lethal = int((sel & lethal).sum())
        n_unknown = int((sel & ~known).sum())
        out[name] = {
            "cells": n,
            "lethal": n_lethal,
            "unknown": n_unknown,
            "lethal_pct": round(100 * n_lethal / max(n, 1), 1),
        }
    out["obstacle_recall_pct"] = out["obstacle"]["lethal_pct"]
    out["floor_false_lethal_pct"] = out["floor"]["lethal_pct"]
    out["stairs_blocked_pct"] = out["stairs"]["lethal_pct"]
    return out


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--db", default="~/datasets/local_planner_test.db")
    ap.add_argument("--gt", default="/tmp/lp_groundtruth.npz")
    ap.add_argument("--at", type=float, default=137.6, help="eval time (s from start)")
    ap.add_argument("--window", type=float, default=45.0, help="scan window before --at")
    ap.add_argument("--no-clearing", action="store_true", help="raw accumulation baseline")
    # clearing knobs (defaults = shipped module config)
    ap.add_argument("--voxel-size", type=float, default=0.1)
    ap.add_argument("--max-range", type=float, default=30.0)
    ap.add_argument("--ray-subsample", type=int, default=1)
    ap.add_argument("--shadow-depth", type=float, default=0.2)
    ap.add_argument("--grace-depth", type=float, default=0.2)
    ap.add_argument("--min-health", type=int, default=-1)
    ap.add_argument("--max-health", type=int, default=6)
    # costmap knobs (defaults = shipped module config)
    ap.add_argument("--resolution", type=float, default=0.1)
    ap.add_argument("--can-pass-under", type=float, default=0.6)
    ap.add_argument("--max-grade", type=float, default=3.0, help="traversable grade (rise/run)")
    ap.add_argument("--half-extent", type=float, default=8.0)
    ap.add_argument("--body-step", type=float, default=0.35)
    ap.add_argument("--body-min-points", type=int, default=0)
    ap.add_argument("--max-step", type=float, default=0.25, help="plateau-step gate (0 disables)")
    args = ap.parse_args()

    from dimos_wavefront import build_costmap

    db = Path(args.db).expanduser()
    gt = np.load(args.gt)
    t_start = args.at - args.window

    if args.no_clearing:
        pts = accumulate_raw(db, args.at, t_start)
        mode = "raw (no clearing)"
    else:
        pts = replay_clearing(
            db,
            args.at,
            t_start,
            voxel_size=args.voxel_size,
            max_range=args.max_range,
            ray_subsample=args.ray_subsample,
            shadow_depth=args.shadow_depth,
            grace_depth=args.grace_depth,
            min_health=args.min_health,
            max_health=args.max_health,
        )
        mode = f"cleared (max_health={args.max_health}, min_health={args.min_health})"

    robot = robot_pose_at(db, args.at)
    cost, origin, res = build_costmap(
        np.ascontiguousarray(pts.astype(np.float32)),
        robot,
        robot[2],
        resolution=args.resolution,
        can_pass_under=args.can_pass_under,
        max_grade=args.max_grade,
        half_extent=args.half_extent,
        body_step=args.body_step,
        body_min_points=args.body_min_points,
        max_step=args.max_step,
    )
    result = score(np.asarray(cost), origin, res, gt)

    print(f"mode: {mode}   points into costmap: {len(pts)}")
    print(f"robot@{args.at}s: ({robot[0]:.2f}, {robot[1]:.2f}, {robot[2]:.2f})")
    for name in ("obstacle", "floor", "stairs"):
        r = result[name]
        print(
            f"  {name:9s} cells={r['cells']:6d}  lethal={r['lethal_pct']:5.1f}%"
            f"  unknown={r['unknown']}"
        )
    print(
        f"  => obstacle recall {result['obstacle_recall_pct']}% | "
        f"floor false-lethal {result['floor_false_lethal_pct']}% | "
        f"stairs blocked {result['stairs_blocked_pct']}%"
    )
    np.savez(
        "/tmp/lp_eval_last.npz",
        cost=np.asarray(cost),
        origin=np.array(origin),
        res=res,
        points=pts[:: max(1, len(pts) // 800_000)],
    )


if __name__ == "__main__":
    main()
