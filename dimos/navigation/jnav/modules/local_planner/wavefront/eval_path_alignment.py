# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

"""Score local-path quality against the goal and the global path on a recording.

For every ``/local_path`` message in a time window this computes, in the world
frame (the recorded path is base-frame; the odometry pose at message time lifts
it):

  * goal pointing   — angle between the local path's net direction (start ->
    end) and the direction to the global path's final point. A plan that runs
    away from the goal scores ~180 deg.
  * global alignment — mean distance from the local path's poses to the nearest
    point on the current global path (how tightly the local plan tracks the
    route corridor).
  * heading churn    — tick-to-tick change of the local path's net direction;
    a smooth planner turns gradually, a flip-flopping one swings by >90 deg.

Usage:

    python eval_path_alignment.py --db recordings/last_run.db --start 140 --end 175
"""

from __future__ import annotations

import math
from pathlib import Path as FsPath

import numpy as np
import typer

from dimos.memory2.cli.dataset import open_store
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.nav_msgs.Path import Path


def _yaw(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def _wrap_deg(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def main(
    db: FsPath = typer.Option(..., help="memory2 recording db"),
    start: float = typer.Option(0.0, help="window start, seconds from recording t0"),
    end: float = typer.Option(1e9, help="window end, seconds from recording t0"),
) -> None:
    store = open_store(db)

    odoms = [(o.ts, o.data) for o in store.stream("odometry", Odometry)]
    t0 = odoms[0][0]
    lo, hi = t0 + start, t0 + end

    globals_ = [(o.ts, o.data) for o in store.stream("global_path", Path) if o.ts <= hi]
    locals_ = [(o.ts, o.data) for o in store.stream("local_path", Path) if lo <= o.ts <= hi]

    def latest_before(seq, ts):
        best = None
        for t, d in seq:
            if t > ts:
                break
            best = d
        return best

    point_errs: list[float] = []
    align_means: list[float] = []
    churns: list[float] = []
    prev_dir: float | None = None
    for ts, lp in locals_:
        od = latest_before(odoms, ts)
        gp = latest_before(globals_, ts)
        if od is None or gp is None or len(lp.poses) < 2 or len(gp.poses) < 2:
            continue
        # Lift the base-frame local path into the world frame.
        yaw = _yaw(od.pose.orientation)
        c, s = math.cos(yaw), math.sin(yaw)
        px, py = od.pose.position.x, od.pose.position.y
        world = [
            (px + c * p.position.x - s * p.position.y, py + s * p.position.x + c * p.position.y)
            for p in lp.poses
        ]
        gpts = np.array([(p.position.x, p.position.y) for p in gp.poses])

        # Goal pointing: net local direction vs direction to the route's end.
        (x0, y0), (x1, y1) = world[0], world[-1]
        net = math.atan2(y1 - y0, x1 - x0)
        goal = math.atan2(gpts[-1, 1] - y0, gpts[-1, 0] - x0)
        point_errs.append(abs(_wrap_deg(math.degrees(net - goal))))

        # Alignment: mean nearest-vertex distance of local poses to the route.
        w = np.array(world)
        d = np.sqrt(((w[:, None, :] - gpts[None, :, :]) ** 2).sum(-1)).min(axis=1)
        align_means.append(float(d.mean()))

        # Churn: tick-to-tick swing of the net direction.
        if prev_dir is not None:
            churns.append(abs(_wrap_deg(math.degrees(net - prev_dir))))
        prev_dir = net

    n = len(point_errs)
    if n == 0:
        typer.echo("no scorable local paths in the window")
        raise typer.Exit(1)
    pe, al, ch = np.array(point_errs), np.array(align_means), np.array(churns)
    typer.echo(f"scored {n} local paths over [{start:.0f}s, {min(end, odoms[-1][0]-t0):.0f}s]")
    typer.echo(
        f"goal pointing : mean {pe.mean():6.1f} deg  p90 {np.percentile(pe, 90):6.1f}  "
        f"within 45 deg {100 * (pe <= 45).mean():5.1f}%  backward(>90) {100 * (pe > 90).mean():5.1f}%"
    )
    typer.echo(
        f"route align   : mean {al.mean():6.2f} m    p90 {np.percentile(al, 90):6.2f}"
    )
    if len(ch):
        typer.echo(
            f"heading churn : mean {ch.mean():6.1f} deg  p90 {np.percentile(ch, 90):6.1f}  "
            f"swings(>90)  {100 * (ch > 90).mean():5.1f}%"
        )


if __name__ == "__main__":
    typer.run(main)
