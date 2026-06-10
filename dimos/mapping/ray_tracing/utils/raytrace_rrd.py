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

"""Replay a lidar+odometry .db through several voxel-mapper variants at once.

Each variant's global map is logged to its own rerun entity (world/maps/<name>),
so they overlay in one view and can be toggled on and off independently. Lidar
and odometry are aligned by timestamp so each frame carries the robot pose used
as the ray-cast origin.

Usage:
    uv run python -m dimos.mapping.ray_tracing.utils.raytrace_rrd go2_mid360_stairs
"""

from __future__ import annotations

from pathlib import Path

import rerun as rr
import typer

from dimos.mapping.ray_tracing.voxel_map import VoxelRayMapper
from dimos.memory2.store.sqlite import SqliteStore
from dimos.memory2.transform import FnTransformer
from dimos.memory2.type.observation import Observation
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.data import resolve_named_path

TIMELINE = "ts"

PairObs = Observation[tuple[Observation[PointCloud2], Observation[Odometry]]]

COLORS = {
    "naive": [90, 200, 90],
    "no_normal_gate": [235, 120, 60],
    "normal_gate": [70, 170, 235],
    "no_recency": [200, 80, 200],
}

# A window large enough to never expire, i.e. recency gating off.
RECENCY_OFF = 1_000_000_000


def _attach_pose_from_odom(pair_obs: PairObs) -> Observation[PointCloud2]:
    lidar_obs, odom_obs = pair_obs.data
    odom = odom_obs.data
    pose_tuple = (
        float(odom.position.x),
        float(odom.position.y),
        float(odom.position.z),
        float(odom.orientation.x),
        float(odom.orientation.y),
        float(odom.orientation.z),
        float(odom.orientation.w),
    )
    return lidar_obs.with_pose(pose_tuple)


def main(
    dataset: str = typer.Argument(..., help="Dataset .db: bare name (cwd or data/) or path"),
    out: Path | None = typer.Option(
        None, "--out", help="Output .rrd path. If omitted, spawn rerun live."
    ),
    lidar_stream: str = typer.Option("fastlio_lidar", "--lidar-stream"),
    odom_stream: str = typer.Option("fastlio_odometry", "--odom-stream"),
    align_tol: float = typer.Option(0.05, "--align-tol", help="Lidar/odom alignment tolerance (s)"),
    voxel_size: float = typer.Option(0.1, "--voxel-size", help="Voxel edge length (m)"),
    max_range: float = typer.Option(30.0, "--max-range", help="Max ray cast distance (m)"),
    emit_every: int = typer.Option(1, "--emit-every", help="Log the maps every N frames"),
    render_voxel: float = typer.Option(0.05, "--render-voxel", help="Voxel render size (m)"),
    raw: bool = typer.Option(True, "--raw/--no-raw", help="Also log the raw sensor cloud"),
) -> None:
    db_path = resolve_named_path(dataset, ".db")

    rr.init("raytrace_rrd", recording_id=db_path.stem)
    if out is not None:
        rr.save(str(out))
    else:
        rr.spawn()

    rr.log(
        "world/robot/axes",
        rr.Arrows3D(
            vectors=[[0.3, 0, 0], [0, 0.3, 0], [0, 0, 0.3]],
            colors=[[255, 0, 0], [0, 255, 0], [0, 0, 255]],
        ),
        static=True,
    )

    # naive accumulates every voxel and never clears. The rest are defaults except:
    # no_normal_gate turns the normal gate off; no_recency keeps the gate but never
    # lets a spare expire (recency off); normal_gate is the full default behavior.
    mappers = {
        "naive": VoxelRayMapper(
            voxel_size=voxel_size,
            max_range=max_range,
            shadow_depth=0.0,
            grace_depth=max_range,
            min_health=0,
        ),
        "no_normal_gate": VoxelRayMapper(voxel_size=voxel_size, max_range=max_range, graze_cos=0.0),
        "no_recency": VoxelRayMapper(
            voxel_size=voxel_size, max_range=max_range, recency_window=RECENCY_OFF
        ),
        "normal_gate": VoxelRayMapper(voxel_size=voxel_size, max_range=max_range),
    }

    store = SqliteStore(path=str(db_path))
    with store:
        lidar = store.stream(lidar_stream, PointCloud2).order_by("ts")
        odom = store.stream(odom_stream, Odometry).order_by("ts")
        pose_tagged = lidar.align(odom, tolerance=align_tol).transform(
            FnTransformer(_attach_pose_from_odom)
        )

        trajectory: list[tuple[float, float, float]] = []
        count = 0
        for obs in pose_tagged:
            if obs.pose_tuple is None:
                continue
            x, y, z, qx, qy, qz, qw = obs.pose_tuple
            pts = obs.data.points_f32()
            for mapper in mappers.values():
                mapper.add_frame(pts, (x, y, z))
            count += 1

            if count % emit_every != 0:
                continue

            rr.set_time(TIMELINE, timestamp=obs.ts)
            for name, mapper in mappers.items():
                rr.log(
                    f"world/maps/{name}",
                    rr.Points3D(mapper.global_map(), colors=[COLORS[name]], radii=render_voxel / 2),
                )
            if raw:
                rr.log("world/raw_points", rr.Points3D(pts, colors=[[90, 90, 90]], radii=0.01))
            rr.log(
                "world/robot",
                rr.Transform3D(
                    translation=[x, y, z], quaternion=rr.Quaternion(xyzw=[qx, qy, qz, qw])
                ),
            )
            trajectory.append((x, y, z))
            if len(trajectory) >= 2:
                rr.log("world/robot_path", rr.LineStrips3D([trajectory], colors=[[255, 165, 0]]))
            print(f"frame={count}", end="\r", flush=True)
        print()

    if out is not None:
        print(f"wrote {out}\nopen with: rerun {out}")


if __name__ == "__main__":
    typer.run(main)
