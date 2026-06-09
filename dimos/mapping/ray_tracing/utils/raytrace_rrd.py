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

"""Render a ray-traced voxel map from a recorded lidar stream into rerun.

Lidar and odometry are aligned by timestamp so each frame carries the robot
pose used as the ray-cast origin. The robot pose axis and trajectory are
logged alongside the map.

Usage:
    uv run python -m dimos.mapping.ray_tracing.utils.raytrace_rrd go2_mid360_stairs
    uv run python -m dimos.mapping.ray_tracing.utils.raytrace_rrd go2_mid360_stairs --out map.rrd && rerun map.rrd
"""

from __future__ import annotations

from pathlib import Path

import rerun as rr
import typer

from dimos.mapping.ray_tracing.transformer import RayTraceMap
from dimos.memory2.store.sqlite import SqliteStore
from dimos.memory2.transform import FnTransformer
from dimos.memory2.type.observation import Observation
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2, register_colormap_annotation
from dimos.utils.data import resolve_named_path

TIMELINE = "ts"

PairObs = Observation[tuple[Observation[PointCloud2], Observation[Odometry]]]


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
    lidar_stream: str = typer.Option(
        "fastlio_lidar", "--lidar-stream", help="Lidar stream name in the recording"
    ),
    odom_stream: str = typer.Option(
        "fastlio_odometry", "--odom-stream", help="Odometry stream name in the recording"
    ),
    align_tol: float = typer.Option(0.05, "--align-tol", help="Lidar/odom alignment tolerance (s)"),
    voxel_size: float = typer.Option(0.1, "--voxel-size", help="Raycaster voxel edge length (m)"),
    max_range: float = typer.Option(
        30.0, "--max-range", help="Max ray cast distance (m); 0 = no limit"
    ),
    ray_subsample: int = typer.Option(1, "--ray-subsample", help="Keep every Nth ray for clearing"),
    shadow_depth: float = typer.Option(
        0.2, "--shadow-depth", help="Ray extension past endpoint (m)"
    ),
    grace_depth: float = typer.Option(
        0.2, "--grace-depth", help="Spare voxels within this dist of endpoint (m)"
    ),
    min_health: int = typer.Option(-2, "--min-health", help="Voxel removal threshold"),
    max_health: int = typer.Option(1, "--max-health", help="Voxel saturation cap"),
    emit_every: int = typer.Option(1, "--emit-every", help="Yield the current map every N frames"),
    render_voxel: float = typer.Option(
        0.05, "--render-voxel", help="Voxel size for rerun rendering (m)"
    ),
) -> None:
    db_path = resolve_named_path(dataset, ".db")

    rr.init("raytrace_rrd", recording_id=db_path.stem)
    if out is not None:
        rr.save(str(out))
    else:
        rr.spawn()
    register_colormap_annotation("turbo")

    rr.log(
        "world/robot/axes",
        rr.Arrows3D(
            vectors=[[0.3, 0, 0], [0, 0.3, 0], [0, 0, 0.3]],
            colors=[[255, 0, 0], [0, 255, 0], [0, 0, 255]],
        ),
        static=True,
    )

    store = SqliteStore(path=str(db_path))
    with store:
        lidar = store.stream(lidar_stream, PointCloud2).order_by("ts")
        odom = store.stream(odom_stream, Odometry).order_by("ts")

        pose_tagged = lidar.align(odom, tolerance=align_tol).transform(
            FnTransformer(_attach_pose_from_odom)
        )
        pipeline = pose_tagged.transform(
            RayTraceMap(
                voxel_size=voxel_size,
                max_range=max_range,
                ray_subsample=ray_subsample,
                shadow_depth=shadow_depth,
                grace_depth=grace_depth,
                min_health=min_health,
                max_health=max_health,
                emit_every=emit_every,
            )
        )

        trajectory: list[tuple[float, float, float]] = []
        for obs in pipeline:
            rr.set_time(TIMELINE, timestamp=obs.ts)
            rr.log("world/raytrace_map", obs.data.to_rerun(voxel_size=render_voxel))

            if obs.pose_tuple is not None:
                x, y, z, qx, qy, qz, qw = obs.pose_tuple
                rr.log(
                    "world/robot",
                    rr.Transform3D(
                        translation=[x, y, z], quaternion=rr.Quaternion(xyzw=[qx, qy, qz, qw])
                    ),
                )
                trajectory.append((x, y, z))
                if len(trajectory) >= 2:
                    rr.log(
                        "world/robot_path",
                        rr.LineStrips3D([trajectory], colors=[[255, 165, 0]]),
                    )

            print(f"frame_count={obs.tags['frame_count']}", end="\r", flush=True)
        print()

    if out is not None:
        print(f"wrote {out}")
        print(f"open with: rerun {out}")


if __name__ == "__main__":
    typer.run(main)
