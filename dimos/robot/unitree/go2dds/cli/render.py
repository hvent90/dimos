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

# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License").

"""Render Go2 odom sources to rerun — memory2 store pipelines (standalone).

Each *pipeline* is a function ``(store, seconds) -> None`` composed from
reusable stream transforms over standard dimos messages. ``leg_odom`` logs both
the per-frame pose (Transform3D) and the accumulated trajectory (nav_msgs/Path)::

    sportmodestate.map_data(pose).tap(log_pose)      # moving frame, full rate
        .transform(throttle(0.1)).transform(accumulate_path).tap(log_path)   # growing path

Standalone — not wired into the dimos CLI:

    uv run python -m dimos.robot.unitree.go2dds.cli.render \
        go2_china_office_indoor.mcap --seconds 120
"""

from __future__ import annotations

from collections.abc import Callable
import shutil
import subprocess
from typing import TYPE_CHECKING, Any

import numpy as np
import typer

from dimos.memory2.transform import throttle
from dimos.memory2.utils.progress import progress
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Path import Path
from dimos.robot.unitree.go2dds.extrinsics import LIDAR_TO_BASE
from dimos.robot.unitree.go2dds.msgs.SportModeState import SportModeState
from dimos.robot.unitree.go2dds.store import Go2McapStore

if TYPE_CHECKING:
    from collections.abc import Iterator

    from dimos.memory2.type.observation import Observation

GRAVITY = np.array([0.0, 0.0, 9.81])
WORLD = "world"


# --- transforms over standard msgs (map / scan / reduce / tap) ---------------
def sportmode_pose(obs: Observation[SportModeState]) -> PoseStamped:
    """map_data: SportModeState -> PoseStamped (leg-inertial pose)."""
    sm = obs.data
    w, x, y, z = (float(v) for v in sm.imu_state.quaternion)  # Unitree order: wxyz
    return PoseStamped(
        ts=obs.ts,
        frame_id=WORLD,
        position=[float(v) for v in sm.position],
        orientation=[x, y, z, w],
    )


def integrate_velocity(state: Any, obs: Observation[Any]) -> tuple[Any, TwistStamped]:
    """scan_data: Imu -> TwistStamped (world accel integrated to velocity)."""
    vel, prev = state
    la = obs.data.linear_acceleration
    a = obs.data.orientation.rotate_vector(Vector3(la.x, la.y, la.z))
    a_world = np.array([a.x, a.y, a.z]) - GRAVITY
    if prev is not None:
        vel = vel + a_world * (obs.ts - prev)
    twist = TwistStamped(ts=obs.ts, frame_id=WORLD, linear=vel.tolist(), angular=[0.0, 0.0, 0.0])
    return (vel, obs.ts), twist


def integrate_position(state: Any, obs: Observation[Any]) -> tuple[Any, PoseStamped]:
    """scan_data: TwistStamped -> PoseStamped (velocity integrated to position)."""
    pos, prev = state
    v = obs.data.linear
    if prev is not None:
        pos = pos + np.array([v.x, v.y, v.z]) * (obs.ts - prev)
    pose = PoseStamped(
        ts=obs.ts, frame_id=WORLD, position=pos.tolist(), orientation=[0.0, 0.0, 0.0, 1.0]
    )
    return (pos, obs.ts), pose


def accumulate_path(upstream: Iterator[Observation[PoseStamped]]) -> Iterator[Observation[Path]]:
    """transform: yield the growing nav_msgs/Path as each pose streams in."""
    path = Path(frame_id=WORLD)
    for obs in upstream:
        path = path.push(obs.data)
        yield obs.derive(data=path)


# --- pipelines: (store, color, seconds) -> None ------------------------------
def leg_odom(store: Go2McapStore, seconds: float | None) -> None:
    """Leg-inertial odometry — pose stream (Transform3D) + accumulated Path line."""
    import rerun as rr

    def log_pose(obs: Observation[PoseStamped]) -> None:
        rr.set_time("time", timestamp=obs.ts)
        # Transform3D carries the pose; TransformAxes3D draws it as a visible gizmo.
        rr.log("world/leg_odom", obs.data.to_rerun(), rr.TransformAxes3D(axis_length=0.2))

    def log_path(obs: Observation[Path]) -> None:
        rr.set_time("time", timestamp=obs.ts)
        rr.log("world/leg_odom_path", obs.data.to_rerun())

    src = store.streams.sportmodestate.to_time(seconds)
    (
        src.tap(progress(src.count(), "leg_odom"))
        .map_data(sportmode_pose)
        .tap(log_pose)
        .transform(throttle(0.1))  # reduce_rate: thin the path to ~10 Hz
        .transform(accumulate_path)  # yield the growing path each step
        .tap(log_path)
        .drain()
    )


def imu_odom(store: Go2McapStore, seconds: float | None) -> None:
    """Dead-reckoned IMU odometry — accel -> velocity -> position -> growing Path (drifts)."""
    import rerun as rr

    def log_path(obs: Observation[Path]) -> None:
        rr.set_time("time", timestamp=obs.ts)
        rr.log("world/imu_odom_path", obs.data.to_rerun(color=(220, 90, 90)))

    src = store.streams.imu.to_time(seconds)
    (
        src.tap(progress(src.count(), "imu_odom"))
        .scan_data((np.zeros(3), None), integrate_velocity)  # -> velocity (TwistStamped)
        .scan_data((np.zeros(3), None), integrate_position)  # -> position (PoseStamped)
        .transform(throttle(0.1))  # thin the path after integrating at full IMU rate
        .transform(accumulate_path)
        .tap(log_path)
        .drain()
    )


def lidar(store: Go2McapStore, seconds: float | None) -> None:
    """Lidar point cloud, under the leg_odom transform (lidar -> base -> world)."""
    import rerun as rr

    def log_lidar(obs: Observation[PoseStamped]) -> None:
        rr.set_time("time", timestamp=obs.ts)
        rr.log("world/leg_odom/lidar", obs.data.to_rerun())

    src = store.streams.lidar.to_time(seconds)
    # Static lidar->base extrinsic (the L1 is mounted ~upside-down). Parent entity
    # world/leg_odom carries base->world, so rerun composes the full chain. Frame-less
    # Transform3D (not LIDAR_TO_BASE.to_rerun, which adds tf-graph frames) for entity-path.
    ext = LIDAR_TO_BASE
    rr.log(
        "world/leg_odom/lidar",
        rr.Transform3D(
            translation=[ext.translation.x, ext.translation.y, ext.translation.z],
            rotation=ext.rotation.to_rerun(),
        ),
        static=True,
    )
    (src.tap(progress(src.count(), "lidar")).tap(log_lidar).drain())


def _interp_pose(
    tt: np.ndarray, pos: np.ndarray, quat: np.ndarray, t: float
) -> tuple[np.ndarray, np.ndarray]:
    """LERP position + NLERP quaternion (xyzw) of a trajectory at scalar time t."""
    i = int(np.clip(np.searchsorted(tt, t), 1, len(tt) - 1))
    t0, t1 = tt[i - 1], tt[i]
    f = 0.0 if t1 == t0 else float(np.clip((t - t0) / (t1 - t0), 0.0, 1.0))
    p = pos[i - 1] * (1 - f) + pos[i] * f
    q0, q1 = quat[i - 1], quat[i].copy()
    if float(q0 @ q1) < 0:
        q1 = -q1
    q = q0 * (1 - f) + q1 * f
    return p, q / np.linalg.norm(q)


def world_lidar(store: Go2McapStore, seconds: float | None) -> None:
    """Lidar transformed into the world frame as data (Transform + PointCloud2.transform).

    Composes the static extrinsic with the leg-odom pose interpolated at each
    cloud's timestamp, then transforms the points — so the cloud genuinely lives
    in world (cf. ``lidar``, which leaves it in the sensor frame for rerun).
    """
    import rerun as rr

    from dimos.mapping.voxels import VoxelMapTransformer

    ext = LIDAR_TO_BASE  # lidar -> base (standard Transform from extrinsics)

    # pre-load the leg-odom trajectory for per-cloud pose interpolation
    odom = store.streams.odom.to_time(seconds).to_list()
    tt = np.array([o.ts for o in odom])
    poses = [o.data.pose.pose for o in odom]
    pos = np.array([[p.position.x, p.position.y, p.position.z] for p in poses])
    quat = np.array(
        [[p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w] for p in poses]
    )

    def to_world(obs: Observation[Any]) -> Any:
        p, q = _interp_pose(tt, pos, quat, obs.ts)
        b2w = Transform.from_pose(
            WORLD,
            PoseStamped(ts=obs.ts, frame_id=WORLD, position=p.tolist(), orientation=q.tolist()),
        )
        return obs.data.transform(b2w.apply(ext))  # lidar -> base -> world

    def log_voxels(obs: Observation[Any]) -> None:
        rr.set_time("time", timestamp=obs.ts)
        rr.log("world/world_lidar", obs.data.to_rerun())

    src = store.streams.lidar.to_time(seconds)
    (
        src.tap(progress(src.count(), "world_lidar"))
        .map_data(to_world)  # lidar cloud -> world-frame cloud
        .transform(VoxelMapTransformer(emit_every=10, voxel_size=0.1))  # global voxel map
        .tap(log_voxels)
        .drain()
    )


# Add a source: write a (store, seconds) -> None function and append it.
PIPELINES: list[Callable[[Go2McapStore, float | None], None]] = [
    leg_odom,
    imu_odom,
    lidar,
    world_lidar,
]


def main(
    mcap: str = typer.Argument(..., help="Go2 .mcap (path or data-dir name)"),
    out: str = typer.Option("go2_odom.rrd", "--out", help="Output .rrd"),
    seconds: float = typer.Option(None, "--seconds", help="Only the first N seconds"),
    no_gui: bool = typer.Option(False, "--no-gui", help="Write the .rrd but don't open the viewer"),
) -> None:
    import rerun as rr

    from dimos.visualization.rerun.init import rerun_init

    store = Go2McapStore(path=mcap)
    rerun_init("go2_odom")  # registers the turbo height colormap for PointCloud2.to_rerun
    rr.save(out)
    for pipeline in PIPELINES:
        pipeline(store, seconds)
    rr.rerun_shutdown()
    print(f"wrote {out}")
    if not no_gui:
        exe = shutil.which("rerun")
        if exe:
            subprocess.Popen([exe, out])
        else:
            print(f"  rerun viewer not on PATH; open manually:\n    rerun {out}")


if __name__ == "__main__":
    typer.run(main)
