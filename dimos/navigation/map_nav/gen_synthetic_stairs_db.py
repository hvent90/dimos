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

"""Generate a synthetic Go2 memory2 .db: three floors + stairs, for replay.

Writes ``lidar`` / ``odom`` / ``color_image`` streams in the same shape as
``go2_short.db``.

Default output: ``data/go2_synthetic_stairs.db`` (under the dimos project root,
via :func:`dimos.utils.data.get_data_dir`).

Usage::

    # default output (data/go2_synthetic_stairs.db)
    uv run python dimos/navigation/map_nav/gen_synthetic_stairs_db.py

    # custom path
    uv run python dimos/navigation/map_nav/gen_synthetic_stairs_db.py --out data/my_stairs.db

    # different random point sampling
    uv run python dimos/navigation/map_nav/gen_synthetic_stairs_db.py --seed 42

Geometry (walk +Y):
  floor0 (z=0) -> stairs (17x0.15 m) -> floor1 (z=2.55) -> stairs -> floor2 (z=5.1)

Riser height 0.15 m matches stock MLS ``step_threshold_m=0.16`` (mls-htc, nav-3d).

Lidar points are true world XYZ (not robot-relative WebRTC Z). Odom Z tracks
floor height so a climb is visible in both streams.

The output is a normal memory2 dataset. ``unitree-go2-map-nav`` can load it
interactively (teleop / click-nav) the same as real Go2 L1 or PointLIO bags
(``go2_bigoffice``, ``mid360_athens_stairs``, etc.).
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from dimos.memory2.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.data import get_data_dir

STEP_H_M = 0.15  # <= MLS step_threshold_m=0.16 (mls-htc, nav-3d)
STEPS_PER_FLIGHT = 17  # 2.55 m rise per flight
TREAD_Y_M = 0.30
FLAT_Y_M = 4.0
HALL_HALF_W_M = 1.2
WALL_H_M = 2.2
LIDAR_HZ = 10.0
ODOM_HZ = 50.0
IMAGE_HZ = 5.0
SPEED_MPS = 0.45
SENSOR_RANGE_M = 6.0
VOXEL_M = 0.05


def _layout() -> tuple[list[tuple[float, float, float]], list[tuple[float, float, float, float]]]:
    """Return (flats, flights).

    flats: (y0, y1, z)
    flights: (y0, z0, n_steps, tread_y) — step i at y=y0+i*tread, z=z0+i*STEP_H
    """
    flats: list[tuple[float, float, float]] = []
    flights: list[tuple[float, float, float, float]] = []
    y = 0.0
    z = 0.0
    for _ in range(3):
        flats.append((y, y + FLAT_Y_M, z))
        y += FLAT_Y_M
        if z >= 2 * STEPS_PER_FLIGHT * STEP_H_M - 1e-9:
            break
        flights.append((y, z, float(STEPS_PER_FLIGHT), TREAD_Y_M))
        y += STEPS_PER_FLIGHT * TREAD_Y_M
        z += STEPS_PER_FLIGHT * STEP_H_M
    return flats, flights


def floor_z_at_y(
    y: float,
    flats: list[tuple[float, float, float]],
    flights: list[tuple[float, float, float, float]],
) -> float:
    for y0, y1, z in flats:
        if y0 <= y <= y1:
            return z
    for y0, z0, n_steps, tread_y in flights:
        y1 = y0 + n_steps * tread_y
        if y0 <= y <= y1:
            i = min(int((y - y0) / tread_y), int(n_steps) - 1)
            return z0 + i * STEP_H_M
    # past the end: last floor
    return flats[-1][2]


def _yaw_quat(yaw: float) -> Quaternion:
    return Quaternion.from_euler(Vector3(0.0, 0.0, yaw))


def _pose(x: float, y: float, z: float, yaw: float) -> Pose:
    return Pose(position=[x, y, z], orientation=_yaw_quat(yaw))


def _sample_horizontal_rect(
    rng: np.random.Generator,
    *,
    x0: float,
    x1: float,
    y0: float,
    y1: float,
    z: float,
    pitch: float,
) -> np.ndarray:
    area = max(abs(x1 - x0) * abs(y1 - y0), 1e-6)
    n = max(int(area / (pitch * pitch)), 1)
    xs = rng.uniform(x0, x1, size=n)
    ys = rng.uniform(y0, y1, size=n)
    zs = np.full(n, z)
    return np.stack([xs, ys, zs], axis=1).astype(np.float32)


def _sample_wall(
    rng: np.random.Generator,
    *,
    x: float,
    y0: float,
    y1: float,
    z0: float,
    z1: float,
    pitch: float,
) -> np.ndarray:
    area = max(abs(y1 - y0) * abs(z1 - z0), 1e-6)
    n = max(int(area / (pitch * pitch)), 1)
    ys = rng.uniform(y0, y1, size=n)
    zs = rng.uniform(z0, z1, size=n)
    xs = np.full(n, x)
    return np.stack([xs, ys, zs], axis=1).astype(np.float32)


def build_world_points(rng: np.random.Generator) -> np.ndarray:
    """Dense-ish world surface samples (floors, treads, side walls)."""
    flats, flights = _layout()
    chunks: list[np.ndarray] = []
    for y0, y1, z in flats:
        chunks.append(
            _sample_horizontal_rect(
                rng,
                x0=-HALL_HALF_W_M,
                x1=HALL_HALF_W_M,
                y0=y0,
                y1=y1,
                z=z,
                pitch=VOXEL_M,
            )
        )
        chunks.append(
            _sample_wall(
                rng, x=-HALL_HALF_W_M, y0=y0, y1=y1, z0=z, z1=z + WALL_H_M, pitch=VOXEL_M * 1.5
            )
        )
        chunks.append(
            _sample_wall(
                rng, x=HALL_HALF_W_M, y0=y0, y1=y1, z0=z, z1=z + WALL_H_M, pitch=VOXEL_M * 1.5
            )
        )
    for y0, z0, n_steps, tread_y in flights:
        n = int(n_steps)
        for i in range(n):
            ty0 = y0 + i * tread_y
            ty1 = ty0 + tread_y
            tz = z0 + i * STEP_H_M
            chunks.append(
                _sample_horizontal_rect(
                    rng,
                    x0=-HALL_HALF_W_M,
                    x1=HALL_HALF_W_M,
                    y0=ty0,
                    y1=ty1,
                    z=tz,
                    pitch=VOXEL_M,
                )
            )
            # riser face at the far edge of this tread
            rz1 = tz + STEP_H_M
            area = max(2 * HALL_HALF_W_M * STEP_H_M, 1e-6)
            nr = max(int(area / (VOXEL_M * VOXEL_M)), 1)
            xs = rng.uniform(-HALL_HALF_W_M, HALL_HALF_W_M, size=nr)
            zs = rng.uniform(tz, rz1, size=nr)
            ys = np.full(nr, ty1)
            chunks.append(np.stack([xs, ys, zs], axis=1).astype(np.float32))
        y1 = y0 + n * tread_y
        chunks.append(
            _sample_wall(
                rng,
                x=-HALL_HALF_W_M,
                y0=y0,
                y1=y1,
                z0=z0,
                z1=z0 + WALL_H_M + n * STEP_H_M,
                pitch=VOXEL_M * 1.5,
            )
        )
        chunks.append(
            _sample_wall(
                rng,
                x=HALL_HALF_W_M,
                y0=y0,
                y1=y1,
                z0=z0,
                z1=z0 + WALL_H_M + n * STEP_H_M,
                pitch=VOXEL_M * 1.5,
            )
        )
    return np.vstack(chunks)


def lidar_frame_at(
    world_pts: np.ndarray,
    *,
    x: float,
    y: float,
    z: float,
    ts: float,
) -> PointCloud2:
    """Keep world points within SENSOR_RANGE of the robot (body height z)."""
    d = world_pts - np.array([x, y, z], dtype=np.float32)
    dist = np.linalg.norm(d, axis=1)
    mask = dist < SENSOR_RANGE_M
    pts = world_pts[mask]
    # Cap density like a voxelized WebRTC frame.
    if len(pts) > 25000:
        idx = np.linspace(0, len(pts) - 1, 25000).astype(np.int64)
        pts = pts[idx]
    return PointCloud2.from_numpy(pts, frame_id="world", timestamp=ts)


def _dummy_image(ts: float, floor_idx: int) -> Image:
    # Tiny placeholder so ReplayConnection.video_stream() has a stream.
    colors = [
        (40, 40, 40),
        (40, 80, 40),
        (40, 40, 100),
    ]
    c = colors[min(floor_idx, 2)]
    img = np.zeros((120, 160, 3), dtype=np.uint8)
    img[:, :] = c
    return Image.from_numpy(img, format=ImageFormat.BGR, frame_id="camera", ts=ts)


def generate(path: Path, *, seed: int = 0) -> dict[str, int | float]:
    if path.exists():
        path.unlink()
    rng = np.random.default_rng(seed)
    flats, flights = _layout()
    world_pts = build_world_points(rng)
    y_end = flats[-1][1]
    duration_s = y_end / SPEED_MPS
    t0 = 1_000_000.0  # stable absolute-ish timestamps

    store = SqliteStore(path=str(path))
    store.start()
    try:
        lidar_s = store.stream("lidar", PointCloud2, codec="lz4+lcm")
        odom_s = store.stream("odom", PoseStamped)
        img_s = store.stream("color_image", Image)

        n_odom = n_lidar = n_img = 0
        n_odom_steps = int(duration_s * ODOM_HZ) + 1
        for i in range(n_odom_steps):
            t = t0 + i / ODOM_HZ
            y = min(i / ODOM_HZ * SPEED_MPS, y_end)
            z = floor_z_at_y(y, flats, flights) + 0.31  # body height above floor
            yaw = np.pi / 2  # +Y
            pose = _pose(0.0, y, z, yaw)
            odom = PoseStamped(
                ts=t, frame_id="world", position=[0.0, y, z], orientation=_yaw_quat(yaw)
            )
            odom_s.append(odom, ts=t, pose=pose)
            n_odom += 1

            if i % int(ODOM_HZ / LIDAR_HZ) == 0:
                cloud = lidar_frame_at(world_pts, x=0.0, y=y, z=z, ts=t)
                lidar_s.append(cloud, ts=t, pose=pose)
                n_lidar += 1

            if i % int(ODOM_HZ / IMAGE_HZ) == 0:
                floor_idx = round(floor_z_at_y(y, flats, flights) / (STEPS_PER_FLIGHT * STEP_H_M))
                img_s.append(_dummy_image(t, floor_idx), ts=t, pose=pose)
                n_img += 1
    finally:
        store.stop()

    return {
        "odom": n_odom,
        "lidar": n_lidar,
        "color_image": n_img,
        "y_end_m": y_end,
        "z_top_m": flats[-1][2],
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--out",
        type=Path,
        default=get_data_dir() / "go2_synthetic_stairs.db",
        help="Output SqliteStore path (default: data/go2_synthetic_stairs.db)",
    )
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()
    stats = generate(args.out, seed=args.seed)
    db = args.out.stem
    print(f"Wrote {args.out}")
    print(stats)
    print(
        f"Interactive nav:\n  uv run dimos --replay-db={db} --viewer=rerun run unitree-go2-map-nav"
    )


if __name__ == "__main__":
    main()
