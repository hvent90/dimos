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

"""Static-map + perfect-odometry plant for virtual Go2 nav (no live robot).

On ``--replay``, ``GO2Connection`` plays recorded odom and ignores ``cmd_vel``.
That is why teleop and downstairs follow fail on ``unitree-go2-mls-htc`` with a
bag: the dog is a tape, not a plant.

This module:
1. Builds a static ``global_map`` once from a memory2 lidar stream (Go2 ``lidar``
   or Mid360 ``pointlio_lidar``, with sensor-frame clouds lifted via ``tf`` /
   stored pose the same way as ``dimos map global``). With ``--map-pgo``, runs
   offline loop-closure PGO (same engine as ``dimos map global --pgo``) and
   caches the cleaned cloud beside the dataset.
2. Integrates ``cmd_vel`` into ``odom`` at a fixed rate (perfect planar odometry).
3. Snaps floor Z to the map under the body (local column, one stair step at a
   time) so teleop and holonomic follow both climb/descend with the surface.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
import threading
import time
from typing import Any

from pydantic import Field
from reactivex.disposable import Disposable

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.mapping.voxels import VoxelGrid
from dimos.memory2.replay import resolve_db_path
from dimos.memory2.store.sqlite import SqliteStore
from dimos.memory2.tf import StreamTF
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

ColumnKey = tuple[int, int]

# Preferred lidar streams when ``lidar_stream`` is empty / missing.
_LIDAR_STREAM_CANDIDATES = ("lidar", "pointlio_lidar")
_WORLD_FRAMES = ("world", "map", "odom")


@dataclass(frozen=True)
class BuiltStaticMap:
    """Result of :func:`build_global_map_from_db`."""

    cloud: PointCloud2
    start_x: float
    start_y: float
    start_yaw: float
    lidar_stream: str
    world_frame: str
    frames_used: int
    pgo: bool = False
    from_cache: bool = False


def resolve_lidar_stream(store: Any, requested: str) -> str:
    """Pick a non-empty PointCloud2 stream; prefer ``requested``, else candidates."""
    names = list(store.list_streams())
    ordered = [requested, *[c for c in _LIDAR_STREAM_CANDIDATES if c != requested]]
    tried: list[str] = []
    for name in ordered:
        if name not in names:
            continue
        tried.append(name)
        first = next(iter(store.stream(name, PointCloud2)), None)
        if first is not None and len(first.data) > 0:
            return name
    raise ValueError(
        f"No non-empty lidar stream in {getattr(store, 'path', store)!r} "
        f"(tried {tried or ordered}; available={names})"
    )


def _detect_world_frame(tf_buf: StreamTF | None, cloud_frame: str, ts: float) -> str:
    if cloud_frame in _WORLD_FRAMES:
        return cloud_frame
    if tf_buf is not None:
        for cand in _WORLD_FRAMES:
            if tf_buf.get(cand, cloud_frame, time_point=ts) is not None:
                return cand
    return "world"


def _yaw_from_transform(tf: Transform) -> float:
    return float(tf.rotation.euler[2])


def _register_transform(
    obs: Any,
    *,
    cloud_frame: str,
    world_frame: str,
    tf_buf: StreamTF | None,
    tf_tolerance: float,
) -> Transform | None:
    """Lift a sensor-frame cloud into ``world_frame`` (tf first, else obs.pose)."""
    if cloud_frame == world_frame:
        return None
    if tf_buf is not None:
        tf = tf_buf.get(
            world_frame, obs.data.frame_id, time_point=obs.ts, time_tolerance=tf_tolerance
        )
        if tf is not None:
            return tf
    pose = obs.pose
    if pose is not None and not (pose.position.is_zero() or pose.orientation.is_zero()):
        return Transform.from_pose(cloud_frame, pose)
    return None


def _trajectory_xy_yaw(tf: Transform | None, obs: Any) -> tuple[float, float, float] | None:
    if tf is not None:
        return (float(tf.translation.x), float(tf.translation.y), _yaw_from_transform(tf))
    pose = obs.pose
    if pose is None or pose.position.is_zero():
        return None
    return (float(pose.position.x), float(pose.position.y), float(pose.orientation.euler[2]))


def map_nav_pgo_cache_paths(db_path: Path) -> tuple[Path, Path]:
    """``{stem}.map_nav_pgo.pc2.lcm`` + meta json beside the dataset."""
    stem = db_path.stem
    parent = db_path.parent
    return (
        parent / f"{stem}.map_nav_pgo.pc2.lcm",
        parent / f"{stem}.map_nav_pgo.meta.json",
    )


def _cli_export_pc2_path(db_path: Path) -> Path:
    """``dimos map global --export`` writes ``cwd/{stem}.pc2.lcm``."""
    return Path.cwd() / f"{db_path.stem}.pc2.lcm"


def _save_pgo_cache(db_path: Path, built: BuiltStaticMap) -> None:
    pc2_path, meta_path = map_nav_pgo_cache_paths(db_path)
    pc2_path.write_bytes(built.cloud.lcm_encode())
    meta = {
        "start_x": built.start_x,
        "start_y": built.start_y,
        "start_yaw": built.start_yaw,
        "lidar_stream": built.lidar_stream,
        "world_frame": built.world_frame,
        "frames_used": built.frames_used,
        "pgo": True,
    }
    meta_path.write_text(json.dumps(meta, indent=2) + "\n", encoding="utf-8")
    logger.info("MapNavPlant wrote PGO cache", pc2=str(pc2_path), meta=str(meta_path))


def _load_pgo_cache(
    db_path: Path,
    *,
    start_x: float,
    start_y: float,
    start_yaw: float,
) -> BuiltStaticMap | None:
    """Load map-nav cache, else ``dimos map global --export`` cloud if present."""
    pc2_path, meta_path = map_nav_pgo_cache_paths(db_path)
    if pc2_path.is_file() and meta_path.is_file():
        cloud = PointCloud2.lcm_decode(pc2_path.read_bytes())
        meta = json.loads(meta_path.read_text(encoding="utf-8"))
        logger.info("MapNavPlant loaded PGO cache", pc2=str(pc2_path), voxels=len(cloud))
        return BuiltStaticMap(
            cloud=cloud,
            start_x=float(meta.get("start_x", start_x)),
            start_y=float(meta.get("start_y", start_y)),
            start_yaw=float(meta.get("start_yaw", start_yaw)),
            lidar_stream=str(meta.get("lidar_stream", "lidar")),
            world_frame=str(meta.get("world_frame", "world")),
            frames_used=int(meta.get("frames_used", 0)),
            pgo=True,
            from_cache=True,
        )

    export_path = _cli_export_pc2_path(db_path)
    if export_path.is_file():
        cloud = PointCloud2.lcm_decode(export_path.read_bytes())
        logger.info(
            "MapNavPlant loaded dimos map --export cloud",
            pc2=str(export_path),
            voxels=len(cloud),
        )
        return BuiltStaticMap(
            cloud=cloud,
            start_x=start_x,
            start_y=start_y,
            start_yaw=start_yaw,
            lidar_stream="lidar",
            world_frame="world",
            frames_used=0,
            pgo=True,
            from_cache=True,
        )
    return None


def build_global_map_from_db(
    dataset: str,
    *,
    lidar_stream: str = "lidar",
    voxel_size: float = 0.05,
    frame_id: str = "world",
    dedup_tol_m: float = 0.3,
    tf_tolerance: float = 0.5,
    start_x: float = 0.0,
    start_y: float = 0.5,
    start_yaw: float = math.pi / 2,
    pgo: bool = False,
) -> BuiltStaticMap:
    """Accumulate lidar into one nav-frame cloud (Go2 world or Mid360 via tf).

    With ``pgo=True``, runs ``dimos.mapping.loop_closure.pgo.PGO`` (same as
    ``dimos map global --pgo``), voxel-rebuilds with drift corrections, and
    caches the result beside the dataset for the next start.
    """
    path = resolve_db_path(dataset)
    if pgo:
        cached = _load_pgo_cache(path, start_x=start_x, start_y=start_y, start_yaw=start_yaw)
        if cached is not None:
            return cached

    store = SqliteStore(path=str(path), must_exist=True)
    store.start()
    try:
        stream_name = resolve_lidar_stream(store, lidar_stream)
        lidar = store.stream(stream_name, PointCloud2)
        first = next(iter(lidar), None)
        if first is None:
            raise ValueError(f"Lidar stream {stream_name!r} is empty in {path}")

        cloud_frame = first.data.frame_id or frame_id
        tf_buf = StreamTF.from_store(store)
        world_frame = _detect_world_frame(tf_buf, cloud_frame, first.ts)
        needs_register = cloud_frame != world_frame

        graph = None
        if pgo:
            from dimos.mapping.loop_closure.pgo import FRAME_BODY, PGO

            logger.info("MapNavPlant running PGO", dataset=str(path), stream=stream_name)
            graph = store.stream(stream_name, PointCloud2).transform(PGO()).last().data
            logger.info(
                "MapNavPlant PGO finished",
                keyframes=len(graph.keyframes),
                loops=len(graph.loops),
            )

        # Spatial dedup (same idea as ``dimos map global --pgo-tol``): keep the
        # latest frame per cell so Mid360 bags don't dump thousands of overlaps.
        kept: dict[Any, Any] = {}
        for i, obs in enumerate(store.stream(stream_name, PointCloud2)):
            if len(obs.data) == 0:
                continue
            reg = (
                _register_transform(
                    obs,
                    cloud_frame=cloud_frame,
                    world_frame=world_frame,
                    tf_buf=tf_buf,
                    tf_tolerance=tf_tolerance,
                )
                if needs_register
                else None
            )
            if needs_register and reg is None:
                continue

            cloud_tf: Transform | None = reg
            traj: tuple[float, float, float] | None
            z_for_key: float
            if graph is not None:
                if obs.pose_tuple is None:
                    continue
                # Clouds live in world_raw; correction lifts them into world_corrected.
                # Robot XY for dedup/start comes from the corrected body pose, not
                # from correction.translation (that vector is drift, often ~0).
                correction = graph.correction_at(obs.ts)
                cloud_tf = correction if reg is None else correction + reg
                corrected = graph.correct(Transform.from_pose(FRAME_BODY, obs.pose_stamped))
                traj = (
                    float(corrected.translation.x),
                    float(corrected.translation.y),
                    _yaw_from_transform(corrected),
                )
                z_for_key = float(corrected.translation.z)
            else:
                traj = _trajectory_xy_yaw(reg, obs)
                if reg is not None:
                    z_for_key = float(reg.translation.z)
                elif obs.pose is not None:
                    z_for_key = float(obs.pose.position.z)
                else:
                    z_for_key = 0.0

            if dedup_tol_m > 0 and traj is not None:
                key: Any = (
                    math.floor(traj[0] / dedup_tol_m),
                    math.floor(traj[1] / dedup_tol_m),
                    math.floor(z_for_key / dedup_tol_m),
                )
            else:
                key = i
            kept[key] = (obs, cloud_tf, traj)

        if not kept:
            raise ValueError(
                f"No registerable lidar frames in {path} stream={stream_name!r} "
                f"cloud_frame={cloud_frame!r} world_frame={world_frame!r}"
            )

        grid = VoxelGrid(
            voxel_size=voxel_size,
            device="CPU:0",
            carve_columns=False,
            frame_id=frame_id,
            show_startup_log=False,
        )
        try:
            start = (start_x, start_y, start_yaw)
            start_set = False
            if graph is not None and graph.keyframes:
                kf0 = graph.keyframes[0]
                start = (
                    float(kf0.optimized.translation.x),
                    float(kf0.optimized.translation.y),
                    _yaw_from_transform(kf0.optimized),
                )
                start_set = True
            n = 0
            for obs, tf, traj in kept.values():
                cloud = obs.data if tf is None else obs.data.transform(tf)
                cloud.frame_id = frame_id
                grid.add_frame(cloud)
                n += 1
                if not start_set and traj is not None:
                    start = traj
                    start_set = True
            cloud_out = grid.get_global_pointcloud2()
            logger.info(
                "MapNavPlant built static map",
                dataset=str(path),
                lidar_stream=stream_name,
                cloud_frame=cloud_frame,
                world_frame=world_frame,
                frames=n,
                voxels=len(cloud_out),
                start=start,
                pgo=pgo,
            )
            if len(cloud_out) == 0:
                raise ValueError(f"Built empty voxel map from {path} stream={stream_name!r}")
            built = BuiltStaticMap(
                cloud=cloud_out,
                start_x=start[0],
                start_y=start[1],
                start_yaw=start[2],
                lidar_stream=stream_name,
                world_frame=world_frame,
                frames_used=n,
                pgo=pgo,
                from_cache=False,
            )
            if pgo:
                _save_pgo_cache(path, built)
            return built
        finally:
            grid.dispose()
    finally:
        store.stop()


def build_surface_columns(cloud: PointCloud2, voxel_size: float) -> dict[ColumnKey, list[float]]:
    """(ix, iy) -> sorted unique surface z samples for snap."""
    pts = cloud.points_f32()
    cols: dict[ColumnKey, list[float]] = {}
    inv = 1.0 / voxel_size
    for x, y, z in pts:
        key = (math.floor(float(x) * inv), math.floor(float(y) * inv))
        cols.setdefault(key, []).append(float(z))
    for key, zs in cols.items():
        zs.sort()
        uniq: list[float] = []
        for z in zs:
            if not uniq or abs(z - uniq[-1]) > 0.5 * voxel_size:
                uniq.append(z)
        cols[key] = uniq
    return cols


def snap_z_to_surface(
    columns: dict[ColumnKey, list[float]],
    *,
    x: float,
    y: float,
    z_hint: float,
    voxel_size: float,
    search_radius_m: float = 0.15,
    max_step_m: float = 0.16,
) -> float:
    """Walkable surface under ``(x, y)`` within one stair step of ``z_hint``.

    Expands ring-by-ring from the robot column so a nearby same-height floor
    (behind you on the flat) cannot win over the tread you are standing on.
    If nothing is reachable in one step, keep ``z_hint`` (no floor teleport).
    """
    inv = 1.0 / voxel_size
    ix0 = math.floor(x * inv)
    iy0 = math.floor(y * inv)
    r_max = max(0, math.ceil(search_radius_m / voxel_size))
    for ring in range(r_max + 1):
        best: float | None = None
        best_d = math.inf
        for dix in range(-ring, ring + 1):
            for diy in range(-ring, ring + 1):
                if ring > 0 and max(abs(dix), abs(diy)) != ring:
                    continue
                zs = columns.get((ix0 + dix, iy0 + diy))
                if not zs:
                    continue
                for z in zs:
                    d = abs(z - z_hint)
                    if d > max_step_m:
                        continue
                    if d < best_d:
                        best_d = d
                        best = z
        if best is not None:
            return best
    return z_hint


class MapNavPlantConfig(ModuleConfig):
    """Config for :class:`MapNavPlant`."""

    map_db: str = Field(
        default_factory=lambda m: m["g"].replay_db,
        description="memory2 dataset name or path used to build the static map",
    )
    lidar_stream: str = "lidar"
    voxel_size: float = 0.05
    frame_id: str = "world"
    rate_hz: float = 50.0
    body_height_m: float = 0.31
    # After a startup burst (see ``map_publish_burst_s``), 0 stops republishing.
    # Continuous 1 Hz of a frozen real cloud re-runs MLS full rebuilds and makes
    # Rerun look chunky; live VoxelGridMapper maps change every tick so they
    # keep a steady publish rate instead.
    map_publish_hz: float = 0.0
    # LCM is not latched: burst so Rerun/MLS subscribers that start after the
    # plant still see the frozen map, then stop.
    map_publish_burst_s: float = 5.0
    map_publish_burst_hz: float = 2.0
    start_x: float = 0.0
    start_y: float = 0.5
    start_yaw: float = math.pi / 2  # +Y, matches synthetic stairs generator
    # Spatial dedup while building the static map (``dimos map global --pgo-tol``).
    dedup_tol_m: float = 0.3
    # Offline loop-closure PGO (``--map-pgo`` / ``dimos map global --pgo``).
    pgo: bool = Field(default_factory=lambda m: bool(m["g"].map_pgo))
    # Only fill small map holes; must stay << tread length so the previous floor
    # cannot pin Z while XY walks onto the stairs.
    surface_search_radius_m: float = 0.15
    max_step_m: float = 0.16


class MapNavPlant(Module):
    """Publish a frozen map and odom driven by ``cmd_vel`` (perfect plant)."""

    config: MapNavPlantConfig

    cmd_vel: In[Twist]
    set_pose: In[PoseStamped]  # agent teleport onto MLS home / m1
    global_map: Out[PointCloud2]
    odom: Out[PoseStamped]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._map_thread: threading.Thread | None = None
        self._vx = 0.0
        self._vy = 0.0
        self._wz = 0.0
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._z_floor = 0.0
        self._map: PointCloud2 | None = None
        self._columns: dict[ColumnKey, list[float]] = {}

    @rpc
    def start(self) -> None:
        super().start()
        self.tf.start()
        cfg = self.config
        built = build_global_map_from_db(
            cfg.map_db,
            lidar_stream=cfg.lidar_stream,
            voxel_size=cfg.voxel_size,
            frame_id=cfg.frame_id,
            dedup_tol_m=cfg.dedup_tol_m,
            start_x=cfg.start_x,
            start_y=cfg.start_y,
            start_yaw=cfg.start_yaw,
            pgo=cfg.pgo,
        )
        self._map = built.cloud
        self._columns = build_surface_columns(self._map, cfg.voxel_size)
        self._x = built.start_x
        self._y = built.start_y
        self._yaw = built.start_yaw
        self._z_floor = snap_z_to_surface(
            self._columns,
            x=self._x,
            y=self._y,
            z_hint=0.0,
            voxel_size=cfg.voxel_size,
            search_radius_m=cfg.surface_search_radius_m,
            max_step_m=cfg.max_step_m,
        )
        self._stop.clear()
        self.register_disposable(Disposable(self.cmd_vel.subscribe(self._on_cmd_vel)))
        self.register_disposable(Disposable(self.set_pose.subscribe(self._on_set_pose)))
        self._thread = threading.Thread(target=self._odom_loop, daemon=True)
        self._thread.start()
        self._map_thread = threading.Thread(target=self._map_loop, daemon=True)
        self._map_thread.start()
        logger.info(
            "MapNavPlant started",
            map_db=cfg.map_db,
            lidar_stream=built.lidar_stream,
            world_frame=built.world_frame,
            frames=built.frames_used,
            pgo=built.pgo,
            from_cache=built.from_cache,
            start=(self._x, self._y, self._z_floor + cfg.body_height_m),
        )

    @rpc
    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(DEFAULT_THREAD_JOIN_TIMEOUT)
        if self._map_thread is not None:
            self._map_thread.join(DEFAULT_THREAD_JOIN_TIMEOUT)
        super().stop()

    def _on_cmd_vel(self, twist: Twist) -> None:
        with self._lock:
            self._vx = float(twist.linear.x)
            self._vy = float(twist.linear.y)
            self._wz = float(twist.angular.z)

    def _on_set_pose(self, pose: PoseStamped) -> None:
        """Teleport plant to a body-frame pose (e.g. MapNavAgent m1 home)."""
        cfg = self.config
        x = float(pose.x)
        y = float(pose.y)
        z_body = float(pose.z)
        yaw = float(pose.orientation.euler[2]) if pose.orientation is not None else 0.0
        z_floor = z_body - cfg.body_height_m
        z_floor = snap_z_to_surface(
            self._columns,
            x=x,
            y=y,
            z_hint=z_floor,
            voxel_size=cfg.voxel_size,
            search_radius_m=max(cfg.surface_search_radius_m, 0.5),
            max_step_m=max(cfg.max_step_m, 2.0),
        )
        with self._lock:
            self._x = x
            self._y = y
            self._yaw = yaw
            self._z_floor = z_floor
            self._vx = 0.0
            self._vy = 0.0
            self._wz = 0.0
        logger.info(
            "MapNavPlant set_pose",
            x=x,
            y=y,
            z_floor=z_floor,
            yaw=yaw,
        )

    def _map_loop(self) -> None:
        cfg = self.config
        burst_hz = max(cfg.map_publish_burst_hz, 0.1)
        burst_period = 1.0 / burst_hz
        deadline = time.monotonic() + max(cfg.map_publish_burst_s, 0.0)
        while not self._stop.is_set():
            if self._map is not None:
                self.global_map.publish(self._map)
            now = time.monotonic()
            if now >= deadline:
                break
            if self._stop.wait(min(burst_period, deadline - now)):
                return

        hz = cfg.map_publish_hz
        if hz <= 0:
            return
        period = 1.0 / hz
        while not self._stop.wait(period):
            if self._map is not None:
                self._map.ts = time.time()
                self.global_map.publish(self._map)

    def _odom_loop(self) -> None:
        period = 1.0 / max(self.config.rate_hz, 1.0)
        next_t = time.monotonic()
        while not self._stop.is_set():
            next_t += period
            self._tick(period)
            sleep = next_t - time.monotonic()
            if sleep > 0:
                self._stop.wait(sleep)
            else:
                next_t = time.monotonic()

    def _tick(self, dt: float) -> None:
        cfg = self.config
        with self._lock:
            vx, vy, wz = self._vx, self._vy, self._wz
            x, y, yaw = self._x, self._y, self._yaw
            z_floor = self._z_floor

        cy, sy = math.cos(yaw), math.sin(yaw)
        x += dt * (cy * vx - sy * vy)
        y += dt * (sy * vx + cy * vy)
        yaw += dt * wz
        if yaw > math.pi:
            yaw -= 2 * math.pi
        elif yaw < -math.pi:
            yaw += 2 * math.pi

        # Holonomic / teleop are XY-only; floor Z always comes from the map under
        # the body (one stair riser per step via max_step_m).
        z_floor = snap_z_to_surface(
            self._columns,
            x=x,
            y=y,
            z_hint=z_floor,
            voxel_size=cfg.voxel_size,
            search_radius_m=cfg.surface_search_radius_m,
            max_step_m=cfg.max_step_m,
        )
        z_body = z_floor + cfg.body_height_m

        with self._lock:
            self._x, self._y, self._yaw, self._z_floor = x, y, yaw, z_floor

        now = time.time()
        quat = Quaternion.from_euler(Vector3(0.0, 0.0, yaw))
        odom = PoseStamped(
            ts=now,
            frame_id=cfg.frame_id,
            position=[x, y, z_body],
            orientation=quat,
        )
        self.odom.publish(odom)
        self.tf.publish(Transform.from_pose("base_link", odom))
