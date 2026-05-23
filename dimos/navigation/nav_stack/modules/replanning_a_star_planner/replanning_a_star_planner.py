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

"""ReplanningAStarPlanner — nav_stack Module that mirrors the original
``dimos/navigation/replanning_a_star`` planner, but:

* Builds its own costmap from PointCloud2 inputs (like SimplePlanner).
* Outputs only the planner-side nav_stack interface (way_point, goal_path,
  costmap_cloud) — no LocalPlanner, no cmd_vel. Downstream LocalPlanner
  + PathFollower handle motion.

Reuses the pure A* core, ReplanLimiter, PositionTracker, and gradient
costmap utilities from the original location.
"""

from __future__ import annotations

import math
import threading
import time
from typing import Any

from dimos_lcm.std_msgs import Bool  # type: ignore[import-untyped]
import numpy as np
import open3d as o3d  # type: ignore[import-untyped]
import open3d.core as o3c  # type: ignore[import-untyped]
from reactivex.disposable import Disposable

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.mapping.occupancy.path_map import make_navigation_map
from dimos.mapping.occupancy.path_resampling import smooth_resample_path
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.OccupancyGrid import CostValues
from dimos.msgs.nav_msgs.Path import Path
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.nav_stack.frames import FRAME_BODY, FRAME_MAP, FRAME_SENSOR
from dimos.navigation.nav_stack.modules.replanning_a_star_planner.costmap_builder import (
    HeightMapCostmap,
)
from dimos.navigation.replanning_a_star.min_cost_astar import min_cost_astar
from dimos.navigation.replanning_a_star.position_tracker import PositionTracker
from dimos.navigation.replanning_a_star.replan_limiter import ReplanLimiter
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


_TF_WARN_THROTTLE = 5.0  # s
_COSTMAP_VIS_Z_LIFT = 0.1  # m above ground
_COLOR_OBSTACLE = (1.0, 40.0 / 255.0, 40.0 / 255.0)  # red
_COLOR_INFLATION = (1.0, 165.0 / 255.0, 0.0)  # orange


def _resolve_tf_chain(tf_buffer: Any, queries: list[tuple[str, str]]) -> Any:
    for parent, child in queries:
        tf = tf_buffer.get(parent, child)
        if tf is not None:
            return tf
    return None


def _distance_point_to_polyline(px: float, py: float, path: list[tuple[float, float]]) -> float:
    """Min perpendicular distance from (px, py) to a polyline of (x, y) tuples."""
    if not path:
        return float("inf")
    if len(path) == 1:
        x0, y0 = path[0]
        return math.hypot(px - x0, py - y0)
    best = float("inf")
    for i in range(len(path) - 1):
        ax, ay = path[i]
        bx, by = path[i + 1]
        vx, vy = bx - ax, by - ay
        seg_len_sq = vx * vx + vy * vy
        if seg_len_sq < 1e-12:
            d = math.hypot(px - ax, py - ay)
        else:
            t = ((px - ax) * vx + (py - ay) * vy) / seg_len_sq
            t = max(0.0, min(1.0, t))
            qx = ax + t * vx
            qy = ay + t * vy
            d = math.hypot(px - qx, py - qy)
        if d < best:
            best = d
    return best


class ReplanningAStarPlannerConfig(ModuleConfig):
    world_frame: str = FRAME_MAP
    body_frame: str = FRAME_BODY
    sensor_frame: str = FRAME_SENSOR

    # Costmap construction
    cell_size: float = 0.15  # m per cell — finer than SimplePlanner so the gradient is meaningful
    obstacle_height_threshold: float = 0.15  # m above ground
    ground_offset_below_robot: float = 1.3  # m (sensor → ground for G1)

    # Costmap window: square of this half-side around robot AND goal
    window_radius: float = 8.0  # m

    # Planner inflation: passed as robot_increase to make_navigation_map.
    # 1.1 matches the original GlobalPlanner._find_wide_path single-size pass.
    inflation_size: float = 1.1
    cost_threshold: int = 100
    unknown_penalty: float = 0.8

    # Loops
    monitor_rate: float = 10.0  # Hz — matches original 0.1s wait
    waypoint_rate: float = 30.0  # Hz — matches SimplePlanner
    lookahead_distance: float = 1.0  # m

    # Tolerances
    goal_reached_threshold: float = 0.39  # m — matches SimplePlanner default
    replan_goal_tolerance: float = 0.5  # m
    max_path_deviation: float = 0.9  # m — matches original

    # Stuck detection (PositionTracker)
    stuck_time_window: float = 8.0  # s
    stuck_threshold: float = 0.4  # m radius

    # ReplanLimiter
    max_replan_attempts: int = 6
    replan_reset_distance: float = 2.0  # m

    # Path resampling spacing
    path_resample_spacing: float = 0.1  # m — matches original

    # Debug viz
    costmap_publish_period: float = 0.5  # s


class ReplanningAStarPlanner(Module):
    """Grid-A* global route planner with event-driven replanning.

    Mirrors the original ``GlobalPlanner`` (in
    ``dimos/navigation/replanning_a_star/global_planner.py``) but lives in
    the nav_stack as a Module and only emits the planner-side outputs.
    """

    config: ReplanningAStarPlannerConfig

    terrain_map_ext: In[PointCloud2]
    terrain_map: In[PointCloud2]
    goal: In[PointStamped]
    stop_movement: In[Bool]
    way_point: Out[PointStamped]
    goal_path: Out[Path]
    costmap_cloud: Out[PointCloud2]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._running = False
        self._monitor_thread: threading.Thread | None = None
        self._waypoint_thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._costmap_lock = threading.Lock()

        # Robot state (refreshed from TF)
        self._robot_x = 0.0
        self._robot_y = 0.0
        self._robot_z = 0.0
        self._has_odom = False
        self._last_tf_warn = 0.0

        # Goal state
        self._goal_x: float | None = None
        self._goal_y: float | None = None
        self._goal_z = 0.0

        # Cached plan
        self._cached_path: list[tuple[float, float]] | None = None
        self._last_costmap_pub = 0.0

        # Stuck / replan tracking
        self._position_tracker = PositionTracker(
            self.config.stuck_time_window, self.config.stuck_threshold
        )
        self._replan_limiter = ReplanLimiter()
        # Override defaults to honor config
        self._replan_limiter._max_attempts = self.config.max_replan_attempts
        self._replan_limiter._reset_distance = self.config.replan_reset_distance

        # Costmap
        self._costmap = HeightMapCostmap(
            cell_size=self.config.cell_size,
            obstacle_height_threshold=self.config.obstacle_height_threshold,
        )

        # Stuck timer (mirrors `last_stuck_check` in original loop)
        self._last_stuck_check = time.perf_counter()
        self._last_unique_pos: tuple[int, int] | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.goal.subscribe(self._on_goal)))
        self.register_disposable(Disposable(self.stop_movement.subscribe(self._on_stop_movement)))
        self.register_disposable(Disposable(self.terrain_map_ext.subscribe(self._on_terrain_ext)))
        self.register_disposable(Disposable(self.terrain_map.subscribe(self._on_terrain)))
        self._running = True
        self._monitor_thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self._monitor_thread.start()
        self._waypoint_thread = threading.Thread(target=self._waypoint_loop, daemon=True)
        self._waypoint_thread.start()
        logger.info("ReplanningAStarPlanner started")

    @rpc
    def stop(self) -> None:
        self._running = False
        for t in (self._waypoint_thread, self._monitor_thread):
            if t is not None:
                t.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        self._waypoint_thread = None
        self._monitor_thread = None
        super().stop()

    # ---------------- TF / pose ----------------

    @property
    def _tf_pose_queries(self) -> list[tuple[str, str]]:
        return [
            (self.config.world_frame, self.config.body_frame),
            (self.config.world_frame, self.config.sensor_frame),
        ]

    def _query_pose(self) -> bool:
        tf = _resolve_tf_chain(self.tf, list(self._tf_pose_queries))
        if tf is None:
            now = time.monotonic()
            if now - self._last_tf_warn > _TF_WARN_THROTTLE:
                self._last_tf_warn = now
                buffers = list(self.tf.buffers.keys()) if hasattr(self.tf, "buffers") else []
                logger.warning(
                    "TF lookup failed — no robot pose available",
                    tried=self._tf_pose_queries,
                    available_frames=buffers,
                )
            return False
        with self._lock:
            self._robot_x = float(tf.translation.x)
            self._robot_y = float(tf.translation.y)
            self._robot_z = float(tf.translation.z)
            self._has_odom = True
        # Feed PositionTracker for stuck detection
        pose = PoseStamped(
            frame_id=self.config.world_frame,
            position=[self._robot_x, self._robot_y, self._robot_z],
            orientation=[0.0, 0.0, 0.0, 1.0],
        )
        self._position_tracker.add_position(pose)
        return True

    # ---------------- Inputs ----------------

    def _on_terrain_ext(self, msg: PointCloud2) -> None:
        """Persistent world view — reset costmap to avoid stale obstacles."""
        points, _ = msg.as_numpy()
        if points is None or len(points) == 0:
            return
        with self._lock:
            rz = self._robot_z if self._has_odom else 0.0
        ground_z = rz - self.config.ground_offset_below_robot
        new_costmap = HeightMapCostmap(
            cell_size=self.config.cell_size,
            obstacle_height_threshold=self.config.obstacle_height_threshold,
        )
        new_costmap.ingest(points, ground_z)
        with self._costmap_lock:
            self._costmap = new_costmap

    def _on_terrain(self, msg: PointCloud2) -> None:
        """Fast local terrain — layer onto existing costmap."""
        points, _ = msg.as_numpy()
        if points is None or len(points) == 0:
            return
        with self._lock:
            rz = self._robot_z if self._has_odom else 0.0
        ground_z = rz - self.config.ground_offset_below_robot
        with self._costmap_lock:
            self._costmap.ingest(points, ground_z)

    def _on_goal(self, msg: PointStamped) -> None:
        if not all(math.isfinite(v) for v in (msg.x, msg.y, msg.z)):
            self._cancel_navigation(source="nan_goal")
            return
        with self._lock:
            self._goal_x = float(msg.x)
            self._goal_y = float(msg.y)
            self._goal_z = float(msg.z)
            self._cached_path = None
        self._replan_limiter.reset()
        self._last_stuck_check = time.perf_counter()
        logger.info("Goal received", x=round(msg.x, 2), y=round(msg.y, 2), z=round(msg.z, 2))
        # Plan once immediately so the robot starts moving without waiting for the loop tick.
        self._plan_once()

    def _on_stop_movement(self, msg: Bool) -> None:
        if msg.data:
            self._cancel_navigation(source="stop_movement")

    # ---------------- Cancel / hold ----------------

    def _cancel_navigation(self, source: str) -> None:
        self._query_pose()
        with self._lock:
            already_idle = self._goal_x is None and self._goal_y is None
            self._goal_x = None
            self._goal_y = None
            self._cached_path = None
            rx, ry, rz = self._robot_x, self._robot_y, self._robot_z
        now = time.time()
        self.way_point.publish(
            PointStamped(ts=now, frame_id=self.config.world_frame, x=rx, y=ry, z=rz)
        )
        self.goal_path.publish(
            Path(
                ts=now,
                frame_id=self.config.world_frame,
                poses=[
                    PoseStamped(
                        ts=now,
                        frame_id=self.config.world_frame,
                        position=[rx, ry, rz],
                        orientation=[0.0, 0.0, 0.0, 1.0],
                    )
                ],
            )
        )
        if not already_idle:
            logger.info("Goal cleared — idle until new goal", source=source)

    # ---------------- Planning ----------------

    def _plan_once(self) -> bool:
        """Run one A* plan from current pose to current goal. Returns True on success."""
        self._query_pose()
        with self._lock:
            if not self._has_odom or self._goal_x is None or self._goal_y is None:
                return False
            rx, ry = self._robot_x, self._robot_y
            gx, gy = self._goal_x, self._goal_y

        # Build binary OccupancyGrid window covering robot + goal.
        with self._costmap_lock:
            binary_grid = self._costmap.to_occupancy_grid(
                center_x=rx,
                center_y=ry,
                radius=self.config.window_radius,
                extra_points=[(gx, gy)],
                frame_id=self.config.world_frame,
            )

        if binary_grid.width == 0 or binary_grid.height == 0:
            return False

        # Apply gradient + inflation to get the soft-cost grid.
        robot_width = max(self.config.g.robot_width, 1e-3)
        try:
            costmap = make_navigation_map(
                binary_grid,
                robot_width * self.config.inflation_size,
                strategy="simple",
                gradient_strategy="voronoi",
            )
        except Exception as exc:
            logger.warning("make_navigation_map failed", exc_info=exc)
            return False

        # Force start and goal cells passable in case inflation made them lethal.
        # This is what SimplePlanner does (via is_blocked wrapper) and what the
        # original replanning_a_star does upstream via find_safe_goal — we do it
        # inline here since we own the costmap.
        threshold = self.config.cost_threshold
        for wx, wy in ((rx, ry), (gx, gy)):
            gp = costmap.world_to_grid((wx, wy))
            gx_i, gy_i = int(gp.x), int(gp.y)
            if 0 <= gx_i < costmap.width and 0 <= gy_i < costmap.height:
                if costmap.grid[gy_i, gx_i] >= threshold:
                    costmap.grid[gy_i, gx_i] = threshold - 1

        # Run A* in world coordinates.
        path = min_cost_astar(
            costmap,
            goal=(gx, gy),
            start=(rx, ry),
            cost_threshold=self.config.cost_threshold,
            unknown_penalty=self.config.unknown_penalty,
        )
        if path is None or not path.poses:
            logger.warning(
                "A* failed; holding position",
                robot=f"({rx:.2f},{ry:.2f})",
                goal=f"({gx:.2f},{gy:.2f})",
            )
            self._publish_hold(rx, ry, gx, gy)
            return False

        # Resample for smoothness.
        try:
            resampled = smooth_resample_path(
                path,
                Vector3(gx, gy, 0.0),
                self.config.path_resample_spacing,
            )
            path_world = [(p.x, p.y) for p in resampled.poses]
        except Exception:
            path_world = [(p.x, p.y) for p in path.poses]

        if not path_world:
            self._publish_hold(rx, ry, gx, gy)
            return False

        with self._lock:
            self._cached_path = path_world

        # Publish goal_path
        now = time.time()
        with self._lock:
            rz = self._robot_z
        poses = [
            PoseStamped(
                ts=now,
                frame_id=self.config.world_frame,
                position=[wx, wy, rz],
                orientation=[0.0, 0.0, 0.0, 1.0],
            )
            for wx, wy in path_world
        ]
        self.goal_path.publish(Path(ts=now, frame_id=self.config.world_frame, poses=poses))

        logger.info(
            "Replan ok",
            path_cells=len(path_world),
            robot=f"({rx:.2f},{ry:.2f})",
            goal=f"({gx:.2f},{gy:.2f})",
        )
        return True

    def _publish_hold(self, rx: float, ry: float, gx: float, gy: float) -> None:
        """A* failed — tell LocalPlanner to hold at the robot's current pose."""
        now = time.time()
        with self._lock:
            rz = self._robot_z
            self._cached_path = None
        self.way_point.publish(
            PointStamped(ts=now, frame_id=self.config.world_frame, x=rx, y=ry, z=rz)
        )
        self.goal_path.publish(
            Path(
                ts=now,
                frame_id=self.config.world_frame,
                poses=[
                    PoseStamped(
                        ts=now,
                        frame_id=self.config.world_frame,
                        position=[rx, ry, rz],
                        orientation=[0.0, 0.0, 0.0, 1.0],
                    ),
                    PoseStamped(
                        ts=now,
                        frame_id=self.config.world_frame,
                        position=[gx, gy, rz],
                        orientation=[0.0, 0.0, 0.0, 1.0],
                    ),
                ],
            )
        )

    # ---------------- Replan trigger logic ----------------

    def _try_replan(self) -> None:
        """Mirror of GlobalPlanner._replan_path — check ReplanLimiter, then plan."""
        with self._lock:
            if self._goal_x is None or self._goal_y is None:
                return
            rx, ry = self._robot_x, self._robot_y
            gx, gy = self._goal_x, self._goal_y

        if math.hypot(gx - rx, gy - ry) < self.config.replan_goal_tolerance:
            self._cancel_navigation(source="arrived")
            return

        if not self._replan_limiter.can_retry(Vector3(rx, ry, 0.0)):
            logger.info("Replan attempts exhausted — cancelling")
            self._cancel_navigation(source="replan_limit")
            return

        self._replan_limiter.will_retry()
        logger.info("Replanning", attempt=self._replan_limiter.get_attempt())
        self._plan_once()

    def _monitor_loop(self) -> None:
        """Mirror of GlobalPlanner._thread_entrypoint — deviation + stuck checks."""
        period = 1.0 / self.config.monitor_rate if self.config.monitor_rate > 0 else 0.1
        last_progress_pos: tuple[float, float] | None = None
        last_progress_time = time.perf_counter()

        while self._running:
            t0 = time.perf_counter()
            try:
                self._query_pose()

                with self._lock:
                    if not self._has_odom or self._goal_x is None or self._goal_y is None:
                        rx = ry = gx = gy = 0.0
                        cached = None
                        has_goal = False
                    else:
                        rx, ry = self._robot_x, self._robot_y
                        gx, gy = self._goal_x, self._goal_y
                        cached = list(self._cached_path) if self._cached_path else None
                        has_goal = True

                if not has_goal:
                    self._publish_costmap_cloud()
                    time.sleep(max(0.0, period - (time.perf_counter() - t0)))
                    continue

                self._publish_costmap_cloud()

                # 1) Goal-reached check
                if math.hypot(gx - rx, gy - ry) <= self.config.goal_reached_threshold:
                    logger.info("Close enough to goal. Accepting as arrived.")
                    self._cancel_navigation(source="goal_reached")
                    time.sleep(max(0.0, period - (time.perf_counter() - t0)))
                    continue

                # 2) Deviation check
                if cached is not None:
                    dev = _distance_point_to_polyline(rx, ry, cached)
                    if dev > self.config.max_path_deviation:
                        logger.info(
                            "Robot veered off track. Replanning.",
                            deviation=round(dev, 2),
                            threshold=self.config.max_path_deviation,
                        )
                        self._try_replan()
                        last_progress_pos = (rx, ry)
                        last_progress_time = time.perf_counter()
                        time.sleep(max(0.0, period - (time.perf_counter() - t0)))
                        continue

                # 3) Stuck check (progress + PositionTracker)
                if last_progress_pos is None:
                    last_progress_pos = (rx, ry)
                    last_progress_time = time.perf_counter()
                else:
                    moved = math.hypot(rx - last_progress_pos[0], ry - last_progress_pos[1])
                    if moved > self.config.stuck_threshold:
                        last_progress_pos = (rx, ry)
                        last_progress_time = time.perf_counter()

                if (
                    time.perf_counter() - last_progress_time > self.config.stuck_time_window
                    and self._position_tracker.is_stuck()
                ):
                    logger.info("Robot is stuck. Replanning.")
                    self._try_replan()
                    last_progress_pos = (rx, ry)
                    last_progress_time = time.perf_counter()

                # 4) Periodic replan if we have no cached path (e.g. A* failed previously
                # and is waiting for the costmap to refresh)
                if cached is None:
                    self._try_replan()

            except Exception as exc:
                logger.error("monitor_loop error", exc_info=exc)

            dt = time.perf_counter() - t0
            sleep = period - dt
            if sleep > 0:
                time.sleep(sleep)

    # ---------------- Waypoint loop (lookahead pursuit) ----------------

    def _waypoint_loop(self) -> None:
        period = 1.0 / self.config.waypoint_rate if self.config.waypoint_rate > 0 else 0.05
        while self._running:
            t0 = time.perf_counter()
            try:
                self._update_waypoint()
            except Exception as exc:
                logger.error("waypoint_loop error", exc_info=exc)
            dt = time.perf_counter() - t0
            sleep = period - dt
            if sleep > 0:
                time.sleep(sleep)

    def _update_waypoint(self) -> None:
        with self._lock:
            if not self._has_odom or self._goal_x is None or self._goal_y is None:
                return
            rx, ry = self._robot_x, self._robot_y
            gz = self._goal_z
            cached = list(self._cached_path) if self._cached_path else None
        if not cached:
            return
        wx, wy = self._lookahead(cached, rx, ry, self.config.lookahead_distance)
        now = time.time()
        self.way_point.publish(
            PointStamped(ts=now, frame_id=self.config.world_frame, x=wx, y=wy, z=gz)
        )

    @staticmethod
    def _lookahead(
        path: list[tuple[float, float]], rx: float, ry: float, distance: float
    ) -> tuple[float, float]:
        if not path:
            return (rx, ry)
        best_idx = 0
        best_d2 = float("inf")
        for i, (wx, wy) in enumerate(path):
            d2 = (wx - rx) ** 2 + (wy - ry) ** 2
            if d2 < best_d2:
                best_d2 = d2
                best_idx = i
        d2_target = distance * distance
        for i in range(best_idx, len(path)):
            wx, wy = path[i]
            if (wx - rx) ** 2 + (wy - ry) ** 2 >= d2_target:
                return (wx, wy)
        return path[-1]

    # ---------------- Costmap visualization ----------------

    def _publish_costmap_cloud(self) -> None:
        now = time.time()
        if now - self._last_costmap_pub < self.config.costmap_publish_period:
            return
        self._last_costmap_pub = now

        with self._lock:
            if not self._has_odom:
                return
            rx, ry, rz = self._robot_x, self._robot_y, self._robot_z
            gx_opt = self._goal_x
            gy_opt = self._goal_y

        extra = [(gx_opt, gy_opt)] if gx_opt is not None and gy_opt is not None else None

        with self._costmap_lock:
            binary_grid = self._costmap.to_occupancy_grid(
                center_x=rx,
                center_y=ry,
                radius=self.config.window_radius,
                extra_points=extra,
                frame_id=self.config.world_frame,
            )

        if binary_grid.width == 0 or binary_grid.height == 0:
            return

        grid_arr = binary_grid.grid
        occ_mask = grid_arr == int(CostValues.OCCUPIED)
        free_mask = grid_arr == int(CostValues.FREE)
        # We render OCC (red) and a thin inflation halo (orange) around it.
        # Inflation pixels = free cells adjacent to an OCC cell.
        if not np.any(occ_mask):
            return

        # Thin inflation: shift the OCC mask by 1 in each cardinal direction.
        inflation_mask = np.zeros_like(occ_mask, dtype=bool)
        inflation_mask[1:, :] |= occ_mask[:-1, :]
        inflation_mask[:-1, :] |= occ_mask[1:, :]
        inflation_mask[:, 1:] |= occ_mask[:, :-1]
        inflation_mask[:, :-1] |= occ_mask[:, 1:]
        inflation_mask &= free_mask  # don't paint inflation on UNK

        cell = binary_grid.resolution
        ox = binary_grid.origin.position.x
        oy = binary_grid.origin.position.y
        z = rz - self.config.ground_offset_below_robot + _COSTMAP_VIS_Z_LIFT

        occ_indices = np.argwhere(occ_mask)
        inf_indices = np.argwhere(inflation_mask)
        n_occ = len(occ_indices)
        n_inf = len(inf_indices)
        n_total = n_occ + n_inf
        if n_total == 0:
            return

        pts = np.empty((n_total, 3), dtype=np.float32)
        colors = np.empty((n_total, 3), dtype=np.float32)
        # occ_indices/inf_indices come as (y, x) since grid is [height, width]
        if n_occ > 0:
            pts[:n_occ, 0] = ox + (occ_indices[:, 1].astype(np.float32) + 0.5) * cell
            pts[:n_occ, 1] = oy + (occ_indices[:, 0].astype(np.float32) + 0.5) * cell
            pts[:n_occ, 2] = z
            colors[:n_occ] = _COLOR_OBSTACLE
        if n_inf > 0:
            pts[n_occ:, 0] = ox + (inf_indices[:, 1].astype(np.float32) + 0.5) * cell
            pts[n_occ:, 1] = oy + (inf_indices[:, 0].astype(np.float32) + 0.5) * cell
            pts[n_occ:, 2] = z
            colors[n_occ:] = _COLOR_INFLATION

        pcd_t = o3d.t.geometry.PointCloud()
        pcd_t.point["positions"] = o3c.Tensor(pts, dtype=o3c.float32)
        pcd_t.point["colors"] = o3c.Tensor(colors, dtype=o3c.float32)
        self.costmap_cloud.publish(
            PointCloud2(pointcloud=pcd_t, ts=now, frame_id=self.config.world_frame)
        )
