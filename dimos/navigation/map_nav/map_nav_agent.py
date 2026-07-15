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

"""Map-nav agent: milestones from MLS ``node_edges`` + MCP click skills.

When ``--map-for-agent`` is set, waits for MLS ``node_edges`` (the walkable
graph on the surface), farthest-point samples N unique edge endpoints, teleports
the plant to m1, and exposes ``@skill`` RPCs that publish ``clicked_point`` into
:class:`MovementManager`.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import threading
import time
from typing import Any

from dimos_lcm.std_msgs import Bool  # type: ignore[import-untyped]
import numpy as np
from numpy.typing import NDArray
from pydantic import Field
from reactivex.disposable import Disposable

from dimos.agents.annotation import skill
from dimos.agents.capabilities import CAP_MOVEMENT
from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.LineSegments3D import LineSegments3D
from dimos.msgs.nav_msgs.Path import Path
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


@dataclass(frozen=True)
class Milestone:
    """One agent-facing route point (1-based id); XYZ is on MLS node_edges."""

    id: int
    x: float
    y: float
    z: float


def points_from_node_edges(
    segments: list[tuple[tuple[float, float, float], tuple[float, float, float]]],
) -> NDArray[np.float64]:
    """Unique XYZ endpoints of MLS ``node_edges`` (walkable graph on the surface)."""
    if not segments:
        return np.zeros((0, 3), dtype=np.float64)
    seen: dict[tuple[int, int, int], tuple[float, float, float]] = {}
    for a, b in segments:
        for p in (a, b):
            key = (
                round(float(p[0]) * 100.0),
                round(float(p[1]) * 100.0),
                round(float(p[2]) * 100.0),
            )
            if key not in seen:
                seen[key] = (float(p[0]), float(p[1]), float(p[2]))
    if not seen:
        return np.zeros((0, 3), dtype=np.float64)
    return np.asarray(list(seen.values()), dtype=np.float64)


def sample_milestones_from_points(
    pts: NDArray[np.float64],
    n: int,
    *,
    seed_xyz: tuple[float, float, float] | None = None,
) -> list[Milestone]:
    """Farthest-point sample ``n`` milestones in 3D."""
    if n <= 0 or len(pts) == 0:
        return []
    n = min(n, len(pts))
    if seed_xyz is None:
        seed = np.array(
            [float(pts[:, 0].mean()), float(pts[:, 1].mean()), float(pts[:, 2].mean())],
            dtype=np.float64,
        )
    else:
        seed = np.asarray(seed_xyz, dtype=np.float64)
    d0 = np.sum((pts - seed) ** 2, axis=1)
    first = int(np.argmin(d0))
    chosen: list[int] = [first]
    min_d2 = np.sum((pts - pts[first]) ** 2, axis=1)
    min_d2[first] = -1.0
    while len(chosen) < n:
        nxt = int(np.argmax(min_d2))
        if min_d2[nxt] < 0:
            break
        chosen.append(nxt)
        d2 = np.sum((pts - pts[nxt]) ** 2, axis=1)
        min_d2 = np.minimum(min_d2, d2)
        min_d2[nxt] = -1.0
    return [
        Milestone(id=i + 1, x=float(pts[j, 0]), y=float(pts[j, 1]), z=float(pts[j, 2]))
        for i, j in enumerate(chosen)
    ]


def sample_milestones_from_node_edges(
    segments: list[tuple[tuple[float, float, float], tuple[float, float, float]]],
    n: int,
    *,
    seed_xyz: tuple[float, float, float],
) -> tuple[list[Milestone], tuple[float, float, float]]:
    """FPS on ``node_edges`` endpoints; m1 / home is the point nearest ``seed_xyz``."""
    pts = points_from_node_edges(segments)
    milestones = sample_milestones_from_points(pts, n, seed_xyz=seed_xyz)
    if not milestones:
        return [], seed_xyz
    home = (milestones[0].x, milestones[0].y, milestones[0].z)
    return milestones, home


def milestones_to_path(milestones: list[Milestone], *, frame_id: str) -> Path:
    """Path of milestone poses (index ``i`` → label ``m{i+1}`` in Rerun)."""
    path = Path(frame_id=frame_id, ts=time.time())
    for m in milestones:
        path.poses.append(
            PoseStamped(
                ts=path.ts,
                frame_id=frame_id,
                position=[m.x, m.y, m.z],
            )
        )
    return path


class MapNavAgentConfig(ModuleConfig):
    """Config for :class:`MapNavAgent`."""

    enabled: bool = Field(default_factory=lambda m: bool(m["g"].map_for_agent))
    n_milestones: int = Field(default_factory=lambda m: int(m["g"].map_milestones))
    frame_id: str = "world"
    body_height_m: float = 0.31
    goal_tolerance_m: float = 0.4
    per_goal_timeout_s: float = 120.0
    milestone_publish_hz: float = 1.0


class MapNavAgent(Module):
    """Sample milestones from MLS node_edges; MCP skills click goals for MLS/HTC."""

    config: MapNavAgentConfig

    node_edges: In[LineSegments3D]
    odom: In[PoseStamped]
    goal_reached: In[Bool]

    clicked_point: Out[PointStamped]
    milestones: Out[Path]
    stop_movement: Out[Bool]
    set_pose: Out[PoseStamped]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = threading.Lock()
        self._milestones: list[Milestone] = []
        self._built = False
        self._pending_edges: LineSegments3D | None = None
        self._odom: PoseStamped | None = None
        self._goal_event = threading.Event()
        self._stop_follow = threading.Event()
        self._follow_thread: threading.Thread | None = None
        self._viz_stop = threading.Event()
        self._viz_thread: threading.Thread | None = None
        self._milestone_path: Path | None = None

    @rpc
    def start(self) -> None:
        super().start()
        if not self.config.enabled:
            logger.info("MapNavAgent disabled (pass --map-for-agent to enable)")
            return
        self.register_disposable(Disposable(self.node_edges.subscribe(self._on_node_edges)))
        self.register_disposable(Disposable(self.odom.subscribe(self._on_odom)))
        self.register_disposable(Disposable(self.goal_reached.subscribe(self._on_goal_reached)))
        self._viz_stop.clear()
        self._viz_thread = threading.Thread(target=self._viz_loop, daemon=True)
        self._viz_thread.start()
        logger.info("MapNavAgent started", n_milestones=self.config.n_milestones)

    @rpc
    def stop(self) -> None:
        self._stop_follow.set()
        self._goal_event.set()
        self._viz_stop.set()
        if self._follow_thread is not None:
            self._follow_thread.join(DEFAULT_THREAD_JOIN_TIMEOUT)
            self._follow_thread = None
        if self._viz_thread is not None:
            self._viz_thread.join(DEFAULT_THREAD_JOIN_TIMEOUT)
            self._viz_thread = None
        super().stop()

    def _on_odom(self, msg: PoseStamped) -> None:
        with self._lock:
            self._odom = msg
        self._try_build_milestones()

    def _on_goal_reached(self, msg: Bool) -> None:
        if bool(getattr(msg, "data", msg)):
            self._goal_event.set()

    def _on_node_edges(self, edges: LineSegments3D) -> None:
        if len(edges) == 0:
            return
        with self._lock:
            if self._built:
                return
            self._pending_edges = edges
        self._try_build_milestones()

    def _try_build_milestones(self) -> None:
        with self._lock:
            if self._built:
                return
            odom = self._odom
            edges = self._pending_edges
            if odom is None or edges is None:
                return
            self._built = True
            self._pending_edges = None

        cfg = self.config
        z_floor = float(odom.z) - cfg.body_height_m
        seed_xyz = (float(odom.x), float(odom.y), z_floor)
        segs = list(edges._segments)
        milestones, home_xyz = sample_milestones_from_node_edges(
            segs, cfg.n_milestones, seed_xyz=seed_xyz
        )
        path_out = milestones_to_path(milestones, frame_id=cfg.frame_id)
        with self._lock:
            self._milestones = milestones
            self._milestone_path = path_out

        if milestones:
            self.set_pose.publish(
                PoseStamped(
                    ts=time.time(),
                    frame_id=cfg.frame_id,
                    position=[home_xyz[0], home_xyz[1], home_xyz[2] + cfg.body_height_m],
                    orientation=odom.orientation,
                )
            )
        logger.info(
            "MapNavAgent milestones ready",
            source="mls_node_edges",
            count=len(milestones),
            edge_points=len(points_from_node_edges(segs)),
            seed=seed_xyz,
            home=home_xyz,
            ids=[m.id for m in milestones],
            z_range=(
                (min(m.z for m in milestones), max(m.z for m in milestones)) if milestones else None
            ),
        )

    def _viz_loop(self) -> None:
        period = 1.0 / max(self.config.milestone_publish_hz, 0.1)
        while not self._viz_stop.wait(period):
            with self._lock:
                path = self._milestone_path
            if path is not None and len(path.poses) > 0:
                path.ts = time.time()
                self.milestones.publish(path)

    def _get_milestone(self, milestone_id: int) -> Milestone | None:
        with self._lock:
            for m in self._milestones:
                if m.id == milestone_id:
                    return m
        return None

    def _publish_goal(self, x: float, y: float, z: float) -> None:
        self._goal_event.clear()
        self.clicked_point.publish(
            PointStamped(
                ts=time.time(),
                frame_id=self.config.frame_id,
                x=float(x),
                y=float(y),
                z=float(z),
            )
        )

    def _near_goal(self, x: float, y: float) -> bool:
        with self._lock:
            odom = self._odom
        if odom is None:
            return False
        return math.hypot(float(odom.x) - x, float(odom.y) - y) <= self.config.goal_tolerance_m

    def _wait_arrival(self, x: float, y: float, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self._stop_follow.is_set():
                return False
            if self._near_goal(x, y):
                return True
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            if self._goal_event.wait(timeout=min(0.25, remaining)):
                self._goal_event.clear()
                if self._near_goal(x, y):
                    return True
        return self._near_goal(x, y)

    @skill
    def list_milestones(self) -> str:
        """List in-memory map milestones (id + xyz on MLS node_edges).

        Returns:
            JSON object with milestones and whether the map has been built.
        """
        with self._lock:
            built = self._built
            ms = list(self._milestones)
        return json.dumps(
            {
                "built": built,
                "count": len(ms),
                "milestones": [
                    {"id": m.id, "x": m.x, "y": m.y, "z": m.z, "label": f"m{m.id}"} for m in ms
                ],
            }
        )

    @skill(uses=[CAP_MOVEMENT])
    def go_to_milestone(self, milestone_id: int) -> str:
        """Navigate to a milestone by id (same as clicking that point in Rerun).

        Args:
            milestone_id: 1-based milestone id from list_milestones.

        Returns:
            Status string.
        """
        m = self._get_milestone(int(milestone_id))
        if m is None:
            with self._lock:
                ids = [x.id for x in self._milestones]
            return f"Unknown milestone_id={milestone_id}. Known ids={ids}."
        self._publish_goal(m.x, m.y, m.z)
        return (
            f"Clicked milestone m{m.id} at ({m.x:.2f}, {m.y:.2f}, {m.z:.2f}). "
            "MLS/holonomic will follow if a path exists. "
            "Call stop_navigation to cancel."
        )

    @skill(uses=[CAP_MOVEMENT])
    def go_to_point(self, x: float, y: float, z: float = 0.0) -> str:
        """Navigate to a world XYZ point (same as a Rerun 3D click).

        Args:
            x: World X (m).
            y: World Y (m).
            z: World Z surface height (m). Default 0.

        Returns:
            Status string.
        """
        self._publish_goal(float(x), float(y), float(z))
        return f"Clicked point ({x:.2f}, {y:.2f}, {z:.2f}). Call stop_navigation to cancel."

    @skill(uses=[CAP_MOVEMENT], lifecycle="background")
    def follow_milestones(self, milestone_ids: list[int]) -> str:
        """Follow a sequence of milestones (e.g. [1, 3, 7, 3, 4, 1]).

        Runs in the background: publishes each goal, waits for arrival (or timeout),
        then advances. Call stop_navigation to cancel.

        Args:
            milestone_ids: Ordered 1-based milestone ids to visit.

        Returns:
            Status string.
        """
        if not milestone_ids:
            return "milestone_ids is empty."
        unknown = [i for i in milestone_ids if self._get_milestone(int(i)) is None]
        if unknown:
            with self._lock:
                ids = [x.id for x in self._milestones]
            return f"Unknown milestone ids {unknown}. Known ids={ids}."

        self.start_tool("follow_milestones")
        self._stop_follow.set()
        if self._follow_thread is not None:
            self._follow_thread.join(DEFAULT_THREAD_JOIN_TIMEOUT)
        self._stop_follow.clear()
        seq = [int(i) for i in milestone_ids]
        self._follow_thread = threading.Thread(target=self._follow_loop, args=(seq,), daemon=True)
        self._follow_thread.start()
        return (
            f"Following milestones {[f'm{i}' for i in seq]} in background. "
            "Call stop_navigation to cancel."
        )

    def _follow_loop(self, milestone_ids: list[int]) -> None:
        try:
            for mid in milestone_ids:
                if self._stop_follow.is_set():
                    self.tool_update("follow_milestones", "cancelled")
                    return
                m = self._get_milestone(mid)
                if m is None:
                    self.tool_update("follow_milestones", f"missing milestone {mid}")
                    return
                self.tool_update(
                    "follow_milestones",
                    f"going to m{m.id} ({m.x:.2f}, {m.y:.2f}, {m.z:.2f})",
                )
                self._publish_goal(m.x, m.y, m.z)
                ok = self._wait_arrival(m.x, m.y, self.config.per_goal_timeout_s)
                if self._stop_follow.is_set():
                    self.tool_update("follow_milestones", "cancelled")
                    return
                if not ok:
                    self.tool_update(
                        "follow_milestones",
                        f"timeout waiting for m{m.id}",
                    )
                    return
            self.tool_update("follow_milestones", f"completed sequence {milestone_ids}")
        finally:
            self.stop_tool("follow_milestones")

    @skill
    def stop_navigation(self) -> str:
        """Cancel active milestone follow and stop holonomic path following."""
        self._stop_follow.set()
        self._goal_event.set()
        self.stop_movement.publish(Bool(data=True))
        self.clicked_point.publish(
            PointStamped(
                ts=time.time(),
                frame_id=self.config.frame_id,
                x=float("nan"),
                y=float("nan"),
                z=float("nan"),
            )
        )
        if self._follow_thread is not None:
            self._follow_thread.join(DEFAULT_THREAD_JOIN_TIMEOUT)
            self._follow_thread = None
        self.stop_tool("follow_milestones")
        return "Navigation cancelled."
