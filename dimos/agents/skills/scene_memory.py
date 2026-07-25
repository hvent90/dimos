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

"""Scene-memory skills: deterministic spatial queries over the occupancy map.

Four reads over the live global costmap, all answered by geometry rather
than by the model's guesswork: is a point free and how much room is around
it (``clearance_at``), where is the closest spot the robot could stand
(``nearest_free``), what stops a ray from a point along a heading
(``raycast``), and which nearby points are open enough to stand on
(``free_space_near``).

They exist because "go next to the couch" is a placement problem, not a
naming problem: the agent needs metric free space to turn an object
position into a goal it can actually reach. Unknown space never counts as
free — an unexplored cell is not a standable cell.
"""

from __future__ import annotations

import math
import threading
from typing import Any, cast

import numpy as np
from numpy.typing import NDArray
from reactivex.disposable import Disposable
from scipy import ndimage

from dimos.agents.annotation import skill
from dimos.agents.skill_result import CommonSkillError, SkillResult
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In
from dimos.msgs.nav_msgs.OccupancyGrid import CostValues, OccupancyGrid

# Costmap cells at or above this cost are obstacles for the map-query skills
# (the same threshold the planning gradient uses); unknown (-1) is never free.
OBSTACLE_THRESHOLD = 50


class SceneMemorySkillContainer(Module):
    """Agent skills reading the robot's occupancy map."""

    global_costmap: In[OccupancyGrid]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._grid_lock = threading.Lock()
        self._latest_grid: OccupancyGrid | None = None
        # Distance-to-nearest-obstacle field, recomputed when the grid changes.
        self._clearance_cache: tuple[OccupancyGrid, NDArray[np.float64]] | None = None

    @rpc
    def start(self) -> None:
        super().start()
        if self.global_costmap.transport:
            self.register_disposable(Disposable(self.global_costmap.subscribe(self._on_costmap)))

    @rpc
    def stop(self) -> None:
        super().stop()

    def _on_costmap(self, grid: OccupancyGrid) -> None:
        with self._grid_lock:
            self._latest_grid = grid

    def _grid_or_none(self) -> OccupancyGrid | None:
        with self._grid_lock:
            return self._latest_grid

    def _clearance_field(self, grid: OccupancyGrid) -> NDArray[np.float64]:
        """Distance in meters from each cell to the nearest obstacle cell."""
        with self._grid_lock:
            cached = self._clearance_cache
        if cached is not None and cached[0] is grid:
            return cached[1]
        obstacles = grid.grid >= OBSTACLE_THRESHOLD
        field = (
            cast("NDArray[np.float64]", ndimage.distance_transform_edt(~obstacles))
            * grid.resolution
        )
        with self._grid_lock:
            self._clearance_cache = (grid, field)
        return field

    @staticmethod
    def _cell_index(grid: OccupancyGrid, x: float, y: float) -> tuple[int, int] | None:
        """(col, row) of the cell containing the world point; None if off-map."""
        g = grid.world_to_grid((x, y, 0.0))
        gx, gy = math.floor(g.x), math.floor(g.y)
        if not (0 <= gx < grid.width and 0 <= gy < grid.height):
            return None
        return gx, gy

    @staticmethod
    def _cell_state(value: int) -> str:
        if value == CostValues.UNKNOWN:
            return "unknown"
        return "occupied" if value >= OBSTACLE_THRESHOLD else "free"

    @staticmethod
    def _map_bounds(grid: OccupancyGrid) -> str:
        ox, oy = grid.origin.position.x, grid.origin.position.y
        return (
            f"mapped area x [{ox:.2f}, {ox + grid.width * grid.resolution:.2f}], "
            f"y [{oy:.2f}, {oy + grid.height * grid.resolution:.2f}]"
        )

    @staticmethod
    def _free_cell_coords(
        grid: OccupancyGrid, mask: NDArray[np.bool_]
    ) -> tuple[NDArray[np.intp], NDArray[np.intp], NDArray[np.float64], NDArray[np.float64]]:
        """(rows, cols, world_x, world_y) cell centers of the masked cells."""
        rows, cols = np.nonzero(mask)
        ox, oy = grid.origin.position.x, grid.origin.position.y
        xs = ox + (cols.astype(np.float64) + 0.5) * grid.resolution
        ys = oy + (rows.astype(np.float64) + 0.5) * grid.resolution
        return rows, cols, xs, ys

    @skill
    def clearance_at(self, x: float, y: float) -> SkillResult[CommonSkillError]:
        """Is a world point free, and how far is the nearest obstacle?

        Deterministic read of the live occupancy map: the point's cell state
        (free / occupied / unknown) plus its clearance — the straight-line
        distance to the nearest obstacle cell. A robot needs clearance of at
        least its own half-width to stand at a point.

        Args:
            x: World x in meters.
            y: World y in meters.
        """
        grid = self._grid_or_none()
        if grid is None:
            return SkillResult.fail(
                "INVALID_STATE", "No occupancy map received yet — is mapping running?"
            )
        cell = self._cell_index(grid, x, y)
        if cell is None:
            return SkillResult.fail(
                "INVALID_INPUT", f"({x:.2f}, {y:.2f}) is outside the {self._map_bounds(grid)}"
            )
        gx, gy = cell
        state = self._cell_state(int(grid.grid[gy, gx]))
        clearance = float(self._clearance_field(grid)[gy, gx])
        return SkillResult.ok(
            f"({x:.2f}, {y:.2f}) is {state}; nearest obstacle is {clearance:.2f} m away.",
            state=state,
            clearance_m=round(clearance, 3),
        )

    @skill
    def nearest_free(
        self, x: float, y: float, min_clearance: float = 0.3
    ) -> SkillResult[CommonSkillError]:
        """Nearest known-free point with at least the given obstacle clearance.

        Turns a rough target (an object position, a spot beside its extent)
        into a standable point: the closest mapped-free cell whose distance
        to every obstacle is at least min_clearance. Unknown space never
        qualifies.

        Args:
            x: World x of the target, in meters.
            y: World y of the target, in meters.
            min_clearance: Required obstacle clearance in meters.
        """
        if min_clearance <= 0:
            return SkillResult.fail("INVALID_INPUT", "min_clearance must be positive")
        grid = self._grid_or_none()
        if grid is None:
            return SkillResult.fail(
                "INVALID_STATE", "No occupancy map received yet — is mapping running?"
            )
        field = self._clearance_field(grid)
        mask = (
            (grid.grid != CostValues.UNKNOWN)
            & (grid.grid < OBSTACLE_THRESHOLD)
            & (field >= min_clearance)
        )
        if not mask.any():
            return SkillResult.ok(
                f"No known-free cell with clearance >= {min_clearance} m exists in the "
                "current map.",
                found=False,
            )
        rows, cols, xs, ys = self._free_cell_coords(grid, mask)
        i = int(np.argmin((xs - x) ** 2 + (ys - y) ** 2))
        px, py = float(xs[i]), float(ys[i])
        distance = float(math.hypot(px - x, py - y))
        clearance = float(field[rows[i], cols[i]])
        return SkillResult.ok(
            f"Nearest standable point to ({x:.2f}, {y:.2f}) with clearance >= "
            f"{min_clearance} m is ({px:.2f}, {py:.2f}), {distance:.2f} m away "
            f"(clearance {clearance:.2f} m).",
            found=True,
            point=[round(px, 3), round(py, 3)],
            distance_m=round(distance, 3),
            clearance_m=round(clearance, 3),
        )

    @skill
    def raycast(
        self, x: float, y: float, angle_deg: float, max_range_m: float = 10.0
    ) -> SkillResult[CommonSkillError]:
        """How far is it from a point along a heading until something blocks?

        Marches the live occupancy map from (x, y) along angle_deg (0 = +x,
        counterclockwise) and reports what ends the ray: an obstacle,
        unknown space, the map edge, or nothing within max_range_m.

        Args:
            x: Ray origin world x, in meters.
            y: Ray origin world y, in meters.
            angle_deg: Heading in degrees (0 = +x, 90 = +y).
            max_range_m: Give up after this distance, in meters.
        """
        if max_range_m <= 0:
            return SkillResult.fail("INVALID_INPUT", "max_range_m must be positive")
        grid = self._grid_or_none()
        if grid is None:
            return SkillResult.fail(
                "INVALID_STATE", "No occupancy map received yet — is mapping running?"
            )
        if self._cell_index(grid, x, y) is None:
            return SkillResult.fail(
                "INVALID_INPUT", f"({x:.2f}, {y:.2f}) is outside the {self._map_bounds(grid)}"
            )
        step = grid.resolution / 2.0
        direction = (math.cos(math.radians(angle_deg)), math.sin(math.radians(angle_deg)))
        outcome, distance = "max_range", max_range_m
        for i in range(1, int(max_range_m / step) + 1):
            d = i * step
            cell = self._cell_index(grid, x + direction[0] * d, y + direction[1] * d)
            if cell is None:
                outcome, distance = "map_edge", d
                break
            value = int(grid.grid[cell[1], cell[0]])
            if value == CostValues.UNKNOWN:
                outcome, distance = "unknown", d
                break
            if value >= OBSTACLE_THRESHOLD:
                outcome, distance = "obstacle", d
                break
        end = [round(x + direction[0] * distance, 3), round(y + direction[1] * distance, 3)]
        described = {
            "obstacle": f"hits an obstacle after {distance:.2f} m",
            "unknown": f"enters unknown space after {distance:.2f} m",
            "map_edge": f"leaves the mapped area after {distance:.2f} m",
            "max_range": f"is clear for the full {distance:.2f} m",
        }[outcome]
        return SkillResult.ok(
            f"Ray from ({x:.2f}, {y:.2f}) at {angle_deg:.0f} deg {described}, "
            f"ending at ({end[0]:.2f}, {end[1]:.2f}).",
            outcome=outcome,
            distance_m=round(distance, 3),
            end=end,
        )

    @skill
    def free_space_near(
        self,
        x: float,
        y: float,
        radius: float = 2.0,
        min_clearance: float = 0.3,
        max_results: int = 8,
    ) -> SkillResult[CommonSkillError]:
        """Standable points near a target, most open first. Deterministic.

        Finds known-free cells within radius of (x, y) whose obstacle
        clearance is at least min_clearance, and returns up to max_results
        of them ranked by clearance, spaced at least 0.4 m apart. Combine
        with an object's position and extent to pick a spot beside it.

        Args:
            x: Target world x, in meters.
            y: Target world y, in meters.
            radius: Search radius in meters.
            min_clearance: Required obstacle clearance in meters.
            max_results: Maximum points to return (1-50).
        """
        if radius <= 0 or min_clearance <= 0:
            return SkillResult.fail("INVALID_INPUT", "radius and min_clearance must be positive")
        if not 1 <= max_results <= 50:
            return SkillResult.fail("INVALID_INPUT", "max_results must be between 1 and 50")
        grid = self._grid_or_none()
        if grid is None:
            return SkillResult.fail(
                "INVALID_STATE", "No occupancy map received yet — is mapping running?"
            )
        field = self._clearance_field(grid)
        mask = (
            (grid.grid != CostValues.UNKNOWN)
            & (grid.grid < OBSTACLE_THRESHOLD)
            & (field >= min_clearance)
        )
        rows, cols, xs, ys = self._free_cell_coords(grid, mask)
        distances = np.hypot(xs - x, ys - y)
        within = distances <= radius
        rows, cols, xs, ys = rows[within], cols[within], xs[within], ys[within]
        distances = distances[within]
        clearances = field[rows, cols]
        # Rank by openness; ties resolve by distance then row-major cell order.
        order = np.lexsort((cols, rows, distances, -clearances))
        points: list[dict[str, float]] = []
        kept_xy: list[tuple[float, float]] = []
        for i in order:
            px, py = float(xs[i]), float(ys[i])
            if any(math.hypot(px - kx, py - ky) < 0.4 for kx, ky in kept_xy):
                continue
            kept_xy.append((px, py))
            points.append(
                {
                    "x": round(px, 3),
                    "y": round(py, 3),
                    "clearance_m": round(float(clearances[i]), 3),
                    "distance_m": round(float(distances[i]), 3),
                }
            )
            if len(points) >= max_results:
                break
        if not points:
            return SkillResult.ok(
                f"No standable point within {radius} m of ({x:.2f}, {y:.2f}) with "
                f"clearance >= {min_clearance} m — the area is occupied, unknown, or "
                "unmapped.",
                points=[],
            )
        best = points[0]
        return SkillResult.ok(
            f"{len(points)} standable point(s) within {radius} m of ({x:.2f}, {y:.2f}); "
            f"most open is ({best['x']}, {best['y']}) with {best['clearance_m']} m "
            "clearance.",
            points=points,
        )
