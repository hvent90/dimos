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

"""Free-space skills: deterministic reads of where the robot can physically be.

Four measurements over the live global costmap, all answered by geometry
rather than by the model's guesswork: how much free space surrounds a point
(``clearance_at``), where the closest free space is (``nearest_free``), how
far free space extends along a heading (``raycast``), and which free space is
available nearby (``free_space_near``).

They exist because "go next to the couch" is a placement problem, not a
naming problem: the agent needs metric free space to turn an object
position into a goal it can actually reach. Unknown space never counts as
free — an unexplored cell is not a standable cell.

The geometry itself lives in :mod:`dimos.mapping.occupancy.clearance` and
:mod:`dimos.mapping.occupancy.raycast`, where the planners can reach it too.
What is left here is the agent-facing half: holding the latest costmap,
turning degrees into radians, and phrasing an answer the model can act on.
"""

from __future__ import annotations

import math
import threading
from typing import Any

from reactivex.disposable import Disposable

from dimos.agents.annotation import skill
from dimos.agents.skill_result import CommonSkillError, SkillResult
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In
from dimos.mapping.occupancy import clearance
from dimos.mapping.occupancy.raycast import raycast as raycast_grid
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid

NO_MAP = "No occupancy map received yet — is mapping running?"


class FreeSpaceSkillContainer(Module):
    """Agent skills measuring free space in the robot's occupancy map."""

    global_costmap: In[OccupancyGrid]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._grid_lock = threading.Lock()
        self._latest_grid: OccupancyGrid | None = None

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

    @staticmethod
    def _map_bounds(grid: OccupancyGrid) -> str:
        ox, oy = grid.origin.position.x, grid.origin.position.y
        return (
            f"mapped area x [{ox:.2f}, {ox + grid.width * grid.resolution:.2f}], "
            f"y [{oy:.2f}, {oy + grid.height * grid.resolution:.2f}]"
        )

    @staticmethod
    def _clearance_value(clearance_m: float) -> float | None:
        """Metadata form of a clearance.

        None rather than infinity, which is what an obstacle-free map measures
        and what ``json.dumps`` would emit as a bare ``Infinity`` literal.
        """
        return None if math.isinf(clearance_m) else round(clearance_m, 3)

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
            return SkillResult.fail("INVALID_STATE", NO_MAP)
        state = clearance.cell_state(grid, x, y)
        if state is None:
            return SkillResult.fail(
                "INVALID_INPUT", f"({x:.2f}, {y:.2f}) is outside the {self._map_bounds(grid)}"
            )
        clearance_m = clearance.clearance_at(grid, x, y)
        assert clearance_m is not None  # same cell lookup already succeeded above
        return SkillResult.ok(
            f"({x:.2f}, {y:.2f}) is {state}; nearest obstacle is {clearance_m:.2f} m away.",
            state=state,
            clearance_m=self._clearance_value(clearance_m),
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
            return SkillResult.fail("INVALID_STATE", NO_MAP)
        point = clearance.nearest_free(grid, x, y, min_clearance)
        if point is None:
            return SkillResult.ok(
                f"No known-free cell with clearance >= {min_clearance} m exists in the "
                "current map.",
                found=False,
            )
        return SkillResult.ok(
            f"Nearest standable point to ({x:.2f}, {y:.2f}) with clearance >= "
            f"{min_clearance} m is ({point.x:.2f}, {point.y:.2f}), {point.distance_m:.2f} m "
            f"away (clearance {point.clearance_m:.2f} m).",
            found=True,
            point=[round(point.x, 3), round(point.y, 3)],
            distance_m=round(point.distance_m, 3),
            clearance_m=self._clearance_value(point.clearance_m),
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
            return SkillResult.fail("INVALID_STATE", NO_MAP)
        ray = raycast_grid(grid, x, y, math.radians(angle_deg), max_range_m)
        if ray is None:
            return SkillResult.fail(
                "INVALID_INPUT", f"({x:.2f}, {y:.2f}) is outside the {self._map_bounds(grid)}"
            )
        described = {
            "obstacle": f"hits an obstacle after {ray.distance_m:.2f} m",
            "unknown": f"enters unknown space after {ray.distance_m:.2f} m",
            "map_edge": f"leaves the mapped area after {ray.distance_m:.2f} m",
            "max_range": f"is clear for the full {ray.distance_m:.2f} m",
        }[ray.outcome]
        end = [round(ray.x, 3), round(ray.y, 3)]
        return SkillResult.ok(
            f"Ray from ({x:.2f}, {y:.2f}) at {angle_deg:.0f} deg {described}, "
            f"ending at ({end[0]:.2f}, {end[1]:.2f}).",
            outcome=ray.outcome,
            distance_m=round(ray.distance_m, 3),
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
            return SkillResult.fail("INVALID_STATE", NO_MAP)
        found = clearance.free_space_near(grid, x, y, radius, min_clearance, max_results)
        if not found:
            return SkillResult.ok(
                f"No standable point within {radius} m of ({x:.2f}, {y:.2f}) with "
                f"clearance >= {min_clearance} m — the area is occupied, unknown, or "
                "unmapped.",
                points=[],
            )
        points = [
            {
                "x": round(p.x, 3),
                "y": round(p.y, 3),
                "clearance_m": self._clearance_value(p.clearance_m),
                "distance_m": round(p.distance_m, 3),
            }
            for p in found
        ]
        best = points[0]
        return SkillResult.ok(
            f"{len(points)} standable point(s) within {radius} m of ({x:.2f}, {y:.2f}); "
            f"most open is ({best['x']}, {best['y']}) with {best['clearance_m']} m "
            "clearance.",
            points=points,
        )
