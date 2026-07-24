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

"""Room-curation skills: view the map as an image, fix and name room geometry.

The agent's eyes and hands for room geometry, mixed into
:class:`dimos.agents.skills.scene_memory.SceneMemorySkillContainer` (split
out of that module for size). ``view_map`` renders the occupancy grid with
room polygons over a labeled metric grid so the agent can read world
coordinates off the picture; rename/boundary/merge/split write agent edits
into the scene graph, where they survive automatic re-derivation (identity
matching + the derive_rooms force gate).
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import threading
import time
from typing import TYPE_CHECKING, Any, cast

import numpy as np
from numpy.typing import NDArray
from scipy import ndimage

from dimos.agents.annotation import skill
from dimos.agents.skill_result import CommonSkillError, SkillResult
from dimos.mapping.occupancy.map_render import MapMarker, MapRegion, grid_step_m, render_map
from dimos.mapping.occupancy.polygons import (
    mask_to_polygon,
    points_in_polygon,
    polygon_from_flat,
)
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.perception.scene_graph import RegionGeometry, RegionSpec, SceneGraph

if TYPE_CHECKING:
    from dimos.agents.skills.scene_memory import PoseTrail


@dataclass
class MapViewResult(SkillResult[CommonSkillError]):
    """A SkillResult that also carries the rendered map image.

    ``agent_encode`` appends the image as its own content block after the
    JSON text block; the MCP client hoists non-text blocks into the agent's
    context as vision input (see McpClient._mcp_tool_to_langchain).
    """

    image: Image | None = None

    def agent_encode(self) -> list[dict[str, Any]]:
        blocks = super().agent_encode()
        if self.image is not None:
            # Image.agent_encode is annotated as one AgentImageMessage but
            # returns a list of content blocks.
            blocks.extend(cast("list[dict[str, Any]]", self.image.agent_encode()))
        return blocks


def _polygon_cell_mask(grid: OccupancyGrid, polygon: NDArray[np.float64]) -> NDArray[np.bool_]:
    """Grid-shaped mask of the cells whose centers fall inside the polygon."""
    ox, oy = float(grid.origin.position.x), float(grid.origin.position.y)
    res = grid.resolution
    c0 = int(np.clip(np.floor((polygon[:, 0].min() - ox) / res) - 1, 0, grid.width))
    c1 = int(np.clip(np.ceil((polygon[:, 0].max() - ox) / res) + 1, 0, grid.width))
    r0 = int(np.clip(np.floor((polygon[:, 1].min() - oy) / res) - 1, 0, grid.height))
    r1 = int(np.clip(np.ceil((polygon[:, 1].max() - oy) / res) + 1, 0, grid.height))
    mask = np.zeros((grid.height, grid.width), dtype=bool)
    if c0 >= c1 or r0 >= r1:
        return mask
    xs = ox + (np.arange(c0, c1, dtype=np.float64) + 0.5) * res
    ys = oy + (np.arange(r0, r1, dtype=np.float64) + 0.5) * res
    gx, gy = np.meshgrid(xs, ys)
    inside = points_in_polygon(np.column_stack([gx.ravel(), gy.ravel()]), polygon)
    mask[r0:r1, c0:c1] = inside.reshape(gy.shape)
    return mask


def _largest_component(mask: NDArray[np.bool_]) -> NDArray[np.bool_]:
    labels, count = ndimage.label(mask)
    if count <= 1:
        return mask
    sizes = ndimage.sum_labels(mask, labels, index=range(1, count + 1))
    keep: NDArray[np.bool_] = labels == (1 + int(np.argmax(sizes)))
    return keep


def _mask_geometry(
    mask: NDArray[np.bool_], grid: OccupancyGrid, polygon: NDArray[np.float64] | None = None
) -> RegionGeometry:
    """Region geometry measured over the mask's cells.

    The anchor is the mask's most open cell (max distance to the mask edge)
    — the same rule room segmentation uses. ``polygon`` overrides the
    outline (an agent-authored boundary is authoritative; the mask only
    measures it).
    """
    res = grid.resolution
    ox, oy = float(grid.origin.position.x), float(grid.origin.position.y)
    area = float(mask.sum()) * res * res
    rows, cols = np.nonzero(mask)
    centroid = (
        ox + (float(cols.mean()) + 0.5) * res,
        oy + (float(rows.mean()) + 0.5) * res,
    )
    edt = cast("NDArray[np.float64]", ndimage.distance_transform_edt(mask)) * res
    anchor_row, anchor_col = np.unravel_index(int(np.argmax(edt)), mask.shape)
    return RegionGeometry(
        polygon=polygon if polygon is not None else mask_to_polygon(mask, res, (ox, oy)),
        area_m2=round(area, 1),
        centroid_xy=(round(centroid[0], 3), round(centroid[1], 3)),
        anchor_xy=(
            ox + (float(anchor_col) + 0.5) * res,
            oy + (float(anchor_row) + 0.5) * res,
        ),
        max_clearance_m=round(float(edt[anchor_row, anchor_col]), 2),
    )


def _analytic_geometry(polygon: NDArray[np.float64]) -> RegionGeometry:
    """Shoelace-based geometry when no occupancy grid is available."""
    x, y = polygon[:, 0], polygon[:, 1]
    x2, y2 = np.roll(x, -1), np.roll(y, -1)
    cross = x * y2 - x2 * y
    area2 = float(cross.sum())
    if abs(area2) < 1e-9:
        centroid = (float(x.mean()), float(y.mean()))
    else:
        centroid = (
            float(((x + x2) * cross).sum() / (3.0 * area2)),
            float(((y + y2) * cross).sum() / (3.0 * area2)),
        )
    return RegionGeometry(
        polygon=polygon,
        area_m2=round(abs(area2) / 2.0, 1),
        centroid_xy=(round(centroid[0], 3), round(centroid[1], 3)),
        anchor_xy=centroid,
        max_clearance_m=0.0,
    )


class RoomCurationSkills:
    """Mixin holding the room-curation skills; the container provides state.

    The declarations under TYPE_CHECKING are the container internals the
    skills use — they exist only for the type checker, so the mixin adds no
    runtime attributes that could shadow the container's own.
    """

    if TYPE_CHECKING:
        _mutate_lock: threading.Lock

        def _grid_or_none(self) -> OccupancyGrid | None: ...
        def _graph(self) -> SceneGraph: ...
        def _ensure_rooms(self, graph: SceneGraph) -> str: ...
        def _trail_or_none(self) -> PoseTrail | None: ...
        def _publish_graph(self, graph: SceneGraph, ts: float) -> None: ...

    @skill
    def view_map(self, bounds: list[float] | None = None) -> SkillResult[CommonSkillError]:
        """Render the occupancy map with rooms, objects and the robot as an image.

        A top-down floor plan built for coordinate reasoning: metric
        gridlines labeled in world meters, each room's tinted polygon with
        id and name, object markers, and the robot pose. The same geometry
        is in the JSON metadata (exact room polygon vertices). Look at it
        before fixing room geometry (set_room_boundary, merge_rooms,
        split_room, rename_room) and again afterwards to verify; zoom in
        for finer gridlines and per-vertex coordinate tags.

        Args:
            bounds: Optional [x_min, y_min, x_max, y_max] world-meter crop
                to zoom into one area; omit for the full map.
        """
        grid = self._grid_or_none()
        if grid is None:
            return SkillResult.fail(
                "INVALID_STATE", "No occupancy map received yet — is mapping running?"
            )
        crop: tuple[float, float, float, float] | None = None
        if bounds is not None:
            if len(bounds) != 4 or bounds[0] >= bounds[2] or bounds[1] >= bounds[3]:
                return SkillResult.fail(
                    "INVALID_INPUT",
                    "bounds must be [x_min, y_min, x_max, y_max] with min < max",
                )
            crop = (bounds[0], bounds[1], bounds[2], bounds[3])

        with self._graph() as graph:
            note = self._ensure_rooms(graph)
            regions = graph.regions()
            objects = [
                n for n in graph.nodes(layer="object") if n.position is not None and not n.retired
            ]
            rooms_meta = [
                {
                    "id": r.id,
                    "name": r.name,
                    "kind": r.layer,
                    "area_m2": r.metadata.get("area_m2"),
                    "origin": r.metadata.get("origin", "derived"),
                    "polygon": [[round(float(x), 2), round(float(y), 2)] for x, y in r.polygon()],
                }
                for r in regions
                if r.extent is not None
            ]
            objects_meta = [
                {
                    "id": n.id,
                    "name": n.name,
                    "x": round(n.xy[0], 2),
                    "y": round(n.xy[1], 2),
                    "room": graph.parent_id(n.id),
                }
                for n in objects
            ]

        agent_xy: tuple[float, float] | None = None
        agent_heading: float | None = None
        trail = self._trail_or_none()
        if trail is not None and len(trail.ts):
            agent_xy = (float(trail.xy[-1, 0]), float(trail.xy[-1, 1]))
            if len(trail.ts) >= 2:
                dx = trail.xy[-1, 0] - trail.xy[-2, 0]
                dy = trail.xy[-1, 1] - trail.xy[-2, 1]
                if abs(dx) > 1e-6 or abs(dy) > 1e-6:
                    agent_heading = math.atan2(dy, dx)

        try:
            img = render_map(
                grid,
                regions=[
                    MapRegion(id=r.id, name=r.name, kind=r.layer, polygon=r.polygon())
                    for r in regions
                    if r.extent is not None
                ],
                markers=[MapMarker(xy=n.xy, label=n.name) for n in objects],
                agent_xy=agent_xy,
                agent_heading=agent_heading,
                bounds=crop,
            )
        except ValueError as e:
            return SkillResult.fail("INVALID_INPUT", str(e))

        x0, y0, x1, y1 = (
            crop
            if crop is not None
            else (
                float(grid.origin.position.x),
                float(grid.origin.position.y),
                float(grid.origin.position.x) + grid.width * grid.resolution,
                float(grid.origin.position.y) + grid.height * grid.resolution,
            )
        )
        step = grid_step_m(max(x1 - x0, y1 - y0))
        message = (
            f"Rendered map view x [{x0:.1f}, {x1:.1f}], y [{y0:.1f}, {y1:.1f}] m: "
            f"{len(rooms_meta)} region(s), {len(objects_meta)} object(s). "
            f"Gridlines every {step:g} m; exact room polygons are in metadata."
        )
        if note:
            message += f" {note}"
        return MapViewResult(
            success=True,
            message=message,
            metadata={
                "view_bounds": [round(v, 2) for v in (x0, y0, x1, y1)],
                "grid_step_m": step,
                "resolution_m": grid.resolution,
                "rooms": rooms_meta,
                "objects": objects_meta,
                "agent": list(agent_xy) if agent_xy is not None else None,
            },
            image=Image(data=img, format=ImageFormat.BGR, ts=time.time()),
        )

    @skill
    def rename_room(self, room_id: str, name: str) -> SkillResult[CommonSkillError]:
        """Rename a room or corridor; the name survives future derivations.

        Args:
            room_id: Region node id, e.g. "room_3".
            name: Human name for it, e.g. "kitchen".
        """
        name = name.strip()
        if not name:
            return SkillResult.fail("INVALID_INPUT", "name must not be empty")
        ts = time.time()
        with self._mutate_lock, self._graph() as graph:
            try:
                renamed = graph.rename_region(room_id, name, ts)
            except KeyError as e:
                return SkillResult.fail("INVALID_INPUT", str(e.args[0]))
            self._publish_graph(graph, ts=ts)
        return SkillResult.ok(
            f"{room_id} is now named '{renamed.name}'.", room_id=room_id, name=renamed.name
        )

    @skill
    def set_room_boundary(
        self, room_id: str, polygon: list[float]
    ) -> SkillResult[CommonSkillError]:
        """Replace a room's outline polygon with corrected geometry.

        The room keeps its id and name; objects and sighting history
        re-check which room contains them. Agent-edited geometry is
        preserved against automatic derivation (derive_rooms refuses
        without force=true). Read coordinates off view_map first.

        Args:
            room_id: Region node id, e.g. "room_3".
            polygon: Flat [x1, y1, x2, y2, ...] world-meter outline,
                at least 3 vertices in order.
        """
        try:
            outline = polygon_from_flat(polygon)
        except ValueError as e:
            return SkillResult.fail("INVALID_INPUT", str(e))
        grid = self._grid_or_none()
        ts = time.time()
        with self._mutate_lock, self._graph() as graph:
            mask = _polygon_cell_mask(grid, outline) if grid is not None else None
            geometry = (
                _mask_geometry(mask, grid, polygon=outline)
                if grid is not None and mask is not None and bool(mask.any())
                else _analytic_geometry(outline)
            )
            try:
                node, moved, rewritten = graph.set_region_geometry(room_id, geometry, ts)
            except KeyError as e:
                return SkillResult.fail("INVALID_INPUT", str(e.args[0]))
            self._publish_graph(graph, ts=ts)
        return SkillResult.ok(
            f"{room_id} boundary replaced ({geometry.area_m2} m^2, "
            f"{len(outline)} vertices); {moved} object(s) changed rooms, "
            f"{rewritten} sighting row(s) reassigned. Automatic derivation "
            "will preserve this edit.",
            room_id=node.id,
            area_m2=geometry.area_m2,
            objects_moved=moved,
            sightings_reassigned=rewritten,
        )

    @skill
    def merge_rooms(self, room_ids: list[str], name: str = "") -> SkillResult[CommonSkillError]:
        """Merge adjacent rooms into one (fixes over-segmentation).

        The merged room gets a fresh id (reported back); objects and
        sighting history move to it, and doorways to outside rooms carry
        over. Preserved against automatic derivation like all agent edits.

        Args:
            room_ids: Two or more region node ids, e.g. ["room_2", "room_3"].
            name: Optional name for the merged room.
        """
        ids = list(dict.fromkeys(room_ids))
        if len(ids) < 2:
            return SkillResult.fail("INVALID_INPUT", "merge_rooms needs at least two distinct ids")
        grid = self._grid_or_none()
        if grid is None:
            return SkillResult.fail(
                "INVALID_STATE", "No occupancy map received yet — merging needs the map geometry."
            )
        ts = time.time()
        with self._mutate_lock, self._graph() as graph:
            try:
                nodes = [graph.region_or_raise(node_id) for node_id in ids]
            except KeyError as e:
                return SkillResult.fail("INVALID_INPUT", str(e.args[0]))
            masks = [_polygon_cell_mask(grid, n.polygon()) for n in nodes]
            union = np.logical_or.reduce(masks)
            closed = cast(
                "NDArray[np.bool_]",
                ndimage.binary_closing(union, structure=np.ones((3, 3)), iterations=2),
            )
            labels, _ = ndimage.label(closed)
            label_sets = [set(np.unique(labels[m])) - {0} for m in masks]
            common = set.intersection(*label_sets) if label_sets else set()
            if not common:
                return SkillResult.fail(
                    "INVALID_INPUT",
                    f"{', '.join(ids)} are not contiguous on the map — merge needs adjacent rooms.",
                )
            merged_mask = labels == min(common)
            geometry = _mask_geometry(merged_mask, grid)
            kind = "corridor" if all(n.layer == "corridor" for n in nodes) else "room"
            created, moved, rewritten = graph.replace_regions(
                ids, [RegionSpec(kind=kind, name=name.strip(), geometry=geometry)], ts
            )
            self._publish_graph(graph, ts=ts)
        new = created[0]
        label = f"{new.id} ('{new.name}')" if new.name != new.id else new.id
        return SkillResult.ok(
            f"Merged {', '.join(ids)} into {label} ({geometry.area_m2} m^2); "
            f"{moved} object(s) and {rewritten} sighting row(s) moved with it. "
            "Automatic derivation will preserve this edit.",
            merged_into=new.id,
            name=new.name,
            area_m2=geometry.area_m2,
            objects_moved=moved,
            sightings_reassigned=rewritten,
        )

    @skill
    def split_room(
        self, room_id: str, line: list[float], names: list[str] | None = None
    ) -> SkillResult[CommonSkillError]:
        """Split a room in two along a straight line (fixes under-segmentation).

        The line extends across the whole room; each side becomes a new
        room (fresh ids, reported back) joined by an adjacency where the
        line crosses. Objects and sighting history re-check containment.
        Preserved against automatic derivation like all agent edits.

        Args:
            room_id: Region node id to split, e.g. "room_3".
            line: [x0, y0, x1, y1] — two world-meter points on the
                dividing line (read them off view_map).
            names: Optional names for the two halves: first the half left
                of the line direction (x0,y0)->(x1,y1), then the right.
        """
        if len(line) != 4:
            return SkillResult.fail("INVALID_INPUT", "line must be [x0, y0, x1, y1]")
        p0 = np.asarray(line[:2], dtype=np.float64)
        p1 = np.asarray(line[2:], dtype=np.float64)
        if float(np.hypot(*(p1 - p0))) < 1e-6:
            return SkillResult.fail("INVALID_INPUT", "line endpoints must differ")
        wanted_names = [n.strip() for n in (names or [])]
        if len(wanted_names) > 2:
            return SkillResult.fail("INVALID_INPUT", "names takes at most two entries")
        wanted_names += [""] * (2 - len(wanted_names))
        grid = self._grid_or_none()
        if grid is None:
            return SkillResult.fail(
                "INVALID_STATE", "No occupancy map received yet — splitting needs the map."
            )
        ts = time.time()
        with self._mutate_lock, self._graph() as graph:
            try:
                node = graph.region_or_raise(room_id)
            except KeyError as e:
                return SkillResult.fail("INVALID_INPUT", str(e.args[0]))
            mask = _polygon_cell_mask(grid, node.polygon())
            rows, cols = np.nonzero(mask)
            ox, oy = float(grid.origin.position.x), float(grid.origin.position.y)
            xs = ox + (cols.astype(np.float64) + 0.5) * grid.resolution
            ys = oy + (rows.astype(np.float64) + 0.5) * grid.resolution
            cross = (p1[0] - p0[0]) * (ys - p0[1]) - (p1[1] - p0[1]) * (xs - p0[0])
            side_a = np.zeros_like(mask)
            side_b = np.zeros_like(mask)
            side_a[rows[cross > 0], cols[cross > 0]] = True
            side_b[rows[cross <= 0], cols[cross <= 0]] = True
            side_a = _largest_component(side_a)
            side_b = _largest_component(side_b)
            min_cells = max(4, int(0.25 / (grid.resolution * grid.resolution)))
            if side_a.sum() < min_cells or side_b.sum() < min_cells:
                return SkillResult.fail(
                    "INVALID_INPUT",
                    f"The line does not divide {room_id} — both sides need map area.",
                )
            specs = [
                RegionSpec(
                    kind=node.layer, name=wanted_names[0], geometry=_mask_geometry(side_a, grid)
                ),
                RegionSpec(
                    kind=node.layer, name=wanted_names[1], geometry=_mask_geometry(side_b, grid)
                ),
            ]
            centroid = node.polygon().mean(axis=0)
            direction = p1 - p0
            t = float(np.dot(centroid - p0, direction) / np.dot(direction, direction))
            doorway_xy = p0 + t * direction
            seam_pairs = int(
                (side_a[1:, :] & side_b[:-1, :]).sum()
                + (side_a[:-1, :] & side_b[1:, :]).sum()
                + (side_a[:, 1:] & side_b[:, :-1]).sum()
                + (side_a[:, :-1] & side_b[:, 1:]).sum()
            )
            created, moved, rewritten = graph.replace_regions([room_id], specs, ts)
            graph.link_adjacent(
                created[0].id,
                created[1].id,
                (float(doorway_xy[0]), float(doorway_xy[1])),
                round(seam_pairs * grid.resolution, 2),
                ts,
            )
            self._publish_graph(graph, ts=ts)
        described = ", ".join(
            f"{n.id} ('{n.name}', {n.metadata['area_m2']} m^2)"
            if n.name != n.id
            else f"{n.id} ({n.metadata['area_m2']} m^2)"
            for n in created
        )
        return SkillResult.ok(
            f"Split {room_id} into {described}; {moved} object(s) and {rewritten} "
            "sighting row(s) reassigned. Automatic derivation will preserve this edit.",
            new_room_ids=[n.id for n in created],
            objects_moved=moved,
            sightings_reassigned=rewritten,
        )
