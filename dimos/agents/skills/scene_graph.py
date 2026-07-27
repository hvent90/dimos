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

"""Scene-graph skills: derive rooms from the map, publish them to the viewer.

``derive_rooms`` segments the live global costmap into rooms and corridors
and writes them into the persistent scene graph
(:mod:`dimos.perception.scene_graph`), where room node ids are stable across
re-derivations: a room re-derived in the same place keeps its node, so its
name and the objects assigned to it survive a remap.

The container republishes the graph on three viewer streams after each
mutation (room outline polygons, labeled node markers, containment and
adjacency edges); the Rerun bridge renders them over the costmap during
replay.

Free-space reads over the same costmap live in
:mod:`dimos.agents.skills.free_space` — placement questions answered from
the grid alone. What is here needs the graph.
"""

from __future__ import annotations

from pathlib import Path
import threading
from typing import Any

import numpy as np
from reactivex.disposable import Disposable

from dimos.agents.annotation import skill
from dimos.agents.skill_result import CommonSkillError, SkillResult
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.mapping.occupancy.rooms.segmentation import (
    RoomSegmentation,
    RoomSegmentationConfig,
    segment_rooms,
)
from dimos.mapping.occupancy.rooms.store import RoomStore, StoredRoomSet
from dimos.msgs.nav_msgs.ContourPolygons3D import ContourPolygons3D
from dimos.msgs.nav_msgs.LineSegments3D import LineSegments3D
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.visualization_msgs.EntityMarkers import EntityMarkers, Marker
from dimos.perception.scene_graph import (
    AGENT_ID,
    ATTACH_RADIUS_M,
    DEFAULT_SIGHTINGS_DB,
    SCENE_GRAPH_ROOM_Z,
    SIGHTING_SNAP_M,
    SceneGraph,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Viewer edge color coding rides the LineSegments3D traversability channel:
# >= 0.9 renders green (contains), 0.4..0.9 yellow (adjacent).
_CONTAINS_TRAV = 1.0
_ADJACENT_TRAV = 0.5


class SceneGraphConfig(ModuleConfig):
    # The scene-graph store (persists across restarts).
    sightings_db: str | Path = DEFAULT_SIGHTINGS_DB
    # Room boundaries form where free space pinches below this half-width.
    # The default suits door-separated buildings; open-plan spaces need it
    # wider so archways count as dividers (DimSim apartment: 1.0).
    door_half_width_m: float = RoomSegmentationConfig().door_half_width_m
    # Fold geometry (see scene_graph module constants).
    sighting_snap_m: float = SIGHTING_SNAP_M
    attach_radius_m: float = ATTACH_RADIUS_M


class SceneGraphSkillContainer(Module):
    """Agent skills over the persistent scene graph."""

    config: SceneGraphConfig
    global_costmap: In[OccupancyGrid]
    scene_graph_rooms: Out[ContourPolygons3D]
    scene_graph_markers: Out[EntityMarkers]
    scene_graph_edges: Out[LineSegments3D]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._grid_lock = threading.Lock()
        self._latest_grid: OccupancyGrid | None = None
        # Serializes graph mutations (fold, derivation, migration).
        self._mutate_lock = threading.Lock()

    @rpc
    def start(self) -> None:
        super().start()
        if self.global_costmap.transport:
            self.register_disposable(Disposable(self.global_costmap.subscribe(self._on_costmap)))
        with self._mutate_lock, self._graph() as graph:
            migrated = graph.ensure_migrated()
            if migrated:
                logger.info("Migrated pre-graph sightings into the scene graph", rows=migrated)
            nodes = graph.nodes()
            if nodes:  # initial load: show the persisted graph in the viewer
                self._publish_graph(graph, ts=max(n.last_seen_ts for n in nodes))

    @rpc
    def stop(self) -> None:
        super().stop()

    def _on_costmap(self, grid: OccupancyGrid) -> None:
        with self._grid_lock:
            self._latest_grid = grid

    def _graph(self) -> SceneGraph:
        return SceneGraph(
            self.config.sightings_db,
            attach_radius_m=self.config.attach_radius_m,
            snap_m=self.config.sighting_snap_m,
        )

    def _derive_into(self, graph: SceneGraph) -> tuple[RoomSegmentation, bool]:
        """Segment the latest grid into the graph; no-op on an unchanged grid.

        Writes the derivation record (evidence) via RoomStore, applies room
        nodes/edges to the graph, and republishes the viewer streams.
        Callers hold ``_mutate_lock``.
        """
        with self._grid_lock:
            grid = self._latest_grid
        assert grid is not None, "callers check a grid exists"
        segmentation = segment_rooms(
            grid, RoomSegmentationConfig(door_half_width_m=self.config.door_half_width_m)
        )
        regions = graph.regions()
        if regions and all(
            r.metadata.get("derived_ts") == segmentation.derived_ts for r in regions
        ):
            # Same grid re-derived: keep the existing nodes — replacing them
            # would churn room ids for zero information.
            return segmentation, False
        source = "scene_graph.derive_rooms"
        with RoomStore(self.config.sightings_db) as store:
            store.save(segmentation, source=source)
        graph.apply_rooms(StoredRoomSet.from_segmentation(segmentation, source=source))
        self._publish_graph(graph, ts=segmentation.derived_ts)
        return segmentation, True

    @skill
    def derive_rooms(self, force: bool = False) -> SkillResult[CommonSkillError]:
        """Segment the current occupancy map into rooms and update the graph.

        Writes room/corridor nodes, containment and doorway-adjacency edges,
        and re-checks which room contains each object. Room ids and names
        are stable: a re-derived room in the same place keeps its node.

        Agent-edited room geometry is preserved: derivation refuses while
        edits exist unless force is true, which discards them and
        re-derives from the map alone.

        Args:
            force: Discard agent-edited room geometry and re-derive.
        """
        with self._grid_lock:
            grid = self._latest_grid
        if grid is None:
            return SkillResult.fail(
                "INVALID_STATE", "No occupancy map received yet — is mapping running?"
            )
        with self._mutate_lock, self._graph() as graph:
            if not force:
                edited = [n.id for n in graph.regions() if n.metadata.get("origin") == "agent"]
                if edited:
                    return SkillResult.ok(
                        f"Rooms carry agent edits ({', '.join(edited)}); kept them. "
                        "Call derive_rooms with force=true to discard the edits and "
                        "re-derive from the map.",
                        kept_agent_edits=edited,
                        region_ids=[n.id for n in graph.regions()],
                    )
            segmentation, changed = self._derive_into(graph)
            region_ids = [n.id for n in graph.regions()]
        rooms = segmentation.rooms()
        corridors = segmentation.corridors()
        note = "" if changed else " The map is unchanged since the last derivation — kept it."
        return SkillResult.ok(
            f"Derived {len(rooms)} room(s) and {len(corridors)} corridor(s) from the "
            f"map ({segmentation.explored_fraction:.0%} explored — the count can rise "
            f"as more area is mapped).{note}",
            n_rooms=len(rooms),
            n_corridors=len(corridors),
            n_doorways=len(segmentation.doorways),
            explored_fraction=segmentation.explored_fraction,
            derived_ts=round(segmentation.derived_ts, 3),
            region_ids=region_ids,
        )

    def _publish_graph(self, graph: SceneGraph, ts: float) -> None:
        """Republish the graph on the viewer streams (rooms, markers, edges).

        Rooms render as filled floor-plan polygons at ground level (the
        polygon id in the cloud is the region's numeric id, so viewer tints
        match the 2D debug renders); objects/agent/room anchors as labeled
        points; contains edges run object to room anchor, adjacent edges
        anchor-doorway-anchor. ``ts`` is recording time so the viewer
        timeline lines up with the replayed camera/lidar streams.
        """
        if not (
            self.scene_graph_rooms.transport
            or self.scene_graph_markers.transport
            or self.scene_graph_edges.transport
        ):
            return  # bare test containers have no wired streams
        regions = graph.regions()
        objects = graph.nodes(layer="object")
        agent = graph.node(AGENT_ID)

        if regions and self.scene_graph_rooms.transport:
            polygons = [r.polygon() for r in regions]
            points = np.vstack([np.column_stack([p, np.zeros(len(p))]) for p in polygons])
            # Polygon id = the region's numeric id ("room_3" -> 3), keying the
            # same palette as the 2D debug renders.
            ids = np.concatenate(
                [
                    np.full(len(p), float(r.id.rsplit("_", 1)[-1]), dtype=np.float64)
                    for r, p in zip(regions, polygons, strict=True)
                ]
            )
            cloud = PointCloud2.from_numpy(points, frame_id="world", timestamp=ts, intensities=ids)
            self.scene_graph_rooms.publish(
                ContourPolygons3D(ts=ts, frame_id="world", raw_bytes=cloud.lcm_encode())
            )

        markers = [
            Marker(
                entity_id=n.id,
                label=n.name,
                entity_type="object",
                x=n.position[0],
                y=n.position[1],
                z=n.position[2],
            )
            for n in objects
            if n.position is not None
        ]
        markers += [
            Marker(
                entity_id=r.id,
                # Display name, so agent renames show up in the viewer (the
                # marker already carries the id). Unnamed regions name == id.
                label=r.name or r.id,
                entity_type="location",
                x=r.xy[0],
                y=r.xy[1],
                z=SCENE_GRAPH_ROOM_Z,
            )
            for r in regions
        ]
        if agent is not None and agent.position is not None:
            markers.append(
                Marker(
                    entity_id=AGENT_ID,
                    label="agent",
                    entity_type="person",
                    x=agent.position[0],
                    y=agent.position[1],
                    z=agent.position[2],
                )
            )
        if self.scene_graph_markers.transport:
            self.scene_graph_markers.publish(EntityMarkers(markers=markers, ts=ts))

        anchors = {r.id: r.xy for r in regions}
        segments: list[tuple[tuple[float, float, float], tuple[float, float, float]]] = []
        traversability: list[float] = []
        for n in objects:
            parent = graph.parent_id(n.id)
            if parent in anchors and n.position is not None:
                ax, ay = anchors[parent]
                segments.append(((ax, ay, SCENE_GRAPH_ROOM_Z), n.position))
                traversability.append(_CONTAINS_TRAV)
        for edge in graph.edges(kind="adjacent"):
            if edge.parent_id in anchors and edge.child_id in anchors:
                pax, pay = anchors[edge.parent_id]
                cax, cay = anchors[edge.child_id]
                mid = edge.metadata.get("xy") or [(pax + cax) / 2, (pay + cay) / 2]
                mx, my = float(mid[0]), float(mid[1])
                segments.append(((pax, pay, SCENE_GRAPH_ROOM_Z), (mx, my, SCENE_GRAPH_ROOM_Z)))
                segments.append(((mx, my, SCENE_GRAPH_ROOM_Z), (cax, cay, SCENE_GRAPH_ROOM_Z)))
                traversability += [_ADJACENT_TRAV, _ADJACENT_TRAV]
        if self.scene_graph_edges.transport:
            self.scene_graph_edges.publish(
                LineSegments3D(
                    ts=ts, frame_id="world", segments=segments, traversability=traversability
                )
            )
