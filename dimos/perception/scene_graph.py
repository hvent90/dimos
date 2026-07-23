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

"""Persistent hierarchical scene graph over recorded robot memory.

One memory2 SQLite DB holds the whole scene memory as four streams:

- ``scene_graph_nodes`` — event-sourced node versions with namespaced ids
  (``object_12``, ``room_3``, ``corridor_14``, ``building_0``, ``agent_0``).
  Current state = the latest row per id; updates append new versions and
  retirement is a flagged row, so history is never destroyed.
- ``scene_graph_edges`` — event-sourced ``contains``/``adjacent`` edges.
  ``contains`` is single-parent, enforced at write time.
- ``sightings`` — one row per observation, carrying the node id assigned by
  the fold and the containing room resolved at fold time (or backfilled by
  the next room-derivation pass). Node timestamps are only caches of this
  log: "when did you last see X in Y" reads the full trail, never the cache.
- ``scan_events`` — one row per scan pass (window, vocabulary, frames), the
  coverage substrate for honest negation.

The node/edge stream names are deliberately prefixed ``scene_graph_``, not
bare ``nodes``/``edges``: this DB may host other graphs later and generic
names would collide.

Object-node identity comes from the fold's attachment rule: a sighting
attaches to the nearest live same-name object node within
``attach_radius_m`` (2D — the go2 lane's lidar z is unreliable), otherwise
it creates a node with a fresh monotonic id. Ids never shift once assigned,
and the fold is deterministic, so rebuilding from the same sightings log
reproduces identical node ids. Two far-apart same-name sightings are two
nodes — "an object instance at a place".

Room *derivation* history (the ``rooms``/``room_derivations`` streams
written via :class:`RoomStore`) is evidence only: no query reads it. Room
nodes in the graph are canonical; geometry is mutable across derivations
while node ids stay unique (a re-derivation retires the old region nodes
and allocates fresh ids — overlap matching old→new identities is a known
deferral). The one exception is :meth:`SceneGraph.ensure_migrated`, which
reads the latest stored derivation once to materialize room nodes when
migrating a pre-graph DB.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
import math
from pathlib import Path
from types import TracebackType
from typing import Any

import numpy as np
from numpy.typing import NDArray

from dimos.constants import STATE_DIR
from dimos.mapping.occupancy.polygons import assign_to_polygons
from dimos.mapping.occupancy.room_store import RoomStore, StoredRoomSet
from dimos.memory2.store.sqlite import SqliteStore
from dimos.memory2.type.observation import Observation

NODES_STREAM = "scene_graph_nodes"
EDGES_STREAM = "scene_graph_edges"
SIGHTINGS_STREAM = "sightings"
SCAN_EVENTS_STREAM = "scan_events"
DEFAULT_SIGHTINGS_DB = STATE_DIR / "scene_memory" / "sightings.db"

BUILDING_ID = "building_0"
AGENT_ID = "agent_0"

# Max 2D distance for a sighting to attach to an existing same-name object
# node. Verified on go2_short (tool_scene_graph.py, 0.75 m): the closest
# distinct same-name nodes land 0.86 m apart (chair rows, twin
# refrigerators) and the far-apart person pair stays two nodes, while
# re-sightings of one instance chain onto its node (within-node sighting
# spread reaches ~1.4 m via chained attachment without merging neighbors).
ATTACH_RADIUS_M = 0.75

# Room polygons outline free space but objects are obstacles — their lidar
# positions sit in occupied cells just outside every polygon. Points inside
# no polygon snap to the nearest room within this distance (on go2_short all
# such sightings were within 0.64 m of their nearest room).
SIGHTING_SNAP_M = 0.75

# A re-scan of the same window is a duplicate, not new history — but track
# ids are per-detector-session counters and detected positions jitter at
# the centimetre level between runs (observed ≤ 0.05 m on go2_short), so
# neither can be part of an exact dedupe key. Same name + same frame ts +
# within this radius = the same sighting; genuinely distinct same-name
# detections in one frame sit ≥ 1.1 m apart on go2_short.
SIGHTING_DEDUPE_M = 0.25

# Render height of the room layer in the viewer (fills, anchors, adjacency
# edges, and the room end of contains edges): a floor-plan look, just above
# the costmap mesh so the fills don't z-fight it. Shared by the publisher
# (scene_memory) and the blueprint's visual overrides.
SCENE_GRAPH_ROOM_Z = 0.05


@dataclass(frozen=True)
class SceneNode:
    """Latest state of one graph node (one event-sourced row per version)."""

    id: str
    layer: str  # "building" | "room" | "corridor" | "object" | "agent"
    name: str
    position: tuple[float, float, float] | None
    extent: list[float] | None  # flat room outline polygon; None for objects
    first_seen_ts: float
    last_seen_ts: float
    sightings: int
    retired: bool
    metadata: dict[str, Any]

    @property
    def xy(self) -> tuple[float, float]:
        assert self.position is not None, f"{self.id} has no position"
        return self.position[0], self.position[1]

    def polygon(self) -> NDArray[np.float64]:
        assert self.extent is not None, f"{self.id} has no extent polygon"
        return np.asarray(self.extent, dtype=np.float64).reshape(-1, 2)


@dataclass(frozen=True)
class SceneEdge:
    """Latest state of one graph edge."""

    parent_id: str
    child_id: str
    kind: str  # "contains" | "adjacent"
    retired: bool
    metadata: dict[str, Any]


@dataclass(frozen=True)
class Sighting:
    """One observation of a named object at a world position and time.

    ``node_id``/``room_id`` are assigned by the fold (empty until then;
    ``room_id`` stays empty when no derived room contains the position).
    """

    name: str
    ts: float
    position: tuple[float, float, float]
    node_id: str = ""
    room_id: str = ""
    object_id: str = ""  # per-scan track id — evidence, not identity
    confidence: float | None = None
    source: str = ""
    vocabulary: tuple[str, ...] = ()


@dataclass(frozen=True)
class ScanEvent:
    """One scan pass: what vocabulary was looked for, over which window."""

    ts: float  # end of the scanned window
    t0: float  # start of the scanned window
    vocabulary: tuple[str, ...]
    source: str
    frames: int
    sightings: int


@dataclass(frozen=True)
class FoldResult:
    """Outcome of folding one scan pass into the graph."""

    appended_sightings: int
    created_node_ids: tuple[str, ...]
    updated_node_ids: tuple[str, ...]


def _node_from_obs(obs: Observation[Any]) -> SceneNode:
    tags = obs.tags
    position: tuple[float, float, float] | None = None
    if obs.pose_tuple is not None:
        position = (obs.pose_tuple[0], obs.pose_tuple[1], obs.pose_tuple[2])
    return SceneNode(
        id=str(tags["node_id"]),
        layer=str(tags["layer"]),
        name=str(obs.data),
        position=position,
        extent=tags.get("extent"),
        first_seen_ts=float(tags["first_seen_ts"]),
        last_seen_ts=float(tags["last_seen_ts"]),
        sightings=int(tags.get("sightings", 0)),
        retired=bool(tags.get("retired", False)),
        metadata=dict(tags.get("metadata", {})),
    )


def _edge_from_obs(obs: Observation[Any]) -> SceneEdge:
    tags = obs.tags
    return SceneEdge(
        parent_id=str(tags["parent_id"]),
        child_id=str(tags["child_id"]),
        kind=str(obs.data),
        retired=bool(tags.get("retired", False)),
        metadata=dict(tags.get("metadata", {})),
    )


def _sighting_from_obs(obs: Observation[Any]) -> Sighting:
    assert obs.pose_tuple is not None
    tags = obs.tags
    return Sighting(
        name=str(obs.data),
        ts=obs.ts,
        position=(obs.pose_tuple[0], obs.pose_tuple[1], obs.pose_tuple[2]),
        node_id=str(tags.get("node_id", "")),
        room_id=str(tags.get("room_id", "")),
        object_id=str(tags.get("object_id", "")),
        confidence=tags.get("confidence"),
        source=str(tags.get("source", "")),
        vocabulary=tuple(tags.get("vocabulary", ())),
    )


def _node_index(node_id: str) -> int:
    return int(node_id.rsplit("_", 1)[1])


class SceneGraph:
    """Append/query API over the scene-graph store. Use as a context manager."""

    def __init__(
        self,
        path: str | Path,
        *,
        attach_radius_m: float = ATTACH_RADIUS_M,
        snap_m: float = SIGHTING_SNAP_M,
    ) -> None:
        self._path = str(path)
        self._store = SqliteStore(path=str(path))
        self._attach_radius_m = attach_radius_m
        self._snap_m = snap_m
        self._nodes: dict[str, SceneNode] | None = None
        self._edges: dict[tuple[str, str, str], SceneEdge] | None = None

    def __enter__(self) -> SceneGraph:
        self._store.start()
        return self

    def __exit__(
        self,
        exctype: type[BaseException] | None,
        excinst: BaseException | None,
        exctb: TracebackType | None,
    ) -> None:
        self._store.stop()

    def refresh(self) -> None:
        """Drop the in-memory state cache; the next read reloads from disk.

        Needed when another SceneGraph instance may have written to the same
        DB since this instance last loaded.
        """
        self._nodes = None
        self._edges = None

    def _load(self) -> tuple[dict[str, SceneNode], dict[tuple[str, str, str], SceneEdge]]:
        """Current state: the latest row per node id / edge key, in row order."""
        if self._nodes is None or self._edges is None:
            nodes: dict[str, SceneNode] = {}
            edges: dict[tuple[str, str, str], SceneEdge] = {}
            existing = self._store.list_streams()
            if NODES_STREAM in existing:
                for obs in self._store.stream(NODES_STREAM, str).to_list():
                    node = _node_from_obs(obs)
                    nodes[node.id] = node
            if EDGES_STREAM in existing:
                for obs in self._store.stream(EDGES_STREAM, str).to_list():
                    edge = _edge_from_obs(obs)
                    edges[(edge.parent_id, edge.child_id, edge.kind)] = edge
            self._nodes = nodes
            self._edges = edges
        return self._nodes, self._edges

    def _append_node(self, node: SceneNode, ts: float) -> None:
        self._store.stream(NODES_STREAM, str).append(
            node.name,
            ts=ts,
            pose=node.position,
            tags={
                "node_id": node.id,
                "layer": node.layer,
                "extent": node.extent,
                "first_seen_ts": node.first_seen_ts,
                "last_seen_ts": node.last_seen_ts,
                "sightings": node.sightings,
                "retired": node.retired,
                "metadata": node.metadata,
            },
        )
        nodes, _ = self._load()
        nodes[node.id] = node

    def _append_edge(self, edge: SceneEdge, ts: float) -> None:
        self._store.stream(EDGES_STREAM, str).append(
            edge.kind,
            ts=ts,
            tags={
                "parent_id": edge.parent_id,
                "child_id": edge.child_id,
                "retired": edge.retired,
                "metadata": edge.metadata,
            },
        )
        _, edges = self._load()
        edges[(edge.parent_id, edge.child_id, edge.kind)] = edge

    def _ensure_building(self, ts: float) -> None:
        nodes, _ = self._load()
        if BUILDING_ID not in nodes:
            self._append_node(
                SceneNode(
                    id=BUILDING_ID,
                    layer="building",
                    name="building",
                    position=None,
                    extent=None,
                    first_seen_ts=ts,
                    last_seen_ts=ts,
                    sightings=0,
                    retired=False,
                    metadata={},
                ),
                ts,
            )

    def _set_parent(self, child_id: str, parent_id: str, ts: float) -> None:
        """Make ``parent_id`` the single ``contains`` parent of ``child_id``."""
        _, edges = self._load()
        current = next(
            (
                e
                for e in edges.values()
                if e.child_id == child_id and e.kind == "contains" and not e.retired
            ),
            None,
        )
        if current is not None:
            if current.parent_id == parent_id:
                return
            self._append_edge(replace(current, retired=True), ts)
        self._append_edge(
            SceneEdge(
                parent_id=parent_id, child_id=child_id, kind="contains", retired=False, metadata={}
            ),
            ts,
        )

    def regions(self) -> list[SceneNode]:
        """Live room/corridor nodes, ordered by node index (deterministic)."""
        nodes, _ = self._load()
        live = [n for n in nodes.values() if n.layer in ("room", "corridor") and not n.retired]
        return sorted(live, key=lambda n: _node_index(n.id))

    def assign_regions(
        self, points_xy: NDArray[np.float64], regions: list[SceneNode] | None = None
    ) -> list[str]:
        """Exclusively assign points to region node ids ("" = no region).

        A point inside a region polygon belongs to it; otherwise it snaps to
        the region with the nearest outline within ``snap_m``. Ties break to
        the lowest region index.
        """
        if regions is None:
            regions = self.regions()
        if len(points_xy) == 0 or not regions:
            return [""] * len(points_xy)
        indices = assign_to_polygons(points_xy, [r.polygon() for r in regions], self._snap_m)
        return [regions[j].id if j >= 0 else "" for j in indices.tolist()]

    def _attach(self, rows: list[Sighting]) -> tuple[list[Sighting], list[str], list[str]]:
        """Fold sighting rows through the attachment rule, in ts order.

        Returns the rows with ``node_id``/``room_id`` assigned, plus the
        created and updated node ids. Appends one node version per touched
        node and re-checks containment; does NOT write sighting rows.
        """
        nodes, _ = self._load()
        regions = self.regions()
        next_object = 1 + max((_node_index(i) for i in nodes if i.startswith("object_")), default=0)
        live_objects = {n.id: n for n in nodes.values() if n.layer == "object" and not n.retired}
        created: list[str] = []
        updated: dict[str, None] = {}  # insertion-ordered set
        assigned: list[Sighting] = []
        for s in sorted(rows, key=lambda r: r.ts):
            xy = np.asarray([s.position[:2]], dtype=np.float64)
            wanted = s.name.strip().lower()
            candidates = [
                n
                for n in live_objects.values()
                if n.name.lower() == wanted
                and float(np.hypot(n.xy[0] - xy[0, 0], n.xy[1] - xy[0, 1])) <= self._attach_radius_m
            ]
            if candidates:
                node = min(
                    candidates,
                    key=lambda n: (
                        float(np.hypot(n.xy[0] - xy[0, 0], n.xy[1] - xy[0, 1])),
                        _node_index(n.id),
                    ),
                )
                node = replace(
                    node,
                    # The position cache tracks the latest sighting by ts, so
                    # folding an older window later must not regress it.
                    position=s.position if s.ts >= node.last_seen_ts else node.position,
                    first_seen_ts=min(node.first_seen_ts, s.ts),
                    last_seen_ts=max(node.last_seen_ts, s.ts),
                    sightings=node.sightings + 1,
                )
                if node.id not in created:
                    updated[node.id] = None
            else:
                node = SceneNode(
                    id=f"object_{next_object}",
                    layer="object",
                    name=s.name,
                    position=s.position,
                    extent=None,
                    first_seen_ts=s.ts,
                    last_seen_ts=s.ts,
                    sightings=1,
                    retired=False,
                    metadata={},
                )
                next_object += 1
                created.append(node.id)
            live_objects[node.id] = node
            room_id = self.assign_regions(xy, regions)[0]
            assigned.append(replace(s, node_id=node.id, room_id=room_id))

        touched = created + list(updated)
        if touched:
            self._ensure_building(min(s.ts for s in assigned))
        for node_id in touched:
            node = live_objects[node_id]
            self._append_node(node, node.last_seen_ts)
            if regions:
                room_id = self.assign_regions(np.asarray([node.xy], dtype=np.float64), regions)[0]
                parent = room_id if room_id else BUILDING_ID
            else:
                parent = BUILDING_ID
            self._set_parent(node_id, parent, node.last_seen_ts)
        return assigned, created, list(updated)

    def fold_scan(
        self,
        sightings: list[Sighting],
        *,
        t0: float,
        t1: float,
        vocabulary: list[str],
        source: str,
        frames: int,
        agent_position: tuple[float, float, float] | None = None,
    ) -> FoldResult:
        """Fold one scan pass: attach sightings to nodes, log everything.

        A row is a duplicate — skipped — when the same name was already
        logged at the same frame ts within SIGHTING_DEDUPE_M, so re-scanning
        the same window doesn't duplicate history. The scan event is
        recorded even when nothing new was seen — coverage counts.
        ``agent_position`` (the robot's pose at the end of the scanned
        window) updates the agent node when given.
        """
        existing: dict[tuple[str, float], list[tuple[float, float, float]]] = {}
        for s in self.sightings():
            existing.setdefault((s.name, s.ts), []).append(s.position)
        fresh: list[Sighting] = []
        for s in sorted(sightings, key=lambda r: r.ts):
            near = existing.setdefault((s.name, s.ts), [])
            if any(math.dist(s.position, p) <= SIGHTING_DEDUPE_M for p in near):
                continue
            near.append(s.position)
            fresh.append(replace(s, source=source, vocabulary=tuple(vocabulary)))

        assigned, created, updated = self._attach(fresh)
        stream = self._store.stream(SIGHTINGS_STREAM, str)
        for s in assigned:
            tags: dict[str, Any] = {
                "object_id": s.object_id,
                "source": s.source,
                "vocabulary": list(s.vocabulary),
                "node_id": s.node_id,
                "room_id": s.room_id,
            }
            if s.confidence is not None:
                tags["confidence"] = s.confidence
            stream.append(s.name, ts=s.ts, pose=s.position, tags=tags)
        self._store.stream(SCAN_EVENTS_STREAM, str).append(
            source,
            ts=t1,
            tags={
                "t0": t0,
                "vocabulary": list(vocabulary),
                "frames": frames,
                "sightings": len(assigned),
            },
        )
        if agent_position is not None:
            self._update_agent(agent_position, t1)
        return FoldResult(
            appended_sightings=len(assigned),
            created_node_ids=tuple(created),
            updated_node_ids=tuple(updated),
        )

    def _update_agent(self, position: tuple[float, float, float], ts: float) -> None:
        """Refresh the agent node's position cache. Containment stays lazy —
        it is resolved from the pose trail at query time, never written."""
        nodes, _ = self._load()
        agent = nodes.get(AGENT_ID)
        if agent is None:
            agent = SceneNode(
                id=AGENT_ID,
                layer="agent",
                name="agent",
                position=position,
                extent=None,
                first_seen_ts=ts,
                last_seen_ts=ts,
                sightings=0,
                retired=False,
                metadata={},
            )
        else:
            agent = replace(agent, position=position, last_seen_ts=max(agent.last_seen_ts, ts))
        self._append_node(agent, ts)

    def apply_rooms(self, room_set: StoredRoomSet) -> None:
        """Write a room derivation into the graph: nodes, edges, backfill.

        Retires any previous region nodes and their edges (region ids are
        never reused; overlap-matching identities across re-derivations is
        deferred), writes room/corridor nodes with fresh monotonic ids,
        ``contains`` edges from the building, ``adjacent`` edges from the
        doorway records, re-checks object containment against the new
        regions, and backfills ``room_id`` on sighting rows that lack one.
        """
        ts = room_set.derived_ts
        nodes, edges = self._load()
        self._ensure_building(ts)

        old_regions = self.regions()
        old_ids = {n.id for n in old_regions}
        for edge in list(edges.values()):
            if not edge.retired and (edge.parent_id in old_ids or edge.child_id in old_ids):
                self._append_edge(replace(edge, retired=True), ts)
        for region_node in old_regions:
            self._append_node(replace(region_node, retired=True), ts)

        next_region = 1 + max(
            (_node_index(i) for i in nodes if i.startswith(("room_", "corridor_"))),
            default=0,
        )
        id_map: dict[int, str] = {}
        for room in sorted(room_set.rooms, key=lambda r: r.id):
            node_id = f"{room.kind}_{next_region}"
            next_region += 1
            id_map[room.id] = node_id
            self._append_node(
                SceneNode(
                    id=node_id,
                    layer=room.kind,
                    name=node_id,  # room naming is deferred; rooms stay "room_3"
                    position=(room.anchor_xy[0], room.anchor_xy[1], 0.0),
                    extent=[round(float(v), 3) for v in room.polygon.ravel()],
                    first_seen_ts=ts,
                    last_seen_ts=ts,
                    sightings=0,
                    retired=False,
                    metadata={
                        "area_m2": room.area_m2,
                        "centroid_xy": [
                            round(room.centroid_xy[0], 3),
                            round(room.centroid_xy[1], 3),
                        ],
                        "max_clearance_m": room.max_clearance_m,
                        "region_id": room.id,
                        "derived_ts": ts,
                        "explored_fraction": room_set.explored_fraction,
                    },
                ),
                ts,
            )
            self._set_parent(node_id, BUILDING_ID, ts)

        for doorway in room_set.doorways:
            a, b = int(doorway["between"][0]), int(doorway["between"][1])
            if a not in id_map or b not in id_map:
                continue
            self._append_edge(
                SceneEdge(
                    parent_id=id_map[a],
                    child_id=id_map[b],
                    kind="adjacent",
                    retired=False,
                    metadata={"xy": list(doorway["xy"]), "width_m": doorway["width_m"]},
                ),
                ts,
            )

        regions = self.regions()
        for node in [n for n in nodes.values() if n.layer == "object" and not n.retired]:
            room_id = self.assign_regions(np.asarray([node.xy], dtype=np.float64), regions)[0]
            self._set_parent(node.id, room_id if room_id else BUILDING_ID, ts)

        rows = self.sightings()
        needs_backfill = [s for s in rows if not s.room_id]
        if needs_backfill:
            xy = np.asarray([s.position[:2] for s in needs_backfill], dtype=np.float64)
            resolved = self.assign_regions(xy, regions)
            fixes = {
                id(s): room_id
                for s, room_id in zip(needs_backfill, resolved, strict=True)
                if room_id
            }
            if fixes:
                self._rewrite_sightings(
                    [replace(s, room_id=fixes[id(s)]) if id(s) in fixes else s for s in rows]
                )

    def ensure_migrated(self) -> int:
        """Fold pre-graph sighting rows (no ``node_id``) through attachment.

        A DB written before the graph existed has bare sighting rows and,
        possibly, a stored room derivation. This materializes room nodes
        from that latest derivation (once, if the graph has none), then
        folds the legacy rows in ts order — assigning node ids and room
        ids — and rewrites the rows. Idempotent; returns how many rows were
        migrated. The fold is deterministic, so a from-scratch rebuild
        reproduces identical node ids.
        """
        rows = self.sightings()
        legacy = [s for s in rows if not s.node_id]
        if not legacy:
            return 0
        if not self.regions():
            with RoomStore(self._path) as room_store:
                room_set = room_store.latest()
            if room_set is not None:
                self.apply_rooms(room_set)
            rows = self.sightings()  # apply_rooms may have backfilled room ids
            legacy = [s for s in rows if not s.node_id]
        assigned, _, _ = self._attach(legacy)
        by_key = {(s.name, s.object_id, s.ts): s for s in assigned}
        self._rewrite_sightings([by_key.get((s.name, s.object_id, s.ts), s) for s in rows])
        return len(legacy)

    def _rewrite_sightings(self, rows: list[Sighting]) -> None:
        """Replace the sightings stream contents (migration/backfill only)."""
        if SIGHTINGS_STREAM in self._store.list_streams():
            self._store.delete_stream(SIGHTINGS_STREAM)
        stream = self._store.stream(SIGHTINGS_STREAM, str)
        for s in rows:
            tags: dict[str, Any] = {
                "object_id": s.object_id,
                "source": s.source,
                "vocabulary": list(s.vocabulary),
                "node_id": s.node_id,
                "room_id": s.room_id,
            }
            if s.confidence is not None:
                tags["confidence"] = s.confidence
            stream.append(s.name, ts=s.ts, pose=s.position, tags=tags)

    def node(self, node_id: str) -> SceneNode | None:
        nodes, _ = self._load()
        return nodes.get(node_id)

    def nodes(self, layer: str | None = None, include_retired: bool = False) -> list[SceneNode]:
        """Current nodes, ordered by id namespace then index."""
        all_nodes, _ = self._load()
        out = [
            n
            for n in all_nodes.values()
            if (layer is None or n.layer == layer) and (include_retired or not n.retired)
        ]
        return sorted(out, key=lambda n: (n.layer, _node_index(n.id)))

    def parent_id(self, node_id: str) -> str | None:
        _, edges = self._load()
        edge = next(
            (
                e
                for e in edges.values()
                if e.child_id == node_id and e.kind == "contains" and not e.retired
            ),
            None,
        )
        return edge.parent_id if edge is not None else None

    def children(self, node_id: str) -> list[SceneNode]:
        nodes, edges = self._load()
        child_ids = [
            e.child_id
            for e in edges.values()
            if e.parent_id == node_id and e.kind == "contains" and not e.retired
        ]
        out = [nodes[c] for c in child_ids if c in nodes and not nodes[c].retired]
        return sorted(out, key=lambda n: (n.layer, _node_index(n.id)))

    def ancestors(self, node_id: str) -> list[SceneNode]:
        """The containment chain from parent up to the root."""
        nodes, _ = self._load()
        chain: list[SceneNode] = []
        current = self.parent_id(node_id)
        while current is not None:
            node = nodes.get(current)
            if node is None:
                break
            chain.append(node)
            current = self.parent_id(current)
        return chain

    def adjacent_rooms(self, node_id: str) -> list[tuple[SceneNode, dict[str, Any]]]:
        """Regions sharing a doorway with ``node_id``, with doorway metadata."""
        nodes, edges = self._load()
        out: list[tuple[SceneNode, dict[str, Any]]] = []
        for e in edges.values():
            if e.kind != "adjacent" or e.retired:
                continue
            other = (
                e.child_id
                if e.parent_id == node_id
                else (e.parent_id if e.child_id == node_id else None)
            )
            if other is None:
                continue
            node = nodes.get(other)
            if node is not None and not node.retired:
                out.append((node, dict(e.metadata)))
        return sorted(out, key=lambda pair: _node_index(pair[0].id))

    def edges(self, kind: str | None = None, include_retired: bool = False) -> list[SceneEdge]:
        _, all_edges = self._load()
        return [
            e
            for e in all_edges.values()
            if (kind is None or e.kind == kind) and (include_retired or not e.retired)
        ]

    def sightings(
        self, name: str | None = None, node_id: str | None = None, room_id: str | None = None
    ) -> list[Sighting]:
        """Sighting rows in ts order; filters combine (name case-insensitive)."""
        if SIGHTINGS_STREAM not in self._store.list_streams():
            return []
        rows = [
            _sighting_from_obs(obs)
            for obs in self._store.stream(SIGHTINGS_STREAM, str).order_by("ts").to_list()
        ]
        if name is not None:
            wanted = name.strip().lower()
            rows = [s for s in rows if s.name.lower() == wanted]
        if node_id is not None:
            rows = [s for s in rows if s.node_id == node_id]
        if room_id is not None:
            rows = [s for s in rows if s.room_id == room_id]
        return rows

    def names(self) -> dict[str, int]:
        """Distinct sighted names with their observation counts."""
        counts: dict[str, int] = {}
        for s in self.sightings():
            counts[s.name] = counts.get(s.name, 0) + 1
        return counts

    def scan_events(self) -> list[ScanEvent]:
        """All scan passes in ts order (coverage: what was looked for, when)."""
        if SCAN_EVENTS_STREAM not in self._store.list_streams():
            return []
        return [
            ScanEvent(
                ts=obs.ts,
                t0=float(obs.tags.get("t0", obs.ts)),
                vocabulary=tuple(obs.tags.get("vocabulary", ())),
                source=str(obs.data),
                frames=int(obs.tags.get("frames", 0)),
                sightings=int(obs.tags.get("sightings", 0)),
            )
            for obs in self._store.stream(SCAN_EVENTS_STREAM, str).order_by("ts").to_list()
        ]

    def ever_in_vocabulary(self, name: str) -> bool:
        """Was ``name`` ever part of a scan's detection vocabulary?"""
        wanted = name.strip().lower()
        return any(wanted in (v.lower() for v in event.vocabulary) for event in self.scan_events())

    def to_json(self) -> dict[str, Any]:
        """Serialize the whole graph (spark_dsg convention): debug/eval snapshot."""
        nodes, edges = self._load()
        return {
            "metadata": {
                "layers": ["building", "room", "corridor", "object", "agent"],
                # Single-floor recordings: no floor layer yet. The hierarchy
                # is extensible — insert one between building and room later.
                "floor_layer": None,
                "counts": {
                    "nodes": len(nodes),
                    "edges": len(edges),
                    "sightings": len(self.sightings()),
                    "scan_events": len(self.scan_events()),
                },
            },
            "nodes": [
                {
                    "id": n.id,
                    "layer": n.layer,
                    "name": n.name,
                    "position": list(n.position) if n.position is not None else None,
                    "extent": n.extent,
                    "first_seen_ts": n.first_seen_ts,
                    "last_seen_ts": n.last_seen_ts,
                    "sightings": n.sightings,
                    "retired": n.retired,
                    "parent": self.parent_id(n.id),
                    "metadata": n.metadata,
                }
                for n in sorted(nodes.values(), key=lambda n: (n.layer, _node_index(n.id)))
            ],
            "edges": [
                {
                    "parent": e.parent_id,
                    "child": e.child_id,
                    "kind": e.kind,
                    "retired": e.retired,
                    "metadata": e.metadata,
                }
                for e in edges.values()
            ],
        }
