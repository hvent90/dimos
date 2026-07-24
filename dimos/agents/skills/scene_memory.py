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

"""Scene-graph memory skills: hierarchical spatial queries over recorded memory.

The query surface over :class:`dimos.perception.scene_graph.SceneGraph`
(design increment 3): nine deterministic reads and two agent-initiated
mutations. Every result that contains a node carries the canonical payload —
id, layer, position, timestamps, ``parent`` and ``ancestors`` — so lineage
("which room is that?") is part of every answer, and temporal queries read
the full sightings log, never node caches.

The robot's pose trail comes from a memory2 recording (the replay DB by
default): the ``odom`` stream answers agent-position questions and feeds
scan coverage. All timestamps are recording-time epoch seconds — the same
time base the recording's streams carry.

The container republishes the graph on three viewer streams after each
mutation (room outline polygons, labeled node markers, containment/adjacency
edges); the Rerun bridge renders them over the costmap during replay.
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import math
from pathlib import Path
import threading
from typing import TYPE_CHECKING, Any, cast

import numpy as np
from numpy.typing import NDArray
from reactivex.disposable import Disposable
from scipy import ndimage

from dimos.agents.annotation import skill
from dimos.agents.skill_result import CommonSkillError, SkillResult
from dimos.agents.skills.scene_memory_rooms import RoomCurationSkills
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.mapping.occupancy.polygons import assign_to_polygons, points_in_polygon
from dimos.mapping.occupancy.room_segmentation import RoomSegmentation, segment_rooms
from dimos.mapping.occupancy.room_store import RoomStore, StoredRoom, StoredRoomSet
from dimos.memory2.replay import resolve_db_path
from dimos.memory2.store.sqlite import SqliteStore
from dimos.memory2.type.observation import Observation
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.nav_msgs.ContourPolygons3D import ContourPolygons3D
from dimos.msgs.nav_msgs.LineSegments3D import LineSegments3D
from dimos.msgs.nav_msgs.OccupancyGrid import CostValues, OccupancyGrid
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.visualization_msgs.EntityMarkers import EntityMarkers, Marker
from dimos.perception.scene_graph import (
    AGENT_ID,
    ATTACH_RADIUS_M,
    BUILDING_ID,
    DEFAULT_SIGHTINGS_DB,
    SCENE_GRAPH_ROOM_Z,
    SIGHTING_SNAP_M,
    ScanEvent,
    SceneGraph,
    SceneNode,
    Sighting,
)
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.perception.detection.detectors.base import Detector

logger = setup_logger()

# Bridge gaps between in-region odom samples up to this long (sensor dropouts
# and doorway-straddling flicker), so one visit doesn't splinter into many.
DEFAULT_VISIT_GAP_S = 2.0

# Detections flicker frame to frame far more than odom does, so bridge larger
# gaps when grouping an object's in-region sightings into stays.
SIGHTING_VISIT_GAP_S = 5.0

# Kept name for callers that predate the graph (see scene_graph.SIGHTING_SNAP_M).
DEFAULT_SIGHTING_SNAP_M = SIGHTING_SNAP_M

# Viewer edge color coding rides the LineSegments3D traversability channel:
# >= 0.9 renders green (contains), 0.4..0.9 yellow (adjacent).
_CONTAINS_TRAV = 1.0
_ADJACENT_TRAV = 0.5

# Costmap cells at or above this cost are obstacles for the map-query skills
# (the same threshold the planning gradient uses); unknown (-1) is never free.
OBSTACLE_THRESHOLD = 50

_AGENT_NAMES = {"agent", "agent_0", "robot"}


def visit_intervals(
    ts: NDArray[np.float64], inside: NDArray[np.bool_], max_gap_s: float = DEFAULT_VISIT_GAP_S
) -> list[tuple[float, float]]:
    """Group in-region samples into visit intervals ``[enter_ts, exit_ts]``.

    Consecutive inside samples whose timestamps differ by at most
    ``max_gap_s`` belong to the same visit; larger gaps split visits.

    Args:
        ts: (N,) sample timestamps, ascending.
        inside: (N,) whether each sample is inside the region.
        max_gap_s: max time gap bridged within one visit.
    """
    idx = np.nonzero(inside)[0]
    if idx.size == 0:
        return []
    inside_ts = ts[idx]
    splits = np.nonzero(np.diff(inside_ts) > max_gap_s)[0]
    starts = np.concatenate(([0], splits + 1))
    ends = np.concatenate((splits, [inside_ts.size - 1]))
    return [(float(inside_ts[s]), float(inside_ts[e])) for s, e in zip(starts, ends, strict=True)]


def _iso(ts: float) -> str:
    return datetime.fromtimestamp(ts, tz=timezone.utc).strftime("%Y-%m-%d %H:%M:%S")


@dataclass(frozen=True)
class PoseTrail:
    """Robot trajectory loaded from a recording: (N,) timestamps + (N, 2) xy."""

    ts: NDArray[np.float64]
    xy: NDArray[np.float64]

    def time_range(self) -> tuple[float, float]:
        return float(self.ts[0]), float(self.ts[-1])


def load_pose_trail(db_path: str, stream_names: list[str]) -> PoseTrail:
    """Load the robot pose trail from a memory2 SQLite recording.

    Uses the first stream in ``stream_names`` present in the store. Positions
    come from each observation's pose columns when stamped, else from the
    payload (expected to expose ``.position`` like ``PoseStamped``).

    Raises:
        LookupError: no candidate stream exists or the stream has no poses.
    """
    with SqliteStore(path=db_path, must_exist=True) as store:
        available = store.list_streams()
        name = next((s for s in stream_names if s in available), None)
        if name is None:
            raise LookupError(f"None of streams {stream_names} in {db_path}; found {available}")
        ts_list: list[float] = []
        xy_list: list[tuple[float, float]] = []
        obs: Observation[Any]
        for obs in store.stream(name).order_by("ts"):
            if obs.pose_tuple is not None:
                x, y = obs.pose_tuple[0], obs.pose_tuple[1]
            else:
                position: Any = obs.data.position
                x, y = float(position.x), float(position.y)
            ts_list.append(float(obs.ts))
            xy_list.append((x, y))
    if not ts_list:
        raise LookupError(f"Stream {name!r} in {db_path} has no observations")
    return PoseTrail(ts=np.asarray(ts_list), xy=np.asarray(xy_list))


def assign_to_rooms(
    points_xy: NDArray[np.float64],
    rooms: tuple[StoredRoom, ...],
    snap_m: float = DEFAULT_SIGHTING_SNAP_M,
) -> NDArray[np.int32]:
    """Exclusively assign each point to a stored-room id (0 = no room).

    A point inside a room polygon belongs to that room; otherwise it snaps
    to the room with the nearest outline if that is within ``snap_m``.
    """
    if len(points_xy) == 0 or not rooms:
        return np.zeros(len(points_xy), dtype=np.int32)
    indices = assign_to_polygons(points_xy, [r.polygon for r in rooms], snap_m)
    ids = np.asarray([r.id for r in rooms], dtype=np.int32)
    assigned: NDArray[np.int32] = np.where(indices >= 0, ids[indices], 0).astype(np.int32)
    return assigned


@dataclass(frozen=True)
class RegionCoverage:
    """Evidence that scan passes actually covered a region."""

    scan_passes: int
    passes_covering_region: int
    last_covered_ts: float | None  # set iff passes_covering_region > 0
    vocabulary: tuple[str, ...] = ()  # union over covering passes


def region_scan_coverage(
    polygon: NDArray[np.float64],
    events: list[ScanEvent],
    sightings: list[Sighting],
    sighting_in_region: NDArray[np.bool_],
    trail: PoseTrail | None,
) -> RegionCoverage:
    """How well past scans covered a region, from the evidence already stored.

    A scan pass covers the region when the robot was inside it during the
    pass's window (pose trail) or one of the pass's sightings — of any
    object — resolved into it (``sighting_in_region``, one flag per
    sighting: the camera positioned objects there).
    """
    sighting_ts = np.asarray([s.ts for s in sightings])
    trail_inside = points_in_polygon(trail.xy, polygon) if trail is not None else None
    covering = 0
    last: float | None = None
    vocabulary: set[str] = set()
    for event in events:
        evidence: list[float] = []
        if trail is not None and trail_inside is not None:
            in_window = trail_inside & (trail.ts >= event.t0) & (trail.ts <= event.ts)
            if in_window.any():
                evidence.append(float(trail.ts[in_window].max()))
        if sightings:
            in_window = sighting_in_region & (sighting_ts >= event.t0) & (sighting_ts <= event.ts)
            if in_window.any():
                evidence.append(float(sighting_ts[in_window].max()))
        if evidence:
            covering += 1
            vocabulary.update(event.vocabulary)
            last = max(evidence) if last is None else max(last, max(evidence))
    return RegionCoverage(
        scan_passes=len(events),
        passes_covering_region=covering,
        last_covered_ts=last,
        vocabulary=tuple(sorted(vocabulary)),
    )


def _vocabulary_sentence(name: str, ever_in_vocab: bool) -> str:
    if ever_in_vocab:
        return f"'{name}' was in the scan vocabulary."
    return (
        f"'{name}' was never in any scan's vocabulary, so it would not have been "
        "detected even if present."
    )


def _coverage_sentence(coverage: RegionCoverage, label: str) -> str:
    if coverage.scan_passes == 0:
        return "Nothing has been scanned yet — run scan_for_objects first."
    if coverage.passes_covering_region == 0:
        return (
            f"No scan pass is known to have covered {label}, so this is weak evidence of absence."
        )
    assert coverage.last_covered_ts is not None
    return (
        f"{coverage.passes_covering_region} of {coverage.scan_passes} scan pass(es) "
        f"covered {label}, most recently at {_iso(coverage.last_covered_ts)} UTC."
    )


def _node_payload(graph: SceneGraph, node: SceneNode) -> dict[str, Any]:
    """The canonical node payload: state + ``parent`` + ``ancestors``.

    ``extent`` summarizes the node's stored outline as an axis-aligned bbox
    ``[x_min, y_min, x_max, y_max]`` (the full polygon stays in storage):
    room/corridor segmentation outlines, or an object footprint unioned from
    the lidar points supporting its sightings. Null when never measured.
    Nodes with a known vertical span also carry ``z_range`` ``[z_min, z_max]``.
    """
    extent: list[float] | None = None
    if node.extent is not None:
        polygon = node.polygon()
        extent = [
            round(float(polygon[:, 0].min()), 2),
            round(float(polygon[:, 1].min()), 2),
            round(float(polygon[:, 0].max()), 2),
            round(float(polygon[:, 1].max()), 2),
        ]
    payload = {
        "id": node.id,
        "name": node.name,
        "layer": node.layer,
        "position": [round(v, 3) for v in node.position] if node.position is not None else None,
        "extent": extent,
        "first_seen_ts": round(node.first_seen_ts, 3),
        "last_seen_ts": round(node.last_seen_ts, 3),
        "sightings": node.sightings,
        "parent": graph.parent_id(node.id),
        "ancestors": [{"id": a.id, "layer": a.layer} for a in graph.ancestors(node.id)],
    }
    z_range = node.metadata.get("z_range")
    if z_range is not None:
        payload["z_range"] = z_range
    return payload


def _lineage_sentence(payload: dict[str, Any]) -> str:
    chain = [payload["id"], *(a["id"] for a in payload["ancestors"])]
    return " -> ".join(chain)


class SceneMemoryConfig(ModuleConfig):
    # Recording holding the robot's pose trail: a dataset name or .db path,
    # resolved like the replay DB. Defaults to the session's replay DB.
    trail_db: str = ""
    # Candidate pose streams, first match wins (naming varies by recording rig).
    trail_streams: list[str] = ["go2_odom", "odom"]
    # The scene-graph store (persists across restarts).
    sightings_db: str | Path = DEFAULT_SIGHTINGS_DB
    # Camera intrinsics + static base_link->camera_optical mount, needed only
    # by scan_for_objects (wired per robot in the blueprint).
    camera_info: CameraInfo | None = None
    base_to_optical: Transform | None = None
    detector_conf: float = 0.4
    scan_sample_period_s: float = 0.5
    # Corroboration gate for scan_for_objects: a NEW object needs this many
    # sightings over this many distinct frames before it becomes a node.
    # Detector confidence can't separate weak open-vocab matches from real
    # objects (true hits score ~0.1 on stylized renders, junk reaches 0.6+),
    # but junk flickers while real objects re-fire across frames.
    # Re-sightings of existing nodes always pass. 1/1 disables the gate.
    scan_min_sightings: int = 3
    scan_min_frames: int = 2
    # Fold geometry (see scene_graph module constants).
    sighting_snap_m: float = SIGHTING_SNAP_M
    attach_radius_m: float = ATTACH_RADIUS_M


class SceneMemorySkillContainer(RoomCurationSkills, Module):
    """Agent skills over the persistent scene graph (rooms, objects, time).

    The room-curation skills (view_map, rename/boundary/merge/split) live in
    :class:`RoomCurationSkills` — same container at runtime, separate module
    for size.
    """

    config: SceneMemoryConfig
    global_costmap: In[OccupancyGrid]
    scene_graph_rooms: Out[ContourPolygons3D]
    scene_graph_markers: Out[EntityMarkers]
    scene_graph_edges: Out[LineSegments3D]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._trail: PoseTrail | None = None
        self._trail_lock = threading.Lock()
        self._detector: Detector | None = None
        self._detector_lock = threading.Lock()
        self._grid_lock = threading.Lock()
        self._latest_grid: OccupancyGrid | None = None
        # Distance-to-nearest-obstacle field, recomputed when the grid changes.
        self._clearance_cache: tuple[OccupancyGrid, NDArray[np.float64]] | None = None
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

    def _graph(self) -> SceneGraph:
        return SceneGraph(
            self.config.sightings_db,
            attach_radius_m=self.config.attach_radius_m,
            snap_m=self.config.sighting_snap_m,
        )

    def _trail_db(self) -> str:
        if self.config.trail_db:
            return self.config.trail_db
        # Only fall back to the replay dataset when actually replaying it —
        # on a live robot that dataset is unrelated history and answering
        # from it would be a confident hallucination.
        if self.config.g.replay:
            return self.config.g.replay_db
        raise LookupError("No trail recording configured (set scenememoryskillcontainer.trail_db)")

    def _get_trail(self) -> PoseTrail:
        with self._trail_lock:
            if self._trail is None:
                path = resolve_db_path(self._trail_db())
                self._trail = load_pose_trail(str(path), self.config.trail_streams)
                logger.info("Loaded pose trail", db=str(path), samples=len(self._trail.ts))
            return self._trail

    def _trail_or_none(self) -> PoseTrail | None:
        """The pose trail if available — coverage answers degrade without it."""
        try:
            return self._get_trail()
        except (FileNotFoundError, LookupError):
            return None

    def _ensure_rooms(self, graph: SceneGraph) -> str:
        """Auto-derive rooms once for a rooms-dependent read.

        Returns a note for the honest-absence case ("" when rooms exist or
        were just derived): with no grid received the read proceeds without
        rooms rather than failing — object/agent nodes still serve.
        """
        if graph.regions():
            return ""
        with self._grid_lock:
            grid = self._latest_grid
        if grid is None:
            return (
                "No rooms are derived yet and no occupancy map has been received, "
                "so answers have no room lineage."
            )
        with self._mutate_lock:
            graph.refresh()
            if not graph.regions():
                self._derive_into(graph)
        return ""

    def _derive_into(self, graph: SceneGraph) -> tuple[RoomSegmentation, bool]:
        """Segment the latest grid into the graph; no-op on an unchanged grid.

        Writes the derivation record (evidence) via RoomStore, applies room
        nodes/edges to the graph, and republishes the viewer streams.
        Callers hold ``_mutate_lock``.
        """
        with self._grid_lock:
            grid = self._latest_grid
        assert grid is not None, "callers check a grid exists"
        segmentation = segment_rooms(grid)
        regions = graph.regions()
        if regions and all(
            r.metadata.get("derived_ts") == segmentation.derived_ts for r in regions
        ):
            # Same grid re-derived: keep the existing nodes — replacing them
            # would churn room ids for zero information.
            return segmentation, False
        source = "scene_memory.derive_rooms"
        with RoomStore(self.config.sightings_db) as store:
            store.save(segmentation, source=source)
        graph.apply_rooms(StoredRoomSet.from_segmentation(segmentation, source=source))
        self._publish_graph(graph, ts=segmentation.derived_ts)
        return segmentation, True

    def _get_detector(self) -> Detector:
        with self._detector_lock:
            if self._detector is None:
                # Lazy import: pulls in torch/ultralytics, which most skills
                # never need.
                from dimos.perception.detection.detectors.yoloe import (
                    Yoloe2DDetector,
                    YoloePromptMode,
                )

                self._detector = Yoloe2DDetector(
                    prompt_mode=YoloePromptMode.PROMPT, conf=self.config.detector_conf
                )
            return self._detector

    @skill
    def scan_for_objects(self, prompt: list[str] | str) -> SkillResult[CommonSkillError]:
        """Scan the recording for objects and fold sightings into the scene graph.

        Runs open-vocabulary detection over the recorded camera frames,
        positions each detection in the world using the recording's lidar,
        and attaches every sighting to a persistent object node (stable ids
        like object_7). Only object types named in the prompt can be found —
        everything else is invisible to this scan. Results persist across
        restarts. Detection matches APPEARANCE, not intent: an object you
        call a "couch" may only fire under a near-synonym ("sofa", "bench",
        "seat"), so scan a broad vocabulary of plausible names and treat any
        hit near the expected spot as your object. Broad vocabularies are
        safe: a new object only enters the graph once it is sighted in
        several frames, so one-frame false matches are filtered out (the
        result reports them).

        Args:
            prompt: Object type names to look for — include synonyms, e.g.
                ["couch", "sofa", "bench", "chair"] or "chair, couch".
        """
        # A bare string over MCP would otherwise iterate per character.
        if isinstance(prompt, str):
            prompt = prompt.split(",")
        vocabulary = sorted({p.strip() for p in prompt if p.strip()})
        if not vocabulary:
            return SkillResult.fail("INVALID_INPUT", "prompt must name at least one object type")
        if self.config.camera_info is None or self.config.base_to_optical is None:
            return SkillResult.fail(
                "NOT_CONFIGURED",
                "scan_for_objects needs camera_info and base_to_optical configured",
            )
        try:
            db_path = resolve_db_path(self._trail_db())
        except (FileNotFoundError, LookupError) as e:
            return SkillResult.fail("NOT_CONFIGURED", f"No recording available: {e}")

        # Import here with the detector: the scan lane is optional heavy metal.
        from dimos.perception.lidar_scan import corroborated_sightings, iter_lidar_scan

        detector = self._get_detector()
        detector.set_prompts(text=vocabulary)  # type: ignore[attr-defined]

        sightings: list[Sighting] = []
        n_frames = 0
        t_lo = float("inf")
        t_hi = float("-inf")
        agent_position: tuple[float, float, float] | None = None
        try:
            with SqliteStore(path=str(db_path), must_exist=True) as store:
                for frame in iter_lidar_scan(
                    store,
                    detector,
                    self.config.camera_info,
                    self.config.base_to_optical,
                    sample_period_s=self.config.scan_sample_period_s,
                ):
                    n_frames += 1
                    t_lo = min(t_lo, frame.ts)
                    t_hi = max(t_hi, frame.ts)
                    agent_position = (frame.robot_xy[0], frame.robot_xy[1], 0.0)
                    for s in frame.sightings:
                        sightings.append(
                            Sighting(
                                name=s.name,
                                ts=s.ts,
                                position=s.position,
                                object_id=str(s.track_id) if s.track_id >= 0 else "",
                                confidence=s.confidence,
                                extent=s.extent,
                            )
                        )
        except LookupError as e:
            return SkillResult.fail(
                "EXECUTION_FAILED", f"Recording is missing a required stream: {e}"
            )
        if n_frames == 0:
            return SkillResult.fail(
                "EXECUTION_FAILED", "No frames with aligned odom+lidar found in the recording"
            )
        with self._mutate_lock, self._graph() as graph:
            # Corroboration gate: a new object must be sighted in several
            # frames before it may become a node; re-sightings of existing
            # nodes pass through. Each scan covers the whole recording, so
            # the gate is cumulative — a dropped one-off can confirm later.
            anchors: dict[str, list[tuple[float, float]]] = {}
            for node in graph.nodes(layer="object"):
                if node.position is not None:
                    anchors.setdefault(node.name, []).append(node.xy)
            sightings, filtered = corroborated_sightings(
                sightings,
                anchors,
                radius_m=self.config.attach_radius_m,
                min_sightings=self.config.scan_min_sightings,
                min_frames=self.config.scan_min_frames,
            )
            result = graph.fold_scan(
                sightings,
                t0=t_lo,
                t1=t_hi,
                vocabulary=vocabulary,
                source="scene_memory.scan_for_objects",
                frames=n_frames,
                agent_position=agent_position,
            )
            self._publish_graph(graph, ts=t_hi)
        by_name: dict[str, int] = {}
        for row in sightings:
            by_name[row.name] = by_name.get(row.name, 0) + 1
        vantage_hint = (
            " Zero sightings usually means the target was never in camera view — "
            "the scan only sees what the camera has faced. Change vantage: turn in "
            "place (navigate_to_pose at the current position with a new yaw_deg) or "
            "move a few meters into free space, then scan again."
            if not sightings
            else ""
        )
        filtered_note = (
            f" Filtered {sum(filtered.values())} uncorroborated detection(s) {filtered} — "
            f"a new object needs >={self.config.scan_min_sightings} sightings over "
            f">={self.config.scan_min_frames} frames; real objects re-fire on rescan "
            "from another vantage."
            if filtered
            else ""
        )
        return SkillResult.ok(
            f"Scanned {n_frames} frames; {result.appended_sightings} new sighting(s): {by_name}. "
            f"Object nodes: {len(result.created_node_ids)} created, "
            f"{len(result.updated_node_ids)} updated. "
            f"Vocabulary was {vocabulary} — other object types were not looked for."
            f"{filtered_note}{vantage_hint}",
            sightings_by_name=by_name,
            appended_sightings=result.appended_sightings,
            created_nodes=list(result.created_node_ids),
            updated_nodes=list(result.updated_node_ids),
            frames=n_frames,
            scanned_t0=round(t_lo, 3),
            scanned_t1=round(t_hi, 3),
            vocabulary=vocabulary,
            filtered_uncorroborated=filtered,
        )

    @skill
    def derive_rooms(self, force: bool = False) -> SkillResult[CommonSkillError]:
        """Segment the current occupancy map into rooms and update the graph.

        Writes room/corridor nodes, containment and doorway-adjacency edges,
        and re-checks which room contains each object. Room ids and names
        are stable: a re-derived room in the same place keeps its node.
        Rooms-dependent reads auto-derive once, so calling this explicitly
        is only needed after the map has grown.

        Agent-edited geometry (set_room_boundary, merge_rooms, split_room)
        is preserved: derivation refuses while edits exist unless force is
        true, which discards them and re-derives from the map alone.

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

    @skill
    def find(self, text: str) -> SkillResult[CommonSkillError]:
        """Find objects or rooms by name or id; every hit carries its lineage.

        Deterministic matching over the scene graph (exact id, exact name,
        then substring). A miss says whether the name was ever in any scan's
        vocabulary — if not, scan_for_objects with that name first.

        Args:
            text: An object/room name or node id, e.g. "couch" or "room_3".
        """
        wanted = text.strip().lower()
        if not wanted:
            return SkillResult.fail("INVALID_INPUT", "text must not be empty")
        with self._graph() as graph:
            note = self._ensure_rooms(graph)
            candidates = [n for n in graph.nodes() if n.layer in ("object", "room", "corridor")]

            def rank(n: SceneNode) -> int:
                if n.id.lower() == wanted:
                    return 0
                if n.name.lower() == wanted:
                    return 1
                return 2

            hits = sorted(
                (n for n in candidates if rank(n) < 2 or wanted in n.name.lower()),
                key=lambda n: (rank(n), n.layer, n.id),
            )
            if not hits:
                ever = graph.ever_in_vocabulary(wanted)
                known = sorted(graph.names())
                region_ids = [r.id for r in graph.regions()]
                return SkillResult.ok(
                    f"No node matches '{text}'. {_vocabulary_sentence(text, ever)} "
                    f"Objects sighted so far: {known}; regions: {region_ids}.{note}",
                    hits=[],
                    ever_in_vocabulary=ever,
                    known_names=known,
                    region_ids=region_ids,
                )
            payloads = [_node_payload(graph, n) for n in hits]
            described = [
                f"{p['id']} ({p['name']}) in {p['parent']}"
                if p["parent"]
                else f"{p['id']} ({p['name']})"
                for p in payloads
            ]
            return SkillResult.ok(
                f"{len(hits)} match(es) for '{text}': {'; '.join(described)}.{note}",
                hits=payloads,
            )

    @skill
    def get_scene(self) -> SkillResult[CommonSkillError]:
        """Collapsed scene overview: rooms, object counts, coverage, the agent.

        Call this first. Returns the recording's absolute time range (epoch
        seconds, UTC — the anchor for relative-time questions), each room and
        corridor with its object count and scan coverage, the doorway count,
        the explored fraction, and which room the agent is in. Drill into any
        node with expand/nodes_in.
        """
        with self._graph() as graph:
            note = self._ensure_rooms(graph)
            regions = graph.regions()
            events = graph.scan_events()
            rows = graph.sightings()
            trail = self._trail_or_none()

            region_entries = []
            for r in regions:
                member = np.asarray([s.room_id == r.id for s in rows], dtype=bool)
                coverage = region_scan_coverage(r.polygon(), events, rows, member, trail)
                region_entries.append(
                    {
                        "id": r.id,
                        "kind": r.layer,
                        "area_m2": r.metadata.get("area_m2"),
                        "objects": sum(1 for c in graph.children(r.id) if c.layer == "object"),
                        "coverage": {
                            "scan_passes_covering": coverage.passes_covering_region,
                            "last_covered_ts": round(coverage.last_covered_ts, 3)
                            if coverage.last_covered_ts is not None
                            else None,
                            "vocabulary": list(coverage.vocabulary),
                        },
                    }
                )

            agent_entry: dict[str, Any] | None = None
            if trail is not None:
                x, y = float(trail.xy[-1, 0]), float(trail.xy[-1, 1])
                room_id = graph.assign_regions(np.asarray([[x, y]]))[0]
                agent_entry = {
                    "id": AGENT_ID,
                    "position": [round(x, 3), round(y, 3), 0.0],
                    "room": room_id or None,
                    "ts": round(float(trail.ts[-1]), 3),
                }

            n_rooms = sum(1 for r in regions if r.layer == "room")
            n_corridors = len(regions) - n_rooms
            objects = graph.nodes(layer="object")
            n_doorways = len(graph.edges(kind="adjacent"))
            explored = regions[0].metadata.get("explored_fraction") if regions else None

            parts = [
                f"Scene: {n_rooms} room(s) + {n_corridors} corridor(s)"
                + (f" ({explored:.0%} explored)" if explored is not None else ""),
                f"{len(objects)} object node(s), {n_doorways} doorway(s).",
            ]
            metadata: dict[str, Any] = {
                "regions": region_entries,
                "total_objects": len(objects),
                "n_doorways": n_doorways,
                "explored_fraction": explored,
                "agent": agent_entry,
            }
            if trail is not None:
                t0, t1 = trail.time_range()
                parts.append(f"Recording {_iso(t0)}..{_iso(t1)} UTC ({t1 - t0:.1f} s).")
                metadata["time_range"] = [round(t0, 3), round(t1, 3)]
            if agent_entry is not None:
                parts.append(
                    f"Agent at ({agent_entry['position'][0]}, {agent_entry['position'][1]})"
                    + (f" in {agent_entry['room']}." if agent_entry["room"] else ".")
                )
            if note:
                parts.append(note)
            return SkillResult.ok(" ".join(parts), **metadata)

    def _children_result(self, node_id: str) -> SkillResult[CommonSkillError]:
        with self._graph() as graph:
            note = self._ensure_rooms(graph)
            node = graph.node(node_id)
            if node is None:
                region_ids = [r.id for r in graph.regions()]
                return SkillResult.fail(
                    "INVALID_INPUT",
                    f"No node '{node_id}'. Known regions: {region_ids}; "
                    "use find() or get_scene() to discover ids.",
                )
            children = graph.children(node_id)
            payloads = [_node_payload(graph, c) for c in children]
            retired_note = (
                " (this node is retired — a newer derivation replaced it)" if node.retired else ""
            )
            described = ", ".join(f"{p['name']} ({p['id']})" for p in payloads) or "nothing"
            return SkillResult.ok(
                f"{node_id}{retired_note} contains {len(children)} node(s): {described}.{note}",
                node_id=node_id,
                count=len(children),
                children=payloads,
            )

    @skill
    def expand(self, node_id: str) -> SkillResult[CommonSkillError]:
        """Expand one node into its children (incremental disclosure).

        Walk the scene top-down: get_scene() first, then expand the node you
        care about (e.g. a room) instead of pulling the whole graph.

        Args:
            node_id: Node to expand, e.g. "building_0" or "room_3".
        """
        return self._children_result(node_id)

    @skill
    def nodes_in(self, node_id: str) -> SkillResult[CommonSkillError]:
        """List (and count) everything contained in a node, deterministically.

        "What's in room 3?" / "how many rooms are there?" — counting happens
        here, not by eyeballing: nodes_in("building_0") counts rooms;
        nodes_in("room_3") counts that room's objects.

        Args:
            node_id: Containment node, e.g. "room_3" or "building_0".
        """
        return self._children_result(node_id)

    @skill
    def adjacent(self, node_id: str) -> SkillResult[CommonSkillError]:
        """Which rooms share a doorway with this room/corridor?

        Answers from the derived doorway records: each neighbor comes with
        the doorway position and approximate width.

        Args:
            node_id: A room or corridor id, e.g. "room_3" or "corridor_6".
        """
        with self._graph() as graph:
            note = self._ensure_rooms(graph)
            node = graph.node(node_id)
            if node is None:
                region_ids = [r.id for r in graph.regions()]
                return SkillResult.fail(
                    "INVALID_INPUT",
                    f"No node '{node_id}'. Known regions: {region_ids}.",
                )
            neighbors = graph.adjacent_rooms(node_id)
            entries: list[dict[str, Any]] = [
                {
                    "node": _node_payload(graph, n),
                    "doorway_xy": [round(float(v), 3) for v in doorway.get("xy", [])],
                    "doorway_width_m": doorway.get("width_m"),
                }
                for n, doorway in neighbors
            ]
            described = ", ".join(
                f"{e['node']['id']} (doorway at {tuple(e['doorway_xy'])}, "
                f"{e['doorway_width_m']} m wide)"
                for e in entries
            )
            message = (
                f"{node_id} shares doorways with: {described}.{note}"
                if entries
                else f"{node_id} has no recorded doorway adjacency.{note}"
            )
            return SkillResult.ok(message, node_id=node_id, neighbors=entries)

    @skill
    def near(
        self, node_id: str = "", xy: list[float] | None = None, radius: float = 2.0
    ) -> SkillResult[CommonSkillError]:
        """What is within a radius of a node or point? Deterministic geometry.

        Distances are between node anchor positions — stored extents are not
        used here, so results say "position within radius", not "surface
        within radius" (hit payloads carry each node's extent bbox when known).

        Args:
            node_id: Center node id, e.g. "object_5" (give this OR xy).
            xy: Center as world [x, y] instead of a node id.
            radius: Search radius in meters.
        """
        if bool(node_id) == (xy is not None):
            return SkillResult.fail("INVALID_INPUT", "Give either node_id or xy, not both")
        if xy is not None and len(xy) != 2:
            return SkillResult.fail("INVALID_INPUT", "xy must be [x, y]")
        if radius <= 0:
            return SkillResult.fail("INVALID_INPUT", "radius must be positive")
        with self._graph() as graph:
            self._ensure_rooms(graph)  # hits carry room lineage
            if node_id:
                center_node = graph.node(node_id)
                if center_node is None or center_node.position is None:
                    return SkillResult.fail(
                        "INVALID_INPUT", f"No positioned node '{node_id}' — use find() first."
                    )
                cx, cy = center_node.xy
            else:
                assert xy is not None
                cx, cy = float(xy[0]), float(xy[1])
            candidates = [
                n
                for n in [*graph.nodes(layer="object"), *graph.nodes(layer="agent")]
                if n.position is not None and n.id != node_id
            ]
            hits = []
            for n in candidates:
                distance = float(np.hypot(n.xy[0] - cx, n.xy[1] - cy))
                if distance <= radius:
                    hits.append((distance, n))
            hits.sort(key=lambda pair: pair[0])
            payloads = [
                {**_node_payload(graph, n), "distance_m": round(distance, 2)}
                for distance, n in hits
            ]
            described = ", ".join(f"{p['name']} ({p['id']}) {p['distance_m']} m" for p in payloads)
            return SkillResult.ok(
                f"{len(hits)} node(s) within {radius} m of ({cx:.2f}, {cy:.2f}): "
                f"{described or 'none'}. Distances are between stored anchor positions, "
                "not surfaces.",
                center=[round(cx, 3), round(cy, 3)],
                radius_m=radius,
                hits=payloads,
            )

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

    @skill
    def where_am_i(self, t: float | None = None) -> SkillResult[CommonSkillError]:
        """Which room is the robot in — now, or at a past time t?

        Resolves the agent's position from the recorded pose trail (t=None
        means the end of the trail) and its room from the scene graph.

        Args:
            t: Optional timestamp in recording-time epoch seconds (see
                get_scene() for the recording's time range).
        """
        try:
            trail = self._get_trail()
        except (FileNotFoundError, LookupError) as e:
            return SkillResult.fail("NOT_CONFIGURED", f"No pose trail available: {e}")
        t0, t1 = trail.time_range()
        if t is None:
            i = len(trail.ts) - 1
        else:
            i = int(np.argmin(np.abs(trail.ts - t)))
            if abs(float(trail.ts[i]) - t) > 2.0:
                return SkillResult.fail(
                    "INVALID_INPUT",
                    f"No pose within 2 s of t={t}; trail covers {t0:.3f}..{t1:.3f} "
                    f"({_iso(t0)}..{_iso(t1)} UTC).",
                )
        ts = float(trail.ts[i])
        x, y = float(trail.xy[i, 0]), float(trail.xy[i, 1])
        with self._graph() as graph:
            note = self._ensure_rooms(graph)
            room_id = graph.assign_regions(np.asarray([[x, y]]))[0]
            ancestors: list[dict[str, Any]] = []
            if room_id:
                room = graph.node(room_id)
                assert room is not None
                ancestors = [
                    {"id": room.id, "layer": room.layer},
                    *({"id": a.id, "layer": a.layer} for a in graph.ancestors(room_id)),
                ]
            elif graph.node(BUILDING_ID) is not None:
                ancestors = [{"id": BUILDING_ID, "layer": "building"}]
            payload = {
                "id": AGENT_ID,
                "name": "agent",
                "layer": "agent",
                "position": [round(x, 3), round(y, 3), 0.0],
                "extent": None,
                "parent": room_id or (BUILDING_ID if ancestors else None),
                "ancestors": ancestors,
            }
            when = f"at {_iso(ts)} UTC" if t is not None else f"as of {_iso(ts)} UTC (end of trail)"
            place = f"in {room_id}" if room_id else "in no derived room"
            return SkillResult.ok(
                f"The robot was at ({x:.2f}, {y:.2f}) {place} {when}."
                + (
                    f" Lineage: {AGENT_ID} -> " + " -> ".join(a["id"] for a in ancestors) + "."
                    if ancestors
                    else ""
                )
                + (f" {note}" if note else ""),
                node=payload,
                ts=round(ts, 3),
            )

    @skill
    def last_seen(self, name: str, in_node: str = "") -> SkillResult[CommonSkillError]:
        """When and where was something last seen? The spatio-temporal workhorse.

        Reads the full sightings log (not node caches), so it stays correct
        when the subject later moved elsewhere: with in_node set, the answer
        is the last sighting INSIDE that node, plus a note if it was later
        seen outside. name="agent" answers for the robot from its pose
        trail ("when were you last in room 4?"). A miss returns a
        coverage-qualified negative, never a bare "no".

        Args:
            name: Object name as scanned (e.g. "couch"), or "agent" for the
                robot itself.
            in_node: Optional containment filter — a room/corridor id from
                get_scene() (e.g. "room_2") or "building_0". Empty = anywhere.
        """
        return self._seen_query(name, in_node, window=None)

    @skill
    def seen_between(
        self, name: str, t0: float, t1: float, in_node: str = ""
    ) -> SkillResult[CommonSkillError]:
        """Was something seen in a time window (optionally inside a node)?

        Same answer shape as last_seen, restricted to sightings with
        t0 <= ts <= t1 (recording-time epoch seconds; get_scene() gives the
        recording's range).

        Args:
            name: Object name as scanned, or "agent" for the robot itself.
            t0: Window start, epoch seconds.
            t1: Window end, epoch seconds.
            in_node: Optional containment filter node id. Empty = anywhere.
        """
        if t1 <= t0:
            return SkillResult.fail("INVALID_INPUT", f"Need t0 < t1, got {t0}..{t1}")
        return self._seen_query(name, in_node, window=(t0, t1))

    def _resolve_filter_node(
        self, graph: SceneGraph, in_node: str
    ) -> tuple[SceneNode | None, SkillResult[CommonSkillError] | None]:
        """Resolve an in_node filter to a region node (None = whole building)."""
        node = graph.node(in_node)
        if node is None:
            region_ids = [r.id for r in graph.regions()]
            return None, SkillResult.fail(
                "INVALID_INPUT",
                f"No node '{in_node}'. Known regions: {region_ids}; "
                "rooms may not be derived yet (see get_scene).",
            )
        if node.layer == "building":
            return None, None  # everything is in the building
        if node.layer not in ("room", "corridor"):
            return None, SkillResult.fail(
                "INVALID_INPUT",
                f"in_node must be a room, corridor, or {BUILDING_ID}; "
                f"'{in_node}' is a {node.layer}.",
            )
        return node, None

    def _seen_query(
        self, name: str, in_node: str, window: tuple[float, float] | None
    ) -> SkillResult[CommonSkillError]:
        with self._graph() as graph:
            # Rooms-dependent even without in_node: the answer's lineage and
            # room_id come from the room set, so auto-derive here too.
            note = self._ensure_rooms(graph)
            if name.strip().lower() in _AGENT_NAMES:
                return self._agent_seen(graph, in_node, window)
            region: SceneNode | None = None
            if in_node:
                region, error = self._resolve_filter_node(graph, in_node)
                if error is not None:
                    return error
            all_rows = graph.sightings(name)
            rows = [s for s in all_rows if window is None or window[0] <= s.ts <= window[1]]
            member = [region is None or s.room_id == region.id for s in rows]
            in_rows = [s for s, m in zip(rows, member, strict=True) if m]
            if not in_rows:
                return self._seen_miss(graph, name, in_node, region, window, all_rows, note)

            visits = visit_intervals(
                np.asarray([s.ts for s in rows]),
                np.asarray(member, dtype=bool),
                max_gap_s=SIGHTING_VISIT_GAP_S,
            )
            last = in_rows[-1]
            enter, exit_ = visits[-1]
            later_elsewhere = [
                s for s, m in zip(rows, member, strict=True) if not m and s.ts > exit_
            ]
            node = graph.node(last.node_id)
            payload = _node_payload(graph, node) if node is not None else None
            x, y, z = (round(v, 2) for v in last.position)
            place = f" in {last.room_id}" if last.room_id else ""
            label = f" in {in_node}" if in_node else ""
            note_later = ""
            extra: dict[str, Any] = {}
            if in_node and later_elsewhere:
                note_later = (
                    f" Note: '{name}' was later seen outside {in_node}, most recently at "
                    f"{_iso(later_elsewhere[-1].ts)} UTC."
                )
                extra["later_elsewhere_ts"] = round(later_elsewhere[-1].ts, 3)
            if window is not None:
                extra["window"] = [round(window[0], 3), round(window[1], 3)]
            return SkillResult.ok(
                f"Last saw '{name}'{label} at {_iso(last.ts)} UTC at ({x}, {y}, {z}){place}; "
                f"that stay spanned {_iso(enter)}..{_iso(exit_)} UTC. "
                f"{len(in_rows)} sighting(s) total.{note_later}{note}",
                name=name,
                last_sighting={
                    "ts": round(last.ts, 3),
                    "position": [x, y, z],
                    "room_id": last.room_id or None,
                    "node_id": last.node_id,
                },
                node=payload,
                visits=[[round(a, 3), round(b, 3)] for a, b in visits],
                last_interval=[round(enter, 3), round(exit_, 3)],
                sightings_matched=len(in_rows),
                in_node=in_node or None,
                **extra,
            )

    def _seen_miss(
        self,
        graph: SceneGraph,
        name: str,
        in_node: str,
        region: SceneNode | None,
        window: tuple[float, float] | None,
        all_rows: list[Sighting],
        note: str,
    ) -> SkillResult[CommonSkillError]:
        """The coverage-qualified negative — never a bare "no"."""
        ever = graph.ever_in_vocabulary(name)
        known = sorted(graph.names())
        events = graph.scan_events()
        trail = self._trail_or_none()
        everything = graph.sightings()
        if region is not None:
            member = np.asarray([s.room_id == region.id for s in everything], dtype=bool)
            coverage = region_scan_coverage(region.polygon(), events, everything, member, trail)
            label = in_node
        else:
            last_event = events[-1].ts if events else None
            coverage = RegionCoverage(
                scan_passes=len(events),
                passes_covering_region=len(events),
                last_covered_ts=last_event,
            )
            label = "anywhere scanned"
        where = f" in {in_node}" if in_node else ""
        when = f" between {_iso(window[0])} and {_iso(window[1])} UTC" if window is not None else ""
        elsewhere = ""
        extra: dict[str, Any] = {}
        if all_rows:
            last_any = all_rows[-1]
            elsewhere = (
                f" It was sighted {len(all_rows)} time(s) outside that filter, last at "
                f"{_iso(last_any.ts)} UTC"
                + (f" in {last_any.room_id}" if last_any.room_id else "")
                + "."
            )
            extra["last_elsewhere_ts"] = round(last_any.ts, 3)
        if window is not None:
            extra["window"] = [round(window[0], 3), round(window[1], 3)]
        return SkillResult.ok(
            f"Never saw '{name}'{where}{when}.{elsewhere} "
            f"{_vocabulary_sentence(name, ever)} {_coverage_sentence(coverage, label)}"
            + (f" Objects sighted so far: {known}." if not all_rows else "")
            + note,
            name=name,
            sightings_matched=0,
            ever_in_vocabulary=ever,
            known_names=known,
            coverage={
                "scan_passes": coverage.scan_passes,
                "passes_covering_region": coverage.passes_covering_region,
                "region_last_scanned_ts": round(coverage.last_covered_ts, 3)
                if coverage.last_covered_ts is not None
                else None,
            },
            in_node=in_node or None,
            **extra,
        )

    def _agent_seen(
        self, graph: SceneGraph, in_node: str, window: tuple[float, float] | None
    ) -> SkillResult[CommonSkillError]:
        """last_seen/seen_between for the robot itself, from the pose trail.

        Region membership uses the strict room polygon (the robot moves
        through free space; the snap rule exists for object positions that
        land in walls). The agent node's containment is resolved here at
        query time — nothing writes it continuously.
        """
        try:
            trail = self._get_trail()
        except (FileNotFoundError, LookupError) as e:
            return SkillResult.fail("NOT_CONFIGURED", f"No pose trail available: {e}")
        region: SceneNode | None = None
        if in_node:
            region, error = self._resolve_filter_node(graph, in_node)
            if error is not None:
                return error
        keep = (
            np.ones(len(trail.ts), dtype=bool)
            if window is None
            else (trail.ts >= window[0]) & (trail.ts <= window[1])
        )
        ts = trail.ts[keep]
        xy = trail.xy[keep]
        t0, t1 = trail.time_range()
        if len(ts) == 0:
            return SkillResult.fail(
                "INVALID_INPUT",
                f"The trail has no samples in that window; it covers {_iso(t0)}..{_iso(t1)} UTC.",
            )
        member = (
            points_in_polygon(xy, region.polygon())
            if region is not None
            else np.ones(len(ts), dtype=bool)
        )
        visits = visit_intervals(ts, member, max_gap_s=DEFAULT_VISIT_GAP_S)
        extra: dict[str, Any] = {}
        if window is not None:
            extra["window"] = [round(window[0], 3), round(window[1], 3)]
        current_room = graph.assign_regions(np.asarray([trail.xy[-1]]))[0]
        agent_payload = {
            "id": AGENT_ID,
            "name": "agent",
            "layer": "agent",
            "position": [round(float(trail.xy[-1, 0]), 3), round(float(trail.xy[-1, 1]), 3), 0.0],
            "extent": None,
            "parent": current_room or None,
            "ancestors": (
                [
                    {"id": current_room, "layer": graph.node(current_room).layer},  # type: ignore[union-attr]
                    *({"id": a.id, "layer": a.layer} for a in graph.ancestors(current_room)),
                ]
                if current_room
                else []
            ),
        }
        if not visits:
            where = f" inside {in_node}" if in_node else ""
            when = (
                f" between {_iso(window[0])} and {_iso(window[1])} UTC"
                if window is not None
                else ""
            )
            return SkillResult.ok(
                f"The robot was never{where}{when} during the recorded trail "
                f"({_iso(t0)}..{_iso(t1)} UTC — full pose coverage, so this is a "
                "confident negative).",
                name="agent",
                sightings_matched=0,
                in_node=in_node or None,
                node=agent_payload,
                visits=[],
                trail_start_ts=round(t0, 3),
                trail_end_ts=round(t1, 3),
                **extra,
            )
        enter, exit_ = visits[-1]
        member_idx = np.nonzero(member)[0]
        last_i = int(member_idx[-1])
        last_ts = float(ts[last_i])
        lx, ly = float(xy[last_i, 0]), float(xy[last_i, 1])
        last_room = (
            region.id if region is not None else graph.assign_regions(np.asarray([[lx, ly]]))[0]
        )
        if in_node and float(ts[-1]) > exit_:
            extra["later_elsewhere_ts"] = round(float(ts[-1]), 3)
        where = f" in {in_node}" if in_node else ""
        after = f" (left {t1 - exit_:.1f} s before the end of the trail)" if in_node else ""
        return SkillResult.ok(
            f"The robot was last{where} from {_iso(enter)} to {_iso(exit_)} UTC{after}. "
            f"{len(visits)} visit(s) total.",
            name="agent",
            last_sighting={
                "ts": round(last_ts, 3),
                "position": [round(lx, 3), round(ly, 3), 0.0],
                "room_id": last_room or None,
                "node_id": AGENT_ID,
            },
            node=agent_payload,
            visits=[[round(a, 3), round(b, 3)] for a, b in visits],
            last_interval=[round(enter, 3), round(exit_, 3)],
            sightings_matched=int(member.sum()),
            in_node=in_node or None,
            **extra,
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
