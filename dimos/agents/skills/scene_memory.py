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

"""Skills answering spatio-temporal queries from recorded robot memory.

The pose trail comes from a memory2 recording (the replay DB by default):
the ``odom`` stream's timestamped poses answer "when was the robot in
region R" and "where was the robot at time t". Regions are 2D polygons in
world coordinates. Object queries read the persistent sightings log
(``dimos.perception.sightings``), populated by the ``scan_for_objects``
skill (color + lidar recordings, no depth needed). All timestamps are
recording-time epoch seconds — the same time base the recording's streams
carry.
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
import threading
from typing import TYPE_CHECKING, Any

import numpy as np
from numpy.typing import NDArray
from reactivex.disposable import Disposable

from dimos.agents.annotation import skill
from dimos.agents.skill_result import CommonSkillError, SkillResult
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.mapping.occupancy.polygons import (
    distance_to_polygon,
    points_in_polygon,
    polygon_from_flat,
)
from dimos.mapping.occupancy.room_segmentation import segment_rooms
from dimos.mapping.occupancy.room_store import RoomStore, StoredRoom, StoredRoomSet
from dimos.memory2.replay import resolve_db_path
from dimos.memory2.store.sqlite import SqliteStore
from dimos.memory2.type.observation import Observation
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.perception.sightings import (
    DEFAULT_SIGHTINGS_DB,
    ScanEvent,
    Sighting,
    SightingsLog,
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


def _sightings_xy(sightings: list[Sighting]) -> NDArray[np.float64]:
    return np.array([[s.position[0], s.position[1]] for s in sightings], dtype=np.float64).reshape(
        -1, 2
    )


# Room polygons outline free space, but objects are obstacles — their lidar
# positions sit in occupied cells just outside every polygon (walls, furniture
# clusters). Points inside no polygon therefore snap to the nearest room
# within this distance; on go2_short every such sighting was within 0.64 m of
# its nearest room. Sightings deep in a furniture band between rooms can be
# nearly equidistant to two rooms, so a snapped assignment near the midline
# is a coin toss — acceptable for room-level queries.
DEFAULT_SIGHTING_SNAP_M = 0.75


def assign_to_rooms(
    points_xy: NDArray[np.float64],
    rooms: tuple[StoredRoom, ...],
    snap_m: float = DEFAULT_SIGHTING_SNAP_M,
) -> NDArray[np.int32]:
    """Exclusively assign each point to a room id (0 = no room).

    A point inside a room polygon belongs to that room; otherwise it snaps
    to the room with the nearest outline if that is within ``snap_m``.
    """
    if len(points_xy) == 0 or not rooms:
        return np.zeros(len(points_xy), dtype=np.int32)
    effective = np.empty((len(points_xy), len(rooms)))
    for j, room in enumerate(rooms):
        inside = points_in_polygon(points_xy, room.polygon)
        effective[:, j] = np.where(inside, 0.0, distance_to_polygon(points_xy, room.polygon))
    ids = np.asarray([r.id for r in rooms], dtype=np.int32)
    assigned: NDArray[np.int32] = np.where(
        effective.min(axis=1) <= snap_m, ids[effective.argmin(axis=1)], 0
    ).astype(np.int32)
    return assigned


@dataclass(frozen=True)
class RegionCoverage:
    """Evidence that scan passes actually covered a region."""

    scan_passes: int
    passes_covering_region: int
    last_covered_ts: float | None  # set iff passes_covering_region > 0


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
            last = max(evidence) if last is None else max(last, max(evidence))
    return RegionCoverage(
        scan_passes=len(events), passes_covering_region=covering, last_covered_ts=last
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


class SceneMemoryConfig(ModuleConfig):
    # Recording holding the robot's pose trail: a dataset name or .db path,
    # resolved like the replay DB. Defaults to the session's replay DB.
    trail_db: str = ""
    # Candidate pose streams, first match wins (naming varies by recording rig).
    trail_streams: list[str] = ["go2_odom", "odom"]
    # Persistent sightings store (survives restarts, unlike RAM-only tracks).
    sightings_db: str | Path = DEFAULT_SIGHTINGS_DB
    # Camera intrinsics + static base_link->camera_optical mount, needed only
    # by scan_for_objects (wired per robot in the blueprint).
    camera_info: CameraInfo | None = None
    base_to_optical: Transform | None = None
    detector_conf: float = 0.4
    scan_sample_period_s: float = 0.5
    # Max distance a sighting outside every room polygon may snap to the
    # nearest room (see assign_to_rooms).
    sighting_snap_m: float = DEFAULT_SIGHTING_SNAP_M


class SceneMemorySkillContainer(Module):
    """Agent skills over recorded spatio-temporal memory (pose trail queries)."""

    config: SceneMemoryConfig
    global_costmap: In[OccupancyGrid]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._trail: PoseTrail | None = None
        self._trail_lock = threading.Lock()
        self._detector: Detector | None = None
        self._detector_lock = threading.Lock()
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

    def _trail_db(self) -> str:
        if self.config.trail_db:
            return self.config.trail_db
        # Only fall back to the replay dataset when actually replaying it —
        # on a live robot that dataset is unrelated history and answering
        # from it would be a confident hallucination.
        if self.config.g.replay:
            return self.config.g.replay_db
        raise LookupError(
            "No trail recording configured (set scene_memory_skill_container.trail_db)"
        )

    def _get_trail(self) -> PoseTrail:
        with self._trail_lock:
            if self._trail is None:
                path = resolve_db_path(self._trail_db())
                self._trail = load_pose_trail(str(path), self.config.trail_streams)
                logger.info(
                    "Loaded pose trail",
                    db=str(path),
                    samples=len(self._trail.ts),
                )
            return self._trail

    def _trail_or_none(self) -> PoseTrail | None:
        """The pose trail if available — coverage answers degrade without it."""
        try:
            return self._get_trail()
        except (FileNotFoundError, LookupError):
            return None

    @skill
    def robot_trail_info(self) -> SkillResult[CommonSkillError]:
        """Describe the robot's recorded pose trail: time range and coverage.

        Call this first to learn the recording's absolute time range (epoch
        seconds, UTC) so you can convert relative times like "30 seconds in"
        into the timestamps other trail skills expect.
        """
        try:
            trail = self._get_trail()
        except (FileNotFoundError, LookupError) as e:
            return SkillResult.fail("NOT_CONFIGURED", f"No pose trail available: {e}")
        t0, t1 = trail.time_range()
        bounds_min = trail.xy.min(axis=0)
        bounds_max = trail.xy.max(axis=0)
        return SkillResult.ok(
            f"Pose trail: {len(trail.ts)} samples from {_iso(t0)} to {_iso(t1)} UTC "
            f"({t1 - t0:.1f} s).",
            start_ts=round(t0, 3),
            end_ts=round(t1, 3),
            duration_s=round(t1 - t0, 1),
            samples=len(trail.ts),
            xy_bounds=[
                [round(float(bounds_min[0]), 2), round(float(bounds_min[1]), 2)],
                [round(float(bounds_max[0]), 2), round(float(bounds_max[1]), 2)],
            ],
        )

    @skill
    def robot_position_at(self, t: float, tolerance: float = 2.0) -> SkillResult[CommonSkillError]:
        """Where was the robot at time t? Returns the nearest recorded pose.

        Args:
            t: Timestamp in recording-time epoch seconds (see robot_trail_info
                for the recording's time range).
            tolerance: Max seconds between t and the nearest pose sample.
        """
        try:
            trail = self._get_trail()
        except (FileNotFoundError, LookupError) as e:
            return SkillResult.fail("NOT_CONFIGURED", f"No pose trail available: {e}")
        t0, t1 = trail.time_range()
        i = int(np.argmin(np.abs(trail.ts - t)))
        dt = float(abs(trail.ts[i] - t))
        if dt > tolerance:
            return SkillResult.fail(
                "INVALID_INPUT",
                f"No pose within {tolerance} s of t={t}; trail covers "
                f"{t0:.3f}..{t1:.3f} ({_iso(t0)}..{_iso(t1)} UTC).",
            )
        x, y = float(trail.xy[i, 0]), float(trail.xy[i, 1])
        return SkillResult.ok(
            f"At {_iso(float(trail.ts[i]))} UTC the robot was at "
            f"({x:.2f}, {y:.2f}) in the world frame.",
            ts=round(float(trail.ts[i]), 3),
            x=round(x, 3),
            y=round(y, 3),
        )

    @skill
    def robot_visits_to_region(self, region: list[float]) -> SkillResult[CommonSkillError]:
        """When was the robot inside a region? Lists visits, most recent last.

        Answers "when were you last in <region>": the last visit's end time is
        when the robot was last there.

        Args:
            region: Region polygon in world coordinates, flattened
                [x1, y1, x2, y2, x3, y3, ...] (at least 3 vertices).
        """
        try:
            polygon = polygon_from_flat(region)
        except ValueError as e:
            return SkillResult.fail("INVALID_INPUT", str(e))
        try:
            trail = self._get_trail()
        except (FileNotFoundError, LookupError) as e:
            return SkillResult.fail("NOT_CONFIGURED", f"No pose trail available: {e}")

        t0, t1 = trail.time_range()
        inside = points_in_polygon(trail.xy, polygon)
        intervals = visit_intervals(trail.ts, inside)
        if not intervals:
            return SkillResult.ok(
                f"The robot was never inside that region during the recorded trail "
                f"({_iso(t0)}..{_iso(t1)} UTC, {t1 - t0:.1f} s of full pose coverage).",
                visits=[],
                trail_start_ts=round(t0, 3),
                trail_end_ts=round(t1, 3),
            )
        enter, exit_ = intervals[-1]
        return SkillResult.ok(
            f"The robot was last in the region from {_iso(enter)} to {_iso(exit_)} UTC "
            f"(left {t1 - exit_:.1f} s before the end of the trail). "
            f"{len(intervals)} visit(s) total.",
            visits=[[round(a, 3), round(b, 3)] for a, b in intervals],
            last_enter_ts=round(enter, 3),
            last_exit_ts=round(exit_, 3),
            trail_start_ts=round(t0, 3),
            trail_end_ts=round(t1, 3),
        )

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
    def scan_for_objects(self, prompt: list[str]) -> SkillResult[CommonSkillError]:
        """Scan the recording for objects and log every sighting to memory.

        Runs open-vocabulary detection over the recorded camera frames and
        positions each detection in the world using the recording's lidar.
        Only object types named in the prompt can be found — everything else
        is invisible to this scan. Sightings persist across restarts.

        Args:
            prompt: Object type names to look for, e.g.
                ["chair", "couch", "potted plant"].
        """
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
        from dimos.perception.lidar_scan import iter_lidar_scan

        detector = self._get_detector()
        detector.set_prompts(text=vocabulary)  # type: ignore[attr-defined]

        sightings: list[Sighting] = []
        n_frames = 0
        t_lo = float("inf")
        t_hi = float("-inf")
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
                    for s in frame.sightings:
                        sightings.append(
                            Sighting(
                                name=s.name,
                                ts=s.ts,
                                position=s.position,
                                object_id=str(s.track_id) if s.track_id >= 0 else "",
                                confidence=s.confidence,
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
        with SightingsLog(self.config.sightings_db) as log:
            log.record_scan(
                sightings,
                t0=t_lo,
                t1=t_hi,
                vocabulary=vocabulary,
                source="scene_memory.scan_for_objects",
                frames=n_frames,
            )
        by_name: dict[str, int] = {}
        for row in sightings:
            by_name[row.name] = by_name.get(row.name, 0) + 1
        return SkillResult.ok(
            f"Scanned {n_frames} frames; logged {len(sightings)} sightings: {by_name}. "
            f"Vocabulary was {vocabulary} — other object types were not looked for.",
            sightings_by_name=by_name,
            frames=n_frames,
            scanned_t0=round(t_lo, 3),
            scanned_t1=round(t_hi, 3),
            vocabulary=vocabulary,
        )

    @skill
    def last_seen_object(self, name: str) -> SkillResult[CommonSkillError]:
        """When and where did you last see an object? Reads the sightings log.

        Answers from all past scans, including objects that are no longer in
        view. Run scan_for_objects first if nothing has been scanned yet.

        Args:
            name: Object type name exactly as scanned, e.g. "couch".
        """
        with SightingsLog(self.config.sightings_db) as log:
            matches = log.sightings(name)
            known = log.names()
            ever_in_vocab = log.ever_in_vocabulary(name)
        if not matches:
            qualifier = (
                "it was in the scan vocabulary but never detected"
                if ever_in_vocab
                else "it was never in any scan's vocabulary, so it would not have been "
                "detected even if present"
            )
            return SkillResult.ok(
                f"No sightings of '{name}' — {qualifier}. Objects sighted so far: {sorted(known)}.",
                sightings=0,
                ever_in_vocabulary=ever_in_vocab,
                known_names=sorted(known),
            )
        last = matches[-1]
        x, y, z = (round(v, 2) for v in last.position)
        return SkillResult.ok(
            f"Last saw '{name}' at {_iso(last.ts)} UTC at world position ({x}, {y}, {z}). "
            f"{len(matches)} sighting(s) total between {_iso(matches[0].ts)} and "
            f"{_iso(last.ts)} UTC.",
            last_ts=round(last.ts, 3),
            position=[x, y, z],
            first_ts=round(matches[0].ts, 3),
            count=len(matches),
        )

    def _resolve_region(
        self, room_id: int | None, region: list[float] | None
    ) -> tuple[NDArray[np.float64], str, StoredRoomSet | None]:
        """Resolve a queried region to (polygon, human label, latest room set).

        Raises:
            ValueError: neither/malformed input, or an unknown room id.
            LookupError: a room id was given but no rooms were derived yet.
        """
        if room_id is not None and region is not None:
            raise ValueError("Give either room_id or region, not both")
        with RoomStore(self.config.sightings_db) as store:
            room_set = store.latest()
        if region is not None:
            return polygon_from_flat(region), "the given region", room_set
        if room_id is None:
            raise ValueError("Provide either a room_id (see rooms()) or a region polygon")
        if room_set is None:
            raise LookupError("No rooms derived yet — call derive_rooms first.")
        room = next((r for r in room_set.rooms if r.id == room_id), None)
        if room is None:
            raise ValueError(
                f"No room with id {room_id}; known ids: {[r.id for r in room_set.rooms]}"
            )
        return room.polygon, f"room {room_id}", room_set

    def _membership(
        self,
        points_xy: NDArray[np.float64],
        polygon: NDArray[np.float64],
        room_id: int | None,
        room_set: StoredRoomSet | None,
    ) -> NDArray[np.bool_]:
        """Which points count as inside the queried region.

        Room queries use exclusive nearest-room assignment (a wall-adjacent
        sighting belongs to exactly one room); raw-polygon queries accept
        anything inside or within the snap distance of the outline.
        """
        if len(points_xy) == 0:
            return np.zeros(0, dtype=bool)
        if room_id is not None and room_set is not None:
            assigned = assign_to_rooms(points_xy, room_set.rooms, self.config.sighting_snap_m)
            member: NDArray[np.bool_] = assigned == room_id
            return member
        inside = points_in_polygon(points_xy, polygon)
        near = distance_to_polygon(points_xy, polygon) <= self.config.sighting_snap_m
        return inside | near

    def _region_coverage(
        self,
        polygon: NDArray[np.float64],
        room_id: int | None,
        room_set: StoredRoomSet | None,
        log: SightingsLog,
    ) -> tuple[RegionCoverage, dict[str, Any]]:
        """Coverage of the queried region, plus metadata for a "never" answer."""
        events = log.scan_events()
        everything = log.sightings()
        trail = self._trail_or_none()
        xy = _sightings_xy(everything)
        member = self._membership(xy, polygon, room_id, room_set)
        coverage = region_scan_coverage(polygon, events, everything, member, trail)
        meta: dict[str, Any] = {
            "scan_passes": coverage.scan_passes,
            "scan_passes_covering_region": coverage.passes_covering_region,
            "region_last_scanned_ts": (
                round(coverage.last_covered_ts, 3) if coverage.last_covered_ts is not None else None
            ),
        }
        if room_set is not None:
            assigned = assign_to_rooms(xy, room_set.rooms, self.config.sighting_snap_m)
            meta["rooms_with_scan_coverage"] = [
                r.id
                for r in room_set.rooms
                if region_scan_coverage(
                    r.polygon, events, everything, assigned == r.id, trail
                ).passes_covering_region
            ]
        return coverage, meta

    @skill
    def last_seen_object_in_region(
        self, name: str, room_id: int | None = None, region: list[float] | None = None
    ) -> SkillResult[CommonSkillError]:
        """When and where did you last see an object in a specific room/region?

        Answers from the sightings log. If the object was later seen somewhere
        else, the answer is still its last sighting inside the queried region.
        Identify the region by room_id (from rooms()) or by a raw polygon.

        Args:
            name: Object type name exactly as scanned, e.g. "couch".
            room_id: Room id from the rooms() skill.
            region: Region polygon in world coordinates, instead of room_id:
                flattened [x1, y1, x2, y2, ...] (at least 3 vertices).
        """
        try:
            polygon, label, room_set = self._resolve_region(room_id, region)
        except ValueError as e:
            return SkillResult.fail("INVALID_INPUT", str(e))
        except LookupError as e:
            return SkillResult.fail("INVALID_STATE", str(e))
        with SightingsLog(self.config.sightings_db) as log:
            matches = log.sightings(name)
            inside = self._membership(_sightings_xy(matches), polygon, room_id, room_set)
            if not matches:
                ever_in_vocab = log.ever_in_vocabulary(name)
                known = sorted(log.names())
                coverage, coverage_meta = self._region_coverage(polygon, room_id, room_set, log)
                return SkillResult.ok(
                    f"No sightings of '{name}' anywhere. "
                    f"{_vocabulary_sentence(name, ever_in_vocab)} "
                    f"Objects sighted so far: {known}.",
                    in_region_count=0,
                    ever_in_vocabulary=ever_in_vocab,
                    known_names=known,
                    **coverage_meta,
                )
            if not inside.any():
                ever_in_vocab = log.ever_in_vocabulary(name)
                coverage, coverage_meta = self._region_coverage(polygon, room_id, room_set, log)
                last = matches[-1]
                x, y = round(last.position[0], 2), round(last.position[1], 2)
                return SkillResult.ok(
                    f"Never saw '{name}' in {label} — though it was sighted "
                    f"{len(matches)} time(s) elsewhere, last at {_iso(last.ts)} UTC "
                    f"at ({x}, {y}). {_coverage_sentence(coverage, label)}",
                    in_region_count=0,
                    ever_in_vocabulary=ever_in_vocab,
                    last_elsewhere_ts=round(last.ts, 3),
                    **coverage_meta,
                )
        in_region = [s for s, flag in zip(matches, inside.tolist(), strict=True) if flag]
        intervals = visit_intervals(
            np.asarray([s.ts for s in matches]), inside, max_gap_s=SIGHTING_VISIT_GAP_S
        )
        last_in = in_region[-1]
        enter, exit_ = intervals[-1]
        later_elsewhere = [
            s for s, flag in zip(matches, inside.tolist(), strict=True) if not flag and s.ts > exit_
        ]
        note = ""
        later_meta: dict[str, Any] = {}
        if later_elsewhere:
            note = (
                f" Note: '{name}' was later seen outside {label}, most recently at "
                f"{_iso(later_elsewhere[-1].ts)} UTC."
            )
            later_meta["later_elsewhere_ts"] = round(later_elsewhere[-1].ts, 3)
        x, y, z = (round(v, 2) for v in last_in.position)
        return SkillResult.ok(
            f"Last saw '{name}' in {label} at {_iso(last_in.ts)} UTC at ({x}, {y}, {z}); "
            f"that stay spanned {_iso(enter)}..{_iso(exit_)} UTC. "
            f"{len(in_region)} sighting(s) in the region total.{note}",
            last_ts=round(last_in.ts, 3),
            position=[x, y, z],
            last_interval=[round(enter, 3), round(exit_, 3)],
            visits=[[round(a, 3), round(b, 3)] for a, b in intervals],
            in_region_count=len(in_region),
            **later_meta,
        )

    @skill
    def object_ever_in_region(
        self, name: str, room_id: int | None = None, region: list[float] | None = None
    ) -> SkillResult[CommonSkillError]:
        """Has an object ever been seen in a specific room or region?

        A "never" answer is qualified by coverage — whether any scan pass
        actually covered the region, and whether the object type was ever in
        a scan's vocabulary — because absence of sightings only counts as
        evidence when the region was scanned for that object type.

        Args:
            name: Object type name exactly as scanned, e.g. "couch".
            room_id: Room id from the rooms() skill.
            region: Region polygon in world coordinates, instead of room_id:
                flattened [x1, y1, x2, y2, ...] (at least 3 vertices).
        """
        try:
            polygon, label, room_set = self._resolve_region(room_id, region)
        except ValueError as e:
            return SkillResult.fail("INVALID_INPUT", str(e))
        except LookupError as e:
            return SkillResult.fail("INVALID_STATE", str(e))
        with SightingsLog(self.config.sightings_db) as log:
            matches = log.sightings(name)
            inside = self._membership(_sightings_xy(matches), polygon, room_id, room_set)
            if inside.any():
                in_region = [s for s, flag in zip(matches, inside.tolist(), strict=True) if flag]
                intervals = visit_intervals(
                    np.asarray([s.ts for s in matches]), inside, max_gap_s=SIGHTING_VISIT_GAP_S
                )
                return SkillResult.ok(
                    f"Yes — '{name}' was seen in {label}: {len(in_region)} sighting(s) "
                    f"between {_iso(in_region[0].ts)} and {_iso(in_region[-1].ts)} UTC.",
                    ever_seen_in_region=True,
                    first_ts=round(in_region[0].ts, 3),
                    last_ts=round(in_region[-1].ts, 3),
                    visits=[[round(a, 3), round(b, 3)] for a, b in intervals],
                    in_region_count=len(in_region),
                )
            ever_in_vocab = log.ever_in_vocabulary(name)
            coverage, coverage_meta = self._region_coverage(polygon, room_id, room_set, log)
        elsewhere = (
            f" It was seen {len(matches)} time(s) outside the region, "
            f"last at {_iso(matches[-1].ts)} UTC."
            if matches
            else ""
        )
        return SkillResult.ok(
            f"No — '{name}' was never seen in {label}.{elsewhere} "
            f"{_vocabulary_sentence(name, ever_in_vocab)} {_coverage_sentence(coverage, label)}",
            ever_seen_in_region=False,
            ever_in_vocabulary=ever_in_vocab,
            sightings_elsewhere=len(matches),
            **coverage_meta,
        )

    @skill
    def derive_rooms(self) -> SkillResult[CommonSkillError]:
        """Segment the current occupancy map into rooms and remember them.

        Runs room segmentation over the latest global costmap and persists
        the result (room polygons, areas, doorways). Call this after the map
        has grown before asking room questions.
        """
        with self._grid_lock:
            grid = self._latest_grid
        if grid is None:
            return SkillResult.fail(
                "INVALID_STATE", "No occupancy map received yet — is mapping running?"
            )
        segmentation = segment_rooms(grid)
        with RoomStore(self.config.sightings_db) as store:
            store.save(segmentation, source="scene_memory.derive_rooms")
        rooms = segmentation.rooms()
        corridors = segmentation.corridors()
        return SkillResult.ok(
            f"Derived {len(rooms)} room(s) and {len(corridors)} corridor(s) from the "
            f"map ({segmentation.explored_fraction:.0%} explored — the count can rise "
            f"as more area is mapped).",
            n_rooms=len(rooms),
            n_corridors=len(corridors),
            n_doorways=len(segmentation.doorways),
            explored_fraction=segmentation.explored_fraction,
            derived_ts=round(segmentation.derived_ts, 3),
        )

    @skill
    def rooms(self) -> SkillResult[CommonSkillError]:
        """How many rooms are there? Lists the remembered rooms with areas.

        Reads the last derive_rooms result. The count is a lower bound while
        the map is partially explored.
        """
        with RoomStore(self.config.sightings_db) as store:
            room_set = store.latest()
        if room_set is None:
            return SkillResult.fail(
                "INVALID_STATE", "No rooms derived yet — call derive_rooms first."
            )
        rooms = room_set.by_kind("room")
        corridors = room_set.by_kind("corridor")
        return SkillResult.ok(
            f"{len(rooms)} room(s) and {len(corridors)} corridor(s) known, from a map "
            f"{room_set.explored_fraction:.0%} explored (counts are lower bounds on a "
            f"partial map).",
            rooms=[
                {
                    "id": r.id,
                    "kind": r.kind,
                    "area_m2": r.area_m2,
                    "centroid": [round(r.centroid_xy[0], 2), round(r.centroid_xy[1], 2)],
                }
                for r in room_set.rooms
            ],
            n_doorways=len(room_set.doorways),
            explored_fraction=room_set.explored_fraction,
            derived_ts=round(room_set.derived_ts, 3),
        )
