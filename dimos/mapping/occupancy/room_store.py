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

"""Persistence for derived room segmentations (memory2 SQLite).

Each ``save`` writes one ``room_derivations`` row (doorways, coverage) and
one ``rooms`` row per region, all stamped with the derivation's grid
timestamp. ``latest`` returns the most recent derivation — earlier ones
are kept as history, never overwritten.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from types import TracebackType
from typing import Any

import numpy as np
from numpy.typing import NDArray

from dimos.mapping.occupancy.room_segmentation import RoomSegmentation
from dimos.memory2.store.sqlite import SqliteStore

ROOMS_STREAM = "rooms"
DERIVATIONS_STREAM = "room_derivations"


@dataclass(frozen=True)
class StoredRoom:
    """One persisted region of a room derivation."""

    id: int
    kind: str  # "room" | "corridor"
    area_m2: float
    centroid_xy: tuple[float, float]
    anchor_xy: tuple[float, float]
    max_clearance_m: float
    polygon: NDArray[np.float64]  # (N, 2) world xy
    derived_ts: float


@dataclass(frozen=True)
class StoredRoomSet:
    """The full result of one derivation pass."""

    derived_ts: float
    source: str
    explored_fraction: float
    rooms: tuple[StoredRoom, ...]
    doorways: tuple[dict[str, Any], ...]  # {"between": [a, b], "xy": [x, y], "width_m": w}

    def by_kind(self, kind: str) -> tuple[StoredRoom, ...]:
        return tuple(r for r in self.rooms if r.kind == kind)

    @classmethod
    def from_segmentation(cls, segmentation: RoomSegmentation, source: str) -> StoredRoomSet:
        """The in-memory equivalent of save-then-latest for one derivation."""
        return cls(
            derived_ts=segmentation.derived_ts,
            source=source,
            explored_fraction=segmentation.explored_fraction,
            rooms=tuple(
                StoredRoom(
                    id=region.id,
                    kind=region.kind,
                    area_m2=region.area_m2,
                    centroid_xy=region.centroid_xy,
                    anchor_xy=region.anchor_xy,
                    max_clearance_m=region.max_clearance_m,
                    polygon=region.polygon,
                    derived_ts=segmentation.derived_ts,
                )
                for region in segmentation.regions
            ),
            doorways=tuple(
                {
                    "between": list(d.between),
                    "xy": [round(v, 3) for v in d.position_xy],
                    "width_m": d.approx_width_m,
                }
                for d in segmentation.doorways
            ),
        )


class RoomStore:
    """Append/query API over persisted room derivations. Context manager."""

    def __init__(self, path: str | Path) -> None:
        self._store = SqliteStore(path=str(path))

    def __enter__(self) -> RoomStore:
        self._store.start()
        return self

    def __exit__(
        self,
        exctype: type[BaseException] | None,
        excinst: BaseException | None,
        exctb: TracebackType | None,
    ) -> None:
        self._store.stop()

    def save(self, segmentation: RoomSegmentation, source: str) -> None:
        derivation_id = f"{segmentation.derived_ts:.6f}"
        rooms = self._store.stream(ROOMS_STREAM, str)
        for region in segmentation.regions:
            rooms.append(
                region.kind,
                ts=segmentation.derived_ts,
                pose=(region.centroid_xy[0], region.centroid_xy[1], 0.0),
                tags={
                    "derivation_id": derivation_id,
                    "room_id": region.id,
                    "kind": region.kind,
                    "area_m2": region.area_m2,
                    "anchor_xy": list(region.anchor_xy),
                    "max_clearance_m": region.max_clearance_m,
                    "polygon": [round(float(v), 3) for v in region.polygon.ravel()],
                },
            )
        self._store.stream(DERIVATIONS_STREAM, str).append(
            source,
            ts=segmentation.derived_ts,
            tags={
                "derivation_id": derivation_id,
                "n_rooms": len(segmentation.rooms()),
                "n_corridors": len(segmentation.corridors()),
                "explored_fraction": segmentation.explored_fraction,
                "doorways": [
                    {
                        "between": list(d.between),
                        "xy": [round(v, 3) for v in d.position_xy],
                        "width_m": d.approx_width_m,
                    }
                    for d in segmentation.doorways
                ],
            },
        )

    def latest(self) -> StoredRoomSet | None:
        try:
            derivation = self._store.stream(DERIVATIONS_STREAM, str).last()
        except LookupError:
            return None
        derivation_id = str(derivation.tags["derivation_id"])
        rooms = []
        for obs in (
            self._store.stream(ROOMS_STREAM, str).tags(derivation_id=derivation_id).order_by("ts")
        ):
            assert obs.pose_tuple is not None
            flat = obs.tags["polygon"]
            anchor = obs.tags["anchor_xy"]
            rooms.append(
                StoredRoom(
                    id=int(obs.tags["room_id"]),
                    kind=str(obs.tags["kind"]),
                    area_m2=float(obs.tags["area_m2"]),
                    centroid_xy=(obs.pose_tuple[0], obs.pose_tuple[1]),
                    anchor_xy=(float(anchor[0]), float(anchor[1])),
                    max_clearance_m=float(obs.tags["max_clearance_m"]),
                    polygon=np.asarray(flat, dtype=np.float64).reshape(-1, 2),
                    derived_ts=obs.ts,
                )
            )
        rooms.sort(key=lambda r: r.id)
        return StoredRoomSet(
            derived_ts=derivation.ts,
            source=str(derivation.data),
            explored_fraction=float(derivation.tags.get("explored_fraction", 0.0)),
            rooms=tuple(rooms),
            doorways=tuple(derivation.tags.get("doorways", [])),
        )
