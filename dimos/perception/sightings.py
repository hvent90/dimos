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

"""Persistent log of object sightings, one row per observation.

Tracks in ``WorldBelief`` (and per-frame detections generally) are RAM-only:
``last_seen_ts`` dies with the process, and a single latest-timestamp field
can't answer "when did you last see X *in Y*" once X moved elsewhere. This
log keeps the full history in a memory2 SQLite stream — ts + world position
per row, with the active detection vocabulary attached so a later "never
saw X" answer can be qualified by whether X was ever looked for.

Two streams in one store: ``sightings`` (one row per object observation)
and ``scan_events`` (one row per scan pass — its time window and vocabulary,
recorded even when nothing was detected; the coverage substrate).
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from types import TracebackType
from typing import Any

from dimos.constants import STATE_DIR
from dimos.memory2.store.sqlite import SqliteStore
from dimos.memory2.type.observation import Observation

SIGHTINGS_STREAM = "sightings"
SCAN_EVENTS_STREAM = "scan_events"
DEFAULT_SIGHTINGS_DB = STATE_DIR / "scene_memory" / "sightings.db"


@dataclass(frozen=True)
class Sighting:
    """One observation of a named object at a world position and time."""

    name: str
    ts: float
    position: tuple[float, float, float]
    object_id: str = ""
    confidence: float | None = None
    source: str = ""
    vocabulary: tuple[str, ...] = ()


@dataclass(frozen=True)
class ScanEvent:
    """One scan pass: what vocabulary was looked for, over which time window."""

    ts: float  # end of the scanned window
    t0: float  # start of the scanned window
    vocabulary: tuple[str, ...]
    source: str
    frames: int
    sightings: int


def _to_sighting(obs: Observation[Any]) -> Sighting:
    assert obs.pose_tuple is not None
    return Sighting(
        name=str(obs.data),
        ts=obs.ts,
        position=(obs.pose_tuple[0], obs.pose_tuple[1], obs.pose_tuple[2]),
        object_id=str(obs.tags.get("object_id", "")),
        confidence=obs.tags.get("confidence"),
        source=str(obs.tags.get("source", "")),
        vocabulary=tuple(obs.tags.get("vocabulary", ())),
    )


class SightingsLog:
    """Append/query API over the sightings store. Use as a context manager."""

    def __init__(self, path: str | Path = DEFAULT_SIGHTINGS_DB) -> None:
        self._store = SqliteStore(path=str(path))

    def __enter__(self) -> SightingsLog:
        self._store.start()
        return self

    def __exit__(
        self,
        exctype: type[BaseException] | None,
        excinst: BaseException | None,
        exctb: TracebackType | None,
    ) -> None:
        self._store.stop()

    def record_scan(
        self,
        sightings: list[Sighting],
        *,
        t0: float,
        t1: float,
        vocabulary: list[str],
        source: str,
        frames: int,
    ) -> None:
        """Append one scan pass: its sightings plus the coverage event."""
        stream = self._store.stream(SIGHTINGS_STREAM, str)
        for s in sightings:
            tags: dict[str, Any] = {
                "object_id": s.object_id,
                "source": source,
                "vocabulary": list(vocabulary),
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
                "sightings": len(sightings),
            },
        )

    def sightings(self, name: str | None = None) -> list[Sighting]:
        """All sightings in ts order; ``name`` filters case-insensitively."""
        rows = [
            _to_sighting(obs) for obs in self._store.stream(SIGHTINGS_STREAM, str).order_by("ts")
        ]
        if name is None:
            return rows
        wanted = name.strip().lower()
        return [s for s in rows if s.name.lower() == wanted]

    def last(self, name: str) -> Sighting | None:
        matches = self.sightings(name)
        return matches[-1] if matches else None

    def names(self) -> dict[str, int]:
        """Distinct sighted names with their observation counts."""
        counts: dict[str, int] = {}
        for s in self.sightings():
            counts[s.name] = counts.get(s.name, 0) + 1
        return counts

    def scan_events(self) -> list[ScanEvent]:
        """All scan passes in ts order (coverage: what was looked for, when)."""
        return [
            ScanEvent(
                ts=obs.ts,
                t0=float(obs.tags.get("t0", obs.ts)),
                vocabulary=tuple(obs.tags.get("vocabulary", ())),
                source=str(obs.data),
                frames=int(obs.tags.get("frames", 0)),
                sightings=int(obs.tags.get("sightings", 0)),
            )
            for obs in self._store.stream(SCAN_EVENTS_STREAM, str).order_by("ts")
        ]

    def ever_in_vocabulary(self, name: str) -> bool:
        """Was ``name`` ever part of a scan's detection vocabulary?"""
        wanted = name.strip().lower()
        return any(wanted in (v.lower() for v in event.vocabulary) for event in self.scan_events())
