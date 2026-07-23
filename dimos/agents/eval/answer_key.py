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

"""Answer keys for the scene-memory eval, stored as reviewable YAML.

A key is generated programmatically as a DRAFT (see
``tool_generate_answer_key.py``) with every entry ``confirmed: false``.
A human flips entries to ``confirmed: true`` after checking them against
the recording — generated labels must never be presented as verified
ground truth, and the system must never be tuned against unconfirmed
labels. All timestamps are recording-epoch seconds (store-row ts).
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

from pydantic import BaseModel
import yaml


class RoomStay(BaseModel):
    """One object's presence in one room: sighting intervals within it."""

    room_id: str  # scene-graph node id, e.g. "room_3"
    first_ts: float
    last_ts: float
    intervals: list[list[float]]


class ObjectEntry(BaseModel):
    """One detected object type: visibility intervals and room assignments."""

    name: str
    sightings: int
    first_ts: float
    last_ts: float
    last_position: list[float]
    last_room_id: str = ""  # region of the last sighting ("" = none)
    rooms: list[RoomStay]
    confirmed: bool = False


class RoomsEntry(BaseModel):
    """The seeded room segmentation the eval runs against."""

    n_rooms: int
    n_corridors: int
    explored_fraction: float
    source: str
    confirmed: bool = False


class CaseEntry(BaseModel):
    """One eval case: the question, how to answer it, and the expected answer."""

    id: str
    query: int
    question: str
    skill: str
    skill_args: dict[str, Any]
    expected: dict[str, Any]
    grading_notes: str = ""
    confirmed: bool = False


class AnswerKey(BaseModel):
    """DRAFT-by-default answer key for one recording."""

    recording: str
    time_base: str = "recording-epoch seconds (store-row ts)"
    trail_start_ts: float
    trail_end_ts: float
    vocabulary: list[str]
    rooms: RoomsEntry
    objects: list[ObjectEntry]
    cases: list[CaseEntry]

    def case(self, case_id: str) -> CaseEntry:
        match = next((c for c in self.cases if c.id == case_id), None)
        if match is None:
            raise KeyError(f"No case {case_id!r}; have {[c.id for c in self.cases]}")
        return match

    def unconfirmed(self) -> list[str]:
        """Labels a human has not yet confirmed (case ids and entry names)."""
        out = [c.id for c in self.cases if not c.confirmed]
        out.extend(o.name for o in self.objects if not o.confirmed)
        if not self.rooms.confirmed:
            out.append("rooms")
        return out


def load_answer_key(path: str | Path) -> AnswerKey:
    return AnswerKey.model_validate(yaml.safe_load(Path(path).read_text()))


def save_answer_key(key: AnswerKey, path: str | Path) -> None:
    Path(path).write_text(yaml.safe_dump(key.model_dump(), sort_keys=False, width=100))
