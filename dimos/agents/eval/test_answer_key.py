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

"""Answer-key YAML round-trip and the confirmation workflow."""

from pathlib import Path

import pytest

from dimos.agents.eval.answer_key import (
    AnswerKey,
    CaseEntry,
    ObjectEntry,
    RoomsEntry,
    RoomStay,
    load_answer_key,
    save_answer_key,
)


def _key() -> AnswerKey:
    return AnswerKey(
        recording="go2_short",
        trail_start_ts=1000.0,
        trail_end_ts=1060.0,
        vocabulary=["chair", "couch"],
        rooms=RoomsEntry(n_rooms=13, n_corridors=1, explored_fraction=0.53, source="offline"),
        objects=[
            ObjectEntry(
                name="couch",
                sightings=8,
                first_ts=1042.9,
                last_ts=1054.6,
                last_position=[-1.4, 4.8, 0.2],
                rooms=[
                    RoomStay(
                        room_id=2, first_ts=1042.9, last_ts=1046.8, intervals=[[1042.9, 1046.8]]
                    )
                ],
            )
        ],
        cases=[
            CaseEntry(
                id="q2_last_seen",
                query=2,
                question="When did you last see a couch?",
                skill="last_seen_object",
                skill_args={"name": "couch"},
                expected={"last_ts": 1054.6},
            )
        ],
    )


def test_yaml_round_trip(tmp_path: Path) -> None:
    key = _key()
    path = tmp_path / "key.yaml"
    save_answer_key(key, path)
    assert load_answer_key(path) == key


def test_unconfirmed_lists_every_draft_entry() -> None:
    assert _key().unconfirmed() == ["q2_last_seen", "couch", "rooms"]


def test_confirmed_entries_drop_out(tmp_path: Path) -> None:
    key = _key()
    key.cases[0].confirmed = True
    key.rooms.confirmed = True
    assert key.unconfirmed() == ["couch"]


def test_case_lookup_by_id() -> None:
    key = _key()
    assert key.case("q2_last_seen").skill == "last_seen_object"
    with pytest.raises(KeyError, match="q9_missing"):
        key.case("q9_missing")
