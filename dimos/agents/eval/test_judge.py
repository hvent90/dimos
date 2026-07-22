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

"""Judge verdict validation and prompt rendering (no live LLM)."""

from pydantic import ValidationError
import pytest

from dimos.agents.eval.answer_key import AnswerKey, CaseEntry, RoomsEntry
from dimos.agents.eval.judge import JudgeVerdict, render_judge_prompt

KEY = AnswerKey(
    recording="go2_short",
    trail_start_ts=1_000_000.0,
    trail_end_ts=1_000_060.0,
    vocabulary=["couch"],
    rooms=RoomsEntry(n_rooms=2, n_corridors=0, explored_fraction=0.5, source="test"),
    objects=[],
    cases=[],
)

CASE = CaseEntry(
    id="q4_last_seen_in_room",
    query=4,
    question="When did you last see a couch in room 2?",
    skill="last_seen_object_in_region",
    skill_args={"name": "couch", "room_id": 2},
    expected={"last_in_room_ts": 1_000_046.8, "global_last_ts": 1_000_054.6},
    grading_notes="TRAP: answering the later elsewhere time scores 0.0.",
)


def test_verdict_scores_are_the_three_grades() -> None:
    assert JudgeVerdict(score=1.0, rationale="right").score == 1.0
    assert JudgeVerdict(score=0.5, rationale="earlier interval").score == 0.5
    with pytest.raises(ValidationError):
        JudgeVerdict(score=0.7, rationale="not a valid grade")


def test_prompt_contains_question_reference_and_answer() -> None:
    prompt = render_judge_prompt(CASE, KEY, "I last saw it at 11:34:06 UTC.")
    assert CASE.question in prompt
    assert '"last_in_room_ts": 1000046.8' in prompt
    assert "TRAP" in prompt
    assert "I last saw it at 11:34:06 UTC." in prompt
    assert "confirmed by a human: False" in prompt


def test_prompt_converts_expected_timestamps_to_utc() -> None:
    prompt = render_judge_prompt(CASE, KEY, "answer")
    # 1_000_046.8 epoch = 1970-01-12 13:47:26 UTC; the judge needs the
    # conversion to compare clock-time answers.
    assert "last_in_room_ts 1000046.8 = 1970-01-12 13:47:26 UTC" in prompt
    assert "trail start 1000000.0 = 1970-01-12 13:46:40 UTC" in prompt


def test_prompt_marks_empty_answer() -> None:
    assert "(no answer produced)" in render_judge_prompt(CASE, KEY, "")
