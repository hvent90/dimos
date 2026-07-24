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

"""LLM judge for scene-memory eval answers: graded, not binary.

The judge compares an agent's natural-language answer against the answer
key and returns a structured verdict — 1.0 (right), 0.5 (real but not the
latest interval, or correct-but-unqualified), 0.0 (wrong) — plus a
``hallucinated_never`` flag for answers that assert something never
happened although the reference shows it did. Prompt rendering is pure so
it can be unit-tested; only :func:`judge_answer` talks to a model.
"""

from __future__ import annotations

import json

from langchain.chat_models import init_chat_model
from langchain_core.messages import HumanMessage, SystemMessage
from pydantic import BaseModel, Field, field_validator

from dimos.agents.eval.answer_key import AnswerKey, CaseEntry
from dimos.agents.eval.scene_eval_cases import iso_utc

DEFAULT_JUDGE_MODEL = "gpt-5.6-luna"

JUDGE_SYSTEM_PROMPT = """\
You grade a robot agent's answer to a question about its spatio-temporal memory
of a recorded run. You get the question, reference data (the expected answer
plus grading notes), and the agent's answer. Timestamps in the reference are
epoch seconds in the recording's time base; UTC clock equivalents are given so
you can compare against answers phrased either way. Treat times within ~2
seconds of a reference time, or inside a referenced interval, as matching.

Score exactly one of:
- 1.0 — the answer matches the reference (the right time/interval, count, or a
  correctly qualified negative), per the grading notes.
- 0.5 — partially right as defined by the grading notes (e.g. a real but
  earlier interval, or a correct answer missing a required qualifier).
- 0.0 — wrong, fabricated, or non-responsive.

Set hallucinated_never=true only when the answer asserts something never
happened or was never seen although the reference shows it did.

If the reference entry is marked unconfirmed, grade against it anyway (the
caller reports confirmation status separately). Be strict: vague answers that
avoid committing to a time or count are non-responsive.
"""


GRADES = (1.0, 0.5, 0.0)


class JudgeVerdict(BaseModel):
    """The judge's structured grade for one answer."""

    score: float = Field(description="Exactly one of 1.0, 0.5, or 0.0.")
    hallucinated_never: bool = False
    rationale: str = Field(description="One or two sentences justifying the score.")

    @field_validator("score")
    @classmethod
    def _score_is_a_grade(cls, v: float) -> float:
        if v not in GRADES:
            raise ValueError(f"score must be one of {GRADES}, got {v}")
        return v


def render_judge_prompt(case: CaseEntry, key: AnswerKey, answer: str) -> str:
    """The human-turn judge prompt for one case. Pure — unit-testable."""
    expected = dict(case.expected)
    ts_notes = [
        f"trail start {key.trail_start_ts} = {iso_utc(key.trail_start_ts)} UTC",
        f"trail end {key.trail_end_ts} = {iso_utc(key.trail_end_ts)} UTC",
    ]
    for field, value in case.expected.items():
        if field.endswith("_ts") and isinstance(value, (int, float)):
            ts_notes.append(f"{field} {value} = {iso_utc(value)} UTC")
    return (
        f"QUESTION the agent was asked:\n{case.question}\n\n"
        f"REFERENCE (confirmed by a human: {case.confirmed}):\n"
        f"{json.dumps(expected, indent=1)}\n\n"
        f"Timestamp conversions:\n" + "\n".join(f"- {n}" for n in ts_notes) + "\n\n"
        f"GRADING NOTES:\n{case.grading_notes}\n\n"
        f"AGENT'S ANSWER:\n{answer or '(no answer produced)'}"
    )


def judge_answer(
    case: CaseEntry, key: AnswerKey, answer: str, model: str = DEFAULT_JUDGE_MODEL
) -> JudgeVerdict:
    """Grade one answer with the LLM judge."""
    llm = init_chat_model(model).with_structured_output(JudgeVerdict)
    verdict = llm.invoke(
        [
            SystemMessage(content=JUDGE_SYSTEM_PROMPT),
            HumanMessage(content=render_judge_prompt(case, key, answer)),
        ]
    )
    assert isinstance(verdict, JudgeVerdict)
    return verdict
