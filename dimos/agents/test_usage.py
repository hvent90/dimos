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

"""UsageTracker: per-trajectory token and call accounting."""

from uuid import uuid4

from langchain_core.messages import AIMessage
from langchain_core.outputs import ChatGeneration, LLMResult

from dimos.agents.usage import UsageTracker


def _llm_result(model: str, input_tokens: int, output_tokens: int) -> LLMResult:
    message = AIMessage(
        content="ok",
        usage_metadata={
            "input_tokens": input_tokens,
            "output_tokens": output_tokens,
            "total_tokens": input_tokens + output_tokens,
        },
        response_metadata={"model_name": model},
    )
    return LLMResult(generations=[[ChatGeneration(message=message)]])


def test_totals_accumulate_across_calls() -> None:
    tracker = UsageTracker()
    tracker.on_llm_end(_llm_result("m1", 100, 20), run_id=uuid4())
    tracker.on_llm_end(_llm_result("m1", 50, 10), run_id=uuid4())
    tracker.on_tool_start({}, "arg", run_id=uuid4())
    tracker.on_tool_start({}, "arg", run_id=uuid4())
    tracker.on_tool_start({}, "arg", run_id=uuid4())
    assert tracker.totals() == {
        "input_tokens": 150,
        "output_tokens": 30,
        "total_tokens": 180,
        "llm_calls": 2,
        "tool_calls": 3,
    }


def test_totals_sum_over_models() -> None:
    tracker = UsageTracker()
    tracker.on_llm_end(_llm_result("m1", 100, 20), run_id=uuid4())
    tracker.on_llm_end(_llm_result("m2", 7, 3), run_id=uuid4())
    totals = tracker.totals()
    assert totals["input_tokens"] == 107
    assert totals["total_tokens"] == 130
    assert set(tracker.usage_metadata) == {"m1", "m2"}


def test_empty_trajectory_is_all_zero() -> None:
    assert UsageTracker().totals() == {
        "input_tokens": 0,
        "output_tokens": 0,
        "total_tokens": 0,
        "llm_calls": 0,
        "tool_calls": 0,
    }
