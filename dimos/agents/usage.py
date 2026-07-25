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

"""Token and step accounting for langchain/langgraph agent trajectories.

Attach a :class:`UsageTracker` to any ``invoke``/``stream`` call via
``config={"callbacks": [tracker]}``; afterwards :meth:`UsageTracker.totals`
reports how many tokens, model calls, and tool calls the trajectory cost.
Per-model token detail stays available on ``usage_metadata`` (from the
langchain base class).
"""

from __future__ import annotations

import threading
from typing import Any

from langchain_core.callbacks import UsageMetadataCallbackHandler
from langchain_core.outputs import LLMResult


class UsageTracker(UsageMetadataCallbackHandler):
    """Aggregates token usage plus LLM/tool call counts for one trajectory."""

    def __init__(self) -> None:
        super().__init__()
        self._counter_lock = threading.Lock()
        self._llm_calls = 0
        self._tool_calls = 0

    def on_llm_end(self, response: LLMResult, **kwargs: Any) -> None:
        with self._counter_lock:
            self._llm_calls += 1
        super().on_llm_end(response, **kwargs)

    def on_tool_start(self, serialized: dict[str, Any], input_str: str, **kwargs: Any) -> Any:
        with self._counter_lock:
            self._tool_calls += 1

    def totals(self) -> dict[str, int]:
        """Tokens summed across models, plus how many LLM and tool calls ran."""
        with self._counter_lock:
            llm_calls, tool_calls = self._llm_calls, self._tool_calls
        usage = list(self.usage_metadata.values())
        return {
            "input_tokens": sum(u.get("input_tokens", 0) for u in usage),
            "output_tokens": sum(u.get("output_tokens", 0) for u in usage),
            "total_tokens": sum(u.get("total_tokens", 0) for u in usage),
            "llm_calls": llm_calls,
            "tool_calls": tool_calls,
        }
