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

"""Drives the full MCP agent in-process for eval layer (c).

Builds the same agent the robot runs — same model, same system prompt,
same tools served by the running blueprint's MCP server — but inside the
eval process, so each trajectory's messages, token usage, and answer are
captured directly instead of scraped from daemon logs. The go2 system
prompt makes the agent deliver answers via ``speak`` tool calls, so the
answer is the final AI text plus every spoken line.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import httpx
from langchain.agents import create_agent
from langchain.chat_models import init_chat_model
from langchain_core.messages import AIMessage, HumanMessage
from langchain_core.messages.base import BaseMessage
from langchain_core.runnables import RunnableConfig
from langchain_core.tools import StructuredTool
from langchain_openai import ChatOpenAI

from dimos.agents.system_prompt import SYSTEM_PROMPT
from dimos.agents.usage import UsageTracker
from dimos.utils.sequential_ids import SequentialIds

DEFAULT_MCP_URL = "http://localhost:9990/mcp"
DEFAULT_AGENT_MODEL = "gpt-5.6-luna"

# Mirrors McpClient's model selection (dimos/agents/mcp/mcp_client.py) so the
# eval exercises the production model configuration.
_RESPONSES_REASONING_MODEL_PREFIXES = ("gpt-5", "o1", "o3", "o4")


def init_agent_model(model_name: str) -> Any:
    if ":" in model_name or not model_name.startswith(_RESPONSES_REASONING_MODEL_PREFIXES):
        return init_chat_model(model=model_name)
    return ChatOpenAI(
        model=model_name,
        use_responses_api=True,
        reasoning={"effort": "medium", "summary": "auto"},
    )


class McpConnection:
    """Minimal JSON-RPC client for a blueprint's MCP server over HTTP."""

    def __init__(self, url: str = DEFAULT_MCP_URL, timeout_s: float = 120.0) -> None:
        self._url = url
        self._client = httpx.Client(timeout=timeout_s)
        self._seq_ids = SequentialIds()

    def close(self) -> None:
        self._client.close()

    def request(self, method: str, params: dict[str, Any] | None = None) -> dict[str, Any]:
        body: dict[str, Any] = {"jsonrpc": "2.0", "id": self._seq_ids.next(), "method": method}
        if params is not None:
            body["params"] = params
        resp = self._client.post(self._url, json=body)
        resp.raise_for_status()
        data = resp.json()
        if "error" in data:
            raise RuntimeError(f"MCP error {data['error']['code']}: {data['error']['message']}")
        result: dict[str, Any] = data.get("result")
        return result

    def reachable(self) -> bool:
        try:
            self.request("initialize")
        except (httpx.ConnectError, httpx.RemoteProtocolError, httpx.TimeoutException):
            return False
        return True

    def fetch_tools(self) -> list[StructuredTool]:
        """The server's tools wrapped for langchain (text content only)."""
        raw_tools = self.request("tools/list").get("tools", [])
        return [self._to_langchain(t) for t in raw_tools]

    def _to_langchain(self, mcp_tool: dict[str, Any]) -> StructuredTool:
        name = mcp_tool["name"]

        def call_tool(**kwargs: Any) -> str:
            result = self.request("tools/call", {"name": name, "arguments": kwargs})
            content = result.get("content", [])
            return "\n".join(c.get("text", "") for c in content if c.get("type") == "text")

        return StructuredTool(
            name=name,
            description=mcp_tool.get("description", ""),
            func=call_tool,
            args_schema=mcp_tool.get("inputSchema", {"type": "object", "properties": {}}),
        )


@dataclass(frozen=True)
class Trajectory:
    """One agent run: full message list, extracted answer, usage totals."""

    messages: list[BaseMessage]
    answer: str
    usage: dict[str, int]


def _content_text(content: str | list[Any]) -> str:
    """Plain text of a message: pass strings through, join list-form text
    blocks (the responses API returns reasoning + text block lists)."""
    if isinstance(content, str):
        return content.strip()
    parts = [b.get("text", "") for b in content if isinstance(b, dict) and b.get("type") == "text"]
    return "\n".join(p for p in parts if p).strip()


def extract_answer(messages: list[BaseMessage]) -> str:
    """The agent's user-facing answer: spoken lines plus the final AI text."""
    parts = []
    for msg in messages:
        if isinstance(msg, AIMessage):
            for call in msg.tool_calls:
                if call["name"] == "speak" and call["args"].get("text"):
                    parts.append(f"(spoken) {call['args']['text']}")
    final = messages[-1] if messages else None
    if isinstance(final, AIMessage):
        text = _content_text(final.content)
        if text:
            parts.append(text)
    return "\n".join(parts)


def run_trajectory(agent: Any, question: str, recursion_limit: int = 50) -> Trajectory:
    """Ask the agent one question and capture the full trajectory."""
    tracker = UsageTracker()
    config: RunnableConfig = {"callbacks": [tracker], "recursion_limit": recursion_limit}
    state = agent.invoke({"messages": [HumanMessage(content=question)]}, config=config)
    messages: list[BaseMessage] = state["messages"]
    return Trajectory(messages=messages, answer=extract_answer(messages), usage=tracker.totals())


def build_agent(tools: list[StructuredTool], model: str = DEFAULT_AGENT_MODEL) -> Any:
    """The production agent shape: same model default and system prompt."""
    return create_agent(model=init_agent_model(model), tools=tools, system_prompt=SYSTEM_PROMPT)


def serialize_messages(messages: list[BaseMessage]) -> list[dict[str, Any]]:
    """Compact JSON-safe trace of a trajectory for the results file."""
    out = []
    for msg in messages:
        entry: dict[str, Any] = {"type": msg.type}
        entry["content"] = (
            msg.content if isinstance(msg.content, str) else _content_text(msg.content)
        )
        if isinstance(msg, AIMessage) and msg.tool_calls:
            entry["tool_calls"] = [{"name": c["name"], "args": c["args"]} for c in msg.tool_calls]
        out.append(entry)
    return out
