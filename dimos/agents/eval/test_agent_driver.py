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

"""Answer extraction and trajectory serialization (no network)."""

from langchain_core.messages import AIMessage, HumanMessage, ToolMessage

from dimos.agents.eval.agent_driver import extract_answer, serialize_messages

TRAJECTORY = [
    HumanMessage(content="When did you last see a couch?"),
    AIMessage(
        content="",
        tool_calls=[
            {"name": "last_seen_object", "args": {"name": "couch"}, "id": "1"},
            {"name": "speak", "args": {"text": "Checking my memory."}, "id": "2"},
        ],
    ),
    ToolMessage(content="Last saw 'couch' at 08:24:53 UTC.", tool_call_id="1"),
    ToolMessage(content="ok", tool_call_id="2"),
    AIMessage(
        content="",
        tool_calls=[{"name": "speak", "args": {"text": "At 08:24:53 UTC."}, "id": "3"}],
    ),
    ToolMessage(content="ok", tool_call_id="3"),
    AIMessage(content="I last saw a couch at 08:24:53 UTC at (-1.42, 4.76)."),
]


def test_extract_answer_collects_speak_and_final_text() -> None:
    assert extract_answer(TRAJECTORY) == (
        "(spoken) Checking my memory.\n"
        "(spoken) At 08:24:53 UTC.\n"
        "I last saw a couch at 08:24:53 UTC at (-1.42, 4.76)."
    )


def test_extract_answer_speak_only_trajectory() -> None:
    # The go2 prompt says users hear speech only — the final AI turn is often
    # empty, and the spoken lines are the whole answer.
    messages = [*TRAJECTORY[:-1], AIMessage(content="")]
    assert extract_answer(messages) == ("(spoken) Checking my memory.\n(spoken) At 08:24:53 UTC.")


def test_extract_answer_empty() -> None:
    assert extract_answer([]) == ""


def test_extract_answer_block_list_content() -> None:
    # The responses API returns content as reasoning + text block lists.
    messages = [
        AIMessage(
            content=[
                {"type": "reasoning", "encrypted_content": "opaque"},
                {"type": "text", "text": "Last seen at 08:24:48 UTC."},
            ]
        )
    ]
    assert extract_answer(messages) == "Last seen at 08:24:48 UTC."


def test_serialize_messages_keeps_tool_calls() -> None:
    trace = serialize_messages(TRAJECTORY)
    assert [t["type"] for t in trace] == ["human", "ai", "tool", "tool", "ai", "tool", "ai"]
    assert trace[1]["tool_calls"] == [
        {"name": "last_seen_object", "args": {"name": "couch"}},
        {"name": "speak", "args": {"text": "Checking my memory."}},
    ]
    assert trace[-1]["content"] == "I last saw a couch at 08:24:53 UTC at (-1.42, 4.76)."
