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

"""Run the scene-memory eval: five queries, three layers per case.

Layer (a) asserts the fact exists in storage (direct store queries),
layer (b) calls the skill directly, layer (c) asks the full MCP agent the
natural-language question and grades the answer with an LLM judge, logging
tokens and steps per trajectory. Layer (c) needs the replay daemon running
against the seeded scene DB (the tool prints the launch command when the
MCP server is unreachable and marks those cells NOT RUN)::

    uv run python dimos/agents/eval/tool_scene_eval.py \
        --key /tmp/scene_eval/go2_short/answer_key.yaml \
        --scene-db /tmp/scene_eval/go2_short/scene_memory.db \
        --out /tmp/scene_eval/go2_short
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

from dotenv import load_dotenv
import numpy as np

from dimos.agents.eval.agent_driver import (
    DEFAULT_MCP_URL,
    McpConnection,
    build_agent,
    run_trajectory,
    serialize_messages,
)
from dimos.agents.eval.answer_key import AnswerKey, CaseEntry, load_answer_key
from dimos.agents.eval.judge import DEFAULT_JUDGE_MODEL, judge_answer
from dimos.agents.eval.scene_eval_cases import object_entries
from dimos.agents.skills.scene_memory import (
    PoseTrail,
    SceneMemorySkillContainer,
    load_pose_trail,
    visit_intervals,
)
from dimos.mapping.occupancy.polygons import points_in_polygon, polygon_from_flat
from dimos.mapping.occupancy.room_store import RoomStore
from dimos.memory2.replay import resolve_db_path
from dimos.perception.scene_graph import SceneGraph


def check_storage(case: CaseEntry, key: AnswerKey, scene_db: Path, trail: PoseTrail) -> str:
    """Layer (a): does the expected fact exist in the stores? '' = pass."""
    expected = case.expected
    if case.query == 1:
        inside = points_in_polygon(trail.xy, polygon_from_flat(case.skill_args["region"]))
        visits = [[round(a, 3), round(b, 3)] for a, b in visit_intervals(trail.ts, inside)]
        if visits != expected["visits"]:
            return f"trail visits {visits} != expected {expected['visits']}"
        return ""
    if case.query == 2:
        with SceneGraph(scene_db) as graph:
            matches = graph.sightings(expected["name"])
        last = matches[-1] if matches else None
        if last is None:
            return f"no sightings of {expected['name']} in store"
        if round(last.ts, 3) != expected["last_ts"]:
            return f"last sighting ts {last.ts} != expected {expected['last_ts']}"
        return ""
    if case.query == 3:
        with RoomStore(scene_db) as store:
            room_set = store.latest()
        if room_set is None:
            return "no room derivation in store"
        n_rooms = len(room_set.by_kind("room"))
        n_corridors = len(room_set.by_kind("corridor"))
        if (n_rooms, n_corridors) != (expected["n_rooms"], expected["n_corridors"]):
            return f"store has {n_rooms}+{n_corridors} rooms, expected {expected}"
        return ""
    if case.query == 4:
        with SceneGraph(scene_db) as graph:
            sightings = graph.sightings()
        with RoomStore(scene_db) as store:
            room_set = store.latest()
        assert room_set is not None
        entry = next(
            (o for o in object_entries(sightings, room_set) if o.name == expected["name"]), None
        )
        if entry is None:
            return f"no sightings of {expected['name']} in store"
        stay = next((r for r in entry.rooms if r.room_id == expected["room_id"]), None)
        if stay is None:
            return f"no {expected['name']} sightings resolve to room {expected['room_id']}"
        if stay.last_ts != expected["last_in_room_ts"]:
            return f"last in-room ts {stay.last_ts} != expected {expected['last_in_room_ts']}"
        if entry.last_ts != expected["global_last_ts"]:
            return f"global last ts {entry.last_ts} != expected {expected['global_last_ts']}"
        return ""
    if case.query == 5:
        with SceneGraph(scene_db) as graph:
            matches = graph.sightings(expected["name"])
            in_vocab = graph.ever_in_vocabulary(expected["name"])
            events = graph.scan_events()
        if matches:
            return f"unexpected sightings of {expected['name']}: {len(matches)}"
        if in_vocab:
            return f"{expected['name']} unexpectedly in a scan vocabulary"
        if not events:
            return "no scan events in store (coverage qualifier would be empty)"
        return ""
    raise ValueError(f"Unknown query {case.query}")


def check_skill(case: CaseEntry, metadata: dict[str, Any]) -> str:
    """Layer (b): does the direct skill call return the expected fact?"""
    expected = case.expected
    if case.query == 1:
        if metadata.get("visits") != expected["visits"]:
            return f"visits {metadata.get('visits')} != expected {expected['visits']}"
        if metadata.get("last_exit_ts") != expected["last_exit_ts"]:
            return f"last_exit_ts {metadata.get('last_exit_ts')} != {expected['last_exit_ts']}"
        return ""
    if case.query == 2:
        if metadata.get("last_ts") != expected["last_ts"]:
            return f"last_ts {metadata.get('last_ts')} != expected {expected['last_ts']}"
        return ""
    if case.query == 3:
        rooms = metadata.get("rooms", [])
        n_rooms = sum(1 for r in rooms if r["kind"] == "room")
        n_corridors = sum(1 for r in rooms if r["kind"] == "corridor")
        if (n_rooms, n_corridors) != (expected["n_rooms"], expected["n_corridors"]):
            return f"skill reports {n_rooms}+{n_corridors}, expected {expected}"
        return ""
    if case.query == 4:
        if metadata.get("last_ts") != expected["last_in_room_ts"]:
            return f"last_ts {metadata.get('last_ts')} != expected {expected['last_in_room_ts']}"
        if metadata.get("later_elsewhere_ts") != expected["global_last_ts"]:
            return (
                f"later_elsewhere_ts {metadata.get('later_elsewhere_ts')} != "
                f"expected {expected['global_last_ts']} (trap evidence missing)"
            )
        return ""
    if case.query == 5:
        if metadata.get("ever_seen_in_region") is not False:
            return f"ever_seen_in_region {metadata.get('ever_seen_in_region')} != False"
        if metadata.get("ever_in_vocabulary") is not False:
            return f"ever_in_vocabulary {metadata.get('ever_in_vocabulary')} != False"
        return ""
    raise ValueError(f"Unknown query {case.query}")


def room_set_summary(scene_db: Path) -> dict[str, Any]:
    """What room derivation is current — the agent may have re-derived."""
    with RoomStore(scene_db) as store:
        room_set = store.latest()
    if room_set is None:
        return {"derived": False}
    return {
        "derived": True,
        "source": room_set.source,
        "derived_ts": room_set.derived_ts,
        "n_rooms": len(room_set.by_kind("room")),
        "n_corridors": len(room_set.by_kind("corridor")),
    }


def daemon_launch_command(scene_db: Path, recording: str) -> str:
    return (
        f"uv run dimos -o scene_memory_skill_container.sightings_db={scene_db} "
        f"--replay --replay-db {recording} run unitree-go2-agentic --daemon"
    )


def print_table(rows: list[dict[str, Any]]) -> None:
    header = (
        f"{'case':<22} {'storage':<8} {'skill':<8} {'agent':<9} {'halluc':<7} "
        f"{'tok in/out':<14} {'llm':<4} {'tools':<5}"
    )
    print(header)
    print("-" * len(header))
    for row in rows:
        agent = row["agent"]
        score = agent.get("score")
        usage = agent.get("usage", {})
        tok = f"{usage['input_tokens']}/{usage['output_tokens']}" if usage else "-"
        print(
            f"{row['case']:<22} "
            f"{'PASS' if row['storage'] == '' else 'FAIL':<8} "
            f"{'PASS' if row['skill'] == '' else 'FAIL':<8} "
            f"{score if score is not None else 'NOT RUN':<9} "
            f"{agent.get('hallucinated_never', '-')!s:<7} "
            f"{tok:<14} {usage.get('llm_calls', '-'):<4} {usage.get('tool_calls', '-'):<5}"
        )
    for row in rows:
        for layer in ("storage", "skill"):
            if row[layer]:
                print(f"  {row['case']} {layer} FAIL: {row[layer]}")


def main() -> None:
    load_dotenv()  # OPENAI_API_KEY for the agent and the judge
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--key", required=True, help="answer_key.yaml from the generator")
    parser.add_argument("--scene-db", required=True, help="seeded scene_memory.db")
    parser.add_argument("--out", default=None, help="results dir (default: alongside the key)")
    parser.add_argument("--layers", default="abc", help="subset of 'abc' to run")
    parser.add_argument("--mcp-url", default=DEFAULT_MCP_URL)
    parser.add_argument("--agent-model", default=None, help="default: the daemon's default model")
    parser.add_argument("--judge-model", default=DEFAULT_JUDGE_MODEL)
    args = parser.parse_args()

    key = load_answer_key(args.key)
    scene_db = Path(args.scene_db)
    out_dir = Path(args.out) if args.out else Path(args.key).parent
    out_dir.mkdir(parents=True, exist_ok=True)
    trail = load_pose_trail(str(resolve_db_path(key.recording)), ["go2_odom", "odom"])

    unconfirmed = key.unconfirmed()
    if unconfirmed:
        print(
            f"WARNING: answer key is a DRAFT — {len(unconfirmed)} unconfirmed "
            f"entries: {unconfirmed}\nScores below are against UNCONFIRMED labels.\n"
        )

    rows: list[dict[str, Any]] = []
    agent = None
    mcp: McpConnection | None = None
    if "c" in args.layers:
        mcp = McpConnection(args.mcp_url)
        if mcp.reachable():
            tools = mcp.fetch_tools()
            print(f"MCP server up, {len(tools)} tools; building in-process agent\n")
            agent = (
                build_agent(tools, model=args.agent_model)
                if args.agent_model
                else build_agent(tools)
            )
        else:
            print(
                f"MCP server not reachable at {args.mcp_url} — layer (c) NOT RUN.\n"
                f"Launch the daemon first:\n  {daemon_launch_command(scene_db, key.recording)}\n"
            )

    container = SceneMemorySkillContainer(trail_db=key.recording, sightings_db=str(scene_db))
    container.start()
    try:
        for case in key.cases:
            row: dict[str, Any] = {"case": case.id, "storage": "", "skill": "", "agent": {}}
            if "a" in args.layers:
                row["storage"] = check_storage(case, key, scene_db, trail)
            if "b" in args.layers:
                result = getattr(container, case.skill)(**case.skill_args)
                if not result.success:
                    row["skill"] = f"skill failed: {result.message}"
                else:
                    row["skill"] = check_skill(case, result.metadata)
                row["skill_message"] = result.message
            if agent is not None:
                print(f"== agent: {case.question}")
                traj = run_trajectory(agent, case.question)
                verdict = judge_answer(case, key, traj.answer, model=args.judge_model)
                print(f"answer: {traj.answer}")
                print(f"judge: {verdict.score} ({verdict.rationale})\n")
                row["agent"] = {
                    "answer": traj.answer,
                    "score": verdict.score,
                    "hallucinated_never": verdict.hallucinated_never,
                    "rationale": verdict.rationale,
                    "usage": traj.usage,
                    "room_set_at_answer": room_set_summary(scene_db),
                    "messages": serialize_messages(traj.messages),
                }
            rows.append(row)
    finally:
        container.stop()
        if mcp is not None:
            mcp.close()

    print_table(rows)
    scores = [r["agent"]["score"] for r in rows if r["agent"].get("score") is not None]
    if scores:
        print(f"\nagent mean score: {float(np.mean(scores)):.2f} over {len(scores)} case(s)")
    results = {
        "recording": key.recording,
        "answer_key": str(Path(args.key).resolve()),
        "unconfirmed_labels": unconfirmed,
        "layers_run": args.layers,
        "cases": rows,
    }
    results_path = out_dir / "results.json"
    results_path.write_text(json.dumps(results, indent=1))
    print(f"results: {results_path}")


if __name__ == "__main__":
    main()
