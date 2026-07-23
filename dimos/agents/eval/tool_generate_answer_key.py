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

"""Seed a scene DB from a recording and draft the eval answer key.

Rebuilds the occupancy grid offline, segments and persists rooms, runs the
broad-vocabulary object scan through the real skill, then derives the DRAFT
answer key (visibility intervals, room stays, the five eval cases). Every
entry is written ``confirmed: false`` — a human must review the YAML against
the recording and flip entries to ``confirmed: true`` before results are
presented as verified::

    uv run python dimos/agents/eval/tool_generate_answer_key.py \
        --db go2_short --out /tmp/scene_eval/go2_short

The seeded ``scene_memory.db`` is the store the eval runs against — point
the replay daemon at it for layer (c):

    uv run dimos -o scene_memory_skill_container.sightings_db=<out>/scene_memory.db \
        --replay --replay-db go2_short run unitree-go2-agentic --daemon
"""

from __future__ import annotations

import argparse
from pathlib import Path

from dimos.agents.eval.answer_key import save_answer_key
from dimos.agents.eval.scene_eval_cases import ALL_QUERIES, build_answer_key
from dimos.agents.skills.scene_memory import SceneMemorySkillContainer, load_pose_trail
from dimos.agents.skills.tool_scene_memory_regions import DEFAULT_VOCAB
from dimos.mapping.occupancy.room_segmentation import segment_rooms
from dimos.mapping.occupancy.room_store import RoomStore
from dimos.mapping.occupancy.tool_room_segmentation_replay import rebuild_grid
from dimos.memory2.replay import resolve_db_path
from dimos.perception.scene_graph import SceneGraph
from dimos.robot.unitree.go2.connection import BASE_TO_OPTICAL, GO2Connection


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--db", default="go2_short", help="recording name or .db path")
    parser.add_argument("--out", default="/tmp/scene_eval/go2_short")
    parser.add_argument("--vocab", nargs="+", default=DEFAULT_VOCAB)
    parser.add_argument(
        "--queries",
        nargs="+",
        type=int,
        default=list(ALL_QUERIES),
        help="which of the five queries to build cases for (e.g. 1 3 for a dark recording)",
    )
    args = parser.parse_args()

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    scene_db = out_dir / "scene_memory.db"
    if scene_db.exists():
        scene_db.unlink()

    print(f"== rebuilding occupancy grid from {args.db} (takes ~2 min)")
    grid = rebuild_grid(str(resolve_db_path(args.db)))
    segmentation = segment_rooms(grid)
    print(
        f"rooms: {len(segmentation.rooms())} + {len(segmentation.corridors())} corridors, "
        f"{segmentation.explored_fraction:.0%} explored"
    )
    with RoomStore(scene_db) as store:
        store.save(segmentation, source=f"tool_generate_answer_key:{args.db}")

    print(f"\n== scanning recording for {args.vocab}")
    module = SceneMemorySkillContainer(
        trail_db=args.db,
        sightings_db=str(scene_db),
        camera_info=GO2Connection.camera_info_static,
        base_to_optical=BASE_TO_OPTICAL,
    )
    module.start()
    try:
        scan = module.scan_for_objects(list(args.vocab))
        print(scan.message)
        assert scan.success, scan
    finally:
        module.stop()

    with SceneGraph(scene_db) as graph:
        sightings = graph.sightings()
    with RoomStore(scene_db) as store:
        room_set = store.latest()
    assert room_set is not None
    trail = load_pose_trail(str(resolve_db_path(args.db)), ["go2_odom", "odom"])

    key = build_answer_key(
        recording=args.db,
        trail=trail,
        sightings=sightings,
        vocabulary=sorted({v.strip() for v in args.vocab}),
        room_set=room_set,
        queries=tuple(args.queries),
    )
    key_path = out_dir / "answer_key.yaml"
    save_answer_key(key, key_path)

    print(f"\n== DRAFT answer key: {key_path}")
    print(f"scene DB (seeded rooms + sightings): {scene_db}")
    print(f"objects: {[o.name for o in key.objects]}")
    for case in key.cases:
        print(f"  {case.id}: {case.question}")
    print(
        f"\nALL {len(key.unconfirmed())} entries are UNCONFIRMED. Review the YAML "
        f"against the recording and set confirmed: true per entry before citing results."
    )


if __name__ == "__main__":
    main()
