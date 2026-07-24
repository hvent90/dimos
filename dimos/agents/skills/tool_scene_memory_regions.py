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

"""Manual check: the graph query surface end to end on a real recording.

Seeds a scene DB (rooms derived into the graph + a full lidar-lifted scan
through the actual skill), then exercises the reads — get_scene, find,
nodes_in, adjacent, and the last_seen trap case on a natural instance (an
object whose last sighting in some room precedes its last sighting overall),
plus the coverage-qualified never for an absurd object. Renders the graph
over the rooms via tool_scene_graph's figure::

    uv run python dimos/agents/skills/tool_scene_memory_regions.py \
        --db go2_short --out /tmp/region_join_check

Viewer check (manual): start the replay daemon with this seeded DB —

    uv run dimos -o scenememoryskillcontainer.sightings_db=<out>/scene_memory.db \
        --replay --replay-db go2_short run unitree-go2-agentic --daemon

— open the Rerun viewer and confirm rooms/markers/edges render over the
costmap (streams world/scene_graph_*). Save the screenshot OUTSIDE the repo,
e.g. /tmp/region_join_check/rerun_scene_graph.png — do not commit binaries.
"""

from __future__ import annotations

import argparse
from collections import defaultdict
import json
from pathlib import Path

import numpy as np
from numpy.typing import NDArray

from dimos.agents.skills.scene_memory import SceneMemorySkillContainer, load_pose_trail
from dimos.mapping.occupancy.polygons import distance_to_polygon, points_in_polygon
from dimos.mapping.occupancy.room_segmentation import segment_rooms
from dimos.mapping.occupancy.room_store import RoomStore
from dimos.mapping.occupancy.tool_room_segmentation_replay import rebuild_grid
from dimos.memory2.replay import resolve_db_path
from dimos.perception.scene_graph import SceneGraph, SceneNode, Sighting
from dimos.perception.tool_scene_graph import render_graph
from dimos.robot.unitree.go2.connection import BASE_TO_OPTICAL, GO2Connection

DEFAULT_VOCAB = [
    "chair",
    "table",
    "couch",
    "bottle",
    "box",
    "potted plant",
    "tv",
    "person",
    "refrigerator",
    "backpack",
]


def assignment_margin(position_xy: NDArray[np.float64], regions: list[SceneNode]) -> float:
    """Gap between the nearest and runner-up region's effective distance.

    A small margin means the room assignment is a near coin toss between two
    rooms — don't build a headline claim on such a sighting.
    """
    point = position_xy.reshape(1, 2)
    effective = sorted(
        0.0
        if points_in_polygon(point, region.polygon())[0]
        else float(distance_to_polygon(point, region.polygon())[0])
        for region in regions
    )
    return effective[1] - effective[0] if len(effective) > 1 else float("inf")


def natural_traps(
    sightings: list[Sighting], regions: list[SceneNode]
) -> list[tuple[str, str, float, float, float]]:
    """(name, room_id, last_in_room_ts, last_overall_ts, margin) instances."""
    by_name_room: dict[str, dict[str, list[Sighting]]] = defaultdict(lambda: defaultdict(list))
    last_overall: dict[str, float] = {}
    for s in sightings:
        last_overall[s.name] = max(last_overall.get(s.name, 0.0), s.ts)
        if s.room_id:
            by_name_room[s.name][s.room_id].append(s)
    traps = []
    for name in sorted(by_name_room):
        for room_id, rows in sorted(by_name_room[name].items()):
            last_in_room = rows[-1]
            if last_in_room.ts < last_overall[name]:
                margin = assignment_margin(np.asarray(last_in_room.position[:2]), regions)
                traps.append((name, room_id, last_in_room.ts, last_overall[name], margin))
    return traps


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--db", default="go2_short")
    parser.add_argument("--out", default="/tmp/region_join_check")
    parser.add_argument("--vocab", nargs="+", default=DEFAULT_VOCAB)
    parser.add_argument("--absent-object", default="crocodile")
    args = parser.parse_args()

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    scene_db = out_dir / "scene_memory.db"
    if scene_db.exists():
        scene_db.unlink()

    print(f"== rebuilding occupancy grid from {args.db} and deriving rooms")
    grid = rebuild_grid(str(resolve_db_path(args.db)))
    segmentation = segment_rooms(grid)
    print(
        f"rooms: {len(segmentation.rooms())} + {len(segmentation.corridors())} corridors, "
        f"{len(segmentation.doorways)} doorways, {segmentation.explored_fraction:.0%} explored"
    )
    with SceneGraph(scene_db) as seed_graph:
        with RoomStore(scene_db) as store:
            store.save(segmentation, source=f"tool_scene_memory_regions:{args.db}")
            room_set = store.latest()
        assert room_set is not None
        seed_graph.apply_rooms(room_set)

    module = SceneMemorySkillContainer(
        trail_db=args.db,
        sightings_db=str(scene_db),
        camera_info=GO2Connection.camera_info_static,
        base_to_optical=BASE_TO_OPTICAL,
    )
    module.start()
    try:
        print("\n== scan_for_objects")
        scan = module.scan_for_objects(list(args.vocab))
        print(scan.message)
        assert scan.success, scan

        print("\n== get_scene")
        scene = module.get_scene()
        print(scene.message)

        print("\n== nodes_in / adjacent spot checks")
        regions_meta = scene.metadata["regions"]
        busiest = max(regions_meta, key=lambda r: r["objects"])
        listing = module.nodes_in(busiest["id"])
        print(listing.message)
        neighbors = module.adjacent(busiest["id"])
        print(neighbors.message)

        with SceneGraph(scene_db) as graph:
            sightings = graph.sightings()
            regions = graph.regions()

        print("\n== natural trap instances (last-in-room < last-overall)")
        traps = natural_traps(sightings, regions)
        t_start = min(s.ts for s in sightings) if sightings else 0.0
        for name, room_id, in_room_ts, overall_ts, margin in traps:
            print(
                f"  {name} in {room_id}: last-in-room t_rel={in_room_ts - t_start:.1f} "
                f"< last-overall t_rel={overall_ts - t_start:.1f} "
                f"(assignment margin {margin:.2f} m)"
            )
        if not traps:
            print("  none found in this recording")

        results: dict[str, object] = {}
        if traps:
            # Exercise the most robustly-assigned instance: prefer a clear
            # margin, break ties by the size of the time gap.
            robust = [t for t in traps if t[4] >= 0.3] or traps
            name, room_id, expect_ts, overall_ts, margin = max(robust, key=lambda t: t[3] - t[2])
            print(f"(picked margin {margin:.2f} m)")
            print(f"\n== last_seen('{name}', in_node='{room_id}')  [trap case]")
            trap_result = module.last_seen(name, in_node=room_id)
            print(trap_result.message)
            got = trap_result.metadata.get("last_sighting", {}).get("ts")
            print(
                f"expected last_ts={expect_ts:.3f} got={got} "
                f"(global last would be {overall_ts:.3f}) -> "
                f"{'PASS' if got == round(expect_ts, 3) else 'FAIL'}"
            )
            results["trap"] = {
                "name": name,
                "room_id": room_id,
                "expected_last_ts": expect_ts,
                "got_last_ts": got,
                "global_last_ts": overall_ts,
                "later_elsewhere_ts": trap_result.metadata.get("later_elsewhere_ts"),
                "assignment_margin_m": round(margin, 2),
                "pass": got == round(expect_ts, 3),
            }

        never_room = regions[0].id
        print(f"\n== last_seen('{args.absent_object}', in_node='{never_room}')  [never case]")
        never_result = module.last_seen(args.absent_object, in_node=never_room)
        print(never_result.message)
        results["never"] = {
            "sightings_matched": never_result.metadata.get("sightings_matched"),
            "ever_in_vocabulary": never_result.metadata.get("ever_in_vocabulary"),
            "coverage": never_result.metadata.get("coverage"),
        }

        print("\n== which room did you last see the person in? (one call now)")
        person = module.last_seen("person")
        print(person.message)

        trail = load_pose_trail(str(resolve_db_path(args.db)), ["go2_odom", "odom"])
        with SceneGraph(scene_db) as graph:
            figure = out_dir / "scene_graph_overlay.png"
            render_graph(grid, segmentation, graph, trail.xy, figure)
        print(f"\noverlay render: {figure}")
        (out_dir / "results.json").write_text(json.dumps(results, indent=1))
        print(f"results: {out_dir / 'results.json'}")
        print(
            "\nviewer check: run the daemon with this DB (see module docstring) and "
            f"save a screenshot to {out_dir / 'rerun_scene_graph.png'}"
        )
    finally:
        module.stop()


if __name__ == "__main__":
    main()
