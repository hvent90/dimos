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

"""Manual check: agent-trail queries through the graph on a real recording.

The first pass verified robot visit intervals on go2_china_office against a
hand polygon (query 1). This is the same check through the closed room join:
rebuild the grid, derive rooms into the graph, pick the room containing the
trail's start pose, and ask ``last_seen("agent", in_node=<that room>)`` — the
skill's visits must equal the independent trail-x-polygon computation the
first pass verified by eye. Also dumps contrast-stretched camera frames at
visit boundaries so a human can re-verify against what the robot saw::

    uv run python dimos/agents/skills/tool_scene_memory_trail.py \
        --db go2_china_office --out /tmp/trail_check
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np

from dimos.agents.skills.scene_memory import (
    SceneMemorySkillContainer,
    load_pose_trail,
    visit_intervals,
)
from dimos.mapping.occupancy.polygons import points_in_polygon
from dimos.mapping.occupancy.room_segmentation import segment_rooms
from dimos.mapping.occupancy.room_store import RoomStore
from dimos.mapping.occupancy.tool_room_segmentation_replay import rebuild_grid
from dimos.memory2.replay import resolve_db_path
from dimos.memory2.store.sqlite import SqliteStore
from dimos.msgs.sensor_msgs.Image import Image
from dimos.perception.scene_graph import SceneGraph


def _contrast_stretch(rgb: np.ndarray) -> np.ndarray:
    """Percentile stretch for dark recordings so scenes are recognizable."""
    lo, hi = np.percentile(rgb, (2, 98))
    if hi <= lo:
        return rgb
    return np.clip((rgb.astype(np.float32) - lo) * 255.0 / (hi - lo), 0, 255).astype(np.uint8)


def _dump_frame(store: SqliteStore, ts: float, label: str, out_dir: Path) -> None:
    candidates = store.stream("color_image", Image).at(ts, tolerance=1.0).to_list()
    if not candidates:
        print(f"  {label}: no frame within 1 s of ts={ts:.2f}")
        return
    obs = min(candidates, key=lambda o: abs(o.ts - ts))
    rgb = _contrast_stretch(obs.data.to_opencv())
    path = out_dir / f"{label}_t{ts:.1f}.jpg"
    cv2.imwrite(str(path), rgb)
    print(f"  {label}: wrote {path} (frame ts offset {obs.ts - ts:+.2f}s)")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--db", default="go2_china_office", help="dataset name or .db path")
    parser.add_argument("--out", default="/tmp/trail_check", help="output directory")
    args = parser.parse_args()

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    scene_db = out_dir / "scene_graph.db"
    if scene_db.exists():
        scene_db.unlink()
    db_path = resolve_db_path(args.db)

    trail = load_pose_trail(str(db_path), ["go2_odom", "odom"])
    t0, t1 = trail.time_range()
    sx, sy = float(trail.xy[0, 0]), float(trail.xy[0, 1])
    print(f"trail: {len(trail.ts)} samples, {t1 - t0:.1f}s, start=({sx:.2f},{sy:.2f})")

    print(f"\n== rebuilding occupancy grid from {args.db} and deriving rooms")
    grid = rebuild_grid(str(db_path))
    segmentation = segment_rooms(grid)
    print(
        f"rooms: {len(segmentation.rooms())} + {len(segmentation.corridors())} corridors "
        f"({segmentation.explored_fraction:.0%} explored)"
    )
    with SceneGraph(scene_db) as graph:
        with RoomStore(scene_db) as store:
            store.save(segmentation, source=f"tool_scene_memory_trail:{args.db}")
            room_set = store.latest()
        assert room_set is not None
        graph.apply_rooms(room_set)
        start_room = graph.assign_regions(np.asarray([[sx, sy]]))[0]
        assert start_room, "start pose resolves to no room — pick another recording"
        polygon = graph.node(start_room).polygon()  # type: ignore[union-attr]
    print(f"start pose is in {start_room}")

    # Independent reference: the first pass's verified trail-x-polygon math.
    expected = [
        [round(a, 3), round(b, 3)]
        for a, b in visit_intervals(trail.ts, points_in_polygon(trail.xy, polygon))
    ]

    container = SceneMemorySkillContainer(trail_db=str(db_path), sightings_db=str(scene_db))
    container.start()
    try:
        scene = container.get_scene()
        print("\nget_scene:", scene.message)

        answer = container.last_seen("agent", in_node=start_room)
        print(f"\nlast_seen('agent', in_node='{start_room}'):", answer.message)
        got = answer.metadata.get("visits")
        match = got == expected
        print(f"skill visits:       {got}")
        print(f"independent visits: {expected}")
        print(f"-> {'PASS' if match else 'FAIL'}")

        here = container.where_am_i()
        print("\nwhere_am_i:", here.message)

        (out_dir / "results.json").write_text(
            json.dumps(
                {
                    "start_room": start_room,
                    "skill_visits": got,
                    "independent_visits": expected,
                    "pass": match,
                },
                indent=1,
            )
        )
    finally:
        container.stop()

    print(f"\ndumping verification frames to {out_dir}:")
    with SqliteStore(path=str(db_path), must_exist=True) as store:
        for i, (enter, exit_) in enumerate(expected):
            _dump_frame(store, enter, f"visit{i}_enter", out_dir)
            _dump_frame(store, exit_, f"visit{i}_exit", out_dir)
        if len(expected) >= 2:
            between = (expected[0][1] + expected[1][0]) / 2
            _dump_frame(store, between, "between_visits", out_dir)


if __name__ == "__main__":
    main()
