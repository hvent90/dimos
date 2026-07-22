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

"""Manual check: the full region join on a real recording (rooms x sightings).

Rebuilds the occupancy grid from the recording, segments and persists rooms,
runs the lidar-lifted object scan through the actual skill, then exercises
the region-join skills — including a hunt for a natural trap instance (an
object whose last sighting in some room precedes its last sighting overall)
and a qualified-negative query. Renders rooms + trail + sightings in one
overlay to verify everything shares the world frame::

    uv run python dimos/agents/skills/tool_scene_memory_regions.py \
        --db go2_short --out /tmp/region_join_check
"""

from __future__ import annotations

import argparse
import itertools
import json
from pathlib import Path

import cv2
import numpy as np
from numpy.typing import NDArray

from dimos.agents.skills.scene_memory import (
    SceneMemorySkillContainer,
    assign_to_rooms,
    load_pose_trail,
)
from dimos.mapping.occupancy.polygons import distance_to_polygon, points_in_polygon
from dimos.mapping.occupancy.room_segmentation import (
    RoomSegmentation,
    render_regions,
    segment_rooms,
)
from dimos.mapping.occupancy.room_store import RoomStore, StoredRoom
from dimos.mapping.occupancy.tool_room_segmentation_replay import rebuild_grid
from dimos.memory2.replay import resolve_db_path
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.perception.sightings import Sighting, SightingsLog
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


def sighting_rooms(
    sightings: list[Sighting], rooms: tuple[StoredRoom, ...]
) -> tuple[dict[str, dict[int, list[Sighting]]], int, int]:
    """name -> room_id -> sightings (exclusive assignment, ts order).

    Also returns how many sightings were strictly inside a polygon vs
    snapped to the nearest room, to keep the assignment honest.
    """
    out: dict[str, dict[int, list[Sighting]]] = {}
    if not sightings:
        return out, 0, 0
    xy = np.array([[s.position[0], s.position[1]] for s in sightings])
    assigned = assign_to_rooms(xy, rooms)
    strict = np.zeros(len(sightings), dtype=bool)
    for room in rooms:
        strict |= points_in_polygon(xy, room.polygon)
    for s, room_id in zip(sightings, assigned.tolist(), strict=True):
        if room_id:
            out.setdefault(s.name, {}).setdefault(room_id, []).append(s)
    snapped = int((assigned > 0).sum()) - int(strict.sum())
    return out, int(strict.sum()), snapped


def assignment_margin(position_xy: NDArray[np.float64], rooms: tuple[StoredRoom, ...]) -> float:
    """Gap between the nearest and runner-up room's effective distance.

    A small margin means the room assignment is a near coin toss between two
    rooms — don't build a headline claim on such a sighting.
    """
    point = position_xy.reshape(1, 2)
    effective = sorted(
        0.0
        if points_in_polygon(point, room.polygon)[0]
        else float(distance_to_polygon(point, room.polygon)[0])
        for room in rooms
    )
    return effective[1] - effective[0] if len(effective) > 1 else float("inf")


def render_overlay(
    grid: OccupancyGrid,
    segmentation: RoomSegmentation,
    trail_xy: NDArray[np.float64],
    sightings: list[Sighting],
    out_path: Path,
    upscale: int = 4,
) -> None:
    """Rooms render + robot trail (magenta) + sightings (yellow dots + names)."""
    img = render_regions(grid, segmentation, upscale=upscale)
    height = grid.grid.shape[0]
    ox, oy = segmentation.origin_xy

    def to_px(x: float, y: float) -> tuple[int, int]:
        cx = (x - ox) / segmentation.resolution
        cy = (y - oy) / segmentation.resolution
        return int(cx * upscale), int((height - cy) * upscale)

    for a, b in itertools.pairwise(trail_xy):
        cv2.line(img, to_px(*a), to_px(*b), (255, 0, 255), 2)
    for s in sightings:
        pt = to_px(s.position[0], s.position[1])
        cv2.circle(img, pt, 5, (0, 255, 255), -1)
        cv2.circle(img, pt, 5, (0, 0, 0), 1)
        cv2.putText(img, s.name, (pt[0] + 6, pt[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1)
    cv2.imwrite(str(out_path), img[:, :, ::-1])


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

    print(f"== rebuilding occupancy grid from {args.db}")
    grid = rebuild_grid(str(resolve_db_path(args.db)))
    segmentation = segment_rooms(grid)
    print(
        f"rooms: {len(segmentation.rooms())} + {len(segmentation.corridors())} corridors, "
        f"{len(segmentation.doorways)} doorways, {segmentation.explored_fraction:.0%} explored"
    )
    with RoomStore(scene_db) as store:
        store.save(segmentation, source=f"tool_scene_memory_regions:{args.db}")

    module = SceneMemorySkillContainer(
        trail_db=args.db,
        sightings_db=str(scene_db),
        camera_info=GO2Connection.camera_info_static,
        base_to_optical=BASE_TO_OPTICAL,
    )
    module.start()
    try:
        print("\n== scanning recording for objects")
        scan = module.scan_for_objects(list(args.vocab))
        print(scan.message)
        assert scan.success, scan

        print("\n== rooms()")
        rooms_result = module.rooms()
        print(rooms_result.message)

        with SightingsLog(scene_db) as log:
            sightings = log.sightings()
        with RoomStore(scene_db) as store:
            room_set = store.latest()
        assert room_set is not None

        print("\n== sightings by room (exclusive nearest-room assignment)")
        by_room, strict, snapped = sighting_rooms(sightings, room_set.rooms)
        t_start = min(s.ts for s in sightings) if sightings else 0.0
        for name in sorted(by_room):
            for room_id, rows in sorted(by_room[name].items()):
                rel = [round(s.ts - t_start, 1) for s in rows]
                print(f"  {name} in room {room_id}: {len(rows)} sightings, t_rel={rel}")
        unassigned = len(sightings) - strict - snapped
        print(
            f"  ({strict} of {len(sightings)} strictly inside a polygon, "
            f"{snapped} snapped to the nearest room, {unassigned} unassigned)"
        )

        print("\n== natural trap instances (last-in-room < last-overall)")
        traps: list[tuple[str, int, float, float, float]] = []
        for name in sorted(by_room):
            last_overall = max(s.ts for s in sightings if s.name == name)
            for room_id, rows in sorted(by_room[name].items()):
                last_in_room = rows[-1]
                if last_in_room.ts < last_overall:
                    margin = assignment_margin(
                        np.asarray(last_in_room.position[:2]), room_set.rooms
                    )
                    traps.append((name, room_id, last_in_room.ts, last_overall, margin))
                    print(
                        f"  {name} in room {room_id}: last-in-room t_rel="
                        f"{last_in_room.ts - t_start:.1f} < last-overall t_rel="
                        f"{last_overall - t_start:.1f} "
                        f"(assignment margin {margin:.2f} m)"
                    )
        if not traps:
            print("  none found in this recording")

        results: dict[str, object] = {}
        if traps:
            # Exercise the most robustly-assigned instance: prefer a clear
            # margin, break ties by the size of the time gap.
            robust = [t for t in traps if t[4] >= 0.3] or traps
            name, room_id, expect_ts, overall_ts, margin = max(robust, key=lambda t: (t[3] - t[2]))
            print(f"(picked margin {margin:.2f} m)")
            print(f"\n== last_seen_object_in_region('{name}', room_id={room_id})  [trap case]")
            trap_result = module.last_seen_object_in_region(name, room_id=room_id)
            print(trap_result.message)
            got = trap_result.metadata.get("last_ts")
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
                "assignment_margin_m": round(margin, 2),
                "pass": got == round(expect_ts, 3),
            }

        print(f"\n== object_ever_in_region('{args.absent_object}', room_id=1)  [never case]")
        never_result = module.object_ever_in_region(args.absent_object, room_id=1)
        print(never_result.message)
        results["never"] = {
            "ever_seen": never_result.metadata.get("ever_seen_in_region"),
            "ever_in_vocabulary": never_result.metadata.get("ever_in_vocabulary"),
            "rooms_with_scan_coverage": never_result.metadata.get("rooms_with_scan_coverage"),
        }

        trail = load_pose_trail(str(resolve_db_path(args.db)), ["go2_odom", "odom"])
        overlay = out_dir / "regions_trail_sightings.png"
        render_overlay(grid, segmentation, trail.xy, sightings, overlay)
        print(f"\noverlay render: {overlay}")
        (out_dir / "results.json").write_text(json.dumps(results, indent=1))
    finally:
        module.stop()


if __name__ == "__main__":
    main()
