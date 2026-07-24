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

"""Manual check: build a scene graph from a real recording and render it.

Rebuilds the occupancy grid from the recording, derives rooms into the
graph, runs the lidar-lifted object scan and folds it, then renders rooms +
object nodes + agent trail in one figure and dumps the graph JSON. Also
prints attachment diagnostics (same-name node separations vs within-node
sighting spread) — the evidence behind ATTACH_RADIUS_M — and verifies that
re-folding the same scan changes nothing::

    uv run python dimos/perception/tool_scene_graph.py \
        --db go2_short --out /tmp/scene_graph_check
"""

from __future__ import annotations

import argparse
from collections import defaultdict
import itertools
import json
from pathlib import Path

import cv2
import numpy as np
from numpy.typing import NDArray

from dimos.agents.skills.scene_memory import load_pose_trail
from dimos.mapping.occupancy.room_segmentation import (
    RoomSegmentation,
    render_regions,
    segment_rooms,
)
from dimos.mapping.occupancy.room_store import RoomStore
from dimos.mapping.occupancy.tool_room_segmentation_replay import rebuild_grid
from dimos.memory2.replay import resolve_db_path
from dimos.memory2.store.sqlite import SqliteStore
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.perception.lidar_scan import iter_lidar_scan
from dimos.perception.scene_graph import SceneGraph, Sighting
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


def scan_recording(
    db_path: str, vocabulary: list[str]
) -> tuple[list[Sighting], int, float, float, tuple[float, float, float] | None]:
    """Run the lidar-lifted detection over the recording (the scan lane)."""
    from dimos.perception.detection.detectors.yoloe import Yoloe2DDetector, YoloePromptMode

    detector = Yoloe2DDetector(prompt_mode=YoloePromptMode.PROMPT, conf=0.4)
    detector.set_prompts(text=vocabulary)
    sightings: list[Sighting] = []
    n_frames = 0
    t_lo, t_hi = float("inf"), float("-inf")
    agent_position: tuple[float, float, float] | None = None
    with SqliteStore(path=db_path, must_exist=True) as store:
        for frame in iter_lidar_scan(
            store, detector, GO2Connection.camera_info_static, BASE_TO_OPTICAL
        ):
            n_frames += 1
            t_lo, t_hi = min(t_lo, frame.ts), max(t_hi, frame.ts)
            agent_position = (frame.robot_xy[0], frame.robot_xy[1], 0.0)
            for s in frame.sightings:
                sightings.append(
                    Sighting(
                        name=s.name,
                        ts=s.ts,
                        position=s.position,
                        object_id=str(s.track_id) if s.track_id >= 0 else "",
                        confidence=s.confidence,
                    )
                )
    return sightings, n_frames, t_lo, t_hi, agent_position


def attachment_diagnostics(graph: SceneGraph) -> None:
    """Same-name node separation vs within-node sighting spread.

    The attachment radius is defensible when distinct same-name nodes sit
    much farther apart than any node's own sightings spread.
    """
    by_node: dict[str, list[tuple[float, float]]] = defaultdict(list)
    for s in graph.sightings():
        by_node[s.node_id].append((s.position[0], s.position[1]))
    print("\n== attachment diagnostics")
    for node in graph.nodes(layer="object"):
        pts = np.asarray(by_node.get(node.id, []), dtype=np.float64)
        spread = 0.0
        if len(pts) > 1:
            spread = max(float(np.hypot(*(a - b))) for a, b in itertools.combinations(pts, 2))
        print(
            f"  {node.id} ({node.name}): {node.sightings} sightings, "
            f"within-node spread {spread:.2f} m"
        )
    by_name: dict[str, list[NDArray[np.float64]]] = defaultdict(list)
    for node in graph.nodes(layer="object"):
        by_name[node.name].append(np.asarray(node.xy))
    for name, positions in sorted(by_name.items()):
        if len(positions) < 2:
            continue
        gap = min(float(np.hypot(*(a - b))) for a, b in itertools.combinations(positions, 2))
        print(f"  '{name}': {len(positions)} distinct nodes, closest pair {gap:.2f} m apart")


def render_graph(
    grid: OccupancyGrid,
    segmentation: RoomSegmentation,
    graph: SceneGraph,
    trail_xy: NDArray[np.float64],
    out_path: Path,
    upscale: int = 4,
) -> None:
    """One figure: rooms + agent trail (magenta) + labeled object nodes."""
    img = render_regions(grid, segmentation, upscale=upscale)
    height = grid.grid.shape[0]
    ox, oy = segmentation.origin_xy

    def to_px(x: float, y: float) -> tuple[int, int]:
        cx = (x - ox) / segmentation.resolution
        cy = (y - oy) / segmentation.resolution
        return int(cx * upscale), int((height - cy) * upscale)

    for a, b in itertools.pairwise(trail_xy):
        cv2.line(img, to_px(*a), to_px(*b), (255, 0, 255), 2)
    region_anchor = {n.id: n.xy for n in graph.nodes() if n.layer in ("room", "corridor")}
    for node in graph.nodes(layer="object"):
        pt = to_px(*node.xy)
        parent = graph.parent_id(node.id)
        if parent in region_anchor:
            cv2.line(img, pt, to_px(*region_anchor[parent]), (60, 60, 60), 1, cv2.LINE_AA)
        cv2.circle(img, pt, 6, (0, 255, 255), -1)
        cv2.circle(img, pt, 6, (0, 0, 0), 1)
        cv2.putText(
            img,
            f"{node.name} ({node.id})",
            (pt[0] + 8, pt[1] + 4),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (0, 0, 0),
            1,
            cv2.LINE_AA,
        )
    agent = graph.node("agent_0")
    if agent is not None and agent.position is not None:
        pt = to_px(*agent.xy)
        cv2.drawMarker(img, pt, (255, 0, 255), cv2.MARKER_TRIANGLE_UP, 18, 3)
        cv2.putText(
            img, "agent_0", (pt[0] + 10, pt[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1
        )
    cv2.imwrite(str(out_path), img[:, :, ::-1])


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--db", default="go2_short")
    parser.add_argument("--out", default="/tmp/scene_graph_check")
    parser.add_argument("--vocab", nargs="+", default=DEFAULT_VOCAB)
    args = parser.parse_args()

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    scene_db = out_dir / "scene_graph.db"
    if scene_db.exists():
        scene_db.unlink()
    db_path = str(resolve_db_path(args.db))

    print(f"== rebuilding occupancy grid from {args.db}")
    grid = rebuild_grid(db_path)
    segmentation = segment_rooms(grid)
    print(
        f"rooms: {len(segmentation.rooms())} + {len(segmentation.corridors())} corridors, "
        f"{len(segmentation.doorways)} doorways, {segmentation.explored_fraction:.0%} explored"
    )

    print("\n== scanning recording for objects")
    vocabulary = sorted({v.strip() for v in args.vocab if v.strip()})
    sightings, n_frames, t_lo, t_hi, agent_position = scan_recording(db_path, vocabulary)
    print(f"{len(sightings)} sightings over {n_frames} frames")

    with SceneGraph(scene_db) as graph:
        with RoomStore(scene_db) as store:
            store.save(segmentation, source=f"tool_scene_graph:{args.db}")
            room_set = store.latest()
        assert room_set is not None
        graph.apply_rooms(room_set)
        result = graph.fold_scan(
            sightings,
            t0=t_lo,
            t1=t_hi,
            vocabulary=vocabulary,
            source=f"tool_scene_graph:{args.db}",
            frames=n_frames,
            agent_position=agent_position,
        )
        print(
            f"fold: {result.appended_sightings} sightings -> "
            f"{len(result.created_node_ids)} new nodes"
        )

        print("\n== object nodes")
        for node in graph.nodes(layer="object"):
            parent = graph.parent_id(node.id)
            lineage = " -> ".join(n.id for n in graph.ancestors(node.id))
            print(
                f"  {node.id}: {node.name} at ({node.position[0]:.2f}, {node.position[1]:.2f}) "  # type: ignore[index]
                f"x{node.sightings}, parent={parent}, lineage: {lineage}"
            )
        with_room = sum(1 for s in graph.sightings() if s.room_id)
        print(f"  ({with_room}/{len(graph.sightings())} sightings resolved to a room)")

        attachment_diagnostics(graph)

        print("\n== re-fold idempotence")
        again = graph.fold_scan(
            sightings,
            t0=t_lo,
            t1=t_hi,
            vocabulary=vocabulary,
            source=f"tool_scene_graph:{args.db}",
            frames=n_frames,
        )
        node_count = len(graph.nodes(layer="object"))
        print(
            f"re-fold appended {again.appended_sightings} sightings, "
            f"created {len(again.created_node_ids)} nodes ({node_count} object nodes total) -> "
            f"{'PASS' if again.appended_sightings == 0 and not again.created_node_ids else 'FAIL'}"
        )

        trail = load_pose_trail(db_path, ["go2_odom", "odom"])
        figure = out_dir / "scene_graph.png"
        render_graph(grid, segmentation, graph, trail.xy, figure)
        print(f"\nfigure: {figure}")

        (out_dir / "scene_graph.json").write_text(json.dumps(graph.to_json(), indent=1))
        print(f"graph json: {out_dir / 'scene_graph.json'}")


if __name__ == "__main__":
    main()
