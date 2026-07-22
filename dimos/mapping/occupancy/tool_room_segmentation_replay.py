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

"""Manual check: room segmentation on a grid rebuilt from a replay recording.

Rebuilds the occupancy grid from the recording's world-frame lidar via the
production ``height_cost_occupancy`` (the CostMapper algorithm), segments it,
renders the regions, and optionally persists via RoomStore::

    uv run python dimos/mapping/occupancy/tool_room_segmentation_replay.py \
        --db go2_china_office --out /tmp/roomseg

Reference results (prototype, Wed 07-22): go2_china_office -> 9 rooms +
0 corridors at ~50% explored; go2_short -> 13 rooms + 1 corridor.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np

from dimos.mapping.occupancy.room_segmentation import render_regions, segment_rooms
from dimos.mapping.occupancy.room_store import RoomStore
from dimos.mapping.pointclouds.occupancy import height_cost_occupancy
from dimos.memory2.replay import resolve_db_path
from dimos.memory2.store.sqlite import SqliteStore
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

VOXEL_M = 0.025  # dedupe quantization, finer than the 5 cm grid


def rebuild_grid(db_path: str):  # type: ignore[no-untyped-def]
    """Accumulate all lidar frames -> voxel dedupe -> production occupancy."""
    voxels: np.ndarray | None = None
    chunk: list[np.ndarray] = []
    last_ts = 0.0
    with SqliteStore(path=db_path, must_exist=True) as store:
        streams = store.list_streams()
        lidar_name = next(s for s in ("go2_lidar", "lidar") if s in streams)
        for obs in store.stream(lidar_name, PointCloud2).order_by("ts"):
            points, _ = obs.data.as_numpy()
            chunk.append(np.round(points / VOXEL_M).astype(np.int32))
            last_ts = max(last_ts, obs.ts)
            if len(chunk) >= 100:
                batch = np.unique(np.vstack(chunk), axis=0)
                voxels = batch if voxels is None else np.unique(np.vstack([voxels, batch]), axis=0)
                chunk = []
    if chunk:
        batch = np.unique(np.vstack(chunk), axis=0)
        voxels = batch if voxels is None else np.unique(np.vstack([voxels, batch]), axis=0)
    assert voxels is not None, "recording has no lidar points"
    points = voxels.astype(np.float64) * VOXEL_M
    print(f"accumulated {len(points)} unique voxels")
    cloud = PointCloud2.from_numpy(points, frame_id="world", timestamp=last_ts)
    return height_cost_occupancy(cloud)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--db", default="go2_china_office")
    parser.add_argument("--out", default="/tmp/roomseg")
    parser.add_argument("--store", default="", help="optional RoomStore db path to persist into")
    args = parser.parse_args()

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    grid = rebuild_grid(str(resolve_db_path(args.db)))
    print("rebuilt grid:", grid)
    segmentation = segment_rooms(grid)

    rooms, corridors = segmentation.rooms(), segmentation.corridors()
    print(
        f"result: {len(rooms)} rooms + {len(corridors)} corridors, "
        f"{len(segmentation.doorways)} doorways, "
        f"{segmentation.explored_fraction:.0%} explored"
    )
    for region in segmentation.regions:
        print(
            f"  id={region.id} {region.kind} area={region.area_m2} m^2 "
            f"anchor=({region.anchor_xy[0]:.2f}, {region.anchor_xy[1]:.2f}) "
            f"max_clearance={region.max_clearance_m} m"
        )

    render_path = out_dir / f"{Path(args.db).stem}_regions.png"
    cv2.imwrite(str(render_path), render_regions(grid, segmentation)[:, :, ::-1])
    print(f"render: {render_path}")

    if args.store:
        with RoomStore(args.store) as store:
            store.save(segmentation, source=f"tool_room_segmentation_replay:{args.db}")
        print(f"persisted to {args.store}")


if __name__ == "__main__":
    main()
