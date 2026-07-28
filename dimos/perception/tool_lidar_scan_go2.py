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

"""Manual gate check: replay in, 3D-positioned detections out (go2, no depth).

Runs the YOLOE + lidar-projection scan over a go2 recording and dumps
per-frame overlays (2D boxes, projected lidar depth points, 3D centers) plus
a top-down map of all sightings, for visual verification that the camera/
lidar extrinsic is right and 3D positions land where the objects are::

    uv run python dimos/perception/tool_lidar_scan_go2.py \
        --db go2_short --out /tmp/lidar_scan_check --render-every 10
"""

from __future__ import annotations

import argparse
from collections import Counter
import itertools
import json
from pathlib import Path

import cv2
import numpy as np

from dimos.memory2.replay import resolve_db_path
from dimos.memory2.store.sqlite import SqliteStore
from dimos.perception.detection.detectors.yoloe import Yoloe2DDetector, YoloePromptMode
from dimos.perception.lidar_scan import LidarScanFrame, iter_lidar_scan, project_points
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


def render_frame(frame: LidarScanFrame, camera_info, out_path: Path) -> None:  # type: ignore[no-untyped-def]
    img = frame.image.to_opencv().copy()
    points, _ = frame.lidar.as_numpy()
    uv, depth = project_points(points, frame.world_to_optical, camera_info)
    if len(uv):
        d = np.clip((depth - 0.5) / 5.0, 0, 1)
        colors = cv2.applyColorMap((d * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
        for (u, v), c in zip(uv[::3].astype(int), colors[::3, 0], strict=True):
            cv2.circle(img, (u, v), 1, tuple(int(x) for x in c), -1)
    for det in frame.detections_2d:
        x1, y1, x2, y2 = (int(v) for v in det.bbox)
        cv2.rectangle(img, (x1, y1), (x2, y2), (255, 255, 255), 2)
    for s in frame.sightings:
        label = (
            f"{s.name} ({s.position[0]:.1f},{s.position[1]:.1f},{s.position[2]:.1f}) {s.n_points}pt"
        )
        match = next(d for d in frame.detections_2d if d.name == s.name)
        x1, y1 = int(match.bbox[0]), int(match.bbox[1])
        cv2.putText(img, label, (x1, max(15, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3)
        cv2.putText(
            img, label, (x1, max(15, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1
        )
    cv2.imwrite(str(out_path), img)


def render_map(frames: list[LidarScanFrame], out_path: Path) -> None:
    all_s = [s for f in frames for s in f.sightings]
    traj = np.array([f.robot_xy for f in frames])
    pts = np.array([s.position[:2] for s in all_s]) if all_s else np.empty((0, 2))
    lo = np.minimum(traj.min(axis=0), pts.min(axis=0) if len(pts) else traj.min(axis=0)) - 1
    hi = np.maximum(traj.max(axis=0), pts.max(axis=0) if len(pts) else traj.max(axis=0)) + 1
    scale = 60.0
    size = ((hi - lo) * scale).astype(int) + 1
    img = np.full((size[1], size[0], 3), 255, np.uint8)

    def to_px(xy):  # type: ignore[no-untyped-def]
        p = ((np.asarray(xy) - lo) * scale).astype(int)
        return int(p[0]), int(size[1] - 1 - p[1])

    for a, b in itertools.pairwise(traj):
        cv2.line(img, to_px(a), to_px(b), (200, 0, 200), 2)
    names = sorted({s.name for s in all_s})
    palette = [
        (31, 119, 180), (255, 127, 14), (44, 160, 44), (214, 39, 40), (148, 103, 189),
        (140, 86, 75), (227, 119, 194), (127, 127, 127), (188, 189, 34), (23, 190, 207),
    ]  # fmt: skip
    for s in all_s:
        c = palette[names.index(s.name) % len(palette)]
        cv2.circle(img, to_px(s.position[:2]), 4, (c[2], c[1], c[0]), -1)
    for i, n in enumerate(names):
        c = palette[i % len(palette)]
        cv2.putText(img, n, (10, 22 + 20 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (c[2], c[1], c[0]), 2)
    cv2.imwrite(str(out_path), img)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--db", default="go2_short")
    parser.add_argument("--out", default="/tmp/lidar_scan_check")
    parser.add_argument("--vocab", nargs="+", default=DEFAULT_VOCAB)
    parser.add_argument("--sample-period", type=float, default=0.5)
    parser.add_argument("--render-every", type=int, default=10, help="render every Nth frame")
    args = parser.parse_args()

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    detector = Yoloe2DDetector(prompt_mode=YoloePromptMode.PROMPT, conf=0.4)
    detector.set_prompts(text=list(args.vocab))
    camera_info = GO2Connection.camera_info_static

    frames: list[LidarScanFrame] = []
    with SqliteStore(path=str(resolve_db_path(args.db)), must_exist=True) as store:
        for i, frame in enumerate(
            iter_lidar_scan(
                store,
                detector,
                camera_info,
                BASE_TO_OPTICAL,
                sample_period_s=args.sample_period,
            )
        ):
            frames.append(frame)
            if i % args.render_every == 0:
                render_frame(frame, camera_info, out_dir / f"frame_{i:04d}_t{frame.ts:.1f}.jpg")
            print(
                f"frame {i} t={frame.ts:.2f} dets2d={len(frame.detections_2d)} "
                f"sightings={[(s.name, [round(v, 2) for v in s.position]) for s in frame.sightings]}"
            )

    all_sightings = [s for f in frames for s in f.sightings]
    print(f"\n{len(frames)} frames, {len(all_sightings)} sightings")
    print("by name:", dict(Counter(s.name for s in all_sightings)))
    lifted = sum(1 for f in frames for _ in f.detections_2d)
    print(f"2D detections: {lifted}, lifted to 3D: {len(all_sightings)}")
    render_map(frames, out_dir / "sightings_map.png")
    summary = [
        {"name": s.name, "ts": round(s.ts, 3), "xyz": [round(v, 2) for v in s.position]}
        for s in all_sightings
    ]
    (out_dir / "sightings.json").write_text(json.dumps(summary, indent=1))
    print(f"renders + sightings.json in {out_dir}")


if __name__ == "__main__":
    main()
