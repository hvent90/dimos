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

"""Manual check of the pose-trail skills against a recorded replay DB.

Runs the trail skills on a recording and dumps contrast-stretched camera
frames at visit boundaries so a human can verify the answers against what
the robot actually saw::

    uv run python dimos/agents/skills/tool_scene_memory_trail.py \
        --db go2_china_office --out /tmp/trail_check

The default region is a 4x4 m box around the recording's start pose - for
go2_china_office the expected result is two visits (the robot starts there,
leaves ~20 s in, and passes through again ~30 s in).
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np

from dimos.agents.skills.scene_memory import SceneMemorySkillContainer, load_pose_trail
from dimos.memory2.replay import resolve_db_path
from dimos.memory2.store.sqlite import SqliteStore
from dimos.msgs.sensor_msgs.Image import Image


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
    parser.add_argument("--out", default="/tmp/trail_check", help="frame dump directory")
    parser.add_argument(
        "--region",
        type=float,
        nargs="+",
        default=None,
        help="flat polygon x1 y1 x2 y2 ... (default: 4x4 m box around start pose)",
    )
    args = parser.parse_args()

    db_path = resolve_db_path(args.db)
    trail = load_pose_trail(str(db_path), ["go2_odom", "odom"])
    t0, t1 = trail.time_range()
    sx, sy = float(trail.xy[0, 0]), float(trail.xy[0, 1])
    print(f"trail: {len(trail.ts)} samples, {t1 - t0:.1f}s, start=({sx:.2f},{sy:.2f})")

    region = args.region
    if region is None:
        h = 2.0
        region = [sx - h, sy - h, sx + h, sy - h, sx + h, sy + h, sx - h, sy + h]
    print(f"region polygon: {region}")

    container = SceneMemorySkillContainer(trail_db=str(db_path))
    container.start()
    try:
        info = container.robot_trail_info()
        print("robot_trail_info:", info.message)

        visits = container.robot_visits_to_region(region)
        print("robot_visits_to_region:", visits.message)
        print(json.dumps(visits.metadata, indent=2))

        mid = (t0 + t1) / 2
        pos = container.robot_position_at(mid)
        print(f"robot_position_at(midpoint {mid - t0:.1f}s in):", pos.message)
    finally:
        container.stop()

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    print(f"dumping verification frames to {out_dir}:")
    with SqliteStore(path=str(db_path), must_exist=True) as store:
        for i, (enter, exit_) in enumerate(visits.metadata.get("visits", [])):
            _dump_frame(store, enter, f"visit{i}_enter", out_dir)
            _dump_frame(store, (enter + exit_) / 2, f"visit{i}_mid", out_dir)
            _dump_frame(store, exit_, f"visit{i}_exit", out_dir)
        # A frame between visits for contrast (should look like a different place).
        v = visits.metadata.get("visits", [])
        if len(v) >= 2:
            between = (v[0][1] + v[1][0]) / 2
            _dump_frame(store, between, "between_visits", out_dir)


if __name__ == "__main__":
    main()
