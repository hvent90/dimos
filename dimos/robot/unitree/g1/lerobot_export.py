# Copyright 2025-2026 Dimensional Inc.
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

"""Export G1 teleop recordings to a LeRobot dataset.

Reads a memory2 SQLite db produced by
:class:`dimos.robot.unitree.g1.episode_recorder.G1EpisodeRecorder`, slices
the continuously-recorded streams by the ``episodes`` start/stop markers
(start→cancel pairs and dangling starts are dropped), resamples everything
onto a uniform ``fps`` grid, and writes a HuggingFace LeRobotDataset:

    observation.state            float32[14]  — measured arm joints
    action                       float32[14]  — commanded arm joints (teleop IK targets)
    observation.images.cam_high  video        — /camera_image

Requires the ``lerobot`` package (not a dimos dependency — it drags torch
and friends). Install it in this venv, or run via the unitree_lerobot repo's
environment with dimos on the path:

    uv pip install lerobot

Usage:

    uv run python -m dimos.robot.unitree.g1.lerobot_export \\
        --db recording_g1_teleop.db \\
        --repo-id you/g1_pick_cube \\
        --task "pick up the cube"
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import sys
from typing import Any

import numpy as np

from dimos.memory2.store.sqlite import SqliteStore
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Must match quest_teleop._ARM_JOINT_NAMES order — which is the Unitree G1
# dex convention (kLeftShoulderPitch … kRightWristYaw). Redefined here so
# the exporter doesn't import pinocchio via quest_teleop.
ARM_JOINT_NAMES: tuple[str, ...] = (
    "left_shoulder_pitch",
    "left_shoulder_roll",
    "left_shoulder_yaw",
    "left_elbow",
    "left_wrist_roll",
    "left_wrist_pitch",
    "left_wrist_yaw",
    "right_shoulder_pitch",
    "right_shoulder_roll",
    "right_shoulder_yaw",
    "right_elbow",
    "right_wrist_roll",
    "right_wrist_pitch",
    "right_wrist_yaw",
)

CAMERA_KEY = "cam_high"


@dataclass(frozen=True)
class Episode:
    index: int
    start_ts: float
    stop_ts: float


def find_episodes(store: SqliteStore) -> list[Episode]:
    """Pair start/stop markers; drop cancelled and dangling episodes."""
    markers = store.stream("episodes", str).to_list()
    episodes: list[Episode] = []
    open_start: tuple[int, float] | None = None  # (episode tag, ts)

    for obs in sorted(markers, key=lambda o: o.ts):
        event = obs.data
        tag = int(obs.tags.get("episode", -1)) if obs.tags else -1
        if event == "start":
            if open_start is not None:
                logger.warning("Episode %d has no stop marker — dropping", open_start[0])
            open_start = (tag, obs.ts)
        elif event == "stop" and open_start is not None:
            episodes.append(Episode(index=open_start[0], start_ts=open_start[1], stop_ts=obs.ts))
            open_start = None
        elif event == "cancel" and open_start is not None:
            logger.info("Episode %d cancelled — skipping", open_start[0])
            open_start = None

    if open_start is not None:
        logger.warning("Episode %d still open at end of recording — dropping", open_start[0])
    return episodes


def _arm_vector(msg: JointState) -> np.ndarray | None:
    """Extract the 14 arm joints (G1 dex order) from a JointState, or None."""
    if not msg.name or not msg.position:
        return None
    by_name = {
        (name.split("/", 1)[1] if "/" in name else name): float(pos)
        for name, pos in zip(msg.name, msg.position, strict=False)
    }
    if not all(name in by_name for name in ARM_JOINT_NAMES):
        return None
    return np.asarray([by_name[name] for name in ARM_JOINT_NAMES], dtype=np.float32)


def _series(
    store: SqliteStore, name: str, type_: type, t0: float, t1: float, pad_s: float = 1.0
) -> tuple[np.ndarray, list[Any]]:
    """Materialize a stream slice as (sorted ts array, data list).

    ``pad_s`` widens the window backwards so 'latest value at episode start'
    resolves even when the last sample landed just before the start marker.
    """
    obs_list = sorted(
        store.stream(name, type_).after(t0 - pad_s).before(t1).to_list(),
        key=lambda o: o.ts,
    )
    return np.asarray([o.ts for o in obs_list]), [o.data for o in obs_list]


def _latest_at(ts_array: np.ndarray, t: float) -> int | None:
    """Index of the latest sample at or before t, or None."""
    i = int(np.searchsorted(ts_array, t, side="right")) - 1
    return i if i >= 0 else None


def _to_rgb(image: Image) -> np.ndarray:
    arr = image.data
    if str(image.format).endswith("BGR"):
        arr = arr[..., ::-1]
    return np.ascontiguousarray(arr)


def export(
    db: Path,
    repo_id: str,
    task: str,
    fps: float = 30.0,
    robot_type: str = "Unitree_G1",
) -> None:
    try:
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
    except ImportError as e:
        sys.exit(
            f"lerobot is not installed ({e}).\n"
            "Install it with `uv pip install lerobot` (or run inside the "
            "unitree_lerobot environment with dimos on the path)."
        )

    store = SqliteStore(path=str(db))
    store.start()
    episodes = find_episodes(store)
    if not episodes:
        sys.exit(f"No complete episodes found in {db}")
    logger.info("Found %d complete episodes in %s", len(episodes), db)

    dataset: LeRobotDataset | None = None
    exported = 0

    for ep in episodes:
        state_ts, state_data = _series(store, "joint_state", JointState, ep.start_ts, ep.stop_ts)
        cmd_ts, cmd_data = _series(store, "joint_command", JointState, ep.start_ts, ep.stop_ts)
        img_ts, img_data = _series(store, "color_image", Image, ep.start_ts, ep.stop_ts)

        if len(state_ts) == 0 or len(img_ts) == 0:
            logger.warning(
                "Episode %d has no %s — skipping",
                ep.index,
                "joint_state" if len(state_ts) == 0 else "camera frames",
            )
            continue

        frames = []
        for t in np.arange(ep.start_ts, ep.stop_ts, 1.0 / fps):
            si = _latest_at(state_ts, t)
            ii = _latest_at(img_ts, t)
            if si is None or ii is None:
                continue
            state = _arm_vector(state_data[si])
            if state is None:
                continue
            ci = _latest_at(cmd_ts, t)
            action = _arm_vector(cmd_data[ci]) if ci is not None else None
            frames.append(
                {
                    "observation.state": state,
                    # No command yet (arms not engaged) → action = hold current state.
                    "action": action if action is not None else state,
                    f"observation.images.{CAMERA_KEY}": _to_rgb(img_data[ii]),
                    "task": task,
                }
            )

        if not frames:
            logger.warning("Episode %d produced no frames — skipping", ep.index)
            continue

        if dataset is None:
            h, w = frames[0][f"observation.images.{CAMERA_KEY}"].shape[:2]
            features = {
                "observation.state": {
                    "dtype": "float32",
                    "shape": (len(ARM_JOINT_NAMES),),
                    "names": [list(ARM_JOINT_NAMES)],
                },
                "action": {
                    "dtype": "float32",
                    "shape": (len(ARM_JOINT_NAMES),),
                    "names": [list(ARM_JOINT_NAMES)],
                },
                f"observation.images.{CAMERA_KEY}": {
                    "dtype": "video",
                    "shape": (h, w, 3),
                    "names": ["height", "width", "channel"],
                },
            }
            dataset = LeRobotDataset.create(
                repo_id=repo_id,
                fps=int(fps),
                robot_type=robot_type,
                features=features,
                use_videos=True,
            )

        for frame in frames:
            dataset.add_frame(frame)
        dataset.save_episode()
        exported += 1
        logger.info(
            "Episode %d exported: %d frames (%.1fs)",
            ep.index,
            len(frames),
            ep.stop_ts - ep.start_ts,
        )

    if dataset is None:
        sys.exit("No episodes produced frames — nothing exported")
    logger.info("Done: %d episodes → %s", exported, dataset.root)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--db", type=Path, required=True, help="memory2 recording db")
    parser.add_argument("--repo-id", required=True, help="HF dataset repo id (user/name)")
    parser.add_argument("--task", required=True, help="task description for all episodes")
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--robot-type", default="Unitree_G1")
    args = parser.parse_args()
    export(args.db, args.repo_id, args.task, fps=args.fps, robot_type=args.robot_type)


if __name__ == "__main__":
    main()
