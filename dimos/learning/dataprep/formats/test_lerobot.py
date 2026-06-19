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

"""Smoke tests for the LeRobot v2 writer/reader.

The state-only test always runs (parquet + meta + stats, exercises the
try/finally cleanup with no writers open). The image test additionally checks
the MP4 path + canonical `observation.images.*` naming, and skips if no mp4v
codec is available in the environment. Skips entirely if pyarrow isn't
installed (`learning` optional-dependency group).
"""

from __future__ import annotations

from collections.abc import Iterator
import json
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("pyarrow")
cv2 = pytest.importorskip("cv2")

from dimos.learning.dataprep.core import OutputConfig, Sample
from dimos.learning.dataprep.formats.lerobot import inspect, write


def _state_samples(n: int = 4) -> Iterator[Sample]:
    for i in range(n):
        yield Sample(
            ts=float(i),
            episode_id="ep_000000",
            observation={"state": np.arange(6, dtype=np.float32)},
            action={"action": np.full(6, float(i), dtype=np.float32)},
        )


def _image_samples(n: int = 4) -> Iterator[Sample]:
    for i in range(n):
        yield Sample(
            ts=float(i),
            episode_id="ep_000000",
            observation={
                "state": np.arange(6, dtype=np.float32),
                "cam": np.full((16, 16, 3), i, dtype=np.uint8),
            },
            action={"action": np.zeros(6, dtype=np.float32)},
        )


def test_lerobot_state_only_layout_and_naming(tmp_path: Path) -> None:
    out = OutputConfig(
        format="lerobot", path=tmp_path / "ds", metadata={"fps": 10.0, "robot": "xarm7"}
    )
    root = write(_state_samples(), out)

    assert (root / "meta" / "info.json").exists()
    assert (root / "meta" / "episodes.jsonl").exists()
    assert (root / "meta" / "tasks.jsonl").exists()
    assert (root / "meta" / "stats.json").exists()
    assert (root / "data" / "chunk-000" / "episode_000000.parquet").exists()

    info = json.loads((root / "meta" / "info.json").read_text())
    assert info["total_episodes"] == 1
    assert info["total_frames"] == 4
    assert info["fps"] == 10.0
    # single low-dim state + single action → canonical names
    assert "observation.state" in info["features"]
    assert "action" in info["features"]


def test_lerobot_inspect_state_only(tmp_path: Path) -> None:
    out = OutputConfig(format="lerobot", path=tmp_path / "ds", metadata={"fps": 10.0})
    root = write(_state_samples(), out)
    info = inspect(root)
    assert info["format"] == "lerobot"
    assert info["episodes"] == 1
    assert info["frames"] == 4
    assert "observation.state" in info["observation"]
    assert "action" in info["action"]
    assert info["has_stats"] is True


def test_lerobot_with_images_writes_mp4_and_video_feature(tmp_path: Path) -> None:
    out = OutputConfig(format="lerobot", path=tmp_path / "ds", metadata={"fps": 10.0})
    try:
        root = write(_image_samples(), out)
    except RuntimeError as e:
        if "VideoWriter" in str(e):
            pytest.skip(f"no mp4v encoder available in this environment: {e}")
        raise

    mp4 = root / "videos" / "chunk-000" / "observation.images.cam" / "episode_000000.mp4"
    assert mp4.exists() and mp4.stat().st_size > 0

    info = json.loads((root / "meta" / "info.json").read_text())
    assert info["features"]["observation.images.cam"]["dtype"] == "video"
    # image column is excluded from parquet; state/action remain
    assert "observation.state" in info["features"]
    assert info["total_frames"] == 4
