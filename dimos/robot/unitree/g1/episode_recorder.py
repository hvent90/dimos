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

"""Episode recorder for G1 Quest teleop demonstrations.

Continuously records joint state, joint commands, camera, and odom to a
memory2 SQLite store. Episode boundaries are just timestamp markers in an
``episodes`` stream, driven by the right Quest controller:

    A (rising edge)  — start episode / stop episode (toggle)
    B (rising edge)  — cancel the in-progress episode

Everything is always recorded; an exporter slices the streams by the
start/stop markers afterwards (start→cancel pairs are dropped). This keeps
the hot path trivial — no buffering or file rotation on button presses.
"""

from __future__ import annotations

from datetime import datetime
from pathlib import Path
import threading
from typing import Any

from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.stream import In
from dimos.memory2.module import Recorder, RecorderConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.teleop.quest.quest_types import Buttons
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class G1EpisodeRecorderConfig(RecorderConfig):
    # Never clobber demonstration data on restart — if the db exists we
    # roll to a timestamped sibling instead (see _unique_db_path).
    db_path: str | Path = "recording_g1_teleop.db"
    overwrite: bool = False


class G1EpisodeRecorder(Recorder):
    """Records G1 teleop sessions with Quest-button episode markers.

    State (``joint_state``) and action (``joint_command`` — the teleop IK
    targets) are recorded separately so an exporter can build proper
    (observation, action) pairs for imitation learning.
    """

    config: G1EpisodeRecorderConfig

    joint_state: In[JointState]
    joint_command: In[JointState]
    color_image: In[Image]
    odom: In[PoseStamped]
    buttons: In[Buttons]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._episode_lock = threading.Lock()
        self._episode_idx = 0
        self._episode_open = False
        self._prev_a = False
        self._prev_b = False

    @rpc
    def start(self) -> None:
        if not self.config.overwrite:
            self.config.db_path = self._unique_db_path(Path(self.config.db_path))
        super().start()
        if self.config.g.replay:
            return
        self._episodes = self.store.stream("episodes", str)
        self.register_disposable(Disposable(self.buttons.subscribe(self._on_buttons)))
        logger.info(
            "G1 episode recorder armed — A starts/stops an episode, B cancels (db: %s)",
            self.config.db_path,
        )

    @staticmethod
    def _unique_db_path(path: Path) -> Path:
        if not path.exists():
            return path
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        rolled = path.with_name(f"{path.stem}_{stamp}{path.suffix}")
        logger.info("Recording %s already exists — rolling to %s", path, rolled)
        return rolled

    def _on_buttons(self, msg: Buttons) -> None:
        """Edge-detect right-controller A/B into episode markers."""
        a, b = msg.right_primary, msg.right_secondary
        with self._episode_lock:
            a_pressed = a and not self._prev_a
            b_pressed = b and not self._prev_b
            self._prev_a, self._prev_b = a, b

            if b_pressed and self._episode_open:
                self._episode_open = False
                self._episodes.append("cancel", tags={"episode": self._episode_idx})
                logger.info("Episode %d CANCELLED", self._episode_idx)
            elif a_pressed:
                if self._episode_open:
                    self._episode_open = False
                    self._episodes.append("stop", tags={"episode": self._episode_idx})
                    logger.info("Episode %d saved", self._episode_idx)
                else:
                    self._episode_idx += 1
                    self._episode_open = True
                    self._episodes.append("start", tags={"episode": self._episode_idx})
                    logger.info("Episode %d recording…", self._episode_idx)


g1_episode_recorder = G1EpisodeRecorder.blueprint

__all__ = ["G1EpisodeRecorder", "G1EpisodeRecorderConfig", "g1_episode_recorder"]
