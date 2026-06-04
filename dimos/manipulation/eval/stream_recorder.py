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

"""Optional memory2 recorder for raw manipulation telemetry streams.

This captures *raw topic streams* (joint states, commanded EE pose) to a memory2
SQLite store — the standard dimos mechanism for time-series topics — as opposed to
the discrete episode-outcome JSONL written by ``EpisodeRecorder``. Add it to the
sim blueprint so streams are captured while ``BenchmarkRunner`` drives episodes,
then read them back (time-sliced per episode) for trajectory / placement analysis.

It is a ``memory2.Recorder`` subclass (so importing this module pulls in memory2,
which is heavy — it transitively loads torch). It is therefore intentionally NOT
imported by the import-light harness (``recorder``/``report``/``runner``/``suite``);
use it only in a blueprint / run context.

Wire it into the sim blueprint (only *connected* ports record, so declaring a port
the blueprint doesn't provide is harmless)::

    from dimos.core.coordination.blueprints import autoconnect
    from dimos.manipulation.blueprints import xarm_perception_sim
    from dimos.manipulation.eval.stream_recorder import ManipulationStreamRecorder

    bench = autoconnect(xarm_perception_sim, ManipulationStreamRecorder.blueprint())

Read back after a run::

    from dimos.memory2.store.sqlite import SqliteStore
    from dimos.msgs.sensor_msgs.JointState import JointState

    store = SqliteStore(path="manip_eval_streams.db")
    store.start()
    joints = store.stream("joint_state", JointState).to_list()
"""

from __future__ import annotations

from dimos.core.core import rpc
from dimos.core.stream import In
from dimos.memory2.module import Recorder, RecorderConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState


class ManipulationStreamRecorderConfig(RecorderConfig):
    db_path: str = "manip_eval_streams.db"


class ManipulationStreamRecorder(Recorder):
    """Records manipulation telemetry streams to a memory2 SQLite store.

    Port names match the ``ControlCoordinator`` outputs so autoconnect wires them
    automatically. memory2 also attaches a world-frame tf pose to every sample, so
    end-effector pose (published on tf, not a topic) is captured implicitly.
    """

    config: ManipulationStreamRecorderConfig

    # Measured joint state from the ControlCoordinator (`joint_state: Out[JointState]`).
    # The canonical arm-telemetry stream.
    joint_state: In[JointState]

    # Commanded end-effector pose setpoint into the coordinator
    # (`cartesian_command: In[PoseStamped]`) — the EE target trajectory. Records
    # only if the running blueprint drives cartesian commands.
    cartesian_command: In[PoseStamped]

    @rpc
    def start(self) -> None:
        super().start()

    @rpc
    def stop(self) -> None:
        super().stop()
