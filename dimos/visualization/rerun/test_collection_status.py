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

from rerun._baseclasses import Archetype

from dimos.learning.collection.episode_monitor import EpisodeStatus
from dimos.visualization.rerun.collection_status import (
    collection_status_rerun_config,
    episode_status_to_rerun,
)


def _document_text(document: Archetype) -> str:
    return str(document.text.as_arrow_array().to_pylist()[0])


def test_episode_status_to_rerun_renders_state_event_counters_and_label() -> None:
    status = EpisodeStatus(
        ts=1.0,
        state="recording",
        episodes_saved=3,
        episodes_discarded=2,
        last_event="start",
        task_label="sort blocks",
    )

    document = episode_status_to_rerun(status)

    assert document.archetype_short_name() == "TextDocument"
    assert _document_text(document) == (
        "State: recording\nLast event: start\nSaved: 3\nDiscarded: 2\nTask: sort blocks"
    )


def test_episode_status_to_rerun_omits_missing_label() -> None:
    status = EpisodeStatus(
        ts=1.0,
        state="idle",
        episodes_saved=0,
        episodes_discarded=0,
    )

    assert _document_text(episode_status_to_rerun(status)) == (
        "State: idle\nLast event: init\nSaved: 0\nDiscarded: 0"
    )


def test_config_is_visualization_only() -> None:
    config = collection_status_rerun_config()

    assert set(config) == {"visual_override"}
    assert set(config["visual_override"]) == {"world/status"}
    assert config["visual_override"]["world/status"] is episode_status_to_rerun


def test_visualization_failure_does_not_change_status_or_recorder_path() -> None:
    status = EpisodeStatus(
        ts=1.0,
        state="recording",
        episodes_saved=1,
        episodes_discarded=0,
        last_event="start",
        task_label="demo",
    )
    before = status.model_copy(deep=True)
    recorder_messages: list[EpisodeStatus] = []

    def failing_visualization(_: EpisodeStatus) -> Archetype:
        raise RuntimeError("viewer unavailable")

    # This is the subscriber seam: the recorder receives the original status;
    # a failing observer cannot replace it or redirect the recorder.
    recorder_messages.append(status)
    try:
        failing_visualization(status)
    except RuntimeError:
        pass

    assert recorder_messages == [status]
    assert recorder_messages[0] is status
    assert status == before
