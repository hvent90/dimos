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

"""Rerun adapter for collection episode status.

The adapter is deliberately a pure message-to-archetype conversion.  It does
not publish, log, or retain an ``EpisodeStatus``; the collection recorder can
therefore remain the authoritative subscriber to the status stream.
"""

from __future__ import annotations

from collections.abc import Callable
from typing import TypedDict

import rerun as rr
from rerun._baseclasses import Archetype

from dimos.learning.collection.episode_monitor import EpisodeStatus

CollectionStatusConverter = Callable[[EpisodeStatus], Archetype]


class CollectionStatusRerunConfig(TypedDict):
    """The visualization-only portion accepted by ``vis_module``."""

    visual_override: dict[str, CollectionStatusConverter]


def episode_status_to_rerun(status: EpisodeStatus) -> Archetype:
    """Render the latest collection state as a persistent Rerun document."""
    lines = [
        f"State: {status.state}",
        f"Last event: {status.last_event}",
        f"Saved: {status.episodes_saved}",
        f"Discarded: {status.episodes_discarded}",
    ]
    if status.task_label is not None:
        lines.append(f"Task: {status.task_label}")
    return rr.TextDocument(  # type: ignore[attr-defined]
        "\n".join(lines), media_type="text/plain"
    )


def collection_status_rerun_config(
    entity_path: str = "world/status",
) -> CollectionStatusRerunConfig:
    """Return a drop-in ``visual_override`` config for the collection status stream.

    Example::

        from dimos.visualization.rerun.collection_status import (
            collection_status_rerun_config,
        )
        from dimos.visualization.vis_module import vis_module

        visualization = vis_module("rerun", collection_status_rerun_config())
    """
    return {"visual_override": {entity_path: episode_status_to_rerun}}


__all__ = [
    "CollectionStatusConverter",
    "CollectionStatusRerunConfig",
    "collection_status_rerun_config",
    "episode_status_to_rerun",
]
