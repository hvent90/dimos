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

from __future__ import annotations

from threading import RLock
from types import SimpleNamespace
from typing import Any

from dimos.manipulation.planning.spec.enums import ObstacleType
from dimos.manipulation.planning.spec.models import Obstacle
from dimos.manipulation.planning.world.drake_world import DrakeWorld, _ObstacleData
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3


def _obstacle(name: str) -> Obstacle:
    return Obstacle(
        name=name,
        obstacle_type=ObstacleType.BOX,
        pose=PoseStamped(position=Vector3(), orientation=Quaternion()),  # type: ignore[call-arg]
        dimensions=(0.1, 0.2, 0.3),
    )


def _make_world() -> DrakeWorld:
    world = DrakeWorld.__new__(DrakeWorld)
    world._lock = RLock()
    world._obstacles = {}
    world._finalized = False
    world._meshcat = None
    world._obstacle_source_id = None
    world._obstacle_add_hook = None
    world._obstacle_remove_hook = None
    world._plant = SimpleNamespace(get_source_id=lambda: "source")
    return world


def test_obstacle_hooks_forward_only_real_mutations(monkeypatch: Any) -> None:
    world = _make_world()
    monkeypatch.setattr(world, "_add_obstacle_to_plant", lambda _obstacle, _id: "geometry")
    events: list[tuple[str, str, Obstacle | None]] = []
    world.set_obstacle_hooks(
        on_add=lambda obstacle_id, obstacle: events.append(("add", obstacle_id, obstacle)),
        on_remove=lambda obstacle_id: events.append(("remove", obstacle_id, None)),
    )
    obstacle = _obstacle("box")

    assert world.add_obstacle(obstacle) == "box"
    assert world.add_obstacle(obstacle) == "box"
    assert world.remove_obstacle("missing") is False
    assert world.remove_obstacle("box") is True
    assert events == [("add", "box", obstacle), ("remove", "box", None)]


def test_obstacle_hook_failure_does_not_rollback_native_mutation(monkeypatch: Any) -> None:
    world = _make_world()
    monkeypatch.setattr(world, "_add_obstacle_to_plant", lambda _obstacle, _id: "geometry")

    def fail(*_args: Any) -> None:
        raise RuntimeError("visualization failed")

    world.set_obstacle_hooks(on_add=fail, on_remove=fail)
    obstacle = _obstacle("retained")

    assert world.add_obstacle(obstacle) == "retained"
    assert world._obstacles["retained"] == _ObstacleData(
        obstacle_id="retained",
        obstacle=obstacle,
        geometry_id="geometry",
        source_id="source",
    )
    assert world.remove_obstacle("retained") is True
    assert world._obstacles == {}


def test_obstacle_hooks_are_replaceable_and_clearable(monkeypatch: Any) -> None:
    world = _make_world()
    monkeypatch.setattr(world, "_add_obstacle_to_plant", lambda _obstacle, _id: "geometry")
    first: list[str] = []
    second: list[str] = []
    obstacle = _obstacle("replaceable")

    world.set_obstacle_hooks(on_add=lambda obstacle_id, _: first.append(obstacle_id))
    world.add_obstacle(obstacle)
    world.set_obstacle_hooks(on_add=lambda obstacle_id, _: second.append(obstacle_id))
    world.remove_obstacle("replaceable")
    world.add_obstacle(obstacle)
    world.set_obstacle_hooks()
    world.remove_obstacle("replaceable")

    assert first == ["replaceable"]
    assert second == ["replaceable"]


def test_clear_obstacles_forwards_removals_and_respects_hook_teardown(
    monkeypatch: Any,
) -> None:
    world = _make_world()
    monkeypatch.setattr(world, "_add_obstacle_to_plant", lambda _obstacle, _id: "geometry")
    removed: list[str] = []
    world.set_obstacle_hooks(on_remove=removed.append)
    for name in ("first", "second"):
        world.add_obstacle(_obstacle(name))

    world.clear_obstacles()
    assert world._obstacles == {}
    assert removed == ["first", "second"]

    world.add_obstacle(_obstacle("cleared-hook"))
    world.set_obstacle_hooks()
    world.clear_obstacles()
    assert world._obstacles == {}
    assert removed == ["first", "second"]


def test_clear_obstacles_survives_remove_hook_failures(monkeypatch: Any) -> None:
    world = _make_world()
    monkeypatch.setattr(world, "_add_obstacle_to_plant", lambda _obstacle, _id: "geometry")

    def fail(_obstacle_id: str) -> None:
        raise RuntimeError("visualization failed")

    world.set_obstacle_hooks(on_remove=fail)
    world.add_obstacle(_obstacle("first"))
    world.add_obstacle(_obstacle("second"))

    world.clear_obstacles()
    assert world._obstacles == {}
