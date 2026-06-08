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

from pathlib import Path
from typing import Any

import pytest

from dimos.simulation.mujoco.entity_scene import add_entities_to_spec


class _FakeBody:
    def __init__(self, name: str) -> None:
        self.name = name
        self.mass: float | None = None
        self.freejoints: list[str] = []
        self.geoms: list[dict[str, Any]] = []

    def add_freejoint(self, *, name: str) -> None:
        self.freejoints.append(name)

    def add_geom(self, **kwargs: Any) -> None:
        self.geoms.append(kwargs)


class _FakeWorldBody:
    def __init__(self) -> None:
        self.bodies: list[_FakeBody] = []

    def add_body(self, *, name: str, pos: list[float], quat: list[float]) -> _FakeBody:
        del pos, quat
        body = _FakeBody(name)
        self.bodies.append(body)
        return body


class _FakeSpec:
    def __init__(self) -> None:
        self.worldbody = _FakeWorldBody()
        self.meshes: list[dict[str, str]] = []

    def add_mesh(self, *, name: str, file: str) -> None:
        self.meshes.append({"name": name, "file": file})


def test_add_entities_to_spec_uses_cooked_collision_paths(tmp_path: Path) -> None:
    pytest.importorskip("mujoco")
    hull_0 = tmp_path / "hull_000.obj"
    hull_1 = tmp_path / "hull_001.obj"
    hull_0.write_text("o hull_000\n")
    hull_1.write_text("o hull_001\n")
    spec = _FakeSpec()

    add_entities_to_spec(
        spec,  # type: ignore[arg-type]
        [
            {
                "id": "chair_016",
                "spawn": "initial",
                "initial_pose": {
                    "x": 1.0,
                    "y": 2.0,
                    "z": 0.5,
                    "qw": 1.0,
                    "qx": 0.0,
                    "qy": 0.0,
                    "qz": 0.0,
                },
                "descriptor": {
                    "entity_id": "chair_016",
                    "kind": "dynamic",
                    "shape_hint": "mesh",
                    "extents": [],
                    "mass": 8.0,
                },
                "collision_paths": [str(hull_0), str(hull_1)],
            }
        ],
    )

    assert [mesh["file"] for mesh in spec.meshes] == [str(hull_0), str(hull_1)]
    body = spec.worldbody.bodies[0]
    assert body.mass == 8.0
    assert body.freejoints == ["entity:chair_016:free"]
    assert [geom["meshname"] for geom in body.geoms] == [
        "entity:chair_016:hull000",
        "entity:chair_016:hull001",
    ]
    assert "size" not in body.geoms[0]
