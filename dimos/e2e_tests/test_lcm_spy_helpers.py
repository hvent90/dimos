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

import json
from pathlib import Path

import pytest

from dimos.e2e_tests.lcm_spy import _load_asset_aabb


def _write_objects(tmp_path: Path) -> Path:
    payload = {
        "frame": "source",
        "objects": [
            {
                "id": "sectional",
                "prim_path": "/Apt/Living/Sectional",
                "aabb": {"min": [-2.0, -1.0, 0.0], "max": [0.0, 1.0, 0.9]},
            },
            {
                "id": "fridge",
                "prim_path": "/Apt/Kitchen/Fridge",
                "aabb": {"min": [3.0, 2.0, 0.0], "max": [3.8, 2.6, 1.8]},
            },
        ],
    }
    path = tmp_path / "objects.json"
    path.write_text(json.dumps(payload))
    return path


def test_load_asset_aabb_by_id(tmp_path: Path) -> None:
    objects_path = _write_objects(tmp_path)
    aabb = _load_asset_aabb("sectional", objects_path)
    assert aabb["min"] == [-2.0, -1.0, 0.0]
    assert aabb["max"] == [0.0, 1.0, 0.9]


def test_load_asset_aabb_by_prim_path(tmp_path: Path) -> None:
    objects_path = _write_objects(tmp_path)
    aabb = _load_asset_aabb("/Apt/Kitchen/Fridge", objects_path)
    assert aabb["min"] == [3.0, 2.0, 0.0]


def test_load_asset_aabb_raises_for_unknown_asset(tmp_path: Path) -> None:
    objects_path = _write_objects(tmp_path)
    with pytest.raises(KeyError, match="oven"):
        _load_asset_aabb("oven", objects_path)
