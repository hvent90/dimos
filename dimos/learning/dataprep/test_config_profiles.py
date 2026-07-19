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

from pathlib import Path

import pytest

from dimos.learning.dataprep.core import DataPrepConfig


@pytest.mark.parametrize(
    ("filename", "anchor", "observation_keys"),
    [
        ("galaxea_a1z_state_config.json", "joint_state", {"joint_state"}),
        ("galaxea_a1z_camera_config.json", "image", {"image", "joint_state"}),
    ],
)
def test_a1z_dataprep_profiles_are_valid(
    filename: str,
    anchor: str,
    observation_keys: set[str],
) -> None:
    path = Path(__file__).with_name(filename)

    config = DataPrepConfig.model_validate_json(path.read_text())

    assert config.sync.anchor == anchor
    assert set(config.observation) == observation_keys
    assert set(config.action) == {"joint_target"}
    assert config.output.metadata["robot"] == "galaxea_a1z"
