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

"""Compatibility import path for shared scene asset geometry loading."""

from dimos.simulation.scene_assets.mesh_scene import (
    SceneMeshAlignment,
    ScenePrimMesh,
    floor_z_under_origin,
    load_scene_mesh,
    load_scene_prims,
    make_raycasting_scene,
)

__all__ = [
    "SceneMeshAlignment",
    "ScenePrimMesh",
    "floor_z_under_origin",
    "load_scene_mesh",
    "load_scene_prims",
    "make_raycasting_scene",
]
