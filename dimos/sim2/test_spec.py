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

from dimos.sim2.spec import (
    EntityDescriptor,
    EntityState,
    WorldManifest,
    WorldStateFrame,
)


def test_world_manifest_roundtrip_keeps_stable_entity_metadata() -> None:
    manifest = WorldManifest(
        scene_revision="office-v3",
        frame_id="world",
        entities=(
            EntityDescriptor(
                entity_id="chair",
                kind="dynamic",
                backend_name="entity:chair",
                mesh_ref="/tmp/chair.glb",
                mass=8.0,
            ),
        ),
    )

    assert WorldManifest.lcm_decode(manifest.lcm_encode()) == manifest


def test_world_state_roundtrip_contains_only_ticked_state() -> None:
    frame = WorldStateFrame(
        episode_id=4,
        physics_tick=12,
        control_tick=3,
        sim_time=0.06,
        scene_revision="office-v3",
        entities=(
            EntityState(
                entity_id="chair",
                frame_id="world",
                position=(1.0, 2.0, 0.0),
                quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
            ),
        ),
    )

    assert WorldStateFrame.lcm_decode(frame.lcm_encode()) == frame
