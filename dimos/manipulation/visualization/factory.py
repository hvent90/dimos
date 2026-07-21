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

"""Factory functions for manipulation visualization backends."""

from __future__ import annotations

from typing import TYPE_CHECKING, Protocol, runtime_checkable

from dimos.manipulation.planning.spec.protocols import VisualizationSpec
from dimos.manipulation.visualization.config import (
    ManipulationVisualizationConfig,
    MeshcatVisualizationConfig,
    NoManipulationVisualizationConfig,
)
from dimos.manipulation.visualization.viser.config import ViserVisualizationConfig

if TYPE_CHECKING:
    from dimos.manipulation.manipulation_module import ManipulationModule
    from dimos.manipulation.planning.monitor.world_monitor import WorldMonitor
    from dimos.manipulation.planning.spec.models import (
        JointPath,
        Obstacle,
        PlanningSceneInfo,
        WorldRobotID,
    )
    from dimos.manipulation.planning.spec.protocols import WorldSpec


@runtime_checkable
class _MeshcatVisualizationSource(Protocol):
    """Native visualization operations exposed by a planning world."""

    def initialize_scene(self, scene: PlanningSceneInfo) -> None: ...
    def get_visualization_url(self) -> str | None: ...
    def publish_visualization(self, ctx: object | None = None) -> None: ...
    def show_preview(self, robot_id: WorldRobotID) -> None: ...
    def hide_preview(self, robot_id: WorldRobotID) -> None: ...
    def animate_path(
        self, robot_id: WorldRobotID, path: JointPath, duration: float = 3.0
    ) -> None: ...
    def close(self) -> None: ...


class _MeshcatVisualizationAdapter:
    """VisualizationSpec facade for worlds that render obstacles natively."""

    def __init__(self, world: _MeshcatVisualizationSource) -> None:
        self._world = world

    def initialize_scene(self, scene: PlanningSceneInfo) -> None:
        self._world.initialize_scene(scene)

    def add_obstacle(self, obstacle_id: str, obstacle: Obstacle) -> None:
        return None

    def remove_obstacle(self, obstacle_id: str) -> None:
        return None

    def clear_obstacles(self) -> None:
        return None

    def get_visualization_url(self) -> str | None:
        return self._world.get_visualization_url()

    def publish_visualization(self, ctx: object | None = None) -> None:
        self._world.publish_visualization(ctx)

    def show_preview(self, robot_id: WorldRobotID) -> None:
        self._world.show_preview(robot_id)

    def hide_preview(self, robot_id: WorldRobotID) -> None:
        self._world.hide_preview(robot_id)

    def animate_path(self, robot_id: WorldRobotID, path: JointPath, duration: float = 3.0) -> None:
        self._world.animate_path(robot_id, path, duration)

    def close(self) -> None:
        self._world.close()


def create_manipulation_visualization(
    config: ManipulationVisualizationConfig,
    *,
    world: WorldSpec,
    world_monitor: WorldMonitor,
    manipulation_module: ManipulationModule,
) -> VisualizationSpec | None:
    """Create an optional manipulation visualization backend."""
    if isinstance(config, NoManipulationVisualizationConfig):
        return None

    if isinstance(config, MeshcatVisualizationConfig):
        if isinstance(world, _MeshcatVisualizationSource):
            return _MeshcatVisualizationAdapter(world)
        raise ValueError("meshcat visualization requires a world that implements VisualizationSpec")

    if isinstance(config, ViserVisualizationConfig):
        from dimos.manipulation.visualization.viser.visualizer import (
            ViserManipulationVisualizer,
        )

        return ViserManipulationVisualizer(
            world_monitor=world_monitor,
            manipulation_module=manipulation_module,
            config=config,
        )

    raise AssertionError(f"Unhandled manipulation visualization config: {config!r}")
