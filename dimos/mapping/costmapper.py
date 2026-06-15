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

from dataclasses import asdict
from typing import Any

import numpy as np
from pydantic import Field

from dimos.core.stream import In, Out
from dimos.mapping.pointclouds.occupancy import (
    OCCUPANCY_ALGOS,
    HeightCostConfig,
    OccupancyConfig,
)
from dimos.memory2.puremodule import PureModule, PureModuleConfig, latest, tick
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_COLOR_UNKNOWN = (0, 0, 0, 0)
_COLOR_FREE = (72, 73, 129, 255)
_COLOR_OCCUPIED = (255, 140, 0, 255)
_COLOR_LETHAL = (220, 30, 30, 255)

# Indexed by grid value + 1: 0 = unknown, 1 = free, 2..101 = cost 1..100.
_COSTMAP_COLOR_LOOKUP_TABLE = np.empty((102, 4), dtype=np.uint8)
_COSTMAP_COLOR_LOOKUP_TABLE[0] = _COLOR_UNKNOWN
_COSTMAP_COLOR_LOOKUP_TABLE[1] = _COLOR_FREE
_COSTMAP_COLOR_LOOKUP_TABLE[2:101] = _COLOR_OCCUPIED
_COSTMAP_COLOR_LOOKUP_TABLE[101] = _COLOR_LETHAL

_COSTMAP_Z_OFFSET = 0.02


def costmap_to_rerun(grid: OccupancyGrid) -> Any:
    return grid.to_rerun(
        color_lookup_table=_COSTMAP_COLOR_LOOKUP_TABLE,
        z_offset=_COSTMAP_Z_OFFSET,
    )


class Config(PureModuleConfig):
    algo: str = "height_cost"
    config: OccupancyConfig = Field(default_factory=HeightCostConfig)
    # for robots that cant see directly below themself
    initial_safe_radius_meters: float = 0.0


class CostMapper(PureModule):
    """Turn the freshest map into an occupancy costmap, one grid per map update.

    ``global_map`` (from the voxel mapper) is always wired and drives the
    ticks; ``relocalized_map`` (from relocalization) is optional and
    preferred when present, matching the original ``combine_latest`` +
    select-merged behaviour. The step is pure: same map in, same grid out.
    """

    config: Config
    global_map: In[PointCloud2] = tick()
    relocalized_map: In[PointCloud2] = latest()
    global_costmap: Out[OccupancyGrid]

    def step(self, global_map: PointCloud2, relocalized_map: PointCloud2 | None) -> OccupancyGrid:
        msg = relocalized_map if relocalized_map is not None else global_map
        return self._calculate_costmap(msg)

    def _calculate_costmap(self, msg: PointCloud2) -> OccupancyGrid:
        occupancy_function = OCCUPANCY_ALGOS[self.config.algo]
        return occupancy_function(msg, **asdict(self.config.config))
