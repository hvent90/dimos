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

"""Recognizing a re-derived region as the same place as an earlier one.

A map grows and the same room segments again with a slightly different
outline. Whoever stores regions needs to know which of the new ones *are* the
old ones, so that identity — ids, names, anything else attached to them —
survives the re-derivation. That question is answered from geometry alone;
this module knows nothing about how regions are stored.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
import math
from typing import TYPE_CHECKING

import numpy as np

from dimos.mapping.occupancy.rooms.polygons import points_in_polygon

if TYPE_CHECKING:
    from numpy.typing import NDArray


@dataclass(frozen=True)
class RegionOutline:
    """The shape of one region, as matching sees it."""

    kind: str  # "room" | "corridor" — only same-kind regions match
    polygon: NDArray[np.float64]  # (N, 2) world xy
    centroid_xy: tuple[float, float]


def match_regions(
    previous: Sequence[RegionOutline], derived: Sequence[RegionOutline]
) -> dict[int, int]:
    """Same-place pairs between two region sets: derived index -> previous index.

    Two regions are the same place when each one's centroid falls inside the
    other's polygon. That tolerates the outline jitter between derivations of
    a growing map while refusing splits and merges: when one region covers the
    ground of two, some centroid lands outside its counterpart and the pair is
    rejected — so a split or a merge retires the old regions and creates new
    ones instead of silently inheriting an identity.

    Conflicts resolve one-to-one, nearest centroids first; unmatched regions
    on either side simply don't appear.
    """
    candidates: list[tuple[float, int, int]] = []
    for d, new in enumerate(derived):
        for p, old in enumerate(previous):
            if old.kind != new.kind:
                continue
            if not points_in_polygon(np.asarray([old.centroid_xy]), new.polygon)[0]:
                continue
            if not points_in_polygon(np.asarray([new.centroid_xy]), old.polygon)[0]:
                continue
            candidates.append(
                (
                    math.hypot(
                        old.centroid_xy[0] - new.centroid_xy[0],
                        old.centroid_xy[1] - new.centroid_xy[1],
                    ),
                    d,
                    p,
                )
            )
    matched: dict[int, int] = {}
    used: set[int] = set()
    for _, d, p in sorted(candidates):
        if d in matched or p in used:
            continue
        matched[d] = p
        used.add(p)
    return matched
