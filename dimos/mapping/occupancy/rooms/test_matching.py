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

import numpy as np

from dimos.mapping.occupancy.rooms.matching import RegionOutline, match_regions


def _box(x0: float, y0: float, x1: float, y1: float, kind: str = "room") -> RegionOutline:
    polygon = np.asarray([[x0, y0], [x1, y0], [x1, y1], [x0, y1]], dtype=np.float64)
    return RegionOutline(kind=kind, polygon=polygon, centroid_xy=((x0 + x1) / 2.0, (y0 + y1) / 2.0))


def test_same_place_with_jittered_outline_matches() -> None:
    previous = [_box(0.0, 0.0, 4.0, 4.0)]
    derived = [_box(-0.15, 0.1, 4.2, 3.9)]
    assert match_regions(previous, derived) == {0: 0}


def test_a_merge_inherits_no_identity() -> None:
    # Two rooms either side of a wall become one open span. Its centroid
    # lands on the old wall — inside neither old polygon.
    previous = [_box(0.0, 0.0, 4.0, 4.0), _box(5.0, 0.0, 9.0, 4.0)]
    derived = [_box(0.0, 0.0, 9.0, 4.0)]
    assert match_regions(previous, derived) == {}


def test_a_split_inherits_no_identity() -> None:
    # One room divided by a new wall: the old centroid falls in the gap.
    previous = [_box(0.0, 0.0, 9.0, 4.0)]
    derived = [_box(0.0, 0.0, 4.0, 4.0), _box(5.0, 0.0, 9.0, 4.0)]
    assert match_regions(previous, derived) == {}


def test_a_lopsided_split_keeps_the_half_that_covers_the_old_centroid() -> None:
    # Splits are only refused where the mutual test actually fails. A wall
    # placed off-center leaves one half sitting on the old centroid, and
    # that half is the same place by every available measure.
    previous = [_box(0.0, 0.0, 9.0, 4.0)]
    derived = [_box(0.0, 0.0, 6.0, 4.0), _box(6.5, 0.0, 9.0, 4.0)]
    assert match_regions(previous, derived) == {0: 0}


def test_a_room_never_inherits_a_corridor() -> None:
    previous = [_box(0.0, 0.0, 4.0, 4.0, kind="corridor")]
    derived = [_box(0.0, 0.0, 4.0, 4.0, kind="room")]
    assert match_regions(previous, derived) == {}


def test_overlapping_candidates_resolve_one_to_one_by_distance() -> None:
    # Both derived rooms mutually contain the old one's centroid; only the
    # nearer may have it, and the other starts fresh.
    previous = [_box(0.0, 0.0, 4.0, 4.0)]
    derived = [_box(0.5, 0.5, 4.5, 4.5), _box(0.1, 0.1, 4.1, 4.1)]
    assert match_regions(previous, derived) == {1: 0}


def test_nothing_matches_across_disjoint_maps() -> None:
    previous = [_box(0.0, 0.0, 4.0, 4.0)]
    derived = [_box(20.0, 20.0, 24.0, 24.0)]
    assert match_regions(previous, derived) == {}
    assert match_regions([], derived) == {}
    assert match_regions(previous, []) == {}
