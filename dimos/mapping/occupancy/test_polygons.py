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
import pytest

from dimos.mapping.occupancy.polygons import points_in_polygon, polygon_from_flat

UNIT_SQUARE = np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]])


def test_points_in_square() -> None:
    points = np.array(
        [
            [0.5, 0.5],  # center
            [0.99, 0.01],  # near corner, inside
            [1.5, 0.5],  # right of square
            [-0.1, 0.5],  # left of square
            [0.5, 1.2],  # above
        ]
    )
    result = points_in_polygon(points, UNIT_SQUARE)
    assert result.tolist() == [True, True, False, False, False]


def test_closed_ring_same_as_open() -> None:
    closed = np.vstack([UNIT_SQUARE, UNIT_SQUARE[0]])
    points = np.array([[0.5, 0.5], [2.0, 2.0]])
    assert points_in_polygon(points, closed).tolist() == [True, False]


def test_concave_polygon() -> None:
    # L-shape: unit square with the top-right quadrant notched out.
    l_shape = np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 0.5], [0.5, 0.5], [0.5, 1.0], [0.0, 1.0]])
    points = np.array(
        [
            [0.25, 0.25],  # bottom-left, inside
            [0.75, 0.25],  # bottom-right, inside
            [0.25, 0.75],  # top-left, inside
            [0.75, 0.75],  # notch, outside
        ]
    )
    assert points_in_polygon(points, l_shape).tolist() == [True, True, True, False]


def test_vertex_winding_direction_irrelevant() -> None:
    clockwise = UNIT_SQUARE[::-1]
    points = np.array([[0.5, 0.5], [1.5, 0.5]])
    assert points_in_polygon(points, clockwise).tolist() == [True, False]


def test_empty_points() -> None:
    result = points_in_polygon(np.empty((0, 2)), UNIT_SQUARE)
    assert result.shape == (0,)


def test_degenerate_polygon_rejected() -> None:
    with pytest.raises(ValueError, match="polygon"):
        points_in_polygon(np.array([[0.0, 0.0]]), np.array([[0.0, 0.0], [1.0, 1.0]]))


def test_polygon_from_flat() -> None:
    polygon = polygon_from_flat([0.0, 0.0, 2.0, 0.0, 2.0, 2.0, 0.0, 2.0])
    assert polygon.shape == (4, 2)
    assert points_in_polygon(np.array([[1.0, 1.0]]), polygon).tolist() == [True]


def test_polygon_from_flat_rejects_odd_and_short() -> None:
    with pytest.raises(ValueError):
        polygon_from_flat([0.0, 0.0, 1.0])
    with pytest.raises(ValueError):
        polygon_from_flat([0.0, 0.0, 1.0, 1.0])
