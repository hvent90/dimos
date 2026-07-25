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

from dimos.msgs.nav_msgs.LineSegments3D import LineSegments3D


def test_lcm_encode_decode_roundtrip() -> None:
    segments = [
        ((0.0, 1.0, 2.0), (3.0, 4.0, 5.0)),
        ((-1.5, 0.25, 2.0), (6.0, -2.0, 0.3)),
    ]
    msg = LineSegments3D(
        ts=1_000_000.25, frame_id="world", segments=segments, traversability=[1.0, 0.5]
    )
    decoded = LineSegments3D.lcm_decode(msg.lcm_encode())
    assert decoded.frame_id == "world"
    assert abs(decoded.ts - 1_000_000.25) < 1e-6
    assert decoded._segments == segments
    assert decoded._traversability == [1.0, 0.5]


def test_lcm_encode_empty() -> None:
    decoded = LineSegments3D.lcm_decode(LineSegments3D(ts=5.0, frame_id="map").lcm_encode())
    assert decoded._segments == []
    assert len(decoded) == 0
