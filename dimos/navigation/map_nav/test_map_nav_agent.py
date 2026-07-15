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

"""Tests for MapNavAgent milestone sampling and click-to-goal skills."""

from __future__ import annotations

import json

import numpy as np

from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.LineSegments3D import LineSegments3D
from dimos.navigation.map_nav.map_nav_agent import (
    MapNavAgent,
    points_from_node_edges,
    sample_milestones_from_node_edges,
    sample_milestones_from_points,
)


def test_points_from_node_edges_unique() -> None:
    segs = [
        ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
        ((1.0, 0.0, 0.0), (2.0, 0.0, 0.0)),
        ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),  # duplicate
    ]
    pts = points_from_node_edges(segs)
    assert len(pts) == 3


def test_sample_from_edges_spreads_in_3d() -> None:
    segs = []
    for i in range(7):
        segs.append(((float(i), 0.0, 0.0), (float(i + 1), 0.0, 0.0)))
        segs.append(((float(i), 0.0, 3.0), (float(i + 1), 0.0, 3.0)))
    segs.append(((7.0, 0.0, 0.0), (7.0, 0.0, 3.0)))
    ms, home = sample_milestones_from_node_edges(segs, 6, seed_xyz=(0.0, 0.0, 0.0))
    assert len(ms) == 6
    assert home == (ms[0].x, ms[0].y, ms[0].z)
    assert min(m.z for m in ms) < 0.5
    assert max(m.z for m in ms) > 2.5


def test_sample_milestones_empty() -> None:
    assert sample_milestones_from_points(np.zeros((0, 3)), 5) == []
    ms, _ = sample_milestones_from_node_edges([], 5, seed_xyz=(0.0, 0.0, 0.0))
    assert ms == []


def test_go_to_milestone_publishes_clicked_point() -> None:
    agent = MapNavAgent(enabled=True, n_milestones=5, body_height_m=0.31)
    try:
        clicked: list[PointStamped] = []
        agent.clicked_point.subscribe(lambda p: clicked.append(p))
        agent._on_odom(PoseStamped(position=[0.1, 0.1, 0.1 + 0.31], frame_id="world"))
        segs = [
            ((float(i) * 0.5, float(j) * 0.5, 0.1), (float(i) * 0.5 + 0.5, float(j) * 0.5, 0.1))
            for i in range(8)
            for j in range(8)
        ]
        agent._on_node_edges(LineSegments3D(frame_id="world", segments=segs))
        assert len(agent._milestones) == 5

        listed = json.loads(agent.list_milestones())
        assert listed["built"] is True
        assert listed["count"] == 5

        mid = agent._milestones[2]
        msg = agent.go_to_milestone(mid.id)
        assert "Clicked milestone" in msg
        assert len(clicked) == 1
        assert abs(clicked[0].x - mid.x) < 1e-9
    finally:
        agent.stop()


def test_build_from_node_edges_teleports_to_m1() -> None:
    agent = MapNavAgent(enabled=True, n_milestones=2, body_height_m=0.0)
    try:
        poses: list[PoseStamped] = []
        agent.set_pose.subscribe(lambda p: poses.append(p))
        agent._on_odom(PoseStamped(position=[0.0, 0.0, 0.0], frame_id="world"))
        assert agent._built is False
        edges = LineSegments3D(
            frame_id="world",
            segments=[
                ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
                ((1.0, 0.0, 0.0), (2.0, 0.0, 0.0)),
            ],
        )
        agent._on_node_edges(edges)
        assert agent._built is True
        assert len(agent._milestones) == 2
        assert len(poses) == 1
        assert abs(poses[0].x - agent._milestones[0].x) < 1e-9
    finally:
        agent.stop()


def test_go_to_point_publishes_click() -> None:
    agent = MapNavAgent(enabled=True, n_milestones=3)
    try:
        clicked: list[PointStamped] = []
        agent.clicked_point.subscribe(lambda p: clicked.append(p))
        agent.go_to_point(1.5, 2.5, 0.3)
        assert len(clicked) == 1
        assert clicked[0].x == 1.5
    finally:
        agent.stop()
