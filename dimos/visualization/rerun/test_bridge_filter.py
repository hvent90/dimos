# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

from dataclasses import dataclass
from unittest.mock import patch

import rerun as rr

from dimos.learning.collection.blueprint import learning_collect_quest_piper_rerun
from dimos.visualization.rerun.bridge import Config, RerunBridgeModule


@dataclass
class Topic:
    name: str


@dataclass
class Message:
    def to_rerun(self) -> rr.TextDocument:
        return rr.TextDocument("message")


def test_default_bridge_config_remains_unfiltered() -> None:
    assert Config().topic_allowlist is None


def test_collection_bridge_allows_only_collection_visualization_streams() -> None:
    rerun_atoms = [
        atom
        for atom in learning_collect_quest_piper_rerun.blueprints
        if atom.module is RerunBridgeModule
    ]

    assert len(rerun_atoms) == 1
    assert rerun_atoms[0].kwargs["topic_allowlist"] == {
        "color_image",
        "coordinator_joint_state",
        "status",
    }


def test_allowlist_is_applied_at_bridge_message_boundary() -> None:
    bridge = RerunBridgeModule(topic_allowlist={"color_image", "coordinator_joint_state", "status"})
    bridge._min_intervals = {}

    try:
        with patch("dimos.visualization.rerun.bridge.rr.log") as mock_log:
            for name in (
                "depth_image",
                "pointcloud",
                "color_image",
                "coordinator_joint_state",
                "status",
            ):
                bridge._on_message(Message(), Topic(f"/{name}"))
    finally:
        bridge.stop()

    assert [call.args[0] for call in mock_log.call_args_list] == [
        "world/color_image",
        "world/coordinator_joint_state",
        "world/status",
    ]
