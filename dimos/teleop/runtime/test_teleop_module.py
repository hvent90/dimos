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

from __future__ import annotations

from collections.abc import Callable, Iterator, Sequence
from typing import Any

import pytest

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.teleop.openarm_mini.config import OpenArmMiniTeleopConfig
from dimos.teleop.runtime.teleop_module import TeleopModule
from dimos.teleop.runtime.types import TeleopCommand


class _Adapter:
    def __init__(self, commands: Sequence[TeleopCommand | None] | None = None) -> None:
        self.commands = list(commands) if commands is not None else []
        self.connected = False
        self.disconnected = False

    def connect(self) -> None:
        self.connected = True

    def disconnect(self) -> None:
        self.disconnected = True

    def get_current_command(self) -> TeleopCommand | None:
        if not self.commands:
            return None
        return self.commands.pop(0)


@pytest.fixture
def module_factory() -> Iterator[Callable[..., TeleopModule]]:
    modules: list[TeleopModule] = []

    def make(runtime_adapter: _Adapter, **kwargs: Any) -> TeleopModule:
        config = {"max_publish_rate_hz": 100.0, "stale_command_timeout_s": 1.0, **kwargs}
        module = TeleopModule(runtime_adapter=runtime_adapter, **config)
        modules.append(module)
        return module

    yield make

    for module in modules:
        module.stop()


def _publishers(mocker: Any, module: TeleopModule) -> dict[str, Any]:
    return {
        "joint_command": mocker.patch.object(module.joint_command, "publish"),
        "coordinator_cartesian_command": mocker.patch.object(
            module.coordinator_cartesian_command, "publish"
        ),
        "twist_command": mocker.patch.object(module.twist_command, "publish"),
    }


def _assert_nothing_published(publishers: dict[str, Any]) -> None:
    for publisher in publishers.values():
        publisher.assert_not_called()


def test_command_envelope_requires_payload_unless_stopping() -> None:
    joint = JointState({"name": ["j0"], "position": [1.0]})

    assert TeleopCommand(joint).payload is joint
    assert TeleopCommand(stop=True).stop
    with pytest.raises(ValueError, match="payload"):
        TeleopCommand()
    with pytest.raises(ValueError, match="payload"):
        TeleopCommand(joint, stop=True)


def test_module_config_creates_adapter_from_discriminated_backend(
    module_factory: Callable[..., TeleopModule],
) -> None:
    module = module_factory(
        _Adapter(), adapter={"backend": "openarm_mini", "enabled_sides": ("right",)}
    )

    assert isinstance(module.teleop_config.adapter, OpenArmMiniTeleopConfig)
    assert module.teleop_config.adapter.enabled_sides == ("right",)


def test_explicit_stop_command_is_not_published(
    mocker: Any, module_factory: Callable[..., TeleopModule]
) -> None:
    module = module_factory(_Adapter([TeleopCommand(timestamp=1.0, stop=True)]))
    publishers = _publishers(mocker, module)
    mocker.patch.object(module, "_now", return_value=1.0)

    module.tick()

    _assert_nothing_published(publishers)


@pytest.mark.parametrize(
    ("payload", "stream_name"),
    [
        (JointState({"name": ["j0"], "position": [1.0]}), "joint_command"),
        (PoseStamped(position=[1.0, 2.0, 3.0]), "coordinator_cartesian_command"),
        (Twist(linear=[1.0, 0.0, 0.0], angular=[0.0, 0.0, 0.0]), "twist_command"),
    ],
)
def test_tick_routes_payload_by_type(
    payload: JointState | PoseStamped | Twist,
    stream_name: str,
    mocker: Any,
    module_factory: Callable[..., TeleopModule],
) -> None:
    module = module_factory(_Adapter([TeleopCommand(payload, timestamp=1.0)]))
    publishers = _publishers(mocker, module)
    mocker.patch.object(module, "_now", return_value=1.0)

    module.tick()

    publishers[stream_name].assert_called_once_with(payload)
    for name, publisher in publishers.items():
        if name != stream_name:
            publisher.assert_not_called()


def test_stale_commands_are_not_published(
    mocker: Any, module_factory: Callable[..., TeleopModule]
) -> None:
    joint = JointState({"name": ["j0"], "position": [1.0]})
    module = module_factory(_Adapter([TeleopCommand(joint, timestamp=1.0)]))
    publishers = _publishers(mocker, module)
    mocker.patch.object(module, "_now", return_value=2.01)

    module.tick()

    _assert_nothing_published(publishers)


def test_rate_limiting_skips_commands(
    mocker: Any, module_factory: Callable[..., TeleopModule]
) -> None:
    first = JointState({"name": ["j0"], "position": [1.0]})
    second = JointState({"name": ["j0"], "position": [2.0]})
    module = module_factory(
        _Adapter([TeleopCommand(first, timestamp=1.0), TeleopCommand(second, timestamp=1.0)]),
        max_publish_rate_hz=10.0,
    )
    publishers = _publishers(mocker, module)
    mocker.patch.object(module, "_now", side_effect=[1.0, 1.0, 1.0, 1.05, 1.05])

    module.tick()
    module.tick()

    publishers["joint_command"].assert_called_once_with(first)


def test_start_stop_connect_disconnect_and_no_publish_after_stop(
    mocker: Any,
    module_factory: Callable[..., TeleopModule],
) -> None:
    joint = JointState({"name": ["j0"], "position": [1.0]})
    adapter = _Adapter([TeleopCommand(joint, timestamp=1.0)])
    module = module_factory(adapter)
    publishers = _publishers(mocker, module)
    mocker.patch("dimos.teleop.runtime.teleop_module.threading.Thread.start")
    mocker.patch("dimos.teleop.runtime.teleop_module.threading.Thread.join")
    mocker.patch.object(module, "_now", return_value=1.0)

    module.start()
    module.stop()
    module.tick()

    assert adapter.connected
    assert adapter.disconnected
    _assert_nothing_published(publishers)
