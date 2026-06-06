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

from __future__ import annotations

from dimos.core.rpc_client import RPCClient
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.msgs.sensor_msgs.JointState import JointState

_client: RPCClient | None = None
_ROBOT_NAMES = ["left_arm", "right_arm"]
_LEFT_TARGET = [0.45, 0.0, 0.0, 0.0, 0.0, 0.0]
_RIGHT_TARGET = [-0.45, 0.0, 0.0, 0.0, 0.0, 0.0]


def client() -> RPCClient:
    global _client
    if _client is None:
        _client = RPCClient(None, ManipulationModule)
    return _client


def robots() -> list[str]:
    return client().list_robots()


def joints() -> dict[str, list[float] | None]:
    return {robot_name: client().get_current_joints(robot_name) for robot_name in _ROBOT_NAMES}


def state() -> str:
    return client().get_state()


def url() -> str | None:
    return client().get_visualization_url()


def dual_plan_joints() -> bool:
    return client().plan_to_joints(
        [JointState(position=_LEFT_TARGET), JointState(position=_RIGHT_TARGET)],
        _ROBOT_NAMES,
    )


def dual_preview(duration: float = 8.0) -> bool:
    return client().preview_path(duration, _ROBOT_NAMES)


def dual_execute() -> bool:
    return client().execute(_ROBOT_NAMES)


def bad_request() -> bool:
    return client().plan_to_joints([JointState(position=_LEFT_TARGET)], _ROBOT_NAMES)


def dual_smoke(execute: bool = False) -> bool:
    if not dual_plan_joints():
        return False
    if not dual_preview():
        return False
    malformed_failed = bad_request() is False
    if execute:
        return malformed_failed and dual_execute()
    return malformed_failed


def stop() -> None:
    if _client is not None:
        _client.stop_rpc_client()


def commands() -> None:
    print("robots()")
    print("joints()")
    print("state()")
    print("url()")
    print("dual_plan_joints()")
    print("dual_preview(duration=8.0)")
    print("bad_request()")
    print("dual_execute()")
    print("dual_smoke(execute=False)")
    print("stop()")


if __name__ == "__main__":
    print("Dual-arm planning RPC client ready.")
    print("Run: dimos run dual-xarm6-mock-planner-coordinator")
    print("Type commands() for available functions.")
