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

"""Canonical robot kinds for hosted teleop.

Single source of truth (within dimos) for the set of cockpits the operator UI
can render. Command modules declare their kind from here and push it to the
broker provider (``BrokerProvider.set_robot_type``) at init, which sends it in
the session POST. The broker is a separate service with no dimos dependency, so
it re-validates the wire string against its own copy — the HTTP value is the
contract, not this class.

Kept out of the transport layer (``BrokerConfig`` takes a plain ``str``) so the
generic provider never depends on this application-level enum.
"""

from enum import StrEnum


class RobotType(StrEnum):
    """Operator cockpit kind. Value is the wire string sent to the broker."""

    GO2 = "go2"
    XARM = "xarm"
