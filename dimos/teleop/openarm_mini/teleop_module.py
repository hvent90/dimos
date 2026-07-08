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

"""OpenArm Mini teleop module using the shared teleop runtime."""

from __future__ import annotations

from dimos.teleop.openarm_mini.calibration import load_calibration
from dimos.teleop.openarm_mini.config import (
    OpenArmMiniCalibrationError,
    OpenArmMiniDependencyError,
    OpenArmMiniSide,
    OpenArmMiniTeleopConfig,
)
from dimos.teleop.openarm_mini.feetech import OpenArmMiniLeaderReader
from dimos.teleop.openarm_mini.mapping import combine_side_commands, map_side_readings
from dimos.teleop.runtime.teleop_module import TeleopModule, TeleopModuleConfig
from dimos.teleop.runtime.types import TeleopCommand
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class OpenArmMiniTeleopModuleConfig(TeleopModuleConfig, OpenArmMiniTeleopConfig):
    """Config for OpenArm Mini leader teleoperation."""

    # Default to one side so running the concrete module directly only requires
    # one leader calibration/port override. Dual-arm blueprints opt into both.
    enabled_sides: tuple[OpenArmMiniSide, ...] = ("left",)


class OpenArmMiniTeleopModule(TeleopModule):
    """Teleop module for OpenArm Mini leader devices."""

    config: OpenArmMiniTeleopModuleConfig  # type: ignore[assignment]

    def __init__(self, **kwargs: object) -> None:
        super().__init__(**kwargs)
        self._buses: dict[OpenArmMiniSide, OpenArmMiniLeaderReader] = {}
        self._previous_positions_by_side: dict[OpenArmMiniSide, dict[str, float]] = {}
        self._last_read_error: str | None = None
        self._teleop_connected = False

    @property
    def openarm_mini_config(self) -> OpenArmMiniTeleopConfig:
        return self.config

    def connect_teleop(self) -> None:
        if self._teleop_connected:
            return
        openarm_mini = self.openarm_mini_config
        buses: dict[OpenArmMiniSide, OpenArmMiniLeaderReader] = {}
        try:
            baudrate = openarm_mini.connection_baudrate()
            for side in openarm_mini.sides():
                calibration = load_calibration(openarm_mini.calibration_path(side), side)
                bus = OpenArmMiniLeaderReader(
                    side,
                    openarm_mini.port(side),
                    calibration,
                    baudrate,
                )
                bus.connect()
                buses[side] = bus
        except (
            OpenArmMiniCalibrationError,
            OpenArmMiniDependencyError,
            ValueError,
            RuntimeError,
            OSError,
        ):
            for bus in buses.values():
                bus.disconnect()
            raise

        self._buses = buses
        self._teleop_connected = True

    def disconnect_teleop(self) -> None:
        for bus in self._buses.values():
            bus.disconnect()
        self._buses = {}
        self._previous_positions_by_side = {}
        self._last_read_error = None
        self._teleop_connected = False

    def get_current_command(self) -> TeleopCommand | None:
        openarm_mini = self.openarm_mini_config
        if not self._teleop_connected or not openarm_mini.authority_active:
            return None

        side_commands = []
        next_previous_positions_by_side: dict[OpenArmMiniSide, dict[str, float]] = {}
        try:
            for side in openarm_mini.sides():
                bus = self._buses[side]
                side_command = map_side_readings(
                    side,
                    bus.read_positions(),
                    target_joint_names=openarm_mini.target_joint_names(side),
                    previous_positions_by_joint=self._previous_positions_by_side.get(side),
                    max_joint_jump_radians=openarm_mini.max_joint_jump_radians,
                )
                side_commands.append(side_command)
                next_previous_positions_by_side[side] = side_command.positions_by_joint
        except (KeyError, ValueError, RuntimeError, OSError) as exc:
            error_message = str(exc)
            if error_message != self._last_read_error:
                logger.warning(
                    "OpenArm Mini teleop read failed; dropping command: %s",
                    error_message,
                )
                self._last_read_error = error_message
            return None

        self._last_read_error = None
        self._previous_positions_by_side = next_previous_positions_by_side
        return TeleopCommand(payload=combine_side_commands(side_commands))
