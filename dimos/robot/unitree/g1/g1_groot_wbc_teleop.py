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

"""WASD teleop panel with an arming gate for the G1 GR00T WBC policy."""

from __future__ import annotations

import os
import threading
import time
from typing import Any

import pygame
from pygame.locals import K_RETURN

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.control.coordinator import ControlCoordinator
from dimos.core.core import rpc
from dimos.core.global_config import global_config
from dimos.core.module import Module
from dimos.core.stream import Out
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

os.environ["SDL_VIDEODRIVER"] = "x11"

DEFAULT_LINEAR_SPEED: float = 0.3
DEFAULT_ANGULAR_SPEED: float = 0.6
DEFAULT_BOOST_MULTIPLIER: float = 2.0
DEFAULT_SLOW_MULTIPLIER: float = 0.5
DEFAULT_RAMP_SECONDS: float = 10.0

_WINDOW_WIDTH = 560
_WINDOW_HEIGHT = 440
_FONT_SIZE = 24
_CONTROL_RATE_HZ = 50
_RAMP_GRACE_SECONDS = 0.5
_BACKGROUND_COLOR = (30, 30, 30)
_HELP_TEXT_COLOR = (150, 150, 150)


class G1GrootWbcTeleop(Module):
    cmd_vel: Out[Twist]
    coordinator: ControlCoordinator

    _stop_event: threading.Event
    _keys_held: set[int] | None = None
    _thread: threading.Thread | None = None
    _screen: pygame.Surface | None = None
    _clock: pygame.time.Clock | None = None
    _font: pygame.font.Font | None = None

    def __init__(
        self,
        linear_speed: float = DEFAULT_LINEAR_SPEED,
        angular_speed: float = DEFAULT_ANGULAR_SPEED,
        boost_multiplier: float = DEFAULT_BOOST_MULTIPLIER,
        slow_multiplier: float = DEFAULT_SLOW_MULTIPLIER,
        ramp_seconds: float = DEFAULT_RAMP_SECONDS,
        **kwargs: Any,
    ) -> None:
        super().__init__(**kwargs)
        self._stop_event = threading.Event()
        self.linear_speed = linear_speed
        self.angular_speed = angular_speed
        self.boost_multiplier = boost_multiplier
        self.slow_multiplier = slow_multiplier
        self.ramp_seconds = ramp_seconds
        self._armed = bool(global_config.simulation)
        self._arming = False

    @rpc
    def start(self) -> None:
        super().start()
        self._keys_held = set()
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._pygame_loop, daemon=True)
        self._thread.start()

    @rpc
    def stop(self) -> None:
        self._publish(0.0, 0.0, 0.0)
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(DEFAULT_THREAD_JOIN_TIMEOUT)
        super().stop()

    def _arm(self) -> None:
        if self._armed or self._arming:
            return
        self._arming = True
        logger.warning("GR00T arm requested, ramping to default over %.0fs", self.ramp_seconds)
        threading.Thread(target=self._arm_worker, daemon=True).start()

    def _arm_worker(self) -> None:
        try:
            self.coordinator.set_dry_run(False)
            self.coordinator.set_activated(True)
        except Exception:
            logger.exception("GR00T arm failed")
            self._arming = False
            return
        time.sleep(self.ramp_seconds + _RAMP_GRACE_SECONDS)
        self._armed = True
        self._arming = False
        logger.warning("GR00T armed, WASD live")

    def _disarm(self) -> None:
        self._armed = False
        self._arming = False
        self._publish(0.0, 0.0, 0.0)
        try:
            self.coordinator.set_activated(False)
        except Exception:
            logger.exception("GR00T disarm failed")
        logger.warning("GR00T disarmed")

    def _pygame_loop(self) -> None:
        if self._keys_held is None:
            raise RuntimeError("_keys_held not initialized")

        pygame.init()
        self._screen = pygame.display.set_mode((_WINDOW_WIDTH, _WINDOW_HEIGHT), pygame.SWSURFACE)
        pygame.display.set_caption("G1 GR00T WBC Teleop")
        self._clock = pygame.time.Clock()
        self._font = pygame.font.Font(None, _FONT_SIZE)

        while not self._stop_event.is_set():
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self._stop_event.set()
                elif event.type == pygame.KEYDOWN:
                    self._keys_held.add(event.key)
                    if event.key == K_RETURN:
                        self._arm()
                    elif event.key == pygame.K_k:
                        self._disarm()
                    elif event.key == pygame.K_SPACE:
                        self._keys_held.clear()
                        self._publish(0.0, 0.0, 0.0)
                        logger.warning("EMERGENCY STOP!")
                    elif event.key == pygame.K_ESCAPE:
                        self._stop_event.set()
                elif event.type == pygame.KEYUP:
                    self._keys_held.discard(event.key)

            vx, vy, wz = self._twist_from_keys()
            self._publish(vx, vy, wz)
            self._update_display(vx, vy, wz)

            if self._clock is None:
                raise RuntimeError("_clock not initialized")
            self._clock.tick(_CONTROL_RATE_HZ)

        self._publish(0.0, 0.0, 0.0)
        pygame.quit()

    def _twist_from_keys(self) -> tuple[float, float, float]:
        if self._keys_held is None or not self._armed:
            return 0.0, 0.0, 0.0

        vx = vy = wz = 0.0
        if pygame.K_w in self._keys_held:
            vx = self.linear_speed
        if pygame.K_s in self._keys_held:
            vx = -self.linear_speed
        if pygame.K_q in self._keys_held:
            vy = self.linear_speed
        if pygame.K_e in self._keys_held:
            vy = -self.linear_speed
        if pygame.K_a in self._keys_held:
            wz = self.angular_speed
        if pygame.K_d in self._keys_held:
            wz = -self.angular_speed

        multiplier = 1.0
        if pygame.K_LSHIFT in self._keys_held or pygame.K_RSHIFT in self._keys_held:
            multiplier = self.boost_multiplier
        elif pygame.K_LCTRL in self._keys_held or pygame.K_RCTRL in self._keys_held:
            multiplier = self.slow_multiplier

        return vx * multiplier, vy * multiplier, wz * multiplier

    def _publish(self, vx: float, vy: float, wz: float) -> None:
        twist = Twist()
        twist.linear = Vector3(vx, vy, 0.0)
        twist.angular = Vector3(0.0, 0.0, wz)
        self.cmd_vel.publish(twist)

    def _state_text(self) -> tuple[str, tuple[int, int, int]]:
        if self._arming:
            return f"ARMING {self.ramp_seconds:.0f}s ramp, hold torso upright", (255, 200, 0)
        if self._armed:
            return "ARMED, WASD live", (0, 255, 0)
        return "DISARMED, hold torso upright, press ENTER to arm", (255, 80, 80)

    def _update_display(self, vx: float, vy: float, wz: float) -> None:
        if self._screen is None or self._font is None or self._keys_held is None:
            raise RuntimeError("Not initialized correctly")

        self._screen.fill(_BACKGROUND_COLOR)
        y_pos = 20

        state, state_color = self._state_text()
        lines: list[tuple[str, tuple[int, int, int]]] = [
            ("G1 GR00T WBC Teleop", (0, 255, 255)),
            ("", (255, 255, 255)),
            (state, state_color),
            ("", (255, 255, 255)),
            (f"Linear X (Forward/Back): {vx:+.2f} m/s", (255, 255, 255)),
            (f"Linear Y (Strafe L/R): {vy:+.2f} m/s", (255, 255, 255)),
            (f"Angular Z (Turn L/R): {wz:+.2f} rad/s", (255, 255, 255)),
        ]
        for text, color in lines:
            if text:
                self._screen.blit(self._font.render(text, True, color), (20, y_pos))
            y_pos += 30

        y_pos = 300
        help_texts = [
            "ENTER: Arm | K: Disarm",
            "WS: Move | AD: Turn | QE: Strafe",
            "Shift: Boost | Ctrl: Slow",
            "Space: E-Stop | ESC: Quit",
        ]
        for text in help_texts:
            self._screen.blit(self._font.render(text, True, _HELP_TEXT_COLOR), (20, y_pos))
            y_pos += 25

        pygame.display.flip()
