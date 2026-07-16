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

"""Constants and pure address/credential helpers shared by the Spot modules.

Intentionally free of any `bosdyn` import so it stays importable — and blueprint
discovery keeps working — on hosts without the SDK.
"""

from __future__ import annotations

import asyncio

from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Spot's fixed default addresses: 192.168.80.3 when it hosts its own WiFi AP,
# 10.0.0.3 on the rear Ethernet port. Probed in order when no ip is given.
SPOT_WIFI_AP_IP = "192.168.80.3"
SPOT_ETHERNET_IP = "10.0.0.3"
IP_LABELS = {SPOT_WIFI_AP_IP: "WiFi", SPOT_ETHERNET_IP: "Ethernet"}

# Spot's gRPC API listens on HTTPS/443; a TCP connect confirms real reachability
# (and matches what the SDK does) better than an ICMP ping.
SPOT_API_PORT = 443
REACHABILITY_PROBE_TIMEOUT_S = 2.0

# Motor power / posture command timeouts (seconds).
POWER_ON_TIMEOUT_S = 20.0
POWER_OFF_TIMEOUT_S = 20.0
STAND_TIMEOUT_S = 10.0
SIT_TIMEOUT_S = 10.0

# Spot's five body fisheye cameras and their matching depth cameras, ordered to
# match the grayscale_image_N / depth_image_N output streams.
GRAYSCALE_SOURCES = [
    "frontleft_fisheye_image",
    "frontright_fisheye_image",
    "left_fisheye_image",
    "right_fisheye_image",
    "back_fisheye_image",
]
DEPTH_SOURCES = [
    "frontleft_depth",
    "frontright_depth",
    "left_depth",
    "right_depth",
    "back_depth",
]

# Spot reports poses in its gravity-agnostic "vision" frame; body is the moving
# base frame. These mirror the bosdyn frame-helper constants, inlined so this
# file imports without the SDK.
VISION_FRAME = "vision"
BODY_FRAME = "body"


def default_candidate_ips() -> list[str]:
    return [SPOT_WIFI_AP_IP, SPOT_ETHERNET_IP]


async def is_reachable(ip: str) -> bool:
    try:
        _, writer = await asyncio.wait_for(
            asyncio.open_connection(ip, SPOT_API_PORT),
            timeout=REACHABILITY_PROBE_TIMEOUT_S,
        )
    except (OSError, asyncio.TimeoutError):
        return False
    # Reachable — a successful TCP handshake is all we need. Close without
    # awaiting wait_closed(); the API port speaks TLS and never completes a
    # clean plaintext close, which would otherwise hang the probe.
    writer.close()
    return True


async def resolve_ip(candidate_ips: list[str]) -> str:
    for candidate in candidate_ips:
        if await is_reachable(candidate):
            logger.info(f"Spot reachable at {candidate}")
            return candidate
    described = " or ".join(
        f"{candidate} ({IP_LABELS[candidate]})" if candidate in IP_LABELS else candidate
        for candidate in candidate_ips
    )
    raise ConnectionError(
        f"I'm unable to connect to {described}. Did you forget to connect to "
        "Spot's WiFi or plug in an Ethernet cable to Spot?"
    )


def resolve_credentials(username: str | None, password: str | None) -> tuple[str, str]:
    if not username or not password:
        raise ValueError(
            "Spot credentials missing — pass username/password in config "
            "(-o <module>.username=... -o <module>.password=...)"
        )
    return username, password
