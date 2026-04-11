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

"""ROS 2 environment setup for Galaxea R1 Pro communication.

The R1 Pro robot runs ROS 2 Humble with FastDDS on a Jetson Orin at
192.168.123.150.  The DimOS laptop runs Jazzy.  To bridge the two
ROS 2 versions reliably we must:

  1. Use FastDDS (rmw_fastrtps_cpp) on both ends — CycloneDDS has
     EDP incompatibilities across Humble/Jazzy.
  2. Use a unicast peer list instead of multicast so discovery works
     across the ethernet link.
  3. Set the same ROS_DOMAIN_ID on both sides.

Call ``ensure_r1pro_ros_env()`` **before** any ``RawROS.start()``
or ``rclpy.init()`` — environment variables must be set before the
RMW is initialised.
"""

from __future__ import annotations

import logging
import os
from pathlib import Path

log = logging.getLogger(__name__)

# Default FastDDS XML shipped with the test scripts.
_DEFAULT_FASTDDS_XML = (
    Path(__file__).resolve().parents[2]
    / "scripts"
    / "r1pro_test"
    / "fastdds_r1pro.xml"
)


def ensure_r1pro_ros_env(
    domain_id: int = 41,
    fastdds_xml: str | Path | None = None,
) -> None:
    """Set environment variables required for R1 Pro ROS 2 communication.

    This is idempotent — variables that are already set are not
    overwritten, so user-level overrides are respected.

    By default only ``ROS_DOMAIN_ID`` and ``RMW_IMPLEMENTATION`` are set.
    Multicast DDS discovery works over a direct ethernet link without a
    custom FastDDS XML profile.  Pass ``fastdds_xml`` explicitly only if
    you need unicast peer discovery (e.g. across subnets).

    Args:
        domain_id: ``ROS_DOMAIN_ID`` value (must match the robot).
        fastdds_xml: Path to the FastDDS XML profile.  When provided, sets
            ``FASTRTPS_DEFAULT_PROFILES_FILE``.  Leave as ``None`` (default)
            to rely on multicast discovery.
    """
    _setdefault("ROS_DOMAIN_ID", str(domain_id))
    _setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")

    if fastdds_xml is not None:
        xml_path = Path(fastdds_xml)
        if xml_path.exists():
            _setdefault("FASTRTPS_DEFAULT_PROFILES_FILE", str(xml_path))
        else:
            log.warning(
                "FastDDS XML not found at %s — unicast discovery disabled.",
                xml_path,
            )


def _setdefault(key: str, value: str) -> None:
    """Set *key* in ``os.environ`` only if it is not already present."""
    if key not in os.environ:
        os.environ[key] = value
        log.debug("R1 Pro env: %s=%s", key, value)
    else:
        log.debug("R1 Pro env: %s already set to %s", key, os.environ[key])
