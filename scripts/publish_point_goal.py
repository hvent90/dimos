#!/usr/bin/env python3
"""Publish a single ``PointStamped`` onto ``/point_goal``.

Used to drive the ``unitree-g1-groot-wbc-point`` blueprint's pointing
pipeline without needing the viser button. ``G1ManipulationModule``
subscribes to ``/point_goal`` and runs the full reset-both-arms-then-
point cycle on each value received.

Usage:
    scripts/publish_point_goal.py 0.6 0.2 1.1
    scripts/publish_point_goal.py 0.6 0.2 1.1 --frame map

Coordinates are world-frame meters; +x = forward, +y = robot's left,
+z = up. Stay within the arm's pointing workspace (front hemisphere,
roughly 0.3-1.0 m radius from each shoulder).
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

# Allow running directly without `uv pip install -e`.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from dimos.core.transport import LCMTransport  # noqa: E402
from dimos.msgs.geometry_msgs.PointStamped import PointStamped  # noqa: E402


def main() -> int:
    p = argparse.ArgumentParser(description="Publish PointStamped to /point_goal")
    p.add_argument("x", type=float, help="World X (forward), meters")
    p.add_argument("y", type=float, help="World Y (robot left), meters")
    p.add_argument("z", type=float, help="World Z (up), meters")
    p.add_argument("--frame", default="map", help="Frame id (default: map)")
    p.add_argument(
        "--topic",
        default="/point_goal",
        help="LCM topic (default: /point_goal)",
    )
    args = p.parse_args()

    pt = PointStamped(
        x=args.x, y=args.y, z=args.z, ts=time.time(), frame_id=args.frame
    )
    tx = LCMTransport(args.topic, PointStamped)
    tx.publish(pt)
    print(f"Published {pt} on {args.topic}")
    # LCM is async; give the publish a tick to flush before the process exits.
    time.sleep(0.1)
    return 0


if __name__ == "__main__":
    sys.exit(main())
