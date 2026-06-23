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

"""Root-cause probe for the Go2 curve OVER-TURN: is it commanded or physical?

Run this ALONGSIDE a benchmark CIRCLE run (constant radius R), on the same host
as the coordinator. It subscribes to /coordinator/joint_state and, at the
robot's ACTUAL forward speed, compares three yaw rates:

  required_wz  = v_forward / R       what the path geometry needs at this speed
  commanded_wz = joint_state.velocity[wz]   what the controller actually sent
  achieved_wz  = d(yaw)/dt           what the robot actually turned

Verdict (printed on Ctrl-C):
  * |commanded| / |required| >> 1   -> the over-turn is COMMANDED. The tracker
    is asking for more yaw than the robot's forward progress warrants, because
    the time-indexed reference is spatially AHEAD of the lagging robot. Fix is
    in the reference (path-following / sample at the robot's arc position), not
    the plant.
  * commanded ~ required but achieved >> required -> the PLANT over-delivers
    yaw (the artifact's wz gain is too high). Fix is re-characterization.

    # terminal 1:  GO2_ESO= ... dimos run unitree-go2-benchmark-trajtrack -o benchmarker.config=<artifact>
    # terminal 2:  python -m dimos.utils.benchmarking.overturn_probe --radius 1.0
"""

from __future__ import annotations

import argparse
import math
from statistics import median
import time

from dimos.core.transport import LCMTransport
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.utils.trigonometry import angle_diff

_TOPIC = "/coordinator/joint_state"
_JX, _JY, _JYAW = "go2/vx", "go2/vy", "go2/wz"


class _Probe:
    def __init__(self, radius: float, v_min: float, out_path: str, alpha: float = 0.3) -> None:
        self.R = radius
        self.v_min = v_min
        self.alpha = alpha
        self.f = open(out_path, "w")
        self.f.write("t,v_fwd,cmd_wz,req_wz,ach_wz\n")
        self._prev: tuple[float, float, float, float] | None = None
        self._v = 0.0  # EMA-smoothed forward speed
        self._ach = 0.0  # EMA-smoothed achieved yaw rate
        self.rows: list[tuple[float, float, float, float]] = []  # (v_fwd, cmd, req, ach)

    def on_msg(self, msg: JointState) -> None:
        if not msg.name or not msg.velocity:
            return
        idx = {n: i for i, n in enumerate(msg.name)}
        try:
            x = float(msg.position[idx[_JX]])
            y = float(msg.position[idx[_JY]])
            yaw = float(msg.position[idx[_JYAW]])
            cmd_wz = float(msg.velocity[idx[_JYAW]])
        except (KeyError, IndexError):
            return
        now = time.perf_counter()
        if self._prev is None:
            self._prev = (now, x, y, yaw)
            return
        pt, px, py, pyaw = self._prev
        dt = now - pt
        if dt <= 1e-3:
            return
        # De-dup HELD poses: the coordinator republishes joint_state faster than
        # odom updates, so consecutive identical poses would make d(pose)/dt
        # alternate between 0 and huge spikes. Only differentiate on real motion.
        if abs(x - px) < 1e-4 and abs(y - py) < 1e-4 and abs(angle_diff(yaw, pyaw)) < 1e-4:
            return
        self._prev = (now, x, y, yaw)
        # measured forward (body-x) speed and yaw rate by differentiation
        v_fwd = ((x - px) / dt) * math.cos(yaw) + ((y - py) / dt) * math.sin(yaw)
        ach = angle_diff(yaw, pyaw) / dt
        self._v = self.alpha * v_fwd + (1 - self.alpha) * self._v
        self._ach = self.alpha * ach + (1 - self.alpha) * self._ach
        if self._v < self.v_min:  # only score when genuinely moving forward
            return
        req = self._v / self.R
        self.rows.append((self._v, cmd_wz, req, self._ach))
        self.f.write(f"{now:.3f},{self._v:.3f},{cmd_wz:.3f},{req:.3f},{self._ach:.3f}\n")
        self.f.flush()
        if len(self.rows) % 20 == 0:
            print(
                f"  v={self._v:.2f}  cmd_wz={cmd_wz:+.2f}  req_wz={req:+.2f}  "
                f"ach_wz={self._ach:+.2f}  (cmd/req={abs(cmd_wz) / max(abs(req), 1e-3):.2f})"
            )

    def summary(self) -> None:
        self.f.close()
        if len(self.rows) < 10:
            print("\n[probe] not enough moving samples — was a circle run active?")
            return
        cmd = median(abs(r[1]) for r in self.rows)
        req = median(abs(r[2]) for r in self.rows)
        ach = median(abs(r[3]) for r in self.rows)
        v = median(r[0] for r in self.rows)
        print(f"\n=== over-turn probe ({len(self.rows)} moving samples, R={self.R} m) ===")
        print(f"  median forward speed v = {v:.2f} m/s")
        print(f"  required_wz  = v/R     = {req:.3f} rad/s   (what the path needs)")
        print(f"  commanded_wz           = {cmd:.3f} rad/s   (cmd/req = {cmd / max(req, 1e-3):.2f})")
        print(f"  achieved_wz  = dyaw/dt = {ach:.3f} rad/s   (ach/req = {ach / max(req, 1e-3):.2f})")
        cmd_ratio = cmd / max(req, 1e-3)
        plant_ratio = ach / max(cmd, 1e-3)
        print("\n  VERDICT:")
        if cmd_ratio > 1.15:
            print(f"  -> COMMANDED over-turn: controller asks {cmd_ratio:.2f}x the needed yaw.")
            print("     The time-indexed reference is ahead of the lagging robot.")
            print("     Fix is in the REFERENCE (path-following), not the plant.")
        elif plant_ratio > 1.15:
            print(f"  -> PLANT over-turn: robot delivers {plant_ratio:.2f}x the commanded yaw.")
            print("     The artifact's wz gain is too high. Fix = re-characterize.")
        else:
            print("  -> neither strongly over-commands nor over-delivers here; the over-turn")
            print("     is elsewhere (heading-FB, frame, or it didn't reproduce this run).")


def main() -> None:
    ap = argparse.ArgumentParser(description="Go2 over-turn root-cause probe")
    ap.add_argument("--radius", type=float, default=1.0, help="circle radius being run (m)")
    ap.add_argument("--v-min", type=float, default=0.15, help="ignore samples below this fwd speed")
    ap.add_argument("--out", default="/tmp/go2_overturn_probe.csv")
    args = ap.parse_args()

    probe = _Probe(args.radius, args.v_min, args.out)
    sub = LCMTransport(_TOPIC, JointState)
    unsub = sub.subscribe(probe.on_msg)
    print(f"[probe] listening on {_TOPIC} (R={args.radius} m). Run a circle, then Ctrl-C.")
    try:
        while True:
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            unsub()
        except Exception:
            pass
        probe.summary()
        print(f"  raw log: {args.out}")


if __name__ == "__main__":
    main()
