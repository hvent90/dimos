#!/usr/bin/env python3
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

"""Convert a go2-station MCAP recording into a Rerun .rrd.

Usage:
    uv run --with mcap --with rerun-sdk --with numpy --with pillow \
        python scripts/mcap_to_rerun.py <in.mcap> [out.rrd]

Logs, on the **onboard `publish_time`** timeline:
  rt/utlidar/cloud       → Points3D (xyz, intensity-shaded) under world/base/lidar
  rt/utlidar/robot_odom  → Transform3D world/base (places the cloud in world frame)
  rt/utlidar/imu         → accel/gyro scalars
  rt/sportmodestate      → mode / body_height / vel / yaw scalars
  rt/frontvideo          → camera image (JPEG, if the recording has it)
  latency/<chan>         → (log_time - publish_time) ms, for channels with an onboard stamp
Static base_link→lidar extrinsic from 20260529-06.
"""

import json
import math
import sys

import go2_cdr as cdr
from mcap.reader import make_reader
import numpy as np
import rerun as rr

# base_link → lidar extrinsic (20260529-06): [x,y,z, roll,pitch,yaw]
BASE_LIDAR_T = [0.28216, 0.0, -0.02467]
BASE_LIDAR_RPY = [0.0, 2.88, 0.0]


def quat_rpy(r, p, y):
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return [
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ]


def scalar(v):
    return rr.Scalars(v) if hasattr(rr, "Scalars") else rr.Scalar(v)


def enc_image(jpeg):
    if hasattr(rr, "EncodedImage"):
        return rr.EncodedImage(contents=jpeg, media_type="image/jpeg")
    import io

    from PIL import Image

    return rr.Image(np.asarray(Image.open(io.BytesIO(jpeg)).convert("RGB")))


def set_t(ns):
    # Rerun's time API changed across versions: old set_time_nanos/_seconds,
    # new set_time(timeline, timestamp=epoch_seconds).
    if hasattr(rr, "set_time_nanos"):
        rr.set_time_nanos("onboard", int(ns))
    elif hasattr(rr, "set_time"):
        try:
            rr.set_time("onboard", timestamp=ns / 1e9)
        except TypeError:
            rr.set_time("onboard", duration=ns / 1e9)
    else:
        rr.set_time_seconds("onboard", ns / 1e9)


def main():
    inp = sys.argv[1]
    out = sys.argv[2] if len(sys.argv) > 2 else inp.rsplit(".", 1)[0] + ".rrd"

    rr.init("go2-station")
    # Embedded blueprint: group multi-axis scalars into single plots (origin =
    # the parent path → all child series in one view) + 3D + camera. Avoids the
    # per-leaf auto-layout (the "only see accel x,y" / stale-view confusion).
    try:
        import rerun.blueprint as rrb

        bp = rrb.Blueprint(
            rrb.Horizontal(
                rrb.Spatial3DView(origin="/world", name="lidar + pose"),
                rrb.Vertical(
                    rrb.Spatial2DView(origin="/camera", name="camera"),
                    rrb.Grid(
                        rrb.TimeSeriesView(origin="/imu/accel", name="accel"),
                        rrb.TimeSeriesView(origin="/imu/gyro", name="gyro"),
                        rrb.TimeSeriesView(origin="/state", name="state"),
                        rrb.TimeSeriesView(origin="/seen", name="telemetry"),
                        rrb.TimeSeriesView(origin="/cmd", name="cmd vel"),
                        rrb.TimeSeriesView(origin="/latency", name="latency (ms)"),
                    ),
                    row_shares=[1, 2],
                ),
                column_shares=[2, 1],
            ),
            collapse_panels=True,
        )
        rr.save(out, default_blueprint=bp)
    except Exception:
        rr.save(out)  # older rerun without blueprint API

    # static lidar mount under the (moving) base.
    rr.log(
        "world/base/lidar",
        rr.Transform3D(
            translation=BASE_LIDAR_T, rotation=rr.Quaternion(xyzw=quat_rpy(*BASE_LIDAR_RPY))
        ),
        static=True,
    )

    counts = {}
    with open(inp, "rb") as f:
        for _sch, ch, msg in make_reader(f).iter_messages():
            t = ch.topic
            counts[t] = counts.get(t, 0) + 1
            set_t(msg.publish_time)
            # latency (receive - onboard), for channels carrying an onboard stamp
            if msg.log_time != msg.publish_time and t in (
                "rt/utlidar/cloud",
                "rt/utlidar/imu",
                "rt/utlidar/robot_odom",
                "rt/sportmodestate",
            ):
                short = t.rsplit("/", 1)[-1]
                rr.log(f"latency/{short}", scalar((msg.log_time - msg.publish_time) / 1e6))
            try:
                if t == "rt/utlidar/cloud":
                    a = cdr.decode_pointcloud2(msg.data)["arr"]
                    if len(a):
                        xyz = np.stack([a["x"], a["y"], a["z"]], axis=1).astype(np.float32)
                        good = np.isfinite(xyz).all(axis=1)
                        xyz = xyz[good]
                        inten = a["intensity"][good]
                        n = np.clip(inten / (inten.max() + 1e-6), 0, 1) if len(inten) else inten
                        col = (
                            (np.stack([n, n, n], axis=1) * 255).astype(np.uint8) if len(n) else None
                        )
                        rr.log("world/base/lidar/points", rr.Points3D(xyz, colors=col, radii=0.01))
                elif t == "rt/utlidar/robot_odom":
                    o = cdr.decode_odometry(msg.data)
                    rr.log(
                        "world/base",
                        rr.Transform3D(
                            translation=o["position"], rotation=rr.Quaternion(xyzw=o["orientation"])
                        ),
                    )
                elif t == "rt/utlidar/imu":
                    m = cdr.decode_imu(msg.data)
                    for ax, v in zip("xyz", m["lin_acc"], strict=False):
                        rr.log(f"imu/accel/{ax}", scalar(v))
                    for ax, v in zip("xyz", m["ang_vel"], strict=False):
                        rr.log(f"imu/gyro/{ax}", scalar(v))
                elif t == "rt/sportmodestate":
                    m = cdr.decode_sportmode(msg.data)
                    rr.log("state/mode", scalar(m["mode"]))
                    rr.log("state/body_height", scalar(m["body_height"]))
                    rr.log("state/yaw_speed", scalar(m["yaw_speed"]))
                    for ax, v in zip("xy", m["velocity"][:2], strict=False):
                        rr.log(f"state/vel/{ax}", scalar(v))
                elif t == "rt/frontvideo":
                    img = cdr.decode_compressed_image(msg.data)
                    rr.log("camera", enc_image(img["data"]))
                elif t == "control_log":
                    # JSON operator-intent events (not CDR).
                    e = json.loads(msg.data)
                    et = e.get("type", "")
                    if et == "velocity_input":
                        rr.log("cmd/vel/lx", scalar(e.get("lx", 0.0)))
                        rr.log("cmd/vel/ly", scalar(e.get("ly", 0.0)))
                        rr.log("cmd/vel/az", scalar(e.get("az", 0.0)))
                    else:
                        extra = " ".join(f"{k}={v}" for k, v in e.items() if k != "type")
                        rr.log("control/events", rr.TextLog(f"{et} {extra}".strip()))
                elif t == "telemetry":
                    # JSON telemetry snapshot (not CDR) — same object the browser sees.
                    m = json.loads(msg.data)
                    rr.log("seen/battery", scalar(m.get("battery", 0.0)))
                    rr.log("seen/current_a", scalar(m.get("current_a", 0.0)))
                    rr.log("seen/body_h", scalar(m.get("body_h", 0.0)))
                    rr.log("seen/yaw", scalar(m.get("yaw", 0.0)))
                    vel = m.get("vel", [0.0, 0.0]) or [0.0, 0.0]
                    rr.log("seen/vel/x", scalar(vel[0] if len(vel) > 0 else 0.0))
                    rr.log("seen/vel/y", scalar(vel[1] if len(vel) > 1 else 0.0))
                    rr.log("seen/lidar_hz", scalar(m.get("lidar_hz", 0.0)))
                    rr.log("seen/imu_hz", scalar(m.get("imu_hz", 0.0)))
                    rr.log("seen/mode", scalar(m.get("mode", 0)))
                    rr.log("seen/obstacle", scalar(1 if m.get("obstacle") else 0))
                    rr.log("seen/lidar", scalar(1 if m.get("lidar") else 0))
                    rr.log("seen/rage", scalar(1 if m.get("rage") else 0))
            except Exception as e:
                # tolerate a bad message rather than aborting the whole file
                pass

    print("channels:", {k: counts[k] for k in sorted(counts)})
    print("wrote", out)


if __name__ == "__main__":
    main()
