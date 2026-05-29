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

"""Minimal little-endian CDR (XCDR1) decoders for the Go2 MCAP channels.

Hand-rolled so the converter needs no ROS/IDL deps — just the field layouts we
recorded (see go2-station/reference/harvest/idl + 20260529-07/08). Alignment is
body-relative (the 4-byte encapsulation header is skipped first).
"""

import struct

import numpy as np


class Cur:
    def __init__(self, b):
        self.b = b
        self.p = 4  # skip 4-byte CDR encapsulation header

    def _al(self, n):
        m = (self.p - 4) % n
        if m:
            self.p += n - m

    def u8(self):
        v = self.b[self.p]
        self.p += 1
        return v

    def u16(self):
        self._al(2)
        v = struct.unpack_from("<H", self.b, self.p)[0]
        self.p += 2
        return v

    def i32(self):
        self._al(4)
        v = struct.unpack_from("<i", self.b, self.p)[0]
        self.p += 4
        return v

    def u32(self):
        self._al(4)
        v = struct.unpack_from("<I", self.b, self.p)[0]
        self.p += 4
        return v

    def f32(self):
        self._al(4)
        v = struct.unpack_from("<f", self.b, self.p)[0]
        self.p += 4
        return v

    def f64(self):
        self._al(8)
        v = struct.unpack_from("<d", self.b, self.p)[0]
        self.p += 8
        return v

    def f32n(self, n):
        return [self.f32() for _ in range(n)]

    def f64n(self, n):
        return [self.f64() for _ in range(n)]

    def s(self):
        n = self.u32()
        v = self.b[self.p : self.p + max(0, n - 1)].decode("ascii", "replace")
        self.p += n
        return v


def _stamp_ns(c):
    sec = c.i32()
    nsec = c.u32()
    return sec * 1_000_000_000 + nsec


# PointCloud2 point dtype for rt/utlidar/cloud (point_step 32): x@0 y@4 z@8
# intensity@16 ring@20(u16) time@24(f32).
_PC_DT = np.dtype(
    {
        "names": ["x", "y", "z", "intensity", "ring", "time"],
        "formats": ["<f4", "<f4", "<f4", "<f4", "<u2", "<f4"],
        "offsets": [0, 4, 8, 16, 20, 24],
        "itemsize": 32,
    }
)


def decode_pointcloud2(data):
    c = Cur(data)
    stamp = _stamp_ns(c)
    c.s()  # header.stamp, frame_id
    c.u32()
    w = c.u32()  # height, width
    nf = c.u32()
    for _ in range(nf):
        c.s()
        c.u32()
        c.u8()
        c.u32()  # name, offset, datatype, count
    c.u8()  # is_bigendian
    ps = c.u32()
    c.u32()  # point_step, row_step
    nd = c.u32()
    blob = data[c.p : c.p + nd]
    arr = np.frombuffer(blob, dtype=_PC_DT) if ps == 32 and nd >= 32 else np.empty(0, _PC_DT)
    return {"stamp_ns": stamp, "width": w, "point_step": ps, "arr": arr}


def decode_imu(data):
    c = Cur(data)
    stamp = _stamp_ns(c)
    c.s()
    o = c.f64n(4)  # orientation x,y,z,w
    c.f64n(9)  # orientation_covariance
    av = c.f64n(3)  # angular_velocity
    c.f64n(9)
    la = c.f64n(3)  # linear_acceleration
    return {"stamp_ns": stamp, "orientation": o, "ang_vel": av, "lin_acc": la}


def decode_odometry(data):
    c = Cur(data)
    stamp = _stamp_ns(c)
    c.s()
    c.s()  # header.stamp, frame_id, child_frame_id
    pos = c.f64n(3)  # pose.pose.position
    quat = c.f64n(4)  # pose.pose.orientation x,y,z,w
    return {"stamp_ns": stamp, "position": pos, "orientation": quat}


def decode_sportmode(data):
    c = Cur(data)
    stamp = _stamp_ns(c)  # TimeSpec
    c.u32()  # error_code
    c.f32n(4)
    c.f32n(3)
    c.f32n(3)
    c.f32n(3)
    c.u8()  # imu_state + temperature
    mode = c.u8()
    c.f32()
    c.u8()
    c.f32()  # mode, progress, gait_type, foot_raise_height
    pos = c.f32n(3)
    body_h = c.f32()
    vel = c.f32n(3)
    yaw = c.f32()
    return {
        "stamp_ns": stamp,
        "mode": mode,
        "position": pos,
        "body_height": body_h,
        "velocity": vel,
        "yaw_speed": yaw,
    }


def decode_compressed_image(data):
    c = Cur(data)
    stamp = _stamp_ns(c)
    c.s()  # header.stamp, frame_id
    fmt = c.s()
    n = c.u32()
    blob = bytes(data[c.p : c.p + n])
    return {"stamp_ns": stamp, "format": fmt, "data": blob}
