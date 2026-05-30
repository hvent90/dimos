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

# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License").

"""Ingest a Go2 MCAP recording into a memory2 SQLite dataset (.db).

Decodes the onboard DDS channels, deskews each lidar scan into the WORLD frame
(the convention the Go2 publishes and that ``dimos map`` assumes), and writes the
streams the mapping CLI reads. Two trajectories are emitted so you can compare
them: the raw leg-inertial ``odom`` and our reconstructed ``odom_bestz`` (leg
xy/yaw + pitch-recovered z, from ``<mcap>_bestz.txt`` in the data dir).

Streams written:
    color_image       Image     RGB, posed at camera_optical in world
    odom              PoseStamped   Go2 leg-inertial odometry
    odom_bestz        PoseStamped   leg xy/yaw + reconstructed z
    lidar             PointCloud2   per-scan world cloud (deskewed by odom)
    lidar_bestz       PointCloud2   per-scan world cloud (deskewed by odom_bestz)
    lidar_1s          PointCloud2   1 s accumulation (odom)
    lidar_bestz_1s    PointCloud2   1 s accumulation (odom_bestz)

Usage:
    uv run python -m dimos.robot.unitree.go2.mcap.ingest \
        data/go2_china_office_indoor.mcap --out go2_china_office_indoor.db --seconds 60
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import typer

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.robot.unitree.go2.mcap import go2_cdr as cdr
from dimos.robot.unitree.go2.mcap.extrinsics import CAM_Q, CAM_T, EXT_R, EXT_T
from mcap.reader import make_reader

CLOUD = "rt/utlidar/cloud"
ODOM = "rt/utlidar/robot_odom"
VIDEO = "rt/frontvideo"


# --- quaternion helpers (xyzw) ----------------------------------------------
def _qrot(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Rotate v[n,3] by xyzw quaternions q[n,4]."""
    xyz, w = q[:, :3], q[:, 3:4]
    t = 2.0 * np.cross(xyz, v)
    return v + w * t + np.cross(xyz, t)


def _qmul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Hamilton product a⊗b for xyzw quats (broadcast over rows)."""
    ax, ay, az, aw = a[..., 0], a[..., 1], a[..., 2], a[..., 3]
    bx, by, bz, bw = b[..., 0], b[..., 1], b[..., 2], b[..., 3]
    return np.stack(
        [
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        ],
        axis=-1,
    )


def _interp(tt: np.ndarray, pos: np.ndarray, quat: np.ndarray, q: np.ndarray):
    """LERP position + NLERP quaternion of a trajectory at query times q."""
    idx = np.clip(np.searchsorted(tt, q), 1, len(tt) - 1)
    i0, i1 = idx - 1, idx
    dt = tt[i1] - tt[i0]
    f = np.clip((q - tt[i0]) / np.where(dt == 0, 1.0, dt), 0.0, 1.0)[:, None]
    p = pos[i0] * (1 - f) + pos[i1] * f
    q0, q1 = quat[i0], quat[i1]
    q1 = np.where(((q0 * q1).sum(1) < 0)[:, None], -q1, q1)
    qi = q0 * (1 - f) + q1 * f
    return p, qi / np.linalg.norm(qi, axis=1, keepdims=True)


def _deskew(pts: np.ndarray, tpt: np.ndarray, tt, pos, quat) -> np.ndarray:
    """Per-point deskew of lidar-frame pts (at times tpt) into the world frame."""
    p, q = _interp(tt, pos, quat, tpt)
    pbase = pts @ EXT_R.T + EXT_T
    return _qrot(q, pbase) + p


def _pose7(tt, pos, quat, t: float) -> tuple:
    """Single interpolated (x,y,z,qx,qy,qz,qw) world pose at time t."""
    p, q = _interp(tt, pos, quat, np.array([t]))
    return (*p[0], *q[0])


def _load_odom(mcap: Path, seconds: float | None):
    """First-cloud anchor + leg-odom trajectory (t_rel, pos, quat)."""
    with open(mcap, "rb") as f:
        anchor = next(m.publish_time for _s, _c, m in make_reader(f).iter_messages(topics=[CLOUD]))
    t, pos, quat = [], [], []
    with open(mcap, "rb") as f:
        for _s, _c, m in make_reader(f).iter_messages(topics=[ODOM]):
            tr = (m.publish_time - anchor) / 1e9
            if tr < 0 or (seconds is not None and tr > seconds + 1):
                continue
            o = cdr.decode_odometry(m.data)
            t.append(tr)
            pos.append(o["position"])
            quat.append(o["orientation"])
    return anchor, np.array(t), np.array(pos, np.float64), np.array(quat, np.float64)


def main(
    mcap: Path = typer.Argument(..., help="Go2 .mcap recording"),
    out: Path = typer.Option(None, "--out", help="Output .db (default: <mcap>.db)"),
    seconds: float = typer.Option(None, "--seconds", help="Only first N seconds"),
    odom_hz: float = typer.Option(30.0, "--odom-hz", help="Downsample odom streams"),
    voxel: float = typer.Option(0.05, "--voxel", help="Voxel size for 1 s accumulation"),
    bestz: Path = typer.Option(
        None, "--bestz", help="Best-z trajectory file (default: <mcap>_bestz.txt, via data utils)"
    ),
    rmin: float = typer.Option(0.4, "--rmin"),
    rmax: float = typer.Option(30.0, "--rmax"),
) -> None:
    """Build a world-frame Go2 dataset (lidar/rgb/two odoms) the dimos map CLI reads."""
    from dimos.memory2.store.sqlite import SqliteStore
    from dimos.utils.data import resolve_named_path

    mcap = resolve_named_path(mcap, ".mcap")  # local path, repo data dir, or LFS pull
    out = out or mcap.with_suffix(".db")
    for p in (out, out.with_suffix(".db-wal"), out.with_suffix(".db-shm")):
        p.unlink(missing_ok=True)  # overwrite: avoid appending to an old db
    anchor, lt, lpos, lquat = _load_odom(mcap, seconds)
    print(f"anchor={anchor}  odom poses={len(lt)}  span={lt[-1]:.1f}s")

    # odom_bestz = leg xy/yaw + reconstructed z, z interpolated onto lt. The best-z
    # trajectory lives next to the mcap in the data dir (resolved via data utils).
    bestz = bestz or resolve_named_path(mcap.stem + "_bestz.txt")
    bz_file = np.loadtxt(bestz)
    bz = np.interp(lt, bz_file[:, 0], bz_file[:, 6])
    bpos = np.column_stack([lpos[:, 0], lpos[:, 1], bz])

    def acc_world(bins: dict, t_rel: float, w: np.ndarray) -> None:
        bins.setdefault(int(t_rel), []).append(w)

    store = SqliteStore(path=str(out))
    with store:
        s_img = store.stream("color_image", Image)
        s_od = store.stream("odom", PoseStamped)
        s_odz = store.stream("odom_bestz", PoseStamped)
        s_li = store.stream("lidar", PointCloud2)
        s_liz = store.stream("lidar_bestz", PointCloud2)
        s_li1 = store.stream("lidar_1s", PointCloud2)
        s_liz1 = store.stream("lidar_bestz_1s", PointCloud2)

        # ts is UNIQUE per stream; keep it strictly increasing (raw publish_times
        # can repeat) so appends never collide.
        _last: dict = {}

        def put(stream, payload, ts: float, pose) -> None:
            last = _last.get(stream)
            if last is not None and ts <= last:
                ts = last + 1e-4
            _last[stream] = ts
            stream.append(payload, ts=ts, pose=pose)

        # ---- odom + odom_bestz (downsampled) ----
        step = max(1, round((len(lt) / max(lt[-1], 1e-6)) / max(odom_hz, 1e-6)))
        for i in range(0, len(lt), step):
            ts = anchor / 1e9 + lt[i]
            qx, qy, qz, qw = lquat[i]
            put(
                s_od,
                PoseStamped(
                    ts=ts, frame_id="world", position=lpos[i].tolist(), orientation=[qx, qy, qz, qw]
                ),
                ts,
                (*lpos[i], qx, qy, qz, qw),
            )
            put(
                s_odz,
                PoseStamped(
                    ts=ts, frame_id="world", position=bpos[i].tolist(), orientation=[qx, qy, qz, qw]
                ),
                ts,
                (*bpos[i], qx, qy, qz, qw),
            )
        print(f"wrote {len(range(0, len(lt), step))} odom poses (x2)")

        # ---- lidar (world) per scan, deskewed by both trajectories + 1s bins ----
        bins_l: dict = {}
        bins_z: dict = {}
        nclouds = 0
        with open(mcap, "rb") as f:
            for _s, _c, m in make_reader(f).iter_messages(topics=[CLOUD]):
                tr = (m.publish_time - anchor) / 1e9
                if tr < 0 or (seconds is not None and tr > seconds):
                    continue
                a = cdr.decode_pointcloud2(m.data)["arr"]
                if len(a) == 0:
                    continue
                xyz = np.stack([a["x"], a["y"], a["z"]], 1).astype(np.float64)
                inten = a["intensity"].astype(np.float32)
                rr = np.linalg.norm(xyz, axis=1)
                keep = np.isfinite(xyz).all(1) & (rr > rmin) & (rr < rmax)
                xyz, inten = xyz[keep], inten[keep]
                if len(xyz) < 10:
                    continue
                tpt = tr + a["time"].astype(np.float64)[keep]
                ts = m.publish_time / 1e9
                wl = _deskew(xyz, tpt, lt, lpos, lquat)
                wz = _deskew(xyz, tpt, lt, bpos, lquat)
                put(
                    s_li,
                    PointCloud2.from_numpy(wl.astype(np.float32), "world", ts, inten),
                    ts,
                    _pose7(lt, lpos, lquat, tr),
                )
                put(
                    s_liz,
                    PointCloud2.from_numpy(wz.astype(np.float32), "world", ts, inten),
                    ts,
                    _pose7(lt, bpos, lquat, tr),
                )
                acc_world(bins_l, tr, wl.astype(np.float32))
                acc_world(bins_z, tr, wz.astype(np.float32))
                nclouds += 1
        print(f"wrote {nclouds} lidar scans (x2)")

        # ---- 1 s accumulations (voxel-downsampled), posed at bin center ----
        def flush(bins: dict, pos, stream) -> None:
            for sec, chunks in sorted(bins.items()):
                ts = anchor / 1e9 + sec + 0.5
                pc = PointCloud2.from_numpy(np.concatenate(chunks), "world", ts)
                pc = pc.voxel_downsample(voxel)
                put(stream, pc, ts, _pose7(lt, pos, lquat, sec + 0.5))

        flush(bins_l, lpos, s_li1)
        flush(bins_z, bpos, s_liz1)
        print(f"wrote {len(bins_l)} 1s accumulations (x2)")

        # ---- color_image, posed at camera_optical in world ----
        import cv2

        nimg = 0
        with open(mcap, "rb") as f:
            for _s, _c, m in make_reader(f).iter_messages(topics=[VIDEO]):
                tr = (m.publish_time - anchor) / 1e9
                if tr < 0 or (seconds is not None and tr > seconds):
                    continue
                img = cdr.decode_compressed_image(m.data)
                bgr = cv2.imdecode(np.frombuffer(img["data"], np.uint8), cv2.IMREAD_COLOR)
                if bgr is None:
                    continue
                ts = m.publish_time / 1e9
                bp, bq = _interp(lt, lpos, lquat, np.array([tr]))
                cam_p = bp[0] + _qrot(bq, CAM_T[None])[0]
                cam_q = _qmul(bq[0], CAM_Q)
                put(
                    s_img,
                    Image.from_numpy(bgr, ImageFormat.BGR, "camera_optical", ts),
                    ts,
                    (*cam_p, *cam_q),
                )
                nimg += 1
        print(f"wrote {nimg} color_image frames")

    print(
        f"\nwrote {out}\n  dimos map summary {out}\n  dimos map global {out} --lidar lidar --voxel 0.1"
    )


if __name__ == "__main__":
    typer.run(main)
