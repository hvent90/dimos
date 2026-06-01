#!/usr/bin/env python3
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

from datetime import datetime
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time
from typing import Any

from pydantic import Field
from reactivex.disposable import Disposable

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In
from dimos.hardware.sensors.lidar.fastlio2.module import FastLio2
from dimos.hardware.sensors.lidar.livox.module import Mid360
from dimos.memory2.module import Recorder, RecorderConfig
from dimos.memory2.stream import Stream
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.robot.unitree.go2.connection import GO2Connection
from dimos.robot.unitree.keyboard_teleop import KeyboardTeleop
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# tcpdump fails fast (EPERM, bad iface) within a few ms; pause briefly so poll() catches that.
_TCPDUMP_STARTUP_PROBE_SEC = 0.3


def _stamp() -> str:
    now = datetime.now()
    return now.strftime("%Y-%m-%d") + "_" + now.strftime("%-I-%M%p").lower() + "-PST"


def _default_recording_dir() -> Path:
    return Path(f"recording_go2_mid360_{_stamp()}")


class Go2Mid360MemoryConfig(RecorderConfig):
    """One recording dir per session: <dir>/data.db plus <dir>/mid360.pcap."""

    recording_dir: Path = Field(default_factory=_default_recording_dir)
    # Filled in by model_post_init below if left at the default.
    db_path: str | Path = ""
    pcap_path: str | Path = ""

    default_frame_id: str = "base_link"

    # tcpdump configuration. Capture is filtered to UDP from the lidar IP.
    record_pcap: bool = True
    record_pcap_iface: str = "enp2s0"
    record_pcap_snaplen: int = 2048
    lidar_ip: str = "192.168.1.107"

    def model_post_init(self, __context: object) -> None:
        super().model_post_init(__context)
        # Resolve db/pcap paths from recording_dir if the caller didn't set them.
        if not self.db_path:
            self.db_path = self.recording_dir / "data.db"
        if not self.pcap_path:
            self.pcap_path = self.recording_dir / "mid360.pcap"


class Go2Mid360Memory(Recorder):
    """Records Go2 camera, native Go2 lidar, Mid-360 (lidar + IMU), FastLio2
    odometry, and Go2 leg odometry.

    Also owns the tcpdump process that captures raw UDP packets from the
    Mid-360. Single session = single timestamped dir holding both the
    sqlite memory store and the pcap.
    """

    config: Go2Mid360MemoryConfig

    color_image: In[Image]
    lidar: In[PointCloud2]
    odom: In[PoseStamped]
    fastlio_lidar: In[PointCloud2]
    fastlio_odometry: In[Odometry]
    livox_lidar: In[PointCloud2]
    livox_imu: In[Imu]

    _pcap_proc: subprocess.Popen[bytes] | None = None

    @rpc
    def start(self) -> None:
        Path(self.config.recording_dir).mkdir(parents=True, exist_ok=True)
        if self.config.record_pcap:
            self._start_pcap()
        super().start()

    @rpc
    def stop(self) -> None:
        super().stop()
        self._stop_pcap()

    def _port_to_stream(self, name: str, input_topic: In[Any], stream: Stream[Any]) -> None:
        """Append each message from *input_topic* to *stream*, attaching world pose via tf.

        Stamped messages use their own ``.frame_id`` and ``.ts``; unstamped
        messages (or ones whose frame isn't in the tf graph, e.g. a payload
        already in world coords) fall back to ``config.default_frame_id`` —
        so every observation gets a robot-pose anchor when tf is publishing.

        Registers the subscription as a disposable on this module.
        """

        default_frame_id = self.config.default_frame_id
        tf_tolerance = self.config.tf_tolerance

        def on_msg(msg: Any) -> None:
            # Force system time for all messages
            ts = time.time()
            frame_id = (
                getattr(msg, "child_frame_id", None)
                or getattr(msg, "frame_id", None)
                or default_frame_id
            )
            transform = self.tf.get("world", frame_id, time_point=ts, time_tolerance=tf_tolerance)
            pose = transform.to_pose() if transform is not None else None

            stream.append(msg, ts=ts, pose=pose)

        self.register_disposable(Disposable(input_topic.subscribe(on_msg)))

    def _start_pcap(self) -> None:
        cfg = self.config
        path = Path(cfg.pcap_path).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)

        # Capture every UDP packet originating from the lidar.
        packet_filter_expression = f"src host {cfg.lidar_ip} and udp"
        tcpdump = shutil.which("tcpdump") or "tcpdump"
        cmd = [
            tcpdump,
            "-i",
            cfg.record_pcap_iface,
            "-w",
            str(path),
            "-s",
            str(cfg.record_pcap_snaplen),
            "-U",
            "-n",
            packet_filter_expression,
        ]

        proc = subprocess.Popen(
            cmd,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            start_new_session=True,
        )
        # tcpdump exits within a few ms on EPERM; wait briefly so we can detect that.
        time.sleep(_TCPDUMP_STARTUP_PROBE_SEC)
        if proc.poll() is not None:
            stderr = proc.stderr.read().decode(errors="replace") if proc.stderr else ""
            self._pcap_proc = None
            logger.error(
                f"Go2Mid360Memory pcap recording failed to start — tcpdump exited"
                f" rc={proc.returncode} stderr={stderr.strip()}"
            )
            print(
                "[go2_record] pcap recording is enabled but tcpdump cannot capture.\n"
                "          Grant capture capability once with:\n"
                f"            sudo setcap cap_net_raw,cap_net_admin=eip {tcpdump}\n"
                "          then restart. (tcpdump stderr above.)",
                flush=True,
            )
            return

        logger.info(
            f"Go2Mid360Memory pcap recording enabled  path={path}  "
            f"iface={cfg.record_pcap_iface}  filter={packet_filter_expression!r}"
        )
        self._pcap_proc = proc

    def _stop_pcap(self) -> None:
        proc = self._pcap_proc
        if proc is None:
            return
        self._pcap_proc = None
        if proc.poll() is not None:
            return
        # SIGINT is tcpdump's documented "stop cleanly" signal — it prints
        # packet counts and flushes the pcap header.
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=self.config.shutdown_timeout)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                proc.wait(timeout=self.config.shutdown_timeout)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()
        logger.info(f"Go2Mid360Memory pcap recording stopped  path={self.config.pcap_path}")


MPH_PER_MPS = 2.23694
SPEED_LIMIT_MPH = 30.0
_SPEED_STATUS_PRINT_INTERVAL_SEC = 1.0


class SpeedWarner(Module):
    """Watches fastlio_odometry; once speed ever exceeds the limit (impossible for the Go2,
    so it indicates the FastLio2 estimate has diverged / sensor is about to crash),
    latches and spams an error on every subsequent odom message until restart.

    FastLio2's C++ publisher hardcodes twist to zero (cpp/main.cpp), so msg.vx/vy/vz
    are always 0. Speed is derived from pose deltas instead.
    """

    fastlio_odometry: In[Odometry]

    _tripped: bool = False
    _max_mph: float = 0.0
    _last_pos: tuple[float, float, float] | None = None
    _last_ts: float | None = None
    _last_print_ts: float = 0.0

    async def handle_fastlio_odometry(self, msg: Odometry) -> None:
        ts = msg.ts or time.time()
        pos = (msg.pose.x, msg.pose.y, msg.pose.z)
        last_pos, last_ts = self._last_pos, self._last_ts
        self._last_pos, self._last_ts = pos, ts
        if last_pos is None or last_ts is None:
            return
        dt = ts - last_ts
        if dt <= 0:
            return
        dx, dy, dz = pos[0] - last_pos[0], pos[1] - last_pos[1], pos[2] - last_pos[2]
        speed_mph = math.sqrt(dx * dx + dy * dy + dz * dz) / dt * MPH_PER_MPS
        if speed_mph > self._max_mph:
            self._max_mph = speed_mph
        if ts - self._last_print_ts >= _SPEED_STATUS_PRINT_INTERVAL_SEC:
            self._last_print_ts = ts
            print(
                f"\rspeed: {speed_mph:6.2f} mph  max: {self._max_mph:6.2f} mph ",
                end="",
                flush=True,
            )
        if not self._tripped and speed_mph > SPEED_LIMIT_MPH:
            self._tripped = True
            logger.error(
                f"!!! FASTLIO ODOMETRY DIVERGED !!! reported {speed_mph:.1f} mph "
                f"(limit {SPEED_LIMIT_MPH:.1f} mph). Latching warnings."
            )


_LIDAR_IP = os.getenv("LIDAR_IP", "192.168.1.107")


unitree_go2_record = autoconnect(
    GO2Connection.blueprint(),
    KeyboardTeleop.blueprint(),
    MovementManager.blueprint(),
    Mid360.blueprint(
        lidar_ip=_LIDAR_IP,
    ).remappings(
        [
            (Mid360, "lidar", "livox_lidar"),
            (Mid360, "imu", "livox_imu"),
        ]
    ),
    FastLio2.blueprint(
        frame_id="world",
        map_freq=-1,
        lidar_ip=_LIDAR_IP,
    ).remappings(
        [
            (FastLio2, "lidar", "fastlio_lidar"),
            (FastLio2, "odometry", "fastlio_odometry"),
        ]
    ),
    Go2Mid360Memory.blueprint(lidar_ip=_LIDAR_IP),
    SpeedWarner.blueprint(),
).global_config(n_workers=10, robot_model="unitree_go2")
