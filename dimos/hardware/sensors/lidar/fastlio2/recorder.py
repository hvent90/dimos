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
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time
from typing import Any

from pydantic import Field
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.stream import In
from dimos.memory2.module import Recorder, RecorderConfig
from dimos.memory2.stream import Stream
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def _stamp() -> str:
    now = datetime.now()
    return now.strftime("%Y-%m-%d") + "_" + now.strftime("%-I-%M%p").lower() + "-PST"


def _default_recording_dir() -> Path:
    return Path("go2_recordings") / _stamp()


class FastLio2RecorderConfig(RecorderConfig):
    """One recording dir per session: <dir>/mem2.db plus <dir>/raw_mid360.pcap."""

    recording_dir: Path = Field(default_factory=_default_recording_dir)
    # Filled in by model_post_init below if left at the default.
    db_path: str | Path = ""

    default_frame_id: str = "base_link"

    # tcpdump configuration. Pcap recording is opt-in: set record_pcap=True to
    # enable. pcap_path defaults to <recording_dir>/mid360.pcap when unset.
    record_pcap: bool = False
    pcap_path: Path | None = None
    record_pcap_iface: str = "enp2s0"
    record_pcap_snaplen: int = 2048
    lidar_ip: str = "192.168.1.107"

    def model_post_init(self, __context: object) -> None:
        super().model_post_init(__context)
        if not self.db_path:
            self.db_path = self.recording_dir / "mem2.db"
        if self.record_pcap and self.pcap_path is None:
            self.pcap_path = self.recording_dir / "raw_mid360.pcap"


class FastLio2Recorder(Recorder):
    """Records FastLio2 inputs and outputs for offline replay: raw Livox
    Mid-360 lidar + IMU into the SDK, FastLio2's registered lidar and
    odometry out, plus any companion streams (e.g. Go2 camera/leg odom)
    that help interpret the run.

    Also owns the tcpdump process that captures the raw Mid-360 UDP
    packets — this is the ground-truth input the FastLio2 binary can be
    replayed against bit-for-bit. Single session = single timestamped dir
    holding both the sqlite memory store and the pcap.
    """

    config: FastLio2RecorderConfig

    livox_lidar: In[PointCloud2]
    livox_imu: In[Imu]
    fastlio_lidar: In[PointCloud2]
    fastlio_odometry: In[Odometry]
    color_image: In[Image]
    lidar: In[PointCloud2]
    odom: In[PoseStamped]

    # tcpdump fails fast (EPERM, bad iface) within a few ms; pause briefly so poll() catches that.
    _TCPDUMP_STARTUP_PROBE_SEC: float = 0.3

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
        time.sleep(self._TCPDUMP_STARTUP_PROBE_SEC)
        if proc.poll() is not None:
            stderr = proc.stderr.read().decode(errors="replace") if proc.stderr else ""
            self._pcap_proc = None
            logger.error(
                f"FastLio2Recorder pcap recording failed to start — tcpdump exited"
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
            f"FastLio2Recorder pcap recording enabled  path={path}  "
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
        logger.info(f"FastLio2Recorder pcap recording stopped  path={self.config.pcap_path}")
