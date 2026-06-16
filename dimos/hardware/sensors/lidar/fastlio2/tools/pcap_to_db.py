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

"""Replay a Livox Mid-360 pcap through FAST-LIO and record its output to a .db.

``virtual_mid360`` replays the pcap over the wire (a fake Mid-360 on a virtual
NIC) and FastLio2 connects in live SDK mode, exactly as to real hardware:

* ``pcap_to_db --pcap capture.pcap``               -> writes ``capture.db``
* ``pcap_to_db --pcap capture.pcap --db mem2.db``  -> appends into an existing db

Records ``fastlio_odometry`` + ``fastlio_lidar``, time-aligned onto the db clock
(``--time-offset`` overrides; ``--force`` overwrites pre-existing fastlio streams).
Stands up two netns joined by a veth, so it needs root — set ``$SUDO`` to a
passwordless escalation (default ``sudo``); netns/veth names override via
``$DRV_NS``/``$LIDAR_NS``/``$VETH_*``.

Build the virtual_mid360 binary once::

    (cd dimos/hardware/sensors/lidar/livox/virtual_mid360 && nix build .#default)
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sqlite3
import subprocess
import sys
import tempfile
import time

from dimos.hardware.sensors.lidar.fastlio2.recorder import FastLio2Recorder

# Poll cadence while recording.
_POLL_SEC = 1.0
# Let FastLio2 drain its scan backlog after the pcap finishes before stopping.
_DRAIN_SEC = 10.0

# lidar ns runs virtual_mid360; drv ns runs the FastLio2 consumer. Defaults are
# distinct from pointlio's harness so both can run concurrently.
_SUDO = os.environ.get("SUDO", "sudo")
_DRV_NS = os.environ.get("DRV_NS", "fl_drv")
_LIDAR_NS = os.environ.get("LIDAR_NS", "fl_lidar")
_VETH_DRV = os.environ.get("VETH_DRV", "veth-fl-drv")
_VETH_LIDAR = os.environ.get("VETH_LIDAR", "veth-fl-lidar")
_HOST_IP = "192.168.1.5"
_LIDAR_IP = "192.168.1.155"
_REPO = Path(__file__).resolve().parents[6]
_VM_BIN = _REPO / "dimos/hardware/sensors/lidar/livox/virtual_mid360/result/bin/virtual_mid360"


def _db_ref_start_ts(db_path: Path) -> float:
    """Min ts across the db's existing streams, or -1.0 if none/absent."""
    if not db_path.exists():
        return -1.0
    con = sqlite3.connect(f"file:{db_path}?mode=ro", uri=True, timeout=2.0)
    try:
        tables = [
            row[0]
            for row in con.execute("SELECT name FROM sqlite_master WHERE type='table'").fetchall()
        ]
        best: float | None = None
        for table in tables:
            if table.startswith("_") or table.startswith("sqlite_"):
                continue
            try:
                # vec0/rtree virtual tables raise "no such module" when the
                # extension isn't loaded -- skip them.
                cols = [c[1] for c in con.execute(f"PRAGMA table_info('{table}')").fetchall()]
                if "ts" not in cols:
                    continue
                row = con.execute(f"SELECT MIN(ts) FROM '{table}'").fetchone()
            except sqlite3.OperationalError:
                continue
            if row and row[0] is not None:
                best = row[0] if best is None else min(best, row[0])
        return best if best is not None else -1.0
    finally:
        con.close()


def _table_stats(db_path: Path, table: str) -> tuple[int, float, float]:
    """(count, min_ts, max_ts) for a stream table; zeros if absent."""
    if not db_path.exists():
        return 0, 0.0, 0.0
    con = sqlite3.connect(f"file:{db_path}?mode=ro", uri=True, timeout=2.0)
    try:
        try:
            row = con.execute(f"SELECT COUNT(*), MIN(ts), MAX(ts) FROM '{table}'").fetchone()
        except sqlite3.OperationalError:
            return 0, 0.0, 0.0
        cnt = row[0] or 0
        return cnt, row[1] or 0.0, row[2] or 0.0
    finally:
        con.close()


# Orchestrator: set up the netns + fake sensor, drive the consumer, tear down.


def _sudo(*args: str, check: bool = True) -> subprocess.CompletedProcess[bytes]:
    return subprocess.run([_SUDO, *args], check=check)


def _teardown_netns() -> None:
    _sudo("pkill", "-9", "-f", "result/bin/virtual_mid360", check=False)
    _sudo("ip", "netns", "del", _DRV_NS, check=False)
    _sudo("ip", "netns", "del", _LIDAR_NS, check=False)
    _sudo("ip", "link", "del", _VETH_DRV, check=False)


def _setup_netns() -> None:
    _teardown_netns()
    _sudo("ip", "netns", "add", _DRV_NS)
    _sudo("ip", "netns", "add", _LIDAR_NS)
    _sudo("ip", "link", "add", _VETH_DRV, "type", "veth", "peer", "name", _VETH_LIDAR)
    _sudo("ip", "link", "set", _VETH_DRV, "netns", _DRV_NS)
    _sudo("ip", "link", "set", _VETH_LIDAR, "netns", _LIDAR_NS)
    _sudo("ip", "netns", "exec", _DRV_NS, "ip", "addr", "add", f"{_HOST_IP}/24", "dev", _VETH_DRV)
    _sudo(
        "ip", "netns", "exec", _LIDAR_NS, "ip", "addr", "add", f"{_LIDAR_IP}/24", "dev", _VETH_LIDAR
    )
    for ns in (_DRV_NS, _LIDAR_NS):
        _sudo("ip", "netns", "exec", ns, "ip", "link", "set", "lo", "up")
        _sudo("ip", "netns", "exec", ns, "ip", "link", "set", "lo", "multicast", "on")
        _sudo("ip", "netns", "exec", ns, "ip", "route", "add", "224.0.0.0/4", "dev", "lo")
    _sudo("ip", "netns", "exec", _DRV_NS, "ip", "link", "set", _VETH_DRV, "up")
    _sudo("ip", "netns", "exec", _LIDAR_NS, "ip", "link", "set", _VETH_LIDAR, "up")
    _sudo("ip", "netns", "exec", _DRV_NS, "ip", "link", "set", _VETH_DRV, "multicast", "on")
    _sudo("ip", "netns", "exec", _LIDAR_NS, "ip", "link", "set", _VETH_LIDAR, "multicast", "on")
    _sudo(
        "ip",
        "netns",
        "exec",
        _LIDAR_NS,
        "ip",
        "route",
        "add",
        "255.255.255.255/32",
        "dev",
        _VETH_LIDAR,
    )
    # Mid-360 multicasts point/IMU to 224.1.1.5 — egress the virtual NIC.
    _sudo(
        "ip", "netns", "exec", _LIDAR_NS, "ip", "route", "add", "224.1.1.5/32", "dev", _VETH_LIDAR
    )


def _orchestrate(args: argparse.Namespace) -> int:
    pcap = Path(args.pcap).expanduser().resolve()
    if not pcap.exists():
        print(f"[pcap_to_db] missing pcap: {pcap}", file=sys.stderr)
        return 2
    if not _VM_BIN.exists():
        print(
            f"[pcap_to_db] missing virtual_mid360 binary at {_VM_BIN}\n"
            f"  build it: (cd {_VM_BIN.parents[1]} && nix build .#default)",
            file=sys.stderr,
        )
        return 2
    db = Path(args.db).expanduser().resolve() if args.db else pcap.with_suffix(".db")
    db.parent.mkdir(parents=True, exist_ok=True)
    print(
        f"[pcap_to_db] {pcap.name} -> {db.name} "
        f"({'append' if db.exists() else 'new'}) via virtual_mid360 (live SDK)",
        flush=True,
    )

    consumer: subprocess.Popen[bytes] | None = None
    tmp = Path(tempfile.gettempdir())
    stopfile = tmp / f"pcap_to_db_stop.{os.getpid()}"
    vmlog = tmp / f"pcap_to_db_vm.{os.getpid()}.log"
    stopfile.unlink(missing_ok=True)
    _setup_netns()
    try:
        # FastLio2 consumer in the drv netns (re-exec self as the recorder).
        cmd = [
            _SUDO,
            "ip",
            "netns",
            "exec",
            _DRV_NS,
            "env",
            f"PYTHONPATH={_REPO}",
            sys.executable,
            "-m",
            "dimos.hardware.sensors.lidar.fastlio2.tools.pcap_to_db",
            "--_consume",
            "--_stopfile",
            str(stopfile),
            "--db",
            str(db),
            "--duration",
            str(args.duration),
            "--odom-freq",
            str(args.odom_freq),
        ]
        if args.config:
            cmd += ["--config", args.config]
        if args.force:
            cmd += ["--force"]
        if args.time_offset is not None:
            cmd += ["--time-offset", str(args.time_offset)]
        # SQLite won't let root (the in-netns recorder) write a user-owned db, so
        # take ownership for the run; the chown back at the end restores it.
        for suffix in ("", "-wal", "-shm"):
            q = Path(str(db) + suffix)
            if q.exists():
                _sudo("chown", "0:0", str(q), check=False)
        consumer = subprocess.Popen(cmd)
        time.sleep(5)  # let the coordinator boot + open the SDK sockets

        # Fake lidar in the lidar netns, replaying the pcap over the wire.
        vm_cfg = json.dumps(
            {
                "topics": {},
                "config": {
                    "pcap": str(pcap),
                    "rate": 1.0,
                    "delay": 2.0,
                    "lidar_ip": _LIDAR_IP,
                    "host_ip": _HOST_IP,
                    "lidar_netns": _LIDAR_NS,
                },
            }
        )
        with open(vmlog, "wb") as vmerr:
            vm = subprocess.Popen(
                [_SUDO, "ip", "netns", "exec", _LIDAR_NS, str(_VM_BIN)],
                stdin=subprocess.PIPE,
                stderr=vmerr,
            )
            assert vm.stdin is not None
            vm.stdin.write((vm_cfg + "\n").encode())
            vm.stdin.close()

            # virtual_mid360 streams the pcap once, then logs "data stream
            # finished"; wait for that (capped by --duration), drain, then stop.
            # (Watching the db for stagnation is unreliable — a diverging FastLio2
            # keeps emitting long after the sensor goes quiet.)
            deadline = time.time() + args.duration
            while time.time() < deadline:
                if vm.poll() is not None:
                    break
                try:
                    if b"data stream finished" in vmlog.read_bytes():
                        break
                except OSError:
                    pass
                time.sleep(1.0)
        time.sleep(_DRAIN_SEC)
        stopfile.touch()
        try:
            consumer.wait(timeout=60)
        except subprocess.TimeoutExpired:
            consumer.terminate()
        rc = consumer.returncode or 0
    finally:
        if consumer is not None and consumer.poll() is None:
            consumer.terminate()
            try:
                consumer.wait(timeout=10)
            except subprocess.TimeoutExpired:
                consumer.kill()
        _teardown_netns()
        stopfile.unlink(missing_ok=True)
        vmlog.unlink(missing_ok=True)

    # The consumer ran as root inside the netns, so the db is root-owned —
    # hand it back to the invoking user.
    for suffix in ("", "-wal", "-shm"):
        p = Path(str(db) + suffix)
        if p.exists():
            _sudo("chown", f"{os.getuid()}:{os.getgid()}", str(p), check=False)
    return rc


# Consumer: FastLio2 live SDK + recorder. Runs inside the drv netns.


def _consume(args: argparse.Namespace) -> int:
    db_path = Path(args.db).expanduser().resolve()
    db_path.parent.mkdir(parents=True, exist_ok=True)

    from dimos.core.coordination.blueprints import autoconnect
    from dimos.core.coordination.module_coordinator import ModuleCoordinator
    from dimos.hardware.sensors.lidar.fastlio2.module import FastLio2
    from dimos.memory2.store.sqlite import SqliteStore

    fastlio_streams = ("fastlio_odometry", "fastlio_lidar")
    store = SqliteStore(path=str(db_path))
    try:
        existing = sorted(set(store.list_streams()) & set(fastlio_streams))
        if existing and not args.force:
            print(
                f"[pcap_to_db] {db_path.name} already has {existing}; pass --force to overwrite",
                file=sys.stderr,
            )
            return 2
        for name in existing:
            store.delete_stream(name)
        if existing:
            print(f"[pcap_to_db] --force: dropped existing {existing}", flush=True)
    finally:
        store.stop()

    ref_start_ts = _db_ref_start_ts(db_path)
    time_offset = float("nan") if args.time_offset is None else args.time_offset

    fastlio_kwargs: dict[str, object] = dict(
        frame_id="world", odom_freq=args.odom_freq, debug=False
    )
    if args.config:
        fastlio_kwargs["config"] = Path(args.config)
    fastlio = FastLio2.blueprint(**fastlio_kwargs).remappings(
        [
            (FastLio2, "odometry", "fastlio_odometry"),
            (FastLio2, "lidar", "fastlio_lidar"),
        ]
    )
    blueprint = autoconnect(
        fastlio,
        FastLio2Recorder.blueprint(
            db_path=str(db_path), ref_start_ts=ref_start_ts, time_offset=time_offset
        ),
    ).global_config(n_workers=4, robot_model="mid360_fastlio2_pcap_to_db")
    coord = ModuleCoordinator.build(blueprint)

    # The orchestrator signals stop via the stop-file; --duration is a safety cap.
    stopfile = Path(args._stopfile) if args._stopfile else None
    t0 = time.time()
    try:
        while time.time() - t0 < args.duration:
            time.sleep(_POLL_SEC)
            if stopfile is not None and stopfile.exists():
                print("[pcap_to_db] stop signalled", flush=True)
                break
        else:
            print(f"[pcap_to_db] reached --duration cap {args.duration:.0f}s", flush=True)
    finally:
        coord.stop()

    o_cnt, o_min, o_max = _table_stats(db_path, "fastlio_odometry")
    l_cnt = _table_stats(db_path, "fastlio_lidar")[0]
    print(
        f"[pcap_to_db] done odom={o_cnt} lidar={l_cnt} "
        f"ts=[{o_min:.3f}, {o_max:.3f}] span={o_max - o_min:.1f}s "
        f"wall={time.time() - t0:.1f}s",
        flush=True,
    )
    return 0


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pcap", default=None, help="Livox Mid-360 pcap capture to replay")
    parser.add_argument(
        "--db",
        default=None,
        help="target memory2 SQLite db; defaults to <pcap>.db. Existing fastlio "
        "streams are time-aligned onto its clock (use --force to overwrite them).",
    )
    parser.add_argument(
        "--config",
        type=str,
        default=None,
        help="FAST-LIO yaml (relative to config/ or absolute); omit for the module default",
    )
    parser.add_argument(
        "--odom-freq",
        type=float,
        default=30.0,
        help="FAST-LIO odometry publish rate in Hz (default 30)",
    )
    parser.add_argument(
        "--time-offset",
        type=float,
        default=None,
        help="seconds added to every output ts; omit to auto-derive from the db clock",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=3600.0,
        help="safety cap in seconds; replay normally stops when the pcap is drained",
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="overwrite existing fastlio_odometry/fastlio_lidar streams in the db",
    )
    # Internal: the in-netns recorder half, spawned by the orchestrator.
    parser.add_argument("--_consume", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--_stopfile", default=None, help=argparse.SUPPRESS)
    args = parser.parse_args(argv)

    if args._consume:
        if not args.db:
            print("[pcap_to_db] --_consume requires --db", file=sys.stderr)
            return 2
        return _consume(args)
    if not args.pcap:
        print("[pcap_to_db] --pcap is required", file=sys.stderr)
        return 2
    return _orchestrate(args)


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
