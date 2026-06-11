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

"""Replay the ruwik2_pt3 5/29 pcap through the Point-LIO native module.

Mirror of the fastlio2 replay_ruwik2_pt3 harness, but driving the PointLio
module. Same orchestrator+worker shape, same pcap, same deterministic clock.
Captures PointLio odometry into a per-attempt sqlite db so the trajectory can
be compared against the fastlio replay (which diverges to ~2257 m/s on this
same wire data).

Run from the dimos6 venv:

    cd ~/repos/dimos6
    source .venv/bin/activate
    python -m dimos.hardware.sensors.lidar.pointlio.tools.replay_ruwik2_pt3
"""

from __future__ import annotations

from collections.abc import AsyncIterator
import json
import os
from pathlib import Path
import subprocess
import sys
import time

from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.msgs.nav_msgs.Odometry import Odometry

PCAP_PATH = Path(
    os.environ.get(
        "REPLAY_PCAP_PATH",
        "/home/dimos/repos/dimos/fastlio2_pcap/mid360_20260529_225346.pcap",
    )
)

RUNS_ROOT = Path("/home/dimos/repos/dimos6/pointlio_ruwik2_pt3_replays")

_ATTEMPT_DIR_ENV = "_REPLAY_POINTLIO_RUWIK2_PT3_ATTEMPT_DIR"

MAX_WALL_SEC = float(os.environ.get("REPLAY_MAX_WALL_SEC", "480.0"))

REPLAY_MAX_SENSOR_SEC = float(os.environ.get("REPLAY_MAX_SENSOR_SEC", "60.0"))

# deterministic_clock=true cuts scan boundaries on the sensor virtual clock read
# atomically under g_pc_mutex, which removes the feeder/main scan-composition race
# and makes replay reproducibly bounded. Set REPLAY_DETERMINISTIC_CLOCK=0 to cut
# scans on wall time (realtime feeder) — that restores the live threading race and
# is the only way replay reproduces the live divergence offline.
REPLAY_DETERMINISTIC_CLOCK = os.environ.get("REPLAY_DETERMINISTIC_CLOCK", "1") != "0"

# Feed point + IMU on two separate threads (mimics the live Livox SDK's
# concurrent delivery). Set REPLAY_DUAL_THREAD=1 with REPLAY_DETERMINISTIC_CLOCK=0
# to reproduce live thread-interleaving offline.
REPLAY_DUAL_THREAD = os.environ.get("REPLAY_DUAL_THREAD", "0") == "1"


def _next_attempt_dir() -> Path:
    RUNS_ROOT.mkdir(parents=True, exist_ok=True)
    existing = sorted(p.name for p in RUNS_ROOT.iterdir() if p.name.startswith("attempt_"))
    n = 0
    for name in existing:
        try:
            n = max(n, int(name.split("_", 1)[1]) + 1)
        except ValueError:
            continue
    attempt = RUNS_ROOT / f"attempt_{n:03d}"
    attempt.mkdir()
    return attempt


def _commit_hash() -> str:
    try:
        return subprocess.check_output(
            ["git", "-C", str(Path(__file__).resolve().parents[6]), "rev-parse", "HEAD"],
            text=True,
        ).strip()
    except subprocess.CalledProcessError:
        return "unknown"


class RecConfig(ModuleConfig):
    """Configures Rec with the per-attempt sqlite db path."""

    db_path: str = ""


_EPS = 1e-9


class Rec(Module):
    """Mirror replay PointLio odometry into a SqliteStore."""

    config: RecConfig
    pointlio_odometry: In[Odometry]
    _last_o: float = 0.0

    async def main(self) -> AsyncIterator[None]:
        from dimos.memory2.store.sqlite import SqliteStore

        self._store = SqliteStore(path=self.config.db_path)
        self._os = self._store.stream("pointlio_odometry", Odometry)
        yield
        self._store.stop()

    async def handle_pointlio_odometry(self, v: Odometry) -> None:
        ts = max(getattr(v, "ts", None) or time.time(), self._last_o + _EPS)
        self._last_o = ts
        pose = getattr(v, "pose", None)
        pose_inner = getattr(pose, "pose", None) if pose is not None else None
        self._os.append(v, ts=ts, pose=pose_inner)


def _orchestrate() -> int:
    if not PCAP_PATH.exists():
        print(f"[replay_pointlio] missing pcap: {PCAP_PATH}", file=sys.stderr)
        return 2
    attempt_dir = _next_attempt_dir()
    stdout_path = attempt_dir / "stdout.txt"
    stderr_path = attempt_dir / "stderr.txt"
    meta = {
        "attempt_dir": str(attempt_dir),
        "pcap_path": str(PCAP_PATH),
        "commit": _commit_hash(),
        "started_at": time.time(),
    }
    print(f"[replay_pointlio] attempt {attempt_dir.name}  commit {meta['commit'][:8]}", flush=True)
    t0 = time.time()
    env = {**os.environ, _ATTEMPT_DIR_ENV: str(attempt_dir)}
    with stdout_path.open("w") as out, stderr_path.open("w") as err:
        rc = subprocess.run(
            [sys.executable, "-m", __spec__.name, "--worker"],
            stdout=out,
            stderr=err,
            env=env,
            check=False,
        ).returncode
    meta["finished_at"] = time.time()
    meta["wall_sec"] = meta["finished_at"] - t0
    meta["worker_rc"] = rc
    (attempt_dir / "meta.json").write_text(json.dumps(meta, indent=2))
    print(
        f"[replay_pointlio] done attempt {attempt_dir.name} rc={rc} wall={meta['wall_sec']:.1f}s",
        flush=True,
    )
    return rc


def _worker() -> int:
    attempt_dir = Path(os.environ[_ATTEMPT_DIR_ENV])
    db_path = attempt_dir / "pointlio.db"
    if db_path.exists():
        db_path.unlink()
    db_path_str = str(db_path)

    from dimos.core.coordination.blueprints import autoconnect
    from dimos.core.coordination.module_coordinator import ModuleCoordinator
    from dimos.hardware.sensors.lidar.pointlio.module import PointLio

    subprocess.run(["kd"], check=False)
    time.sleep(1.0)

    bp = autoconnect(
        PointLio.blueprint(
            frame_id="world",
            map_freq=-1,
            replay_pcap=PCAP_PATH,
            deterministic_clock=REPLAY_DETERMINISTIC_CLOCK,
            replay_dual_thread=REPLAY_DUAL_THREAD,
            debug=False,
        ).remappings(
            [
                (PointLio, "odometry", "pointlio_odometry"),
            ]
        ),
        Rec.blueprint(db_path=db_path_str),
    ).global_config(n_workers=4, robot_model="mid360_pointlio_replay_ruwik2")
    coord = ModuleCoordinator.build(bp)

    import sqlite3

    t0 = time.time()
    last_ts_seen = 0.0
    first_ts_seen = 0.0
    stagnant_since: float | None = None
    saw_first_row = False
    try:
        while time.time() - t0 < MAX_WALL_SEC:
            time.sleep(1.0)
            if not db_path.exists():
                continue
            try:
                con = sqlite3.connect(f"file:{db_path_str}?mode=ro", uri=True, timeout=0.5)
                row = con.execute(
                    "SELECT MIN(ts), MAX(ts), COUNT(*) FROM pointlio_odometry"
                ).fetchone()
                con.close()
            except Exception:
                continue
            first_ts = row[0] if row and row[0] else 0.0
            last_ts = row[1] if row and row[1] else 0.0
            cnt = row[2] if row else 0
            if cnt > 0:
                saw_first_row = True
                if first_ts_seen == 0.0:
                    first_ts_seen = first_ts
            if not saw_first_row:
                continue
            if (
                REPLAY_MAX_SENSOR_SEC > 0
                and first_ts_seen > 0
                and (last_ts - first_ts_seen) >= REPLAY_MAX_SENSOR_SEC
            ):
                print(
                    f"[replay_pointlio.worker] reached REPLAY_MAX_SENSOR_SEC="
                    f"{REPLAY_MAX_SENSOR_SEC:.1f} s — stopping",
                    flush=True,
                )
                break
            if last_ts == last_ts_seen:
                if stagnant_since is None:
                    stagnant_since = time.time()
                elif time.time() - stagnant_since > 3.0:
                    break
            else:
                last_ts_seen = last_ts
                stagnant_since = None
    finally:
        coord.stop()

    if db_path.exists():
        size_mb = db_path.stat().st_size / 1e6
        print(
            f"[replay_pointlio.worker] db_size={size_mb:.2f}MB wall={time.time() - t0:.1f}s",
            flush=True,
        )
    return 0


def main(argv: list[str]) -> int:
    if len(argv) >= 2 and argv[1] == "--worker":
        return _worker()
    return _orchestrate()


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
