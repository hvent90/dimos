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

"""Convert FAST-LIO JSONL recording to memory2 sqlite store."""

import json
import sys
import time

from dimos.memory2.store.sqlite import SqliteStore


def main() -> None:
    jsonl_path = (
        sys.argv[1]
        if len(sys.argv) > 1
        else "/Volumes/USB/fastlio_recordings/mid360_20260528_190850.jsonl"
    )
    db_path = sys.argv[2] if len(sys.argv) > 2 else jsonl_path.rsplit(".", 1)[0] + ".db"

    print(f"Source: {jsonl_path}")
    print(f"Output: {db_path}")

    store = SqliteStore(path=db_path)

    accel_x = store.stream("accel_x", float)
    accel_y = store.stream("accel_y", float)
    accel_z = store.stream("accel_z", float)
    gyro_x = store.stream("gyro_x", float)
    gyro_y = store.stream("gyro_y", float)
    gyro_z = store.stream("gyro_z", float)

    imu_count = 0
    skipped = 0
    t0 = time.time()

    with open(jsonl_path) as f:
        for line in f:
            obj = json.loads(line)
            if obj["type"] != "imu":
                skipped += 1
                if skipped % 500000 == 0:
                    print(f"  skipped {skipped} non-IMU lines...")
                continue

            ts = obj["sensor_ts_ns"] / 1e9
            ax, ay, az = obj["accel"]
            gx, gy, gz = obj["gyro"]

            accel_x.append(ax, ts=ts)
            accel_y.append(ay, ts=ts)
            accel_z.append(az, ts=ts)
            gyro_x.append(gx, ts=ts)
            gyro_y.append(gy, ts=ts)
            gyro_z.append(gz, ts=ts)

            imu_count += 1
            if imu_count % 10000 == 0:
                elapsed = time.time() - t0
                rate = imu_count / elapsed
                print(f"  {imu_count} IMU samples ({elapsed:.1f}s, {rate:.0f}/s)")

    elapsed = time.time() - t0
    print(f"Done: {imu_count} IMU samples, {skipped} lidar skipped → {db_path} ({elapsed:.1f}s)")
    store.stop()


if __name__ == "__main__":
    main()
