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

"""Add IMU data (with epoch timestamps) and ZUPT speed estimate to a memory2 DB."""

import json
import sys
import time

import numpy as np

from dimos.memory2.store.sqlite import SqliteStore


def compute_zupt_speed(
    ts: np.ndarray,
    accel: np.ndarray,
    gyro: np.ndarray,
) -> np.ndarray:
    """LP gravity tracking + aggressive velocity decay + ZUPT."""
    G = 9.81
    n = len(ts)

    gravity = accel[:200].mean(axis=0).copy()
    lp_alpha = 0.002
    vel_decay = 0.99

    zupt_accel_tol = 0.08
    zupt_gyro_tol = 0.05
    zupt_window = 40

    velocity = np.zeros(3)
    speed_vals = np.zeros(n)
    zupt_count = 0
    stationary_counter = 0

    for i in range(1, n):
        dt = ts[i] - ts[i - 1]
        if dt <= 0 or dt > 0.1:
            speed_vals[i] = speed_vals[i - 1]
            continue

        gravity = lp_alpha * accel[i] + (1 - lp_alpha) * gravity
        dynamic = (accel[i] - gravity) * G
        velocity = velocity * vel_decay + dynamic * dt

        accel_mag = np.linalg.norm(accel[i])
        gyro_mag = np.linalg.norm(gyro[i])
        if abs(accel_mag - 1.0) < zupt_accel_tol and gyro_mag < zupt_gyro_tol:
            stationary_counter += 1
        else:
            stationary_counter = 0
        if stationary_counter >= zupt_window:
            velocity[:] = 0.0
            zupt_count += 1

        speed_vals[i] = np.linalg.norm(velocity)

        if i % 100000 == 0:
            print(f"  {i}/{n} speed={speed_vals[i]:.2f} m/s, zupts={zupt_count}")

    print(f"  Speed: mean={speed_vals.mean():.3f}, max={speed_vals.max():.3f} m/s")
    print(f"  ZUPT resets: {zupt_count}")
    return speed_vals


def main() -> None:
    jsonl_path = (
        sys.argv[1]
        if len(sys.argv) > 1
        else ("/Volumes/USB/fastlio_recordings/mid360_20260528_190850.jsonl")
    )
    db_path = (
        sys.argv[2]
        if len(sys.argv) > 2
        else (
            "/Volumes/USB/fastlio_recordings/recording_go2_mid360_2026-05-28_7-37pm-PST_with_imu.db"
        )
    )

    print(f"JSONL: {jsonl_path}")
    print(f"DB:    {db_path}")

    print("Reading IMU from JSONL (epoch timestamps)...")
    t0 = time.time()
    imu_ts_list: list[float] = []
    imu_accel_list: list[list[float]] = []
    imu_gyro_list: list[list[float]] = []

    with open(jsonl_path) as f:
        for line in f:
            obj = json.loads(line)
            if obj["type"] != "imu":
                continue
            imu_ts_list.append(obj["pcap_ts_ns"] / 1e9)
            imu_accel_list.append(obj["accel"])
            imu_gyro_list.append(obj["gyro"])

    ts = np.array(imu_ts_list)
    accel = np.array(imu_accel_list)
    gyro_arr = np.array(imu_gyro_list)
    n = len(ts)
    print(f"Loaded {n} IMU samples in {time.time() - t0:.1f}s")

    print("Computing ZUPT speed...")
    speed_vals = compute_zupt_speed(ts, accel, gyro_arr)

    print("Storing streams in DB...")
    store = SqliteStore(path=db_path)

    streams = {
        "imu_accel_x": accel[:, 0],
        "imu_accel_y": accel[:, 1],
        "imu_accel_z": accel[:, 2],
        "imu_gyro_x": gyro_arr[:, 0],
        "imu_gyro_y": gyro_arr[:, 1],
        "imu_gyro_z": gyro_arr[:, 2],
        "imu_speed": speed_vals,
    }

    for name, values in streams.items():
        print(f"  Storing {name}...")
        stream = store.stream(name, float)
        for i in range(n):
            stream.append(float(values[i]), ts=float(ts[i]))
            if i % 100000 == 0 and i > 0:
                print(f"    {i}/{n}")

    elapsed = time.time() - t0
    print(f"Done in {elapsed:.1f}s")
    store.stop()


if __name__ == "__main__":
    main()
