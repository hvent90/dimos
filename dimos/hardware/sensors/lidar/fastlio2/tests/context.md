# FAST-LIO IMU Analysis Context

## What was done

Converted a Mid-360 PCAP-derived JSONL recording into memory2 sqlite stores and generated speed comparison plots to debug a FAST-LIO velocity spike issue.

## Recording

- Source JSONL: `/Volumes/USB/fastlio_recordings/mid360_20260528_190850.jsonl`
  - 3.58M lidar messages + 344K IMU messages (~28.7 min, 200Hz IMU)
  - Each line is `{"type": "lidar"|"imu", "pcap_ts_ns": ..., "sensor_ts_ns": ..., ...}`
  - IMU: `"accel": [x,y,z]` in g, `"gyro": [x,y,z]` in rad/s
  - Lidar: `"points": [[x,y,z,reflectivity,tag], ...]`, 96 points per packet

## Timestamp alignment

- `sensor_ts_ns`: relative to sensor boot (~1583s start). Used in `mid360_20260528_190850.db`.
- `pcap_ts_ns`: epoch time (~1780020531s start). Aligns with Go2 DB timestamps.
- Offset: `epoch = sensor_ts + 1780018948.01`
- The Go2 DB (`recording_go2_mid360_2026-05-28_7-37pm-PST.db`) uses epoch timestamps.

## Generated DBs

| File | Description |
|------|-------------|
| `mid360_20260528_190850.db` | IMU-only, sensor-boot timestamps. Streams: accel_x/y/z, gyro_x/y/z |
| `recording_go2_mid360_2026-05-28_7-37pm-PST_with_imu.db` | Copy of Go2 DB + 7 IMU streams (epoch timestamps) |
| `spike_window.db` | ±10s around first fastlio velocity >10 m/s event (87.7 MB) |

## IMU speed estimation

Three approaches were tried for estimating speed from raw IMU:

1. **Gyro AHRS + ZUPT** (v1): Rotation-based orientation tracking with zero-velocity updates. Failed — rotation composition numerically unstable over 344K iterations, drifted to thousands m/s.

2. **Complementary filter + ZUPT + mild decay** (v2): Added accel-based orientation correction and 0.995/sample velocity decay. Still drifted to 19 m/s mean.

3. **LP gravity tracking + aggressive decay + ZUPT** (v3, final): No rotation math. Low-pass filter (alpha=0.002, ~2.5s time constant) tracks gravity directly in sensor frame. Velocity decay 0.99/sample (retains 13%/s). ZUPT when |accel| within 0.08g of 1g AND |gyro| < 0.05 rad/s for 40 consecutive samples. Results: mean=0.18 m/s, max=1.83 m/s.

The ZUPT and non-ZUPT versions were nearly identical because the aggressive decay already dominates.

## Scripts

- `jsonl_to_memory2.py` — Converts JSONL to standalone memory2 DB (sensor-boot timestamps, IMU only)
- `add_imu_to_db.py` — Adds IMU streams + ZUPT speed estimate to an existing memory2 DB (epoch timestamps)

## Sensor notes

- Mid-360 IMU mounted ~42 degrees from vertical
- Accel range in recording: [-4, 4]g with significant dynamic content
- Mean accel magnitude: 1.43g (lots of motion)

## Plots generated

All in `/Volumes/USB/fastlio_recordings/`:
- `odom_vs_imu_speed.svg` — 3-line comparison: fastlio odom speed, IMU+ZUPT speed, IMU no-ZUPT speed (clamped to 10 m/s)
- `accel_z_full.svg`, `accel_z_smoothed.svg`, `accel_xyz.svg`, `accel_z_2hz.svg` — various acceleration plots

## Key finding

FAST-LIO odometry velocity has spikes up to ~9800 m/s. First spike at 19:10:05 (50 m/s), ~75 seconds into the recording. The `spike_window.db` captures ±10s around this first event for detailed investigation.
