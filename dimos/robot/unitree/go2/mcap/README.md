# Go2 MCAP → dataset ingest

Turn a Go2 onboard MCAP recording into a memory2 SQLite dataset (`.db`) that the
`dimos map` CLI reads. Lidar scans are deskewed into the **world frame** (Go2
convention); two trajectories are emitted for comparison.

## Build

```sh
uv run python -m dimos.robot.unitree.go2.mcap.ingest \
    data/go2_china_office_indoor.mcap --out data/go2_china_office_indoor.db --seconds 60
```
Drop `--seconds` for the full recording. `mcap` must be installed (`uv pip install mcap`).
The best-z trajectory defaults to `<mcap>_bestz.txt` (data dir, via data utils);
override with `--bestz`.

## Streams

| stream | type | content |
|---|---|---|
| `color_image` | Image | RGB, posed at `camera_optical` in world |
| `odom` | PoseStamped | Go2 leg-inertial odometry |
| `odom_bestz` | PoseStamped | leg xy/yaw + pitch-reconstructed z |
| `lidar` | PointCloud2 | per-scan world cloud (deskewed by `odom`) |
| `lidar_bestz` | PointCloud2 | per-scan world cloud (deskewed by `odom_bestz`) |
| `lidar_1s` / `lidar_bestz_1s` | PointCloud2 | 1 s accumulation per trajectory |

## Validate

```sh
dimos map summary data/go2_china_office_indoor.db
dimos map global  data/go2_china_office_indoor.db --lidar lidar       --voxel 0.1
dimos map global  data/go2_china_office_indoor.db --lidar lidar_bestz --voxel 0.1   # z-corrected
dimos map replay  data/go2_china_office_indoor.db --duration 60
```

Extrinsic (`extrinsics.py`): the L1 is mounted nearly upside-down; `EXT_R` is the
official `pitch=2.88` flip + `rotate_yaw_bias` (~-123°), ground-leveled.
`data/<mcap>_bestz.txt` is the reconstructed trajectory (column 6 = z).

## Full-length run (go2_china_office_indoor, 1157 s ≈ 19 min)

Ingest: `python -m ...ingest data/go2_china_office_indoor.mcap` (no `--seconds`)
→ **3 min 26 s**, `data/go2_china_office_indoor.db` ≈ **2.3 GB**.

```
anchor=1780066187180359363  odom poses=173616  span=1157.1s
wrote 34724 odom poses (x2)
wrote 17776 lidar scans (x2)
wrote 1158 1s accumulations (x2)
wrote 5703 color_image frames
```

`dimos map summary data/go2_china_office_indoor.db`:

```
Stream("color_image"):    5703 items, 2026-05-29 14:49:47 — 15:09:04 (1156.8s)
Stream("lidar"):         17776 items, 2026-05-29 14:49:47 — 15:09:04 (1157.0s)
Stream("lidar_1s"):       1158 items, 2026-05-29 14:49:47 — 15:09:04 (1157.0s)
Stream("lidar_bestz"):   17776 items, 2026-05-29 14:49:47 — 15:09:04 (1157.0s)
Stream("lidar_bestz_1s"): 1158 items, 2026-05-29 14:49:47 — 15:09:04 (1157.0s)
Stream("odom"):          34724 items, 2026-05-29 14:49:47 — 15:09:04 (1157.1s)
Stream("odom_bestz"):    34724 items, 2026-05-29 14:49:47 — 15:09:04 (1157.1s)
```

`dimos map global … --lidar lidar_bestz` reconstructs the z-corrected world map
(GPU voxel grid, pose-dedup) and writes an `.rrd`. Notes: odom is downsampled to
~30 Hz; a few onboard JPEG frames log a harmless "Corrupt JPEG data" warning and
are skipped.
