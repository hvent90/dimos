# Point-LIO + VirtualMid360

Adds **VirtualMid360** (replay a recorded Livox Mid-360 `.pcap` over the Livox
wire protocol) and **Point-LIO** (an alternative LIO backend to FastLIO).

`pcap_to_db.py` wires three autoconnected modules and stops once the pcap has
drained:

- **`VirtualMid360`** — replays the pcap (aliasing the host/lidar IPs onto a
  dummy interface on Linux, or `lo0` on macOS).
- **`PointLio`** — an unmodified, live Point-LIO that consumes the replay as if
  it were real hardware.
- **`PointlioRecorder`** — appends the `pointlio_odometry` / `pointlio_lidar`
  streams into the db.

## Usage / Testing

### Pcap to DB

```bash
# A missing --pcap path is fetched via get_data, so you can pass the LFS-relative
# path directly (here: the sample shake-stairs capture).
PCAP="mid360_shake_stairs/mid360_shake_stairs.pcap"

# gen .db from pcap (defaults to <pcap>.db next to the pcap)
python -m dimos.hardware.sensors.lidar.pointlio.scripts.pcap_to_db --pcap "$PCAP"

# ...with config overrides (any PointLioConfig field; see "Example config" below)
python -m dimos.hardware.sensors.lidar.pointlio.scripts.pcap_to_db \
    --config overrides.yaml --pcap "$PCAP"

# ...or append into an existing .db
DB="mem2.db"
python -m dimos.hardware.sensors.lidar.pointlio.scripts.pcap_to_db --db "$DB" --pcap "$PCAP"

# a quick-look <db>.rrd (aggregated world lidar + pose path) is written next to the db
dimos-viewer "${DB%.db}.rrd"
# dimos map global --lidar pointlio_lidar --pgo-tol=0 --no-carve
```

`--config` is optional — omit it to use the `PointLioConfig` defaults
(`dimos/hardware/sensors/lidar/pointlio/module.py`). There is no shipped default
yaml; the YAML you pass is a sparse override doc.

#### Options

| Flag | Default | Meaning |
|------|---------|---------|
| `--pcap` | *(required)* | Livox Mid-360 pcap (a missing path is fetched via `get_data`) |
| `--db` | `<pcap>.db` | Target memory2 db. Existing → append/align; missing → built from scratch (or fetched via `get_data`) |
| `--config` | `""` | YAML/JSON of `PointLioConfig` field overrides |
| `--rate` | `1.0` | Replay-speed multiplier |
| `--odom-freq` | `30.0` | Point-LIO odometry rate (Hz) |
| `--max-sensor-sec` | `0` (whole pcap) | Stop after N sensor seconds |
| `--warmup-sec` | `4.0` | Seconds the fake lidar waits before streaming (lets Point-LIO come up) |
| `--no-rrd` | off | Skip writing the `<db>.rrd` quick-look |
| `--voxel` | `0.2` | Voxel size (m) for the `.rrd` aggregated map |
| `--host-ip` | `192.168.1.5` | Host IP (override to run two replays at once) |
| `--lidar-ip` | `192.168.1.155` | Synthetic lidar IP |
| `--alias-iface` | `dimos-mid360` | Dummy iface the host/lidar IPs live on |
| `--no-network-setup` | off | Don't let the module alias the NIC via sudo — you've set up the IPs + routes yourself |

#### macOS

The module aliases the synthetic IPs onto `lo0`, which needs sudo. A tty-less
worker can't prompt, so set up the interface by hand, then pass
`--no-network-setup`:

```bash
sudo ifconfig lo0 alias 192.168.1.5 netmask 255.255.255.0
sudo ifconfig lo0 alias 192.168.1.155 netmask 255.255.255.0
sudo route -n add -host 224.1.1.5 -interface lo0
sudo route -n add -host 255.255.255.255 -interface lo0

python -m dimos.hardware.sensors.lidar.pointlio.scripts.pcap_to_db \
    --pcap "$PCAP" --no-network-setup
```

### Record a Mid-360 pcap

```bash
# Allow using tcpdump without sudo (only need to do once)
sudo setcap cap_net_raw,cap_net_admin=eip "$(which tcpdump)"

# set these (env var or pass to the blueprint below)
export DIMOS_MID360_PCAP_IFACE=enp2s0
export DIMOS_MID360_LIDAR_IP=192.168.1.155
export DIMOS_MID360_HOST_IP=192.168.1.5
export DIMOS_MID360_NETNS=lidar
```

```python
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.hardware.sensors.lidar.virtual_mid360.recorder import Mid360PcapRecorder

record = autoconnect(
    Mid360PcapRecorder.blueprint(
        pcap_path="recordings/run1.pcap",
        # lidar_ip="192.168.1.155",
        # iface="enp2s0",
    ),
)
ModuleCoordinator.build(record).loop()
```

### Replay a pcap to a module

```python
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.hardware.sensors.lidar.virtual_mid360.module import VirtualMid360
from dimos.hardware.sensors.lidar.pointlio.module import PointLio
from dimos.visualization.vis_module import vis_module

replay = autoconnect(
    VirtualMid360.blueprint(
        pcap="recordings/run1.pcap",
        # lidar_ip="192.168.1.155",
    ),
    PointLio.blueprint(
        # lidar_ip="192.168.1.155",
    ),
    vis_module("rerun"),
).global_config(n_workers=3)
ModuleCoordinator.build(replay).loop()
```

## Example config

Every field is a `PointLioConfig` override. Mid-360-specific values to retune for
a different sensor: `preprocess.lidar_type` (Livox), `blind`/`scan_line`,
`mapping.extrinsic_T`/`extrinsic_R` (Mid-360 IMU→lidar mount), `det_range`,
`fov_degree`.

```yaml
common:
    con_frame: false
    con_frame_num: 1
    cut_frame: false
    cut_frame_time_interval: 0.1
    time_lag_imu_to_lidar: 0.0

preprocess:
    # LID_TYPE enum (Point-LIO src/preprocess.h):
    #   1 = AVIA (Livox), 2 = VELO16, 3 = OUST64, 4 = HESAIxt32, 5 = UNILIDAR
    # 1 selects the Livox branch (preprocess.cpp avia_handler), which expects the
    # Livox CustomMsg point layout the Mid-360 emits:
    #   https://github.com/Livox-SDK/livox_ros_driver2/blob/master/msg/CustomMsg.msg
    lidar_type: 1
    scan_line: 4
    scan_rate: 10
    timestamp_unit: 3        # 3 = nanosecond
    blind: 0.5
    # Pre-KF input decimation: keep every Nth raw point. 1 = keep all (disable).
    point_filter_num: 3

mapping:
    use_imu_as_input: false  # false = IMU-as-output model (Point-LIO's robust path)
    prop_at_freq_of_imu: true
    check_satu: true
    init_map_size: 10
    # Pre-KF voxel downsample of each scan before the filter. false = feed the
    # full scan (disable). Leaf size is filter_size_surf below.
    space_down_sample: true
    satu_acc: 3.0            # g; accel >= this is treated as saturated (residual zeroed) to bound velocity
    satu_gyro: 35.0
    acc_norm: 1.0           # IMU accel unit: g
    plane_thr: 0.1
    filter_size_surf: 0.2   # pre-KF scan downsample leaf size (m), used iff space_down_sample
    filter_size_map: 0.5
    ivox_grid_resolution: 2.0   # iVox local-map grid (m)
    ivox_nearby_type: 6         # NEARBY6
    cube_side_length: 1000.0
    det_range: 100.0
    fov_degree: 360.0
    imu_en: true
    start_in_aggressive_motion: false
    extrinsic_est_en: false
    imu_time_inte: 0.005
    lidar_meas_cov: 0.01
    acc_cov_input: 0.1
    vel_cov: 20.0
    gyr_cov_input: 0.01
    gyr_cov_output: 1000.0
    acc_cov_output: 500.0
    b_gyr_cov: 0.0001
    b_acc_cov: 0.0001
    imu_meas_acc_cov: 0.01
    imu_meas_omg_cov: 0.01
    match_s: 81.0
    gravity_align: true
    gravity: [0.0, 0.0, -9.810]
    gravity_init: [0.0, 0.0, -9.810]
    extrinsic_T: [-0.011, -0.02329, 0.04412]   # Mid-360 IMU->lidar offset (m)
    extrinsic_R: [1.0, 0.0, 0.0,
                  0.0, 1.0, 0.0,
                  0.0, 0.0, 1.0]

odometry:
    publish_odometry_without_downsample: false
    odom_only: false
```

## Notes

- Replay runs in **real time** and Point-LIO is **not deterministic**, so
  successive runs differ. The largest run-to-run divergence is vertical (Z)
  drift — the gravity-aligned, least-constrained axis indoors. If a run lags
  (drops/delays lidar+IMU frames), scan-matching stops correcting the
  accelerometer/gravity bias and Z drifts steadily. For repeatable output,
  process offline from the pcap rather than trusting a live recording.
- Each db row carries a `ts` plus `pose_x/y/z` and an orientation quaternion;
  read any `pointlio_odometry*` table to diff trajectories.
