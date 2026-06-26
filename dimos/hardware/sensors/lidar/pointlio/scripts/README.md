# pcap_to_db.py

Replay a recorded Livox Mid-360 `.pcap` through **Point-LIO** and record the
resulting odometry + lidar into a memory2 SQLite db (with an auto-generated
`.rrd` quick-look).

It wires up three autoconnected modules and stops once the pcap has drained:

- **`VirtualMid360`** — replays the pcap over the Livox wire protocol (aliasing
  the host/lidar IPs onto a dummy interface on Linux, or `lo0` on macOS).
- **`PointLio`** — an unmodified, live Point-LIO that consumes the replay as if
  it were real hardware.
- **`PointlioRecorder`** — appends Point-LIO's `pointlio_odometry` /
  `pointlio_lidar` streams into the db.

## Quick start

```bash
# fetch a sample Mid-360 capture (arg = path inside the LFS archive)
PCAP_PATH="$(python -c "from dimos.utils.data import get_data; print(get_data('mid360_shake_stairs/mid360_shake_stairs.pcap'))")"

# build <pcap>.db next to the pcap
python -m dimos.hardware.sensors.lidar.pointlio.scripts.pcap_to_db --pcap "$PCAP_PATH"

# view the auto-written quick-look
rerun "${PCAP_PATH%.pcap}.rrd"
```

A missing `--pcap` or `--db` is fetched via `get_data` before falling back to
building from scratch.

## Config overrides

Override any `PointLioConfig` field with a small YAML/JSON doc:

```bash
# overrides.yaml  ->  {filter_size_surf: 0.15, filter_size_map: 0.5}
python -m dimos.hardware.sensors.lidar.pointlio.scripts.pcap_to_db \
    --pcap "$PCAP_PATH" --config overrides.yaml
```

## Options

| Flag | Default | Meaning |
|------|---------|---------|
| `--pcap` | *(required)* | Livox Mid-360 pcap capture |
| `--db` | `<pcap>.db` | Target memory2 db. Existing → append/align; missing → built from scratch |
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

## macOS

The module aliases the synthetic IPs onto `lo0`, which needs sudo. A tty-less
worker can't prompt, so set up the interface by hand, then pass
`--no-network-setup`:

```bash
sudo ifconfig lo0 alias 192.168.1.5 netmask 255.255.255.0
sudo ifconfig lo0 alias 192.168.1.155 netmask 255.255.255.0
sudo route -n add -host 224.1.1.5 -interface lo0
sudo route -n add -host 255.255.255.255 -interface lo0

python -m dimos.hardware.sensors.lidar.pointlio.scripts.pcap_to_db \
    --pcap "$PCAP_PATH" --no-network-setup
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
