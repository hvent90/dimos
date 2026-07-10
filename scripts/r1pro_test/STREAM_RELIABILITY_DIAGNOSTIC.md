# R1 Pro stream reliability — diagnostic & remediation plan

Date opened: 2026-05-21
Branch: `mustafa/task/r1pro-coordinator-integration`

> ## ⚠️ CURRENT UNDERSTANDING (2026-07-03) — supersedes the May root cause below
>
> **Root cause: the robot's CSI/VI camera-capture subsystem cannot run
> multiple chassis cameras simultaneously.** When N chassis cams stream at
> once, the Tegra capture engine enters a timeout(2500 ms)→reset loop with a
> kernel SMMU/IRQ fault storm (~300+ faults/s) that starves the whole box —
> killing ALL streams (incl. ethernet lidar), crashing camera driver nodes
> (they stay dead until reboot), and once even the Wi-Fi firmware. Because
> v4l2 nodes start capturing on first subscriber, every earlier "bandwidth
> threshold" was actually **camera-count on the CSI bus**, not network
> byte-rate. → **Vendor escalation to Galaxea/TZTEK** with dmesg + the
> robot-local reproduction (`stream_dashboard.py --only chassis --stagger 10`
> on the robot, no network involved).
>
> Verified NOT the bottleneck: the PC↔robot wire (USB3 AX88179A, gigabit PHY,
> ~117 MB/s ceiling, 938 Mbit/s TCP measured, 0 loss, 1.4 ms RTT), DDS
> engines (C-level ≥700 MB/s), socket buffers (40 MB both ends).
> Real but secondary: robot eth1 qdisc tail-drops (fix: `txqueuelen 10000`),
> LCM 10k-deep subscription queue in the rerun bridge (dimos-side lag,
> experiment pending), May's uncompressed-image-over-LCM issue (fixed via
> JpegLcmTransport).
>
> **Interim configuration that is healthy indefinitely:** head + wrist
> colors + lidar (~10 MiB/s; adding wrist depths ≈ 25 MiB/s also OK).
> Skip the chassis surround cams until the vendor issue is resolved.
> Full session evidence: status log below, 2026-07-02 → 2026-07-03.

## Symptom

Cameras + lidar publish fine, then **all topics intermittently drop to 0 Hz**
at random intervals and recover on their own. The robot itself is healthy —
ROS feedback/commands flow continuously robot↔PC. CPU is pathological:

| Process | Observed CPU |
|---|---|
| `R1ProConnection` | 700–800% |
| `RerunBridgeModule` | ~400% |
| `ControlCoordinator` | oscillates 3% → 107% |
| all host cores | oscillate 15% ↔ 90% every ~1 s |

PC is also running low on memory. Teleop is responsive but occasional
keypresses are dropped.

## Root cause: uncompressed raw images flooding LCM UDP multicast

### The data path (current)

```
robot ROS topic            R1ProConnection (own process)        LCM UDP            RerunBridgeModule
/hdas/.../compressed  ───▶  cv2.imdecode(JPEG) → raw BGR  ──▶  Image.lcm_encode  ──▶  multicast  ──▶  subscribe_all
  (JPEG, ~30–90 KB)         (decode workers, 14 threads)       RAW uint8, NO       239.255.76.67     → Image.lcm_decode
                                                               compression          :7667            → rr.log()
```

Confirmed in code:

- [Image.lcm_encode()](../../dimos/msgs/sensor_msgs/Image.py#L471-L503) — serializes
  the **raw uint8 numpy array uncompressed**. `LCMTransport` (the transport pinned
  for every image topic in [r1pro_coordinator.py](../../dimos/robot/galaxea/r1pro/blueprints/basic/r1pro_coordinator.py#L167-L178))
  uses this path.
- [connection.py:767-788](../../dimos/robot/galaxea/r1pro/connection.py#L767-L788) — every
  chassis/wrist camera worker does `cv2.imdecode` then `out.publish(Image(bgr,...))`.
  The robot already sent JPEG; we throw the compression away.
- LCM transport is **UDP multicast** (`udpm://239.255.76.67:7667`). There is **no
  in-process shortcut** — even same-host modules serialize + go over UDP.
- `RerunBridgeModule` does `subscribe_all` → it receives **every** image message.

### Bandwidth math

Per published frame, uncompressed:

| Stream | Encoding | @1280×720 | @640×480 |
|---|---|---|---|
| head_color + 5 chassis + 2 wrist color (8×) | BGR8 (3 B/px) | 2.76 MB ea | 0.92 MB ea |
| head_depth (1×) | 32FC1 (4 B/px) | 3.69 MB | 1.23 MB |
| wrist depth (2×) | 16UC1 (2 B/px) | 1.84 MB ea | 0.61 MB ea |
| **per full frame-set** | | **~29.5 MB** | **~9.9 MB** |
| **aggregate @15 Hz** | | **~440 MB/s** | **~148 MB/s** |

The robot's actual JPEG streams total only ~5–15 MB/s on the ROS side.
**DiMOS inflates the data ~20–40× by decoding to raw and shipping it uncompressed.**
No UDP multicast link reliably carries 150–440 MB/s.

### Why "everything goes to 0" (not just images)

1. A 2.76 MB image is **fragmented into ~44 UDP datagrams** by LCM (datagram max
   ~64 KB). head_depth = ~58 fragments.
2. LCM's reassembler discards the **entire message if any one fragment is lost**.
   So a single dropped packet wastes 44 packets' worth of bandwidth.
3. Under CPU saturation the LCM receive thread can't drain the kernel UDP socket
   fast enough → `SO_RCVBUF` overflows → fragments dropped → whole frames vanish.
4. **All LCM topics share one multicast socket.** When image fragments overflow
   that buffer, the tiny control messages (`motor_states`, `cmd_vel`, `odom`,
   `joint_command`) are collateral damage — dropped in the same overflow.
   → that is why control feedback *and* images flatline together, and why
   teleop keypresses are occasionally lost.

The CPU oscillation + per-core 15↔90% sawtooth is the buffer fill→overflow→drain
cycle plus memory pressure / swap. `ControlCoordinator`'s 3→107% swing is it
being starved then bursted by the contended socket — it is a victim, not a cause.

### Where the CPU goes

- **Connection 700–800%**: 8–9 parallel `cv2.imdecode` workers (imdecode releases
  the GIL → genuinely uses 7–8 cores) + a large `lcm_encode` memcpy per frame-set
  + thousands of `sendmsg` syscalls fragmenting it.
- **Bridge 400%**: receiving + reassembling the flood, `lcm_decode`, `rr.log`
  of 8 uncompressed images.

### Acute trigger: the box is out of RAM (confirmed by measurement)

The *random total stalls* are not (only) UDP loss — the dominant acute failure is
**memory exhaustion**. M3 measured: Mem 13/15 GiB used, **Swap 4.0/4.0 GiB — 100%
full**, 765 MiB available. The machine is thrashing.

- One worker (PID 5412) holds **~6.7 GB RSS** — almost certainly the rerun bridge:
  `RerunBridgeModule` defaults to `memory_limit="25%"` = ~3.75 GB just for rerun's
  in-memory recording store, plus Python/cv2/turbojpeg, plus in-flight raw images.
- When RAM + swap are full, every page fault blocks on disk I/O. Processes that
  *should* burn 700% CPU can't get scheduled — `ps` caught them capped at 251%
  because they were swap-blocked. That fill→thrash→drain cycle **is** the
  1-second, all-core 15↔90% sawtooth and the coordinator's 3↔107% swing.
- Swap thrash also starves the LCM + ROS DDS receive threads → UDP buffer
  overflow → dropped fragments → the 27–32 s topic stalls seen in M1.

So: uncompressed images are the **root cause**; memory exhaustion is the **acute
amplifier** that turns it into total random blackouts. Both are fixed by the same
remediation (ship compressed, hold less).

This is a **bandwidth + transport** problem, not a robot problem and not "DiMOS
being slow." DiMOS is doing exactly what the blueprint told it to: ship raw pixels.

## Measurement protocol (fill in before/after)

Run with the system up. The user can execute these.

### M1 — robot-side stream size & rate — MEASURED 2026-05-21
- Chassis cam (`/hdas/camera_chassis_front_left/rgb/compressed`): **494 KB per
  JPEG message** (consistent 494.3–495.3 KB). That is a high-resolution camera —
  a 494 KB JPEG decodes to roughly a 1080p+ BGR frame (~6 MB raw), not the 720p
  /~50 KB JPEG assumed above. **The bandwidth estimates above are LOW.**
- `ros2 topic hz`: effective **0.1–0.5 Hz** with `min 0.13 s` (bursts ~7 Hz) and
  **`max 27.9 s` / `32.7 s`** — the topic is dead most of the time, in long
  blackouts. Measured *with dimos running*: the PC cannot even receive the ROS
  stream while it is thrashing.

### M2 — LCM / UDP packet loss on the PC — MEASURED 2026-05-21
- `net.core.rmem_max` = **67108864 (64 MB)** — already raised, *double* the
  runbook's Phase 3 value.
- `UdpInErrors` = `UdpRcvbufErrors` = **4543** — every UDP error is a receive-
  buffer overflow. **Overflows happen despite a 64 MB buffer** → we cannot tune
  our way out with `rmem_max`; the data rate is simply too high.

### M3 — memory / CPU — MEASURED 2026-05-21 — CRITICAL
- `free -h`: Mem **13 / 15 GiB used**, 765 MiB available · **Swap 4.0 / 4.0 GiB —
  100% full.** The machine is thrashing.
- `ps`: PID 5412 = **44.8% MEM ≈ 6.7 GB RSS**, 79.7% CPU (likely rerun bridge);
  PID 5413 = 251% CPU, 3.6% MEM. CPU figures are capped low because swap-blocked
  processes cannot be scheduled.

### M4 — still open
- Confirm PID 5412's module: `cat /proc/5412/cmdline | tr '\0' ' '; echo`
- head/wrist depth resolution: `ros2 topic echo <topic> --once --no-arr`

## Remediation plan

### Tier 1 — stop the bleeding (immediate, low risk, fixes reliability)

Switch the **color** image topics from `LCMTransport` to `JpegLcmTransport` in
[r1pro_coordinator.py](../../dimos/robot/galaxea/r1pro/blueprints/basic/r1pro_coordinator.py).
Precedent: [go2 `_with_jpeg.py`](../../dimos/robot/unitree/go2/blueprints/smart/_with_jpeg.py)
does exactly this for `color_image`.

- Wire payload per color frame: 2.76 MB → ~40–90 KB (JPEG q75). **~30–60× smaller.**
- Sub-64 KB messages are **single-datagram** → no fragment-loss amplification →
  atomic delivery. The LCM socket stops overflowing → control messages survive.
- head_depth (32FC1) / wrist depth (16UC1) cannot JPEG. Mitigate by **throttling
  hard** (`RerunBridgeModule` `max_hz`, ~3–5 Hz) and/or downscaling; head_depth at
  3.69 MB is the single biggest payload — consider dropping it from the default
  layout until Tier 2.
- Lower rerun `memory_limit` from `"25%"` to a fixed small value (e.g. `"2GB"`).

Expected after Tier 1: reliability problem gone; connection CPU still elevated
(~300–400% — still decoding + re-encoding); bridge CPU lower; memory way down.

### Tier 1.5 — throttle + downscale for the viewer

- Bridge `max_hz` per entity: surround chassis cams 5 Hz, wrist 10 Hz, depth 3 Hz.
- Downscale chassis surround cams to ~640×360 at the source (situational
  awareness, not detail work). Make it a `R1ProConnectionConfig` knob.

### Tier 2 — true compressed passthrough (fixes CPU + memory at the root)

The connection should **not decode at all**. The robot sends JPEG; carry the JPEG
bytes end-to-end and let the **viewer** decode (`rr.EncodedImage`).

- Needs a compressed-image carrier (the LCM `Image` type already supports
  `encoding="jpeg"`; the dimos-side `Image` dataclass does not — it requires a
  numpy array). Decision: new lightweight `CompressedImage` dimos msg vs. extend
  `Image`. **Open — see decision points.**
- Connection: `ROS CompressedImage.data` (JPEG) → publish straight through. Zero
  `cv2`, zero decode workers.
- Bridge: `to_rerun()` → `rr.EncodedImage(contents=jpeg, media_type="image/jpeg")`
  → viewer GPU-decodes.
- Expected: connection ~700% → ~80%; bridge ~400% → ~80%.

### Tier 3 — depth + lidar payload reduction (if still needed after #1)

**SHM is ruled out.** The rerun bridge consumes topics via `LCM().subscribe_all`,
and no SHM pubsub implements `subscribe_all` (`SharedMemoryPubSubBase` does not) —
moving a stream to SHM removes it from the viewer. The working robots confirm the
approach: go2, g1, and the Livox/FastLIO modules all carry `PointCloud2` over
plain `LCMTransport`. go2's lidar-over-LCM works because (a)
`VoxelGridMapper(emit_every=5)` voxel-downsamples before visualization and (b) its
bus is not flooded by cameras. (go2's `connection.py` has a `pSHMTransport` for
pointcloud, but only inside an unused `deploy()` function — dead code; if active
it would break rerun.)

So depth/lidar reduction stays on LCM:
- **Lidar**: throttle via bridge `max_hz`, and/or adopt the go2 pattern — compose
  a `VoxelGridMapper` and visualize the downsampled `global_map`.
- **Depth** (r1pro-specific — no go2 equivalent): throttle via `max_hz`,
  downscale, convert 32FC1→16UC1 (mm), or drop from the default layout.
- Persist `net.core.rmem_max` via `/etc/sysctl.d/`.

## Decision points

1. **Tier 2 message model** — new `CompressedImage` type, or extend `Image` to
   optionally hold compressed bytes? New type is cleaner but touches msgs/.
2. **Depth in the viewer** — keep head/wrist depth (throttled + downscaled), or
   drop from the default layout? It is the costliest payload class.
3. **Lidar reduction** — bridge `max_hz` throttle alone, or also add a
   `VoxelGridMapper` (the go2 pattern) for r1pro?

## Status log

- 2026-05-21 — diagnostic written. Root cause: uncompressed raw images over LCM
  UDP multicast → fragment-loss amplification → shared-socket overflow kills
  control too. Plan tiered.
- 2026-05-21 — M1–M3 measured. Confirmed + escalated: chassis cams are ~1080p+
  (494 KB JPEG, not 50 KB — bandwidth estimates were low). `UdpRcvbufErrors=4543`
  despite a 64 MB `rmem_max` → cannot tune the buffer out of it. **Swap 100% full,
  13/15 GiB RAM used — the box is thrashing; that is the acute trigger for the
  random blackouts.** Memory relief folded into Tier 1; implement immediately.
- 2026-05-21 — **Change #1 applied**: the 8 color streams (head_color, 5×
  chassis_*, 2× wrist_*_color) switched `LCMTransport` → `JpegLcmTransport` in
  `r1pro_coordinator.py`. Depth/lidar/control untouched. turbojpeg confirmed
  working in the r1pro direnv env. Awaiting re-measurement.
- 2026-05-21 — SHM follow-up scoped. Two blockers found for "depth/lidar →
  SHM": (1) `subscribe_all` is implemented **only** by the LCM pubsub
  (`lcmpubsub.py`) — SHM pubsubs lack it, and the rerun bridge consumes
  everything via `LCM().subscribe_all`, so SHM topics would vanish from the
  viewer. (2) For control, no `transport_shm` adapter exists (adapter types:
  mock/xarm/piper/sim_mujoco/mock_twist_base/flowbase) — the coordinator side
  is `transport_lcm`, so control must stay on LCM.
- 2026-05-21 — Checked how go2/g1 carry point clouds: **all on plain
  `LCMTransport`** (go2 connection, Livox `mid360`, `FastLio2`, `VoxelGridMapper`).
  The rerun bridge sees go2's point cloud *because* it is on LCM. go2's
  lidar-over-LCM is reliable because it voxel-downsamples (`VoxelGridMapper`,
  `emit_every=5`) and its bus is not camera-flooded. **SHM dropped entirely** —
  it is not how the working robots do it and it breaks the bridge. Lidar/depth
  reduction = throttle + voxelize, all on LCM.
- 2026-05-22 — **OOM ruled out.** After a failed run, `free -h` showed 10 GiB
  available, no `dmesg` OOM kill, and the dimos process tree was alive (main +
  2 workers ~380 MB each). Symptom is now "modules alive but idle — no data
  flows," recurring as "works a few seconds, then publishing stops." Points to
  ROS 2 discovery staleness carried across runs, not resources. **Change #2
  applied**: rerun `memory_limit` "25%"→"1GB"; added `_RERUN_MAX_HZ` per-entity
  caps (surround cams 5 Hz, color 10 Hz, depth 3 Hz). Still open: the
  few-seconds-then-stop stall — bisect via lcm-spy (is the connection still
  publishing when it stalls?) + dtop.
- 2026-07-02 — **New symptom analyzed: unbounded growing lag in rerun** (fresh
  2–3 s, then lag climbs). Different signature from the May 0-Hz drops: growing
  lag = a FIFO absorbing a rate mismatch, not loss. Suspect identified:
  `lcmpubsub.py` sets `set_queue_capacity(10000)` on every subscription; the
  rerun bridge drains its `subscribe_all` on a **single thread** doing
  `lcm_decode` → *then* the `max_hz` throttle → `rr.log`. Decode is paid for
  100% of messages (throttle sits after it); at ~150+ msg/s aggregate the
  thread falls behind and the 10k queue replays an ever-older backlog. When
  the queue fills, LCM **drops newest, keeps oldest** — streams appear to
  freeze (0 Hz) then recover as the queue drains. **Experiment applied**:
  queue capacity 10000 → 4 (temp, marked in code). Expected if theory holds:
  lag bounded ~constant indefinitely; choppiness OK. If lag still grows:
  suspect rerun SDK/viewer buffering instead. Real fix (after confirmation):
  per-entity latest-wins mailbox in the bridge (the `_enqueue_drop_oldest`
  pattern connection.py already uses), throttle before decode, optionally
  Tier 2 JPEG passthrough.
- 2026-07-02 (later) — **First run with queue=4: connection stats reveal the
  bottleneck is UPSTREAM of dimos.** Per-stream ingress (ROS side, before LCM):
  chassis cams `rx=1.3–1.5/s` (should be ~15), silence gaps (`age` = time since
  last rx) up to **9.3 s** — and even `imu_chassis` (tiny msgs, 167/s) showed a
  1.5 s gap. **All topics on the sensor participant stall together**, incl.
  payloads far too small for bandwidth to matter → suspect **discovery/
  liveliness flapping** (robot participant announcements lost under load → PC
  drops the participant → everything 0 Hz → rediscovery → brief revival →
  cycle), NOT per-topic bandwidth. Subscriber QoS already BEST_EFFORT+VOLATILE.
  **Tool added**: `monitor_topic_hz.py` — standalone raw-subscription monitor
  logging hz / MiB/s / gap / staleness / **count_publishers** per topic per
  tick. `pubs→0` during a stall = discovery flap; `pubs=1, hz=0` = data-path
  starvation. Protocol: baseline WITHOUT dimos, then WITH dimos (`--only` to
  limit added robot egress — FastDDS unicasts per reader). The LCM queue=4
  experiment (rerun-lag layer) is still pending a verdict — can't judge lag in
  rerun while ingress itself is starved.
- 2026-07-02 (verdict) — **monitor_topic_hz.py runs analyzed
  (`/tmp/hz_baseline.csv`, `/tmp/hz_with_dimos.csv`). DiMOS is exonerated;
  discovery flapping is ruled out. Root cause: robot egress saturation when
  many large-frame topics are subscribed at once.** Evidence:
  (1) Baseline, **no dimos**, monitor subscribing to all 14 topics: full rate
  for ~2 s (chassis 15.5 Hz / 7.5 MiB/s each), then **every large topic silent
  42 s straight**; IMUs perfect throughout. The "2–3 s then death" symptom
  reproduces with zero dimos code running.
  (2) Dimos running + monitor on **one** camera + IMUs: **25 Hz / 10.5 MiB/s,
  max gap 0.1 s, for 214 s**. One camera is flawless end-to-end.
  (3) `pubs` stayed 1 through every stall → discovery healthy; flap
  hypothesis dead.
  (4) Only multi-fragment messages die; single-datagram IMU survives → any
  lost fragment kills a whole best-effort frame. Robot 208 KB `wmem` holds
  ~145 fragments; one ~500 KB JPEG is ~350.
  (5) IMU stalls (gaps to 15 s) appear only with dimos + monitor both
  subscribed (double egress) → send-path saturation cascades participant-wide
  (Fast DDS default SYNCHRONOUS publish: writer threads block on full send
  buffer).
  (6) Arithmetic: all topics ≈ 45 (6 chassis) + ~26 (wrist depth raw) + head
  + lidar ≈ 75–80 MiB/s ≈ the robot's measured ~80 MB/s UDP send ceiling.
  **Separate hardware findings:** `head_depth` `pubs=0` in every run incl.
  baseline (nothing publishes `/hdas/camera_head/depth/depth_registered` —
  check driver/topic name); `wrist_right_*` `pubs=0` in runs 4–6 (known D405
  dropout: replug USB + restart HDAS tmux).
  **Next:** (A) raise robot `net.core.wmem_max`/`wmem_default` (sysctl) and
  re-run the all-topic baseline; (B) if still collapsing, `ros2 topic hz` ON
  the robot during a PC-side stall (producer vs wire); (C) binary-search the
  camera budget with `--only` (1 cam works; find N). Do NOT run monitor(all)
  + dimos together — Fast DDS unicasts per reader, doubling egress.
- 2026-07-02 (stagger runs, `stream_dashboard.py`) — **Egress budget measured
  + a new failure mode: the robot's camera drivers DIE under overload.**
  Timeline plot: `/tmp/flow_pc_timeline.png` (from `/tmp/flow_pc.csv`, 2 runs).
  (1) **Budget:** collapse reproduced twice at **26.7 / 26.8 MiB/s aggregate
  (~225 Mbit/s)**, both ~10 s after the 2nd chassis cam joined. All big topics
  die simultaneously; IMUs immune (though 180–190 Hz under load vs clean 200
  after — mild robot-side jitter). Below ~25 MiB/s: stable at full rate.
  (2) **Wire exonerated:** May iperf showed 938 Mbit/s TCP, 0 retransmits
  through the same USB3 GbE adapter — collapse is at 24% of wire capacity.
  The USB hub is not the bottleneck; the robot's DDS/UDP send path is
  (208 KB `wmem` per-burst mechanism; raw UDP was clean at 500 Mbit/s).
  (3) **Driver deaths:** in run 2, ~80 s into the collapsed state, **8
  publishers vanished from discovery permanently**: lidar, head_color,
  chassis_front_left/right, chassis_left, chassis_rear, wrist_right
  color+depth. The robot-local dashboard run confirms the same set dead
  ON the robot (survivors chassis_right 30 Hz/15.5 MiB/s + wrist_left pair
  publish at full rate locally). → overload kills/hangs HDAS driver
  processes; they stay dead until the HDAS launch is restarted (matches the
  known wrist-D405 recovery procedure). The dead/survivor split suggests
  per-process camera grouping — check `ros2 node list` + HDAS tmux logs on
  the robot. `head_depth` has never published in any run.
  (4) **Answer to "does subscriber count matter":** not count — aggregate
  egress bytes. Robot-local (SHM) tolerates everything; over the wire the
  budget is ~25 MiB/s until robot `wmem` is raised. The full dimos blueprint
  demands ~95+ MiB/s → guaranteed collapse + driver deaths.
  **Actions:** (a) restart HDAS to revive drivers; (b) robot sysctl
  `wmem_max`/`wmem_default` then re-measure budget with the stagger run;
  (c) until then keep dimos subscriptions ≤ ~20 MiB/s (head + wrist colors +
  lidar ≈ 10 MiB/s is safe; adding wrist depths ≈ 25 MiB/s is the edge);
  (d) investigate HDAS driver fragility (robot logs/dmesg at death time).
- 2026-07-02 (correction) — **The robot's `wmem` is NOT 208 KB anymore.**
  Measured now: `wmem_max` = `wmem_default` = **40960000 (~39 MiB)** — matching
  `rmem`. The May addendum's "208 KB stock send buffer" is stale; someone
  raised it since (find how: `grep -r wmem /etc/sysctl.d/ /etc/sysctl.conf`
  on the robot — if it's not persisted there, it was a manual sysctl and will
  NOT survive a robot reboot). **Consequences:**
  (a) The earlier recommendation to `sysctl -w net.core.wmem_max=16777216`
  is RETRACTED — it would lower the buffers. Do not apply.
  (b) The send-buffer-overflow hypothesis no longer explains the measured
  ~27 MiB/s collapse — kernel buffers are already huge. UNLESS Fast DDS
  overrides them per-socket with a small explicit value: verify on the robot
  while streaming with `ss -u -m` (look at `sb`/`tb` on the RTPS ports) and
  check `FASTRTPS_DEFAULT_PROFILES_FILE` / any XML profile in the HDAS env.
  (c) The May UDP send-cap numbers (~640–700 Mbit/s, `-w 10M` refused) were
  measured under 208 KB wmem and are stale too — re-run iperf UDP robot→PC
  to re-baseline.
  **Robot-side forensics during the next stagger-induced collapse** (run
  `stream_dashboard.py --stagger 15` on the PC to trigger it):
  1. `ss -u -m` — actual SO_SNDBUF on the DDS sockets (kernel vs DDS override)
  2. `nstat | grep -iE 'SndbufErrors|UdpOutDatagrams'` — send-path drops
  3. `tc -s qdisc show dev eth1` + `ip -s link show eth1` — qdisc/NIC TX drops
  4. `tegrastats` (or `top` per-core) — CPU saturation / throttling at collapse
  5. `dmesg -Tw` — **USB resets at the moment drivers die** (prime suspect for
     the grouped driver deaths, given the known wrist-D405 USB dropout)
  6. HDAS tmux panes / `ros2 node list` before vs after — which process group
     died and what it logged at death time.
- 2026-07-03 — **Robot-side forensics: two decisive findings.**
  (1) **Drop point localized: the eth1 transmit queue.** Robot counters:
  `UdpSndbufErrors = 13689` and `tc -s qdisc` shows the mq root dropped
  **exactly 13689** packets (13408 on tx queue `:6` — all DDS bulk traffic
  hashes to one queue). Perfect match → every failed send is a **qdisc
  tail-drop**: pfifo_fast holds only ~1000 packets (txqueuelen default), one
  ~500 KB frame is ~350 fragments, several cameras bursting together overflow
  it instantly. The 40 MB socket buffers are fine — the bottleneck moved one
  layer down from May's 208 KB socket buffer to the interface queue.
  **Fix to test:** `sudo ip link set eth1 txqueuelen 10000` (instant,
  reversible, not persistent until added to udev/systemd). If control-latency
  jitter appears afterwards, refine with fq_codel instead of pfifo_fast.
  (2) **Kernel camera-DMA fault storm (robot is currently sick):** `dmesg`
  floods ~300/s with `arm-smmu unhandled context fault` + `tegra-mc vi/vi2
  VPR violation` — the Tegra **VI camera-capture engine** is DMA-writing into
  unmapped memory, the signature of camera drivers that died while capture
  hardware kept running (the 8 dead HDAS drivers). Continuous IRQ storm
  (~950 memory-controller IRQs/5 s) degrades the whole box → all current
  measurements contaminated. **Reboot required** (tmux restart won't reset
  the wedged VI/SMMU state). Check `dmesg -T | grep -m1 smmu` vs `uptime -s`
  to date the storm's onset (consequence of driver death vs chronic).
  (3) Timestamp sync PC↔robot: clock offset already measured ~0.1 s (stale
  column); `dmesg -T` is wall time and the dashboard CSV now has a `wall`
  column — align by wall clock, second-level precision suffices.
  (4) **Driver architecture mapped** (`ros2 node list` on robot): each
  chassis cam is its own `/v4l2_chassis_*` node (V4L2 → Tegra CSI/VI capture —
  the VI fault storm belongs to this path); wrist cams are per-side nodes
  (`/hdas/camera_wrist_left` present, **`camera_wrist_right` MISSING
  entirely** → the D405 USB dropout, needs replug + HDAS restart); lidar =
  `/livox_lidar_publisher` (alive despite topic pubs=0 earlier — wedged or
  zombie entry); head = `/HDAS` / `/signal_camera`. **Duplicate node names**
  (gripper ×3, jointTracker ×4, robot_state_publisher ×2) = zombie discovery
  entries from crashed/restarted processes and/or double launches — bloats
  discovery, cleared by reboot. Disambiguate wedged-vs-zombie per node with
  `ros2 node info /v4l2_chassis_front_left` after reboot.
  **Order of operations:** reboot robot → verify dmesg quiet + clean node
  list (no dupes, wrist_right present) + all 14 topics publish → snapshot
  counters (`nstat >/dev/null; tc -s qdisc show dev eth1`) → raise
  txqueuelen → stagger run → compare collapse threshold + counter deltas.
- 2026-07-03 (reboot) — Robot rebooted; boot dmesg clean (normal TZTEK/Orin
  bring-up). Two notes from the boot log: (1) **chassis cameras are powered
  in PAIRS** via carrier-board GPIOs (`camera_0_1-power` … `camera_6_7-power`)
  behind a TCA9548 I2C mux — hardware pair-grouping, a clean explanation for
  why cameras/drivers die in groups rather than individually. (2) EXT4
  recovered 6 orphan inodes → the previous shutdown was unclean, consistent
  with how wedged the box was. Post-reboot verification pending: quiet dmesg
  after HDAS starts, clean node list, 14/14 topics at rate robot-side.
- 2026-07-03 (post-reboot run — **DIAGNOSIS REFRAMED**) — Stagger run on the
  freshly rebooted robot collapsed at **~10–15 MiB/s aggregate — far below
  the old 27 MiB/s** — and the robot dmesg during the run shows the real
  killer: `tegra-camrtc-capture-vi: uncorr_err: request timed out after
  2500 ms → err_rec: reset the capture channel` **looping every ~2.5 s, two
  channels at a time**, with the SMMU/VPR fault storm back (~1000+/5 s).
  The dashboard corroborates it exactly: chassis_front_left/right delivered
  **one frame per ~2.5 s, alternating** (one frame per reset cycle) from the
  moment they were subscribed — broken at the *capture* layer, before any
  network involvement. When chassis_left joined, the whole box melted:
  lidar (ethernet-sourced!), head_color, and wrist streams all died together
  → kernel IRQ storm starves everything.
  **Reframe:** the CSI/VI camera-capture subsystem cannot run this many
  chassis cameras simultaneously; subscribing (lazily) starts captures, so
  every "bandwidth threshold" we measured was really **camera-count on the
  CSI bus**, not network MiB/s. Explains all prior data: collapses always
  correlated with the Nth chassis cam joining, never with pure byte-rate;
  single cams always ran clean; Galaxea demos (few cams) never trip it.
  The eth1 qdisc drops (13689) are real but secondary.
  Also in dmesg: RealSense USB endpoint stall loop (`CLEAR_HALT` ×3, wrist
  cam), and an **iwlwifi firmware crash** — multiple subsystems failing
  hints at possible carrier-board power/thermal angle; check `tegrastats`
  temps/throttling during the storm.
  **Decisive next test (no PC, no network):** on the robot itself run
  `stream_dashboard.py --only chassis --stagger 10` while watching
  `dmesg -Tw`. If VI timeouts begin as cam #2/#3 starts with zero network
  load → confirmed capture-subsystem defect → collect dmesg + reproduce
  steps and escalate to Galaxea/TZTEK (vendor-level: CSI lane/clock config,
  per-pair camera power rails, or driver bug).
- 2026-07-03 (link verification) — **PC↔robot wire formally verified clean.**
  Adapter AX88179A on USB 3.x (negotiated 5000 Mbit/s bus), gigabit PHY up,
  MTU 1500 confirmed end-to-end (DF pings pass), RTT 1.41 ms avg / 0.016 ms
  jitter, 0% loss, 0 RX errors/drops over 2.8 GB. Theoretical payload
  ceiling ~117–118 MB/s (gigabit minus framing); May iperf3 measured
  938 Mbit/s TCP with 0 retransmits through this exact path. The wire could
  carry the entire sensor suite (~100 MB/s) — it is definitively not the
  constraint. Fresh iperf3 sweep pending an `iperf3 -s` on the robot.
- 2026-07-03 — **Change #3 applied: chassis surround cameras removed from the
  default pipeline.** `R1ProConnectionConfig.enable_chassis_cameras` added
  (default **False**) — the connection now subscribes head_color + wrists +
  head_depth + lidar + imus only; the 5 `v4l2_chassis_*` topics are never
  touched, so their CSI captures are never started by dimos. Blueprint
  (`r1pro_coordinator.py`): chassis transports, rerun views, and max_hz caps
  removed (re-enable path documented inline). Decode loop renamed
  `_chassis_camera_decode_loop` → `_jpeg_camera_decode_loop`. Expected robot
  demand: head ~2.3 + wrist colors ~2.3 + wrist depths ~16 + lidar ~5 ≈
  **~25 MiB/s** — at the measured safe edge; wrist depths are now the largest
  payload, so Tier-2 (depth downscale/compression) is the next lever if any
  jitter remains after the txqueuelen fix.
- 2026-07-03 (later) — **Unsubscribing is NOT sufficient: storm confirmed
  active with ZERO chassis-cam subscribers** (dimos running change #3, dmesg
  shows live VI/SMMU faults, 518 lines in ring buffer). So the lazy-capture
  assumption was wrong OR the VI engine has been wedged since the 07:26
  meltdown. All big streams (head/wrist/lidar) still die after seconds; IMUs
  fine — same IRQ-storm signature. Also applied: blueprint
  `n_workers=3` (connection / coordinator / rerun bridge each in own
  process — PC-side hygiene, cannot fix the robot-side storm).
  **Disambiguation + mitigation on the robot:**
  1. `pkill -f v4l2_chassis` (SIGTERM, NOT -9 — drivers must stop the
     capture channels cleanly on exit).
  2. Storm stops within ~10 s → drivers were actively streaming with no
     subscribers → permanent old-robot fix = remove the 5 v4l2_chassis nodes
     from the HDAS launch.
  3. Storm persists → VI engine wedged → reboot, then kill/disable the
     chassis nodes BEFORE first stream, then re-test dimos.
  Success criterion either way: head + wrists + lidar stay live indefinitely
  once no chassis camera is capturing.
- 2026-07-03 (clean-boot trap — **TRIGGER IDENTIFIED**) — Clean boot verified:
  0 faults across full bring-up (IMX185 sensors behind MAX9296 GMSL desers
  probe + bind cleanly; both D405s enumerate once; quiet at rest). Starting
  HDAS: **first fault fires 1.4 ms after `signal_camera_n` makes its first
  NvMap allocation** (`/signal_camera` node) — faults begin independent of
  the v4l2_chassis nodes, which is why `pkill v4l2_chassis` didn't stop the
  storm. **Mechanism pinned:** every fault IOVA (0x60240000, 0x60270000,
  0x603f/604b/… earlier) lies inside the **host1x syncpoint aperture**
  (base 0x60000000 size 0x4000000, per boot log), and `vifalw`/`vi2falw` =
  VI *falcon writes* = per-frame **syncpoint writes** hitting unmapped SMMU
  space → ~2 faults/frame/camera, scaling with active cameras until capture
  `uncorr_err` timeouts + IRQ storm melt the box. Frames can flow while
  faulting (data writes land; bookkeeping faults) — explains cameras
  streaming at full rate "despite" the storm. **TZTEK/Galaxea BSP
  camera-driver bug** (capture-channel SMMU context lacks the syncpt
  mapping). Next: fault-rate delta over 10 s per active camera; kill
  signal_camera → expect rate drop; determine which node publishes the head
  topics (/HDAS vs /signal_camera) to know if head_color survives without
  it. Galaxea ticket: clean-boot log + first-fault line + syncpt analysis.
- 2026-05-22 — First run after change #1 errored on every color stream
  ("R1Pro camera <name> decode error" — actually an *encode* failure, caught by
  the decode loop's broad `except`). Root cause: the native `libturbojpeg`
  library was missing. `PyTurboJPEG` (Python wrapper) imports fine, but
  `Image.lcm_jpeg_encode()` constructs `TurboJPEG()`, which needs the native
  `.so`. Fixed: `apt-get install -y libturbojpeg` (Ubuntu 22.04 package name is
  `libturbojpeg`, not `libturbojpeg0`). **Caveat: this apt install does not
  survive a container rebuild — `libturbojpeg` must be added to the dev-container
  image. It is a hidden native dependency of `JpegLcmTransport` /
  `JpegShmTransport` (go2's `_with_jpeg` blueprint would hit the same).**
