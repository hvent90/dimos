---
title: "Hosted Teleop"
---

Operate a DimOS robot remotely from any browser or Quest headset over WebRTC.
The robot dials out to a hosted broker
([teleop.dimensionalos.com](https://teleop.dimensionalos.com)) — Cloudflare
Realtime or LiveKit SFU — so you don't need to open any inbound ports on the
robot's network. It works behind a home router, on Wi-Fi, wired LAN, or
cellular.

## Quick Start

```bash
TRANSPORTS__BROKER__API_KEY=dtk_live_... \
dimos run teleop-hosted-go2-transport
```

The robot registers with the broker. Open
[teleop.dimensionalos.com](https://teleop.dimensionalos.com), log in, and your
robot appears under **Available Robots**. Click **Connect** and you're driving.

The API key alone is enough — the broker derives the robot identity from it.
`TRANSPORTS__BROKER__ROBOT_ID` / `TRANSPORTS__BROKER__ROBOT_NAME` are optional
overrides. All broker settings can also be passed on the CLI, e.g.
`-o transports.broker.api_key=dtk_live_...`.

## Get an API key

1. Visit [teleop.dimensionalos.com](https://teleop.dimensionalos.com) and sign up.
2. On the dashboard, **API Keys → + New Key**.
3. Copy the key (shown once) and pass it as `TRANSPORTS__BROKER__API_KEY`.

## Available blueprints

| Blueprint | Backend | Notes |
|-----------|---------|-------|
| `teleop-hosted-go2-transport` | Cloudflare | Drive + camera + minimap + click-to-nav (recommended) |
| `teleop-hosted-go2-livekit` | LiveKit | Drive + camera + state; no minimap/click-nav yet |
| `teleop-hosted-go2-multicam` | Cloudflare | Adds a second RealSense, operator-selectable, mux'd into one video track |
| `teleop-hosted-go2` | Cloudflare | Legacy `HostedTwistTeleopModule` wrapper (transport-swap above is preferred) |

The transport blueprints bind `Cloudflare*` / `LiveKit*` transports directly to
one module's streams (`Go2HostedConnection`), which shares a single broker
session. LiveKit uses the same `transports.broker.*` config key as Cloudflare.

Enable the glass-to-glass latency benchmark with
`-o go2hostedconnection.latency_stamp=true` (adds a timestamp strip the operator
reads then crops).

## Operator inputs

The browser is modality-agnostic — it streams whatever the device gives it, and
the robot blueprint decides what to do with it.

| Device | Input | Maps to |
|--------|-------|---------|
| Desktop browser | **WASD** keyboard | `TwistStamped` → `cmd_vel` |
| Phone | **On-screen WASD** | same path as keyboard |
| Quest 3 | **Left thumbstick** Y → fwd/back, X → strafe; **Right thumbstick** X → yaw | `Joy` → derived twist on the robot |

Shift = 2× speed, Ctrl = ½×. The operator can also send allow-listed posture
commands (Stand, Sit, Damp, …), toggle obstacle avoidance / the head LED, pick
the camera, E-STOP, and click the minimap to navigate.

## Live metrics HUD

While connected, the operator sees a metrics overlay, color-coded on video and
command-plane health:

| Metric | Source |
|--------|--------|
| `fps`, `bitrate`, `loss`, `jitter buffer`, `decode time`, `freezes` | Operator's `getStats()` on the inbound video track |
| `e2e latency` (glass-to-glass) | Frame-embedded capture-time stamp, decoded operator-side (needs `latency_stamp=true`) |
| `RTT` | NTP-style min-RTT clock sync over the reliable datachannel |
| `cmd latency`, `jitter`, `rate` | Robot-measured over the inbound command wire, sent back on `state_reliable_back` |

Pair with the recorder to log a session and emit a stats report:

```bash
dimos run teleop-hosted-go2-transport teleop-recorder
```

This writes `recording_teleop_<ts>.db` + a `report_<ts>.json` on disconnect;
regenerate from an old .db with `python -m dimos.teleop.utils.report path.db`.

## Configuration

`Go2HostedConnectionConfig` — commonly-tuned fields (config key
`go2hostedconnection`):

| Field | Default | Notes |
|-------|---------|-------|
| `telemetry_hz` | `3.0` | Robot → operator HUD push rate |
| `cmd_stale_after_sec` | `0.5` | Drop `cmd_vel` older than this |
| `latency_stamp` | `false` | Paint the glass-to-glass timestamp strip |
| `video_max_width` / `video_max_fps` | `0` (source) | Publish-side caps for constrained uplinks |
| `map_hz` / `odom_hz` | `2.0` / `15.0` | Minimap grid + robot-pose push rates (`0` = off) |
| `speaker` | `true` | Play operator mic on the dog's speaker (needs broker `audio_in`) |
| `max_nav_goal_m` | `100.0` | Reject click-to-nav goals beyond this (axis-aligned bound) |
| `allow_acrobatics` | `false` | Gate FrontJump / FrontPounce off the default command allow-list |

Broker settings live under `transports.broker.*`: `api_key` (required),
`broker_url`, `robot_id`, `robot_name`, and — Cloudflare only — `audio_in`
(operator→robot mic, opt-in) and `video_codec` (`h264` default).

## How it connects

The per-process `BrokerProvider` (Cloudflare) / `LiveKitBrokerProvider` owns the
session; blueprint transports bind to it. Channels:

```text
robot                          broker (Cloudflare / LiveKit)      operator browser/Quest
─────                          ─────────────────────────────      ──────────────────────
  POST /api/v1/sessions  ───►  session + datachannels     ◄───    operator joins
  cmd_unreliable        ◄────  (operator → robot, lossy)  ◄────    WASD / Joy
  state_reliable        ◄────  (operator → robot, json)   ◄────    ping, video_stats, estop
  state_reliable_back   ────►  (robot → operator, json)    ────►   pong, robot_telemetry, cmd_ack
  map_unreliable        ────►  (robot → operator, lossy)   ────►   minimap grid + odom
  video track           ────►  broker publishes + pulls    ────►   <video> sink
```

For the WebRTC / aiortc / Cloudflare implementation details (MAX_BUNDLE, SCTP
id-0 channel, candidate propagation, thread model), see
[`dimos/teleop/quest_hosted/README.md`](/dimos/teleop/quest_hosted/README.md).

## Known Limitations

- **Single operator** per robot session today.
- **TURN** is fetched best-effort from the broker; STUN-only fallback, so
  symmetric-NAT / some cellular networks may fail to connect.
- **No robot-side auto-reconnect.** If the link drops, the operator clicks
  **Connect** again; the robot side stays up.
- **LiveKit** ships command/state/video only — no minimap or click-to-nav yet.
- **Operator→robot audio** is opt-in (broker `audio_in=true`) and Cloudflare-only.
