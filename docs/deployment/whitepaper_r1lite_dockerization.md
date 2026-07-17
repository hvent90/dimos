# Whitepaper: Dockerizing dimos for the Galaxea R1 Lite

**Status:** hardware-validated end-to-end, 2026-07-17
**Scope:** the final design — how dimos is packaged, shipped, run, maintained,
upgraded and rolled back on an R1 Lite, and how the same machinery extends to
the rest of the fleet.

This document is self-contained. Part 1 defines every term used later, so it
can be read without prior Docker or Python-packaging knowledge. Parts 2–5
describe the architecture as built. Parts 6–8 cover operations: what to do when
something changes, how a fleet gets updated, and how to roll back.

The companion document [`galaxea-r1lite.md`](galaxea-r1lite.md) is the
practical runbook (bring-up steps, gotchas, porting notes). The full evidence
trail — every hypothesis and dead end — is in
[`scripts/r1lite_test/BRINGUP_LOG.md`](../../scripts/r1lite_test/BRINGUP_LOG.md).

---

# Part 1 — Bootstrap: the vocabulary

Skip this if the terms are familiar. Nothing later depends on knowing them in
advance.

## 1.1 Containers

**Image** — a sealed, read-only snapshot of a complete filesystem: an operating
system, libraries, a Python interpreter, and our software, all installed and
frozen. Think of a shipping container: packed once, sealed, behaves identically
wherever it goes.

**Container** — an image *running*. It borrows the machine's CPU, memory and
(optionally) network, but its files are its own. Deleting a container leaves the
host exactly as it was. This is the property that lets us add dimos to a robot
whose operating system belongs to the vendor without touching their software.

**Layer** — images are built in stacked layers, one per build instruction.
Unchanged layers are reused from cache, which is why the second build of an
image is far faster than the first.

**Tag** — a human-readable name for an image, e.g.
`dimos-r1lite:0.0.14b1-r1lite.1`. **Tags are mutable**: someone can publish
different bytes under the same tag later. This matters enormously and is why we
do not rely on them alone.

**Digest** — a cryptographic hash of an image's exact content, e.g.
`dimos-r1lite@sha256:8e8b61c6…`. A digest is **immutable**: it always names one
exact image, forever. If the content changed, the digest would change.

**Registry** — a server that stores images (`ghcr.io` is GitHub's). This is
separate storage from the git repository: `git clone` fetches **source code**,
`docker pull` fetches **built images**. Same account, different shelves.

**Dockerfile** — the recipe: an ordered list of instructions that builds an
image.

**Build context** — the set of files sent to the Docker engine to build from.
Large contexts make builds slow, so we control ours deliberately.

**Multi-stage build** — a Dockerfile with more than one `FROM`. Early stages can
compile things; later stages copy out only the results. The compilers never
reach the shipped image.

**ENTRYPOINT / CMD** — `ENTRYPOINT` is the program the container always runs;
`CMD` is its default arguments. `docker run <image> list` replaces `CMD`, so the
container behaves like the `dimos` command itself.

**Compose** — a file (`compose.yaml`) describing one or more containers
(**services**) and how to run them, so the whole set starts with one command.

**`.env`** — a file of `KEY=value` lines that compose reads. This is the
per-robot configuration surface: which image, which uid, which ROS domain.

**Restart policy** — `restart: unless-stopped` means Docker restarts the
container if it crashes or the robot reboots, until someone explicitly stops it.

**Healthcheck** — a command Docker runs periodically to ask "is this actually
working?". Important nuance: Docker's restart policy does **not** act on health.
A healthcheck is *observability*, not automation — which also means a wrong one
cannot restart-loop a working robot.

**uid / gid** — the numeric user and group a process runs as. Linux file
permissions are enforced on these numbers, not on names. A container can run as
any uid; matching the right one turns out to be load-bearing (§4.4).

**`network_mode: host`** — the container shares the robot's real network stack
instead of getting a private, NAT'd one.

**`ipc: host`** — the container shares the robot's shared-memory namespace
(`/dev/shm`) instead of getting a private one.

## 1.2 Python packaging

**PyPI** — the global public shelf of Python packages (~600k of them). It is
live: new versions land daily, published by strangers on their schedule.

**Wheel (`.whl`)** — a pre-built, installable package. Fast: no compilation.

**Source distribution (sdist)** — source code that must be **compiled** at
install time, which needs a compiler and the right system headers.

**ABI tag (`cp310`, `cp312`)** — which Python version a wheel was compiled for.
`cp310` = CPython 3.10. A cp312-only wheel **cannot** install on Python 3.10.

**`manylinux`** — the platform tag for a wheel that works across Linux
distributions. A wheel can be `cp310` yet only exist for macOS — the Python tag
and the platform tag must **both** match, a distinction that is easy to miss.

**Dependency range** — what a project declares, e.g. `typer>=0.19.2,<1`: "any
version from 0.19.2 up to 1.0". Ranges are ambiguous by design — *"whatever fits
today"* changes over time.

**Lockfile (`uv.lock`)** — the exact resolved answer: every package, one exact
version, with hashes. Not "a wrench, 10–14mm" but "this wrench, serial 0.23.1".
Committed to git. This is what developers and CI actually run.

**Extra** — an optional dependency bundle, e.g. `dimos[perception]`. Not
installed unless requested.

**Dependency group** — like an extra, but for development tooling (tests, lint).
`pyproject.toml` sets `default-groups = ["tests"]`, meaning a normal dev sync
installs the test dependencies **too**.

**Environment marker** — a condition on a dependency, e.g.
`; python_version >= '3.12'`. The installer evaluates it and skips the package
when false. This is how a dependency that only exists for some Pythons or
platforms is declared honestly.

**`uv`** — a fast Python package manager. Crucially, **uv reads `uv.lock`; pip
does not.** pip resolves dependencies itself, from scratch, taking the newest
version that fits each range.

**`exclude-newer`** — a uv policy in `pyproject.toml` (set to `"7 days"` here)
meaning: ignore anything published in the last week. Deliberately living
slightly in the past so the rest of the world finds the fresh bugs first. **pip
does not honour this** — it is a uv concept.

**`override-dependencies`** — a uv mechanism to force a version past what some
package claims to need, when we know better. These overrides live in
`pyproject.toml`, not in an exported requirements file — which is why anything
that re-resolves an exported list can "rediscover" a conflict the lock already
settled.

## 1.3 ROS 2 and DDS

**ROS 2** — the robotics middleware the Galaxea stack speaks.

**Topic** — a named channel, e.g. `/hdas/feedback_arm_left`.

**DDS / FastDDS** — the transport underneath ROS 2. Two behaviours matter:

- **Discovery** ("who is out there, what topics exist?") happens over
  **multicast UDP** — a broadcast on the network.
- **Data delivery**, when both parties are on the **same machine**, happens
  through **shared memory**: the writer places bytes in a chunk of RAM
  (`/dev/shm`) and hands over a pointer. Far faster than copying through the
  network stack.

These are **two different roads**, which produces the single most confusing
failure mode in this whole system: *topics are visible, and no messages ever
arrive.* Discovery works, delivery does not.

**`ROS_DOMAIN_ID`** — an integer partitioning ROS traffic. Ours is `2`. Nodes on
different domains cannot see each other.

**`rclpy`** — ROS 2's Python library. It is **compiled against a specific Python
version**: ROS 2 Humble ships Ubuntu 22.04's system Python **3.10** (cp310). Any
process that imports `rclpy` must therefore run on Python 3.10. This constraint
propagates through everything below.

## 1.4 dimos

**Module** — a unit of dimos that does one thing (owns a robot connection, runs
a controller, bridges a viewer).

**Blueprint** — a composition of modules describing a runnable system.
`r1lite-coordinator` is a blueprint. `dimos run r1lite-coordinator` runs it.

**LCM** — dimos' **internal** message bus. dimos does not speak ROS internally;
it speaks LCM.

**Connection module** — the translator between a robot's native interface and
dimos. `R1LiteConnection` is the **only** component that speaks ROS. Everything
above it is robot-agnostic.

**ControlCoordinator** — runs the control loop (100Hz here), owns hardware
components and tasks.

**rerun / dimos-viewer** — the visualisation system. dimos *serves* a data
stream; a viewer *connects* to it and draws.

---

# Part 2 — The problem

The R1 Lite's onboard PC already runs Galaxea's software: Ubuntu 22.04, ROS 2
Humble, their motor drivers. That machine works, and it is not ours.

We must add dimos to it such that:

1. **It cannot break the vendor stack.** Their Python and ours must not collide.
2. **It is reproducible.** The same version must mean the same software on every
   robot, forever — including a robot deployed months from now.
3. **It survives reboots** without a human.
4. **It upgrades and rolls back** across a fleet, quickly and safely.
5. **It needs no laptop, no compiler, and no credentials** at a customer site.

The naive approach — `pip install dimos` on the robot — fails every one of
these. dimos needs specific versions of ~150 packages; the vendor stack needs
its own. There is no clean undo, no way to know what a working robot looked
like, and no way to do it identically twice.

---

# Part 3 — The final architecture

## 3.1 Three layers, one machine

```
┌────────────────────────────────────────────────────────────────────────┐
│  R1 Lite onboard PC  —  Ubuntu 22.04, x86-64                           │
│                                                                         │
│  ┌──────────────────────────────────────────────────────────────────┐  │
│  │  LAYER 1 — Galaxea vendor stack   (host, NOT ours)               │  │
│  │  tmux: hdas · mobiman · ros_discovery · system · tools           │  │
│  │  Owns the motors. Speaks ROS 2 Humble. Runs as uid 1000.         │  │
│  │      HDAS ── arms · torso · grippers                             │  │
│  │      VCU  ── chassis (holonomic swerve, RC-gated)                │  │
│  └───────────────┬──────────────────────────────────────────────────┘  │
│                  │  ROS 2 · DDS · ROS_DOMAIN_ID=2                      │
│    feedback  ────┤  /hdas/feedback_arm_left|right, _chassis, _torso    │
│    (robot →)     │  /hdas/feedback_gripper_*, /hdas/imu_*              │
│    commands  ────┤  /motion_target/target_speed_chassis                │
│    (→ robot)     │  /motion_target/target_joint_state_arm_*            │
│                  │                                                      │
│  ┌───────────────┴──────────────────────────────────────────────────┐  │
│  │  LAYER 2 — dimos    container: dimos-dimos-1                     │  │
│  │  network_mode: host · ipc: host · user 1000 · restart always     │  │
│  │                                                                   │  │
│  │    R1LiteConnection   ← THE ONLY MODULE THAT SPEAKS ROS          │  │
│  │      · translates ROS ⇄ LCM                                       │  │
│  │      · owns the chassis dead-man                                  │  │
│  │            │  LCM (internal bus — never leaves this box)          │  │
│  │            ▼                                                      │  │
│  │    ControlCoordinator  @100Hz                                     │  │
│  │      · servo_r1lite → 16-DOF upper body   (transport_lcm)         │  │
│  │      · vel_chassis  → chassis vx,vy,wz    (transport_lcm)         │  │
│  │            ▲                                                      │  │
│  │            │  twist_command  ←── LCM /cmd_vel (public Twist bus)  │  │
│  │      any Twist publisher: teleop, nav, agents                     │  │
│  │                                                                   │  │
│  │    RerunBridge → serves gRPC :9877   RerunWebSocketServer :3030   │  │
│  │    WebsocketVisModule :7779                                        │  │
│  └───────────────────────────────────────────────────────────────────┘ │
│                                                                         │
│  ┌───────────────────────────────────────────────────────────────────┐ │
│  │  LAYER 3 — viewer   container: dimos-viewer-1  (same image)       │ │
│  │  hosts the web viewer app on :9090                                │ │
│  └───────────────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────────────┘
```

## 3.2 The central design idea

**dimos does not speak ROS. Its internal bus is LCM. `R1LiteConnection` is the
single translation boundary.**

Everything above that line — coordinator, tasks, teleop, navigation, agents — is
robot-agnostic and would work identically over any other transport.

Two consequences:

- **Adding a robot to dimos is mostly writing one connection module**, not
  plumbing ROS through the stack.
- **The R1 Pro reuses ~all of this deployment machinery unchanged.** Only the
  connection module, joint lists, units and blueprint names differ.

## 3.3 The control path

Driving the base, end to end:

```
teleop / nav / agent  ──►  LCM /cmd_vel          (public Twist bus)
                      ──►  ControlCoordinator.twist_command
                      ──►  vel_chassis task      (chassis/vx, vy, wz)
                      ──►  transport_lcm adapter ──► LCM /chassis/cmd_vel
                      ──►  R1LiteConnection
                      ──►  ROS /motion_target/target_speed_chassis
                      ──►  VCU  ──►  wheels
```

`/cmd_vel` is deliberately a **public bus**: the coordinator's `twist_command`
input and any module's `cmd_vel` output are pinned to the same LCM topic. Any
Twist publisher drives the chassis with no extra wiring. That is the extension
point for navigation, agents, or a new teleop source.

**Safety property.** The chassis VCU **latches its last velocity target
forever** — it has no dead-man of its own. `R1LiteConnection` supplies one: it
streams the chassis command every tick and collapses to an explicit
zero-velocity stream when `cmd_vel` goes stale (`cmd_vel_timeout_s = 0.3`), and
sends a courtesy zero on shutdown. **Never command this chassis from anything
that does not stream.**

## 3.4 Ports

| Port | Served by | Purpose |
|---|---|---|
| **9877** | dimos (`RerunBridge`) | gRPC data stream. **Viewers connect here.** |
| **3030** | dimos (`RerunWebSocketServer`) | Viewer→dimos events (clicks, WASD) |
| **7779** | dimos (`WebsocketVisModule`) | Web dashboard; also the healthcheck target |
| **9090** | viewer container | Hosts the web viewer app |
| **9878** | viewer container | Its own unused gRPC proxy, parked off 9877 |

Direction matters: **dimos serves 9877 and viewers connect to it.** A laptop
viewer needs no sidecar:

```bash
dimos-viewer --connect rerun+http://<robot-ip>:9877/proxy --ws-url ws://<robot-ip>:3030/ws
```

The container prints these hints for every interface at startup.

---

# Part 4 — The image

`scripts/galaxea/docker/Dockerfile` — ~5.6 GB, two stages.

## 4.1 Base: public, and pinned by digest

```dockerfile
ARG ROS_BASE=ros@sha256:afb40d6b…      # == ros:humble-ros-base-jammy, 2026-07-16
```

`ros-base`, not `desktop`: no GUI stack, no RViz, no Gazebo. A customer robot
pulls or builds this with **zero credentials**.

Pinned by **digest**, not tag, because `ros:humble-ros-base-jammy` is mutable and
rebuilt regularly. Reproducibility that stops at the OS is not reproducibility.

## 4.2 Builder stage — compile, then throw it away

```dockerfile
FROM ${ROS_BASE} AS builder
ENV CIBUILDWHEEL=1
RUN pip install "setuptools>=70" "packaging>=24" wheel "pybind11>=2.12" tomli uv
COPY . /src
RUN pip wheel --no-deps --no-build-isolation --wheel-dir /wheels /src
RUN cd /src && uv export --frozen --no-emit-project --no-default-groups \
        --format requirements-txt -o /wheels/requirements.txt
```

Four decisions, each load-bearing:

- **`CIBUILDWHEEL=1`** drops `-march=native` from the C++ extension. Without it
  the wheel is compiled for *this* CPU and may fault on a different customer
  machine.
- **`setuptools>=70`** — jammy ships setuptools 59, which silently ignores
  pyproject's `[project]` table and produces an empty `UNKNOWN-0.0.0` wheel.
  dimos has no `__init__.py` files, so the package name and contents *must* come
  from pyproject.
- **`packaging>=24`** — setuptools 70 calls an API jammy's packaging 21.3 lacks.
- **`--no-default-groups`** — `pyproject.toml` sets `default-groups = ["tests"]`,
  so the ordinary flags still export the **entire test suite** (torch, mujoco,
  ultralytics, pyaudio) into a runtime image. This flag is the difference between
  **~150 packages and ~380**, and between building and not: `pyaudio` has no
  Linux wheel and needs a compiler.

The compiler toolchain never reaches the shipped image.

## 4.3 Runtime stage — install exactly the lock

```dockerfile
FROM ${ROS_BASE}
RUN apt-get install -y --no-install-recommends \
        python3-pip python-is-python3 \
        libturbojpeg0-dev liblcm-dev libgl1 libglib2.0-0 iproute2
COPY --from=builder /wheels /tmp/wheels
RUN pip install --no-cache-dir uv \
    && uv pip install --system --no-cache --no-deps -r /tmp/wheels/requirements.txt \
    && uv pip install --system --no-cache --no-deps /tmp/wheels/dimos-*.whl "pygame==2.6.1" \
    && pip uninstall -y uv \
    && rm -rf /tmp/wheels
```

**This is the heart of the design.** Dependencies come from `uv.lock`, exported
to a pinned list — **not** from a resolver.

*Why not just `pip install dimos.whl`?* Because pip resolves dependencies
itself: it never reads `uv.lock`, and it ignores `exclude-newer = "7 days"`. It
takes latest-wins inside every declared range. That makes the image
**non-reproducible** — build the same commit on two days, get different
software — which silently destroys the rollback guarantee the whole design rests
on.

Both flags matter:

- **`uv`, not pip** — the export carries hashes, which put pip in
  `--require-hashes` mode where every requirement must be `==`-pinned.
  Transitive extras are not (`chromadb` asks for `uvicorn[standard]>=0.18.3`), so
  pip aborts.
- **`--no-deps`** — the export is already the complete closure, so there is
  nothing to resolve, and resolving actively breaks: pyproject's
  `override-dependencies` are not in the exported file, so any resolver
  rediscovers conflicts the lock already settled.

The apt list is the minimum: turbojpeg (camera encode), lcm (dimos' bus), gl/glib
(opencv/open3d import-time libraries), iproute2 (`ip`), pip + `python`-as-python3.

`pygame` is pinned by hand because `r1lite-keyboard-teleop` needs it and it is
otherwise only packaged in the heavy `sim` extra.

**No venv, on purpose.** dimos installs into the image's **system Python 3.10**,
which *is* Humble's rclpy Python. One interpreter, so the entire class of
"wrong Python / rclpy won't import" bugs is structurally impossible.

## 4.4 Identity: uid 1000, not root

```dockerfile
RUN useradd --uid 1000 --user-group --create-home --shell /bin/bash dimos
USER dimos
ENV HOME=/home/dimos
```

This is not hygiene — **root here is a data-loss bug.**

FastDDS delivers same-host data by writing into the **reader's** `/dev/shm`
segment. The vendor stack runs as `r1lite` (uid 1000). A root container creates
**root-owned** reader segments, which the vendor's uid-1000 publishers cannot
write into. Discovery still succeeds over UDP, so **topics are visible and not
one message arrives** — silently, with no error anywhere.

Matching the uid keeps **zero-copy shared memory**, which matters for the camera
streams. Measured: **1600 messages in 8 s (~185 Hz, full rate).**

`setup.sh` writes `DIMOS_UID`/`DIMOS_GID` from `id -u`/`id -g`, so this stays
correct by construction on a robot that ships a different uid.

Note dimos keeps its logs and run registry under `$HOME/.local/state/dimos`, so
whatever uid runs must own a writable home.

## 4.5 The build gate

```dockerfile
RUN dimos list | grep -q r1lite-coordinator \
    && python3 -c "from ...r1lite_coordinator import r1lite_coordinator" \
    && python3 -c "from ...r1lite_keyboard_teleop import r1lite_keyboard_teleop"
```

The build **tests itself**, and a broken image therefore cannot be produced.

Two checks, because the first alone is insufficient: `dimos list` exercises the
CLI but only reads registry *strings* — it never imports a blueprint. Importing
the blueprints pulls the real graph (connection, coordinator, `vis_module`, every
transitive import) — the thing the robot actually runs.

Placed **after `USER dimos`**, so it also proves the runtime user can write its
state directory.

## 4.6 Entrypoint: boot-order-proof

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-2}"
# for `run ...`: poll up to 120s for /hdas/* topics
exec dimos "$@"
```

The container may boot before the vendor stack. It waits (up to 120 s, skip with
`DIMOS_NO_WAIT=1`), so `restart: unless-stopped` plus this wait makes power-cycle
order irrelevant. `exec dimos "$@"` makes dimos **PID 1**, which is what lets
Docker's SIGTERM reach it — see §5.2.

## 4.7 Build context

```bash
./scripts/galaxea/docker/build.sh [revision]
```

Stages `git archive HEAD` into a temp directory and **deletes `data/`** before
building. Building from the repo root instead ships ~33 GB of LFS assets to the
daemon for files no R1 Lite blueprint loads. `--network=host` is used because
guest and corporate networks routinely block Docker's default DNS.

Two consequences worth internalising:

- **It builds the last *commit*.** Uncommitted changes are not included.
- **Untracked files cannot leak in**, which is exactly what you want from a
  release artifact.

Tag: `dimos-r1lite:<pyproject-version>-r1lite.<rev>` — e.g.
`dimos-r1lite:0.0.14b1-r1lite.1`. The image is labelled with the git revision it
was built from (`org.opencontainers.image.revision`), so provenance is always
one command away:

```bash
docker image inspect <tag> --format '{{index .Config.Labels "org.opencontainers.image.revision"}}'
```

---

# Part 5 — The deployment

## 5.1 Compose

```yaml
x-dimos-common: &dimos-common
  image: ${DIMOS_IMAGE:?set in /opt/dimos/.env}
  network_mode: host        # DDS discovery is multicast; bridge NAT drops it
  ipc: host                 # FastDDS same-host shared memory
  restart: unless-stopped
  logging:
    driver: json-file
    options: { max-size: "10m", max-file: "3" }

services:
  dimos:
    <<: *dimos-common
    env_file: .env
    user: "${DIMOS_UID:-1000}:${DIMOS_GID:-1000}"
    stop_grace_period: 30s
    command: ["run", "r1lite-coordinator"]
    healthcheck:
      test: ["CMD", "python3", "-c",
             "import socket; socket.create_connection(('127.0.0.1', 7779), timeout=5).close()"]
      interval: 30s
      start_period: 120s

  viewer:
    <<: *dimos-common
    entrypoint: ["rerun"]
    command: ["--serve-web", "--port", "9878", "--memory-limit", "2GB"]
```

## 5.2 Why each setting exists

- **`network_mode: host`** — ROS 2 finds peers by shouting on the network
  (multicast). Docker's default bridge NAT drops that, so a default container
  would see *nothing at all*.
- **`ipc: host`** — data delivery is via shared memory. A private `/dev/shm`
  means discovery succeeds and delivery silently fails. **Necessary but not
  sufficient** — the uid must match too (§4.4).
- **`stop_grace_period: 30s`** — **safety-critical.** The VCU latches its last
  velocity; `R1LiteConnection.stop()` sends the courtesy zero and SIGTERM
  triggers it. Docker's default grace is **10 s, then SIGKILL** — and a killed
  process sends no zero, leaving a robot driving at its last commanded velocity.
  Teardown joins publisher threads and shuts down the sensor executor, so 10 s is
  not obviously enough.
- **`logging` caps** — the json-file driver is unbounded by default. An
  always-on 100 Hz coordinator fills the disk and takes the robot down weeks
  later, at a customer site.
- **`healthcheck`** — a **TCP connect**, deliberately not an HTTP GET (`/`
  redirects to `/command-center`, which returns 503 unless the React app was
  built — it is not, in this image — so an HTTP probe would report healthy
  robots as unhealthy). Informational only: `restart: unless-stopped` catches a
  process that *exits*, never one alive but wedged, and `compose ps` would
  otherwise report "Up" for a dead robot.
- **viewer as its own container** — dimos' in-process rerun web mode
  GIL-deadlocks inside forkserver workers. A separate process keeps the Rust
  server off dimos' GIL. Its gRPC proxy is parked on **9878** because **9877 is
  dimos'**; the viewer service exists only to host the web app on 9090.

## 5.3 The per-robot configuration surface

`/opt/dimos/.env` — written once by `setup.sh`, never overwritten:

```ini
DIMOS_IMAGE=dimos-r1lite@sha256:8e8b61c6…   # what to run  ← the version pin
DIMOS_UID=1000                              # match the vendor stack
DIMOS_GID=1000
ROS_DOMAIN_ID=2
VIEWER=rerun                                # compose the rerun bridge
RERUN_OPEN=none                             # serve gRPC; spawn no local viewer
```

**That file is the robot's entire identity.** Everything else is immutable.

## 5.4 Installation

```bash
git clone https://github.com/dimensionalOS/dimos.git ~/dimos
cd ~/dimos
./scripts/r1lite_test/roslaunch.sh                       # vendor stack up
bash scripts/galaxea/r1lite/setup.sh [--tar <file>]
```

`setup.sh` is idempotent and prompts before every host change:

| Step | What |
|---|---|
| 1 | Preflight — x86-64, >20 GB free |
| 2 | Docker + compose (apt), with a sudo fallback since the `docker` group isn't active until re-login |
| 3 | Image: already present → registry pull → `--tar` load → build on robot |
| 4 | Sysctls `/etc/sysctl.d/60-dimos.conf` — 64 MB UDP buffers for DDS/LCM |
| 5 | Deploy `/opt/dimos/{compose.yaml,.env}` + `/usr/local/bin/dimos` wrapper |
| 6 | `docker compose up -d` |
| 7 | **Verify**: subscribe to `/hdas/feedback_arm_left` for 8 s inside the container |

**Step 7 is the only check that matters.** >100 messages proves the whole chain:
the vendor stack is publishing, the container can hear it, and dimos can read
it. It is the one thing that silently doesn't work, and everything else is easy
to fix by comparison.

Step 5 prefers an immutable **digest** when the image came from a registry,
falling back to the tag for a tarball or on-robot build (which is local and
cannot be overwritten from outside anyway).

## 5.5 The wrapper

`/usr/local/bin/dimos` makes dimos a normal command on the robot:

```bash
dimos list
dimos run r1lite-keyboard-teleop      # needs ssh -X (pygame window)
```

It runs `docker compose run --rm dimos "$@"`, forwarding `DISPLAY` and X11 mounts
when present. Nobody needs to think about Docker.

> **The `dimos` service is already running `r1lite-coordinator`.** Running a
> blueprint that contains a coordinator (like teleop) starts a **second** one
> commanding the same chassis. Stop the service first:
> `docker compose -f /opt/dimos/compose.yaml stop dimos`.

---

# Part 6 — Maintenance: what to do when something changes

The single most useful fact: **not every change needs a rebuild.** The system has
three tiers, and the cost differs by ~100×.

## 6.1 The change matrix

| What changed | Rebuild image? | Steps | Time |
|---|---|---|---|
| **`.env`** (version pin, uid, domain, viewer) | ❌ | edit + `up -d` | **~10 s** |
| **`compose.yaml`** (ports, limits, healthcheck, grace) | ❌ | `git pull`, copy to `/opt/dimos`, `up -d` | **~30 s** |
| **`setup.sh` / `roslaunch.sh` / test scripts** | ❌ | `git pull` on the robot | **~5 s** |
| **dimos Python code** (blueprint, connection, throttle) | ✅ | commit → build → save → scp → load → redeploy | **~15 min** |
| **Dependencies** (`pyproject.toml`) | ✅ | + `uv lock` first | ~15 min |
| **Base OS** (new ROS base) | ✅ | + update the digest in the Dockerfile | ~30 min (cold) |

The distinction is simply **what lives inside the sealed box**: Python code and
dependencies are baked in; deploy files and configuration are not.

## 6.2 Tier 1 — configuration only (~10 seconds)

```bash
sudo vi /opt/dimos/.env
docker compose -f /opt/dimos/compose.yaml up -d
```

This is how you change version, uid, ROS domain, or viewer behaviour. **This is
also how you upgrade and roll back** (§7, §8).

## 6.3 Tier 2 — deploy files (~30 seconds)

```bash
cd ~/dimos && git pull
sudo cp scripts/galaxea/r1lite/compose.yaml /opt/dimos/compose.yaml
docker compose -f /opt/dimos/compose.yaml up -d --force-recreate
```

`compose.yaml` is a *deploy* file, not part of the image. Ports, resource
limits, healthcheck, `stop_grace_period` and the uid mapping are all tunable
without touching the image at all.

## 6.4 Tier 3 — code or dependencies (~15 minutes)

```bash
# 1. laptop — change, and COMMIT (build.sh builds the last commit)
git add -A && git commit -m "..." && git push

#    if pyproject.toml changed:
uv lock && git add uv.lock && git commit --amend --no-edit

# 2. build — the image self-tests; a broken one cannot be produced
./scripts/galaxea/docker/build.sh

# 3. verify provenance matches what you just committed
docker image inspect dimos-r1lite:0.0.14b1-r1lite.1 \
  --format '{{index .Config.Labels "org.opencontainers.image.revision"}}'
git rev-parse --short HEAD          # must match

# 4. ship
docker save dimos-r1lite:0.0.14b1-r1lite.1 | gzip > /tmp/dimos-r1lite.tar.gz
scp /tmp/dimos-r1lite.tar.gz r1lite:~/

# 5. robot
cd ~/dimos && git pull
docker compose -f /opt/dimos/compose.yaml down
docker load -i ~/dimos-r1lite.tar.gz
sudo rm /opt/dimos/.env             # ← see below
bash scripts/galaxea/r1lite/setup.sh --tar ~/dimos-r1lite.tar.gz
```

> ### ⚠️ The one non-obvious step: `sudo rm /opt/dimos/.env`
>
> If you rebuild **under the same tag**, two things bite:
>
> 1. `setup.sh` step 3 sees the tag present and skips loading your new file.
> 2. `.env` pins the **old digest**, which still exists locally — so compose
>    happily keeps running the **old image**, forever, ignoring the new one.
>
> Deleting `.env` makes `setup.sh` re-derive the digest from the freshly loaded
> image. **Step 5 must print a different `sha256:`** — that is your proof the new
> code is live.
>
> This is not a flaw. It is immutable pinning doing exactly its job: an image
> reference never changes meaning by accident. **Updates are explicit, always.**
>
> The cleaner alternative, once images are published: **bump the revision**
> (`build.sh 2` → `…-r1lite.2`), so old and new are different references and
> nothing needs deleting.

## 6.5 Iterating quickly (the dev path)

Rebuilding for every experiment is too slow. For active development there is a
second path — `scripts/r1lite_test/` — that bind-mounts your checkout into a dev
container with a `uv` venv, so edits are live with no rebuild.

Trade-off: it is **not** reproducible or versioned, and it uses a UDP-only DDS
profile (losing zero-copy) instead of matching the uid.

> ### ⚠️ Never run both paths on one robot.
> They collide on ports and on truth. A leftover dev container held 9877 for 8
> hours, so dimos never bound its gRPC port **while still reporting `healthy`**
> (the healthcheck probes 7779, not rerun), and a laptop viewer silently rendered
> stale code. Before deploying the runtime path:
> `docker stop dimos-dev-r1lite`.

**Rule of thumb:** dev path for exploration; runtime path for anything a robot
should still be doing tomorrow.

---

# Part 7 — Fleet operations

## 7.1 The intended flow

For a blank robot, once the image is published to a registry:

```bash
git clone https://github.com/dimensionalOS/dimos.git ~/dimos
cd ~/dimos && bash scripts/galaxea/r1lite/setup.sh
```

**~10 minutes, no laptop, no build, no compiler.** `setup.sh` pulls the sealed
image and starts it. This is the design's destination.

> **Current gap.** The image is not yet published, so today's flow is
> `docker save | scp` + `setup.sh --tar` — which works and validates everything,
> but keeps a laptop in the loop. Publishing to ghcr, ideally from CI on merge so
> it happens without anyone remembering, is what closes this. One decision
> attached: ghcr packages are private even when the repo is public, so either
> robots get a read-only token or the package is made public.

## 7.2 Upgrading N robots

Because the image is immutable and a robot's version is **one line of `.env`**:

```bash
# once: publish
docker push ghcr.io/dimensionalos/dimos-r1lite:0.0.15-r1lite.1

# per robot: ~10 seconds
sudo sed -i 's|^DIMOS_IMAGE=.*|DIMOS_IMAGE=ghcr.io/dimensionalos/dimos-r1lite@sha256:<new>|' /opt/dimos/.env
docker compose -f /opt/dimos/compose.yaml up -d
```

Properties that matter at fleet scale:

- **Nothing is built on a robot.** Every robot runs bytes that were built once
  and tested once.
- **Robots may sit on different versions deliberately** — canary one, hold the
  rest.
- **No robot's state depends on what was on someone's laptop that day.**
- **The upgrade is a config change**, so it is scriptable over SSH and needs no
  interactive session.

## 7.3 Why this is fast

- **Layer caching** — a rebuild after a code-only change reuses the OS and
  dependency layers; only the wheel and its install re-run.
- **The heavy stuff is cached** — apt and the ~150 dependencies do not move
  unless `uv.lock` moves.
- **Tier 1 and 2 changes need no build at all**, and most operational tuning
  (ports, limits, grace periods, uid) is Tier 1 or 2 by construction.

---

# Part 8 — Rollback

## 8.1 The procedure

```bash
sudo vi /opt/dimos/.env        # DIMOS_IMAGE=<the previous reference>
docker compose -f /opt/dimos/compose.yaml up -d
```

**That is the whole thing.** Seconds, one line, no build, no network if the old
image is still in the robot's local Docker.

## 8.2 Why it actually works

Rollback is only as trustworthy as the immutability underneath it. Three
references decide what a robot runs, and **all three are pinned**:

| Layer | Pinned by | If it were loose |
|---|---|---|
| OS + ROS | `ARG ROS_BASE=ros@sha256:…` | base drifts under you |
| ~150 Python deps | `uv.lock` → exported → `--no-deps` | pip takes latest-wins |
| The image itself | `DIMOS_IMAGE=…@sha256:…` in `.env` | a tag can be pushed over |

If **any** of those floats, "roll back to the known-good version" silently gets
different bytes, and you are debugging a ghost. That is why the design pins the
whole chain rather than the convenient parts.

**Digest > tag.** `setup.sh` writes a digest automatically when the image came
from a registry. Tags are mutable; digests are not. Pin by digest and "the
version we shipped in July" means one exact set of bytes, in July and in
December.

## 8.3 What rollback does not cover

- **`.env` itself is not versioned.** If an upgrade also changed `ROS_DOMAIN_ID`
  or `DIMOS_UID`, reverting `DIMOS_IMAGE` alone won't restore those. Keep `.env`
  changes minimal and deliberate; the image reference should be the only line
  that routinely moves.
- **Deploy files are versioned in git, not in the image.** If a `compose.yaml`
  change is implicated, roll that back with `git checkout <old> -- ` and re-copy.
- **Robot-side state** (vendor stack, sysctls, docker itself) is host
  configuration, not container content. `setup.sh` is idempotent and prompts, so
  it is safe to re-run, but it does not "undo".

## 8.4 Verifying which version a robot is on

```bash
grep DIMOS_IMAGE /opt/dimos/.env
docker compose -f /opt/dimos/compose.yaml ps          # IMAGE column shows the digest
docker image inspect <ref> --format '{{index .Config.Labels "org.opencontainers.image.revision"}}'
```

That last command returns the **git commit** the running image was built from.
Provenance is never a guess.

---

# Part 9 — Extending this to other robots

The deployment machinery is robot-agnostic. For the R1 Pro (or any Galaxea
robot):

**Reusable as-is** — `Dockerfile` (only the final `CMD`), `build.sh` (only the
tag), `entrypoint.sh` (the `/hdas/*` wait is Galaxea-wide), `compose.yaml` (only
the `command:`), `setup.sh` (only the tag + deploy dir), `dimos-wrapper.sh`
(verbatim).

Suggested shape: `scripts/galaxea/r1pro/` beside `r1lite/`, sharing
`scripts/galaxea/docker/`. Parameterise the blueprint via `ARG ROBOT` / compose
`command:` rather than forking the Dockerfile.

**Must be robot-specific** — the connection module (the real work: topics, DOF
counts, units, QoS, dead-man), the joint list and hardware components, the
blueprint and its registration, and a bring-up test ladder under
`scripts/<robot>_test/`.

**Transferable rules** (see `galaxea-r1lite.md` §7 for the full list): prove the
robot obeys plain ROS *before* introducing dimos; command publishers need
RELIABLE QoS; one-shot commands are ignored — the robot needs a stream; never
joint-command a coupled linkage; Python 3.10 is not negotiable while Humble is.

---

# Appendix A — One-page summary

**What we built:** dimos runs on the R1 Lite's own PC, inside a sealed,
versioned container, alongside Galaxea's stack, touching nothing of theirs.

**How it's packaged:** a ~5.6 GB image from a **digest-pinned** public ROS base;
dimos compiled to a wheel in a throwaway builder stage; ~150 dependencies
installed **exactly from `uv.lock`** with no resolver involved; core deps only —
no torch, no simulator, no test tooling; runs as **uid 1000** to match the
vendor; **self-tested at build time** so a broken image cannot exist.

**How it runs:** two compose services on `network_mode: host` + `ipc: host`,
`restart: unless-stopped`, log-rotated, health-probed, with a **30 s stop grace**
so the chassis always gets its courtesy zero.

**How it's configured:** one file — `/opt/dimos/.env` — holding the image
digest, uid, ROS domain and viewer mode.

**How it's maintained:** three tiers. Config ~10 s. Deploy files ~30 s. Code or
dependencies ~15 min, gated by a self-testing build.

**How it's upgraded:** publish an image, change one line per robot, `up -d`.
Nothing builds on a robot.

**How it's rolled back:** put the previous digest back and `up -d`. It is
trustworthy because the OS, the dependencies and the image are **all** pinned by
content — so a version means the same bytes forever.
