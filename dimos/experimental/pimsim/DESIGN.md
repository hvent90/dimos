# PimSim — Design from First Principles

A study/architecture document. The companion `README.md` lists *what each
file does*; this explains *why the system is shaped this way*, builds the
concepts up one at a time, and works through the open design decisions and
what each entails. Reachability appears here as one consumer among several,
not the centerpiece.

---

## 1. The problem PimSim solves

Start from what a "normal" simulator is: one process that welds together
four concerns —

1. **physics** (how bodies move and collide),
2. **scene** (what geometry exists and where),
3. **rendering** (what it looks like),
4. **sensing** (what a lidar/camera would see).

When those four are welded into one engine, three things you actually want
become impossible:

- **You can't swap the physics engine.** MuJoCo is great for deterministic,
  headless, control-grade rollout; a browser engine is great for
  interactive, multi-user, visual teleop. A monolith forces one or the
  other forever.
- **You can't share the scene.** A Rust lidar raycaster, a motion planner,
  and a renderer each need to know "what's in the world." In a monolith each
  reaches into the engine's private structures, so each is rewritten per
  engine.
- **You can't bridge sim and reality.** Perception produces "objects in the
  world"; sim produces "objects in the world"; if they don't share a
  representation, nothing that consumes one can consume the other.

**PimSim's thesis:** stop treating "the simulator" as the unit. Make
*physics authority* a pluggable role, and decouple every other concern from
it through three shared contracts. Then any authority can drive any
consumer.

```
              the three shared contracts
   ┌───────────────────────────────────────────────────┐
   │  1. SCENE PACKAGE   what geometry exists (cooked, portable)
   │  2. ENTITY STREAM   where everything is now (wire format)
   │  3. LCM BUS         the transport everything speaks
   └───────────────────────────────────────────────────┘

   AUTHORITY (pluggable)            CONSUMERS (authority-blind)
   ┌──────────────────┐            ┌────────────────────────────┐
   │ Babylon + Havok  │            │ Rust lidar raycaster        │
   │  (interactive)   │ ─stream──► │ splat / camera views        │
   │                  │  entities  │ planning world (collision)  │
   │ MuJoCo           │            │ reachability builder        │
   │  (headless,      │            │ any future renderer/recorder│
   │   deterministic) │            └────────────────────────────┘
   │ perception       │
   │  (eventually)    │   No consumer knows which authority is upstream.
   └──────────────────┘
```

The rest of this document is: the three contracts (§2–4), the pluggable
authorities (§5), the consumers (§6), and the decisions that are still open
(§7), then reachability as a worked example of a consumer (§8).

---

## 2. Contract one — the scene package

**First principle:** geometry that is computed at runtime is geometry you
can't trust, share, or reproduce. CoACD convex decomposition of a mesh takes
seconds to minutes; doing it at startup means slow boots, nondeterministic
collision shapes, and a different result on every machine.

So PimSim **cooks** the scene offline, once. The cooker
(`dimos/simulation/scene_assets/`) takes a source mesh + a `<scene>.cook.json`
sidecar and emits a *portable package*:

- **visual GLB** — what it looks like,
- **decimated collision GLB** — a cheap mesh for raycasting,
- **per-entity GLBs** — each movable object as its own asset,
- **CoACD collision hulls** — convex decomposition, computed once, written
  into the package (never at runtime),
- **`objects.json`** — per-prim semantic labels,
- **a MuJoCo wrapper MJCF** — the robot model + assets bundled so MuJoCo can
  load the same scene.

The package is the single source of "what geometry exists," and it is
consumed *identically* by the browser sim, by MuJoCo, by the lidar, and by
the planner. **What entails from this:** the cooking pipeline is a
dependency of everything else — it is the supply side, and it lands as its
own PR. Anything that consumes a package is an argument for landing it.

---

## 3. Contract two — the entity stream

**First principle:** "what geometry exists" (stable) and "where it is right
now" (changes every tick) are different facts with different lifetimes.
Bundling them means a pose update drags the whole mesh description with it,
and a consumer can't cache identity across motion.

So PimSim splits them (`entity.py`):

- **`EntityDescriptor`** — *what an entity is*: stable id, `kind`
  (dynamic / kinematic / static), `mesh_ref` (GLB), `shape_hint` + `extents`
  (for primitives), `mass`, optional `rgba`. Everything needed to
  *instantiate* the object once.
- **`EntityStateBatch`** — *where everything is now*: a timestamped snapshot
  of `(descriptor, pose)` entries, restreamed every tick by whoever owns
  physics.

**Why a wire format and not a Python object?** Because the consumers aren't
all Python. The Rust lidar and the browser both decode this. So the batch is
**versioned, length-prefixed JSON over LCM** (the same pattern as the
existing `EntityMarkers` message) — hand-decodable in any language, no code
generation, unknown keys ignored for forward-compatibility. It deliberately
depends on nothing heavier than `Pose`.

**Why "authority-agnostic" matters:** the consumer subscribes to the stream
and never learns whether Havok, MuJoCo, or a perception stack produced it.
That is the whole decoupling, expressed in one message type.

---

## 4. Contract three — the LCM bus

The transport is dimos's existing LCM bus, with one bridge: the browser
can't speak LCM natively, so `BabylonSceneViewerModule` exposes a
`@dimos/msgs`-compatible **LCM-over-WebSocket** bridge on `/lcm-ws`. This is
why a browser tab can be a first-class participant — it publishes `/odom`
and `/entity_state_batch` onto the same bus a headless MuJoCo process would,
and dimos modules can't tell the difference. Nothing else about the bus is
PimSim-specific; it's just the medium the three contracts travel over.

---

## 5. The pluggable authorities

This is the payoff of the contracts: physics authority becomes a *role* that
different engines fill, selected by an `entity_authority` switch.

### Babylon.js + Havok (the interactive authority)

`BabylonSceneViewerModule` boots Babylon.js + the Havok physics engine
(WASM) in a browser tab, served by uvicorn, in right-handed z-up to match
ROS. Physics steps inside Babylon's render loop;
`scene.onAfterPhysicsObservable` drives the entity broadcast.

- `entity_authority="browser"` — Havok owns `/odom` and
  `/entity_state_batch`; you can grab and throw objects in the tab and the
  planner sees them move.
- `entity_authority="external"` — the browser mirrors states from another
  publisher (MuJoCo) as kinematic bodies; it's a *viewer*, not the authority.

**Why have this at all?** Interactive, multi-user, zero-install (it's a URL),
good for teleop and demos. The cost: a browser engine isn't deterministic or
headless-friendly for control-grade work.

### MuJoCo (the headless, deterministic authority)

The existing `MujocoSimModule`, reworked to participate: it gained a
`scene_entities` config and an `entity_state_batch: Out[EntityStateBatch]`,
and `entity_scene.py` composes the cooked wrapper MJCF with one body per
scene-package entity (auditing spawn contact, welding anything embedded in
the static scene). Crucially, **the robot MJCFs are now scene-free** — the
old hardcoded `manip_table`/`manip_cube` rig is gone; those spawn as
synthetic entities from the office sidecar instead.

**Why both?** They're complementary roles, not rivals: Havok for
interaction/visualization, MuJoCo for deterministic/headless/control. Same
scene package, same entity stream, so a blueprint swaps authority with a
flag and nothing downstream changes.

---

## 6. The consumers

Each subscribes to the contracts and is blind to the authority:

- **Rust scene lidar** (`scene_lidar/`) — a BVH-accelerated raycaster
  against the cooked collision GLB *plus* dynamic entities from
  `/entity_state_batch`. 720×16 rays @ 10 Hz. It raycasts the same scene the
  sim simulates because both read the same package + stream.
- **Splat cameras** (`splat_camera.py`) — render a Gaussian-splat view from
  the robot camera pose and composite live entity poses + arm hulls on top.
- **Planning world** (`MujocoWorld`) — `sync_entity_poses` writes streamed
  entity poses into the planner's collision world, so a plan routes around a
  chair where it *is now*, not where the package said it started.
- **Reachability builder** (§8) — samples arm FK against the same world.

The lesson to present: adding a consumer is "subscribe to the stream," not
"integrate with the simulator." That asymmetry is the architecture working.

---

## 7. The open decisions (and what each entails)

These are the genuine forks. Understanding them is the point of this
section — each is stated as *the question*, *the options*, *what choosing
entails*, and *the recommendation*.

### Decision A — one scene-object noun, or three?

**Question.** Three types describe "a shaped thing at a pose":
`Obstacle` (planning collision input), `EntityDescriptor` (PimSim scene
state), and the perception `Object` (`Detection3D`: pointcloud + mask +
detector output). Do they unify?

**Options.**
- *Keep all three.* Minimum work; leaves duplicated geometry description
  across the planner and PimSim, which reviewers correctly flag.
- *Merge all three.* Wrong — the perception `Object` pulls in open3d + cv2
  and is detector output (observations, not spawnable geometry); forcing it
  into the scene type would drag that dependency into the Rust/browser
  consumers.
- *Merge the two that are genuinely the same, keep perception separate
  (recommended).* `Obstacle` and `EntityDescriptor` collapse into one
  `SceneObject`; the world exposes two verbs — `add_object` (inject new
  geometry, what perception and `add_obstacle` do) and `update_object_pose`
  (reposition known geometry, what `sync_entity_poses` does). Perception
  `Object` stays upstream and is *converted into* a `SceneObject` by the
  obstacle monitor.

**What it entails.** ~11 files in `manipulation/planning/` plus `entity.py`;
no mechanism bodies change (type merge + renaming `add_obstacle`→
`add_object` across the three world backends). Because it touches every
backend, it is agreed jointly, not landed unilaterally. The win: one noun
for "a thing in the scene," and `EntityStateBatch` becomes the streaming
form of that one noun.

**Why two verbs and not one pipeline.** "Inject newly-seen geometry" and
"move known geometry" are genuinely different operations (one mutates the
body set, one writes a pose). Collapsing them into a single pipeline is the
over-abstraction trap — it would force runtime-add through the entity path,
which can't currently add bodies, only move them. Keep the noun singular,
keep the verbs plural.

### Decision B — how do fast planners coexist with a shared scene?

**Question.** A motion planner wants speed; speed pushes it to own its world
(stay in native, compiled collision structures). But we want a shared scene.
Do these conflict?

**Key realization (this is the one to internalize): no, because "shared
scene" and "shared runtime world" are different layers.** A backend ingests
the shared *package* once at `finalize` and compiles it into its own native
runtime structure. After that, planning runs entirely native — the package
was just loader input. So a shared scene costs the planner nothing.

**What follows.** There are two legitimate kinds of planner, and the
`PlannerSpec` protocol should name both instead of pretending all planners
are alike:
- *Backend-agnostic* (`RRT-Connect`, `RRT*`, `JacobianIK`): Python, consume
  *any* `WorldSpec` via its collision/FK methods. These must keep working on
  every backend — that is the line the abstraction protects.
- *Backend-coupled* (`roboplan`): a compiled library whose planner owns its
  world so it can collision-check in-process without a Python call per edge.
  Exposed through `PlannerSpec` but valid only with its own world; the
  factory already gates this (`planner_name="roboplan"` requires
  `world_backend="roboplan"`).

**What it entails.** Almost nothing to build — it's a *naming* clarification
in the spec that sanctions the coupling roboplan already has, and protects
the generic planners. The practical consequence: don't try to out-plan a
compiled library with a Python planner over `WorldSpec`; instead make sure
every backend can ingest the shared package, which is performance-neutral.

### Decision C — where does reachability live?

**Question.** Mustafa's point: reachability "must not need a mujoco
dependency; it sits higher, on the planning level." Is that right?

**Answer: yes, for the part that matters.** Reachability has two phases:
- *Query* ("is pose T reachable?") is a pure array lookup into a saved map —
  **zero backend dependency, zero mujoco.** This is the part that sits high
  on the planning level, and it already does.
- *Construction* (offline, minutes, once per robot) needs FK +
  self-collision sampling. **Decision:** sample through the `WorldSpec`
  interface rather than calling mujoco directly, so a map can be built on any
  backend. It's offline, so the Python-per-query speed ceiling is an
  acceptable one-time cost in exchange for backend-independence. (Today it
  calls `mujoco.MjSpec` directly; this is the refactor that lifts it above
  the backend.)

**What it entails.** The query/map layer ships dependency-free. Construction
gets re-expressed against `WorldSpec` FK + collision — a contained change to
the sampler, not a redesign.

---

## 8. Reachability as a worked consumer

With the architecture in place, reachability is "just another consumer," and
that framing is the point.

**What it is.** A capability map per G1 arm: in a gravity-aligned,
heading-quotiented **pelvis frame**, a discretized record of which
end-effector poses are reachable and how dexterously. The pelvis frame is
valid because the WBC gives a true SE(2) base, so a heading-free query means
"reachable, possibly after turning in place." It is RM4D-style but **5D** —
an explicit in-plane wrist dimension, because the G1's ±92.5° wrist yaw
breaks RM4D's 4D collapse (measured, IK-verified: 4.9% false-positive rate
for 5D vs 13.9% for the 4D marginal at equal recall).

**How it uses the contracts.** Construction samples arm FK + self-collision
through `WorldSpec` (§7-C), on the same model the sim uses, so "reachable in
the map" means "reachable in the world the robot actually inhabits." Query
needs none of that — it's a lookup.

**What it's for.** An *instant feasibility oracle*: a planner or a visualizer
asks "reachable here?" in microseconds instead of running IK. Near-term uses:
gate/seed planning targets; later, propose where the base should stand so a
target becomes reachable (stance selection). Deliverable artifacts: the
`.npz` maps, the green→red reachability plots, an IK-verified accuracy
report, and a one-shot inspection viewer.

**The presentation line:** reachability didn't need a new subsystem. It
needed a fast world (MujocoWorld), a shared scene (the package), and a place
to sit (above the planner). PimSim already provided all three. That is the
argument for the architecture — a genuinely new capability dropped in as one
more consumer.
