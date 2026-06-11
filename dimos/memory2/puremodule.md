# Pure Modules — design notes

Usage, tutorial, and the exact rules live in
[docs/usage/pure_modules.md](/docs/usage/pure_modules.md). This page
records the *why* — the reasoning behind the design decisions and the
plans for what isn't built yet. Read it before changing the
implementation in [puremodule.py](puremodule.py) / [tick.py](tick.py) /
[health.py](health.py).

## Why ticks

If modules are stateless — or their state is fed externally, react/redux
style — replay, time-travel, live migration, restarts that resume, and
parallel execution stop being features built into each module and become
properties of the runtime. The blocker for robotics is that "call the
module on its inputs" is ill-posed when sensors don't share a clock: the
declaration must say *when* the module runs (the tick) and *how* every
other input is sampled at that moment (`latest` / `interpolate` /
`window`). The sampler language is the smallest vocabulary we found that
covers the real cases (pose-at-image-time, hold-with-expiry, IMU
batching); `every(hz)` clock ticks and multi-input triggers are deferred
until a concrete module needs them.

One machine drives both modes: the `TickMachine` is a plain
events-in/rows-out state machine, fed from a timestamp-ordered merge
offline (exact, deterministic) and from an arrival-ordered queue live
(best-effort under jitter). Keeping it free of threads and streams is
what makes alignment unit-testable.

## Backpressure: the tick is the unit of load

The system has two regimes, and the store converts between them. Pull
(offline): backpressure is intrinsic — the consumer's iteration is the
clock and nothing accumulates beyond pruned alignment buffers. Push
(live): sensors can't be paused, so backpressure must be a declared
drop/coalesce policy, and the tick is the right unit — secondaries are
cheap to ingest, all the expense is `step()`. Hence the
`BackpressureBuffer` between the alignment thread and the step thread,
speaking the existing `buffer.py` vocabulary rather than inventing one.

The invariant to preserve when changing the live path: **every queue is
bounded** — the tick buffer by policy, alignment buffers by pruning
(including the dead-trigger case, 1 s arrival-jitter slack), pending
ticks by `max_pending_ticks` (a dead `interpolate()` input must not
accumulate ticks), and the monitor's reservoirs by fixed-size deques.

## Health: drops are metrics, not errors

Under `KeepLast` a controller dropping most ticks is the system working
as designed, so per-drop warnings are categorically wrong. The ladder:
count always (by reason — `backpressure`, `missing_input`, `blocked` are
three different problems: slow step, dead sensor, clock skew), report
continuously (the `_health` stream rides the same store as the data, so
recordings capture health next to the frames it explains), log on state
transitions only, alert on declared contracts. The real SLO is output
freshness and rate; drop counters are diagnosis. Contracts split
deliberately: semantic tolerances (`max_age`, `tolerance`) belong in the
declaration because they're algorithm truths; rates (`expected_hz`,
`min_output_hz`) belong in deployment config because sim, replay, and
the robot legitimately differ.

## Replay fidelity under drops (planned: record tick rows)

With a dropping policy, a live run processes a *subsample* of triggers,
so replaying raw inputs offline (which processes all of them) diverges
for stateful modules. The fix is to record the **resolved tick rows** —
the aligned inputs actually consumed — making replay-of-a-run exact by
construction, drops and all. This is the prerequisite for trusting
time-travel on stateful modules and should land before production
relies on them.

## State persistence (planned: the journal design)

Today, Mealy state lives in a loop variable — initialized from
`initial_state`, threaded by the runtime, gone when the run ends.
Deliberately not on `self` (concurrent `over()` runs stay independent),
deliberately not yet persisted. The plan:

- **Snapshots are a stream.** The runtime appends post-tick state to a
  `_state` stream in the module store (like `_health`), on a cadence
  policy — every tick for small states, every N seconds for big ones,
  on-stop minimum. Store choice = persistence policy; codecs, ts
  indexing, and replay tooling already exist.
- **The DB is a journal, not the hot path.** The working copy stays in
  memory; appends are write-through; reads happen only at start or seek.
  No round-trips inside a control loop.
- **What it buys**: resume (`start()` loads `_state.last()` under a
  `resume` config), migration (the snapshot is a value in a file),
  time-travel (snapshots are checkpoints, the tick log is the WAL —
  `state = fold(step, ticks)`, seek = load snapshot ≤ T + replay),
  counterfactual debugging (replay from a snapshot with edited inputs or
  edited step code). `state` is a reserved input name, so
  `over(state=snapshot, pose=db.pose.after(t0))` is collision-free.
- **Contract on state values**: plain serializable data (dataclass /
  LCM message / numpy — an LCM-typed state gets cross-language replay),
  treated as immutable (`step` returns new state; serializing at append
  time is the aliasing fix), sized for its cadence.
- **Endgame**: keyed state (e.g. per-marker buffers) shards — the
  runtime partitions ticks by key across processes, each owning a
  shard. Only possible because state is a declared value the runtime
  owns.

## Multi-output offline shape (planned: run handle)

Offline, multi-output modules yield `{port: value}` dict rows while live
publishes per-port — an asymmetry. The planned fix is a run handle:
`run = M.over(..., store=...)` executes once into a store and exposes one
stream per output (`run.detections`, `run.alerts`), independently
re-iterable and queryable. Materializing through a store also makes
offline structurally identical to live (both are "module + store") and is
the substrate a future module-graph would build on. The lazy dict-row
form stays for single-pass pipelines.

## Deliberately deferred

- `every(hz)` clock triggers and multi-input triggers (`on_any`).
- A live timeout policy for `interpolate()` when its input dies
  (currently ticks wait until evicted; shutdown resolves via the
  nearest-fallback).
- Live-side input gating — offline, gating composes onto the input
  stream (`over(color_image=imgs.transform(QualityWindow(...)))`); live
  has no per-port hook yet (chain a gating module). Possibly
  `tick(via=...)`.
- Modules that *query* memory (semantic search) — impure capability,
  stays on `MemoryModule`.
- `Annotated[In[X], sampler]` syntax — core `Module` introspection
  doesn't unwrap `Annotated`, so ports would silently not be created;
  the default-value syntax is canonical.
- `Recorder` subsumption — a recorder is a PureModule deployment with a
  storage-backed store and no step; fold once the API is stable.
