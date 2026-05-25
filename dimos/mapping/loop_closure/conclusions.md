# loop-closure autoresearch — conclusions

## TL;DR

Baseline `TOTAL_SPREAD = 48.655 m`. Tuned `pgo.py` to **22.289 m**, a
**-54.2%** reduction over 37 commits.

| dataset      | baseline | final | change |
|--------------|---------:|------:|-------:|
| hk_village1  |     5.83 |  6.20 |  +6%   |
| hk_village2  |    19.53 |  6.49 | **-67%** |
| hk_village3  |     5.00 |  3.78 |  -24%  |
| hk_village4  |    13.14 |  2.39 | **-82%** |
| hk_village5  |     2.80 |  2.14 |  -24%  |
| hk_village6  |     1.74 |  1.59 |   -9%  |

hk_village2 and hk_village4 were dominating the baseline metric; both
contain a clear "robot was disturbed" event mid-trajectory that the
unmodified loop-closure code couldn't bridge.

## The problem, after looking at it

Two failure modes drove the baseline number, both visible from the
per-dataset breakdown but only diagnosable by looking at the actual
data:

### 1. PGO can't relocalize through a meter-scale disturbance

`hk_village2` had drift growing from 0 to 4.7 m across one trajectory.
Loop closure search ran against optimized-frame positions with a 2 m
radius — once optimized drift exceeded 2 m, every nearby revisit fell
outside the search window. The robot kept revisiting the same physical
area, but the optimizer never saw a match. Marker spread for the only
multi-tracked marker (id 10) was 19 m, all because detections from after
the disturbance landed wherever the unconstrained chain put them.

Diagnostic script (`/tmp/hk2_marker10.py`) dumped per-detection raw +
corrected positions plus the nearest keyframe's drift. The drift
trace (`/tmp/drift_trace.py`) printed kf-by-kf when drift crossed
threshold. These were the two scripts that made every subsequent
tuning decision obvious.

### 2. Chain factors were overconfident

Inter-keyframe BetweenFactors had translation variance `1e-4` (sigma
~1cm). Accumulated over 262 keyframes that implies a total uncertainty
of ~16 cm — but actual drift in hk_village2 hit 4 m. PGO under-corrected
even when good loop closures fired, because the chain factors fought
back. Forensic comparison of marker raw positions vs. PGO-corrected
positions showed the correction matched the *measured* drift but
under-corrected the *true* drift by ~36%.

## What worked (in order they were found)

Each line in the table is roughly a real win — bigger pieces of the
total improvement at the top.

| change | spread |
|---|---:|
| baseline | 48.66 |
| tighten loop rot_var 0.05 → 0.01 then → 0.003 | 47.75 → 37.98 |
| `loop_submap_half_range` 10 → 40 (more ICP context) | 39.92 |
| tighten ICP `max_correspondence_dist` 1.0 → 0.5 | 38.82 |
| drift-gated local-frame fallback (radius 15 m, drift > 0.5 m) | 34.06 |
| multi-loop accept inside fallback (K=7 candidates) | 31.40 |
| `min_loop_detect_duration` 5s → 3s | 33.66 → 31.26 |
| **drift-aware chain variance** (8e-4 if prev kf drift > 1.3 m) | 27.39 |
| nearest-neighbor interp (vs linear blend) | 27.33 |
| ISAM2 Dogleg optimizer | 27.33 |
| trans_var floor 0.01 → 0.005 | 30.24 |
| **second-pass loop rescan** over converged optimized poses | 24.62 |
| rescan opt-only (no fallback) + `submap_half_range=38` | 22.60 |
| rescan `time_thresh_override=13s` | **22.29** |

The two structural changes (chain-variance-by-drift and second-pass
rescan) account for most of the improvement. The rest is parameter
tuning at their joint sweet spot, which is narrow — moving any one knob
±20% usually regressed by 1 m+.

## What didn't work (a non-exhaustive list)

So future tuners don't repeat these:

- **Loosening trajectory variance globally** (1e-4 → 1e-3, 5e-4, etc.) —
  helps hk2 dramatically (sometimes drops it to 8 m) but blows up
  hk1/hk5/hk6 because chain becomes effectively unconstrained.
- **RANSAC + FPFH global registration** for fallback init — admits
  wrong alignments (geometrically similar but different actual places)
  that the optimizer can't reject.
- **Coarse-to-fine ICP** for fallback — same problem; coarse pass found
  wrong-but-locally-tight alignments.
- **ICP init from local-frame relative pose** — marginal at best,
  occasionally worse.
- **Huber robust kernel on loop factors** (both global and
  fallback-only) — down-weights good loops along with bad ones.
- **Forcing multi-loop accept everywhere** (not just fallback) — adds
  redundant constraints in easy datasets, increases noise.
- **Two-pass rescan** (run rescan twice) — over-constrains.
- **Tighter chain factors when many loops exist** — they keep being the
  bottleneck even with abundant loop evidence.
- **Variations on the drift threshold for chain loosening** (1.0, 1.2,
  1.4, 1.5, 1.7, 1.8, 2.0, 3.0, multi-tier) — 1.3 m is sharply optimal.
- **Variations on rescan submap_half_range** (20, 30, 33, 35, 36, 37,
  39, 40, 50, 80) — 38 is sharply optimal; first-pass needs 40, not 38.
- **Rescan with wider opt search radius, force-fallback, smaller source
  submap, force-multi-loop, tighter ICP correspondence, lower
  candidate count, tighter early-exit** — all worse.
- **Windowed-mean / Catmull-Rom interpolation in the corrections** —
  nearest-neighbor wins; the noise isn't in interp blending.
- **Final batch LM re-optimization after ISAM2** — ISAM2 was already
  converged; LM finds the same answer.

## How the process went

The first half of the experiment was parameter twiddling — small wins
adding up to ~10 m of improvement, then a hard plateau around 27–28 m
where every knob was at its joint optimum and ±20% perturbations
regressed.

The breakthroughs all came from **investigation runs that didn't change
`pgo.py`**:

1. Per-dataset breakdown of spread (already in the eval table). Showed
   hk_village2 + hk_village4 were 68% of the baseline metric.
2. `_diag.py` / `hk2_marker10.py` — per-marker pose dump. Revealed
   marker 10 in hk_village2 had detections 5 m apart in the corrected
   frame, all in the disturbance area. Concretely showed the
   relocalization-after-disturbance failure mode.
3. `drift_trace.py` — kf-by-kf drift trace. Showed when each dataset
   crossed which drift threshold and how long it stayed there;
   directly motivated the "drift-aware chain variance" change.
4. Comparing raw vs. corrected marker positions plus the correction
   translation. Showed the correction was matching the *measured*
   drift but falling 36% short of the true drift — pointed straight
   at the chain-factor overconfidence problem.

Each of these scripts ran in 30–60s, didn't count toward the
experiment log, and led to a 1–3 m improvement that knob tuning hadn't
been finding. The second-pass rescan idea came from "we converged once,
the optimized poses are now actually trustworthy, what would loop
detection do if we re-ran it?" — and that single structural change
dropped the metric by ~3 m.

The final ~2 m came from narrow-window tuning of the rescan
parameters: `submap_half_range=38` (not 37 or 39) and
`time_thresh_override=13s` (not 12 or 14). At this point most
perturbations regress by 0.1–2 m, suggesting we're at a real local
optimum and further gains likely need a different structural change
(e.g., refining the marker_transformer integration, which is out of
scope here).

## What's still on the table

- **Marker_transformer rotation accuracy.** Forensic of hk_village2
  marker 10 shows det 3 (the worst remaining) has 1.4 m of x-error
  that doesn't move with chain or loop tuning. The robot saw the
  marker from different angles at det 1 vs det 3, and small yaw drift
  in the optimized pose projects to large marker-position error
  because the marker is ~1 m from the camera. PGO can't fix the
  camera-to-marker geometry; this is approaching the noise floor.
- **Per-recording adaptive parameters.** The current params are a
  global optimum across all 6 recordings; recordings 1 and 2/4 have
  pretty different dynamics. A per-recording tuner would probably
  squeeze another 1–2 m total but isn't well-aligned with deployment.

## Code shape

The final `pgo.py` is bigger than the original (the `_search_for_loops`
function in particular took on several optional kwargs to support the
rescan code path). Worth a cleanup pass before merging — the original
inline code path stayed for the first pass; only the rescan path needs
the extra knobs. A targeted refactor could pull rescan into its own
method and drop most of the kwargs.

## What data would have helped next time

Some signals would have shortened the loop substantially:

- **Ground-truth marker positions.** A `markers.csv` with the true
  world position of each `marker_id` per recording. Right now the eval
  scores "consistency across detections of the same marker", which
  only tells you when corrections diverge — not whether they converge
  to the *right* place. Half my forensic time was spent inferring true
  positions from the most consistent cluster. With ground truth you
  could also distinguish "PGO under-correcting" from "marker detection
  is wrong" without having to reason about it.
- **Ground-truth trajectories.** Even just one or two recordings with
  pose ground truth (e.g., logged from a more accurate localizer)
  would let you compute keyframe-pose error directly, instead of using
  marker-spread as a proxy. The proxy is informative but indirect:
  there were several experiments where loop count and quality both
  improved but spread stayed flat because the underlying poses moved
  along the marker viewing rays.
- **Annotation of disturbance events.** Marking the timestamps in each
  recording where the robot got pushed / lifted / kicked would have
  saved a lot of inference. I built `drift_trace.py` to detect these
  from PGO output, but the timestamps would have been cleaner ground
  truth and would let an automated tuner segment "easy" from
  "disturbed" trajectory regions.
- **Loop closure log structured beyond `[inf]` log lines.** Per-loop
  records of (source kf, target kf, score, was-fallback, accepted-vs-
  rejected-by-ICP) emitted as JSONL or a sqlite table would have made
  many forensics one-liner queries instead of grep + Python parsing.
- **The eval's per-marker breakdown printed by default**, not derived
  from a custom diag script. The dataset row tells you which recording
  is hurting; one more level of detail (which marker, how many
  detections, max pair distance) would have pointed at hk_village2
  marker 10 in the first 5 minutes instead of after several
  knob-tuning rounds.

## What approach would help next time

Roughly in order of expected payoff:

1. **Spend the first iteration on tooling, not tuning.** I tuned for
   ~20 commits before writing the first diagnostic script and got the
   biggest single wins (drift-aware chain, second-pass rescan) only
   *after* the forensics existed. The program.md update mid-experiment
   ("Investigation is part of the loop") was the right correction. The
   diagnostics paid back their cost in the first iteration they were
   used.
2. **Run dimos map with rerun for the worst recording before
   touching anything.** Even a 5-minute look at the actual trajectory,
   marker positions, and loop closure edges would have made the
   disturbance failure mode obvious and saved several speculative
   tuning rounds. I never actually opened rerun this session and the
   most important visualization was reconstructed badly from log
   parsing.
3. **State a hypothesis before each change, log the prediction, then
   check.** I did this informally in the responses to the user but
   the experiment log records *what changed and the resulting
   spread*, not *what was predicted*. When a change regresses, the
   prediction-vs-reality gap is the actual learning; without it you
   only learn "that one didn't work."
4. **Search the loose-chain / drift-threshold parameter space jointly
   with a grid, not greedy.** The threshold and the loose variance
   value interact (1.5/5e-4 worked, 1.3/8e-4 worked better; some
   combinations weren't tested because the greedy walk didn't go
   there). A 3×3 or 5×5 grid would have taken 25 evals (≈40 min) and
   probably found the joint optimum directly. I spent more time than
   that on greedy walks that revisited the same region.
5. **Try the structural change ("rescan with converged poses") much
   earlier.** It was obvious in retrospect — if loop search depended
   on the optimized poses being accurate, and those poses are most
   accurate after PGO converges, then re-searching after convergence
   *had to* find more loops. I sat on this for many iterations because
   it felt like a bigger change. The cost was ~30 minutes of
   refactoring `_search_for_loops` to take a `cur_idx`, and the
   payoff was the largest single improvement.
6. **Keep a "did not work" register that's easier to grep than the
   results.tsv `discard` rows.** Several ideas resurfaced under
   different names (e.g., "loosen rot_var in fallback" got tried
   three times across iterations). Cross-referencing against past
   attempts before each new experiment is cheap; not doing it is
   compounding waste.
7. **Resist twiddling the same knob multiple times.** Once a
   parameter is at a local optimum, ±1 unit perturbations are usually
   in the noise; the energy is better spent on a structural change
   somewhere else. I did several rounds of "try 1.3 / try 1.4 / try
   1.5" once a knob hit its sweet spot, and the wins from those
   sub-percent moves were almost always reverted by the next
   experiment downstream.
