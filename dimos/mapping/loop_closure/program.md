# loop-closure autoresearch

Adapted from [karpathy/autoresearch](https://github.com/karpathy/autoresearch).
The agent iteratively tunes ICP / pose-graph code to minimize marker
position drift across recorded sessions.

| nanochat | here |
|---|---|
| `train.py` (edit) | `pgo.py` |
| `prepare.py` (read-only) | `eval.py` |
| `val_bpb` (lower better) | `TOTAL_SPREAD` (lower better) |
| 5-min wall budget | data-bounded, ~2 min sequential |

## Setup

1. **Agree on a run tag**: propose one based on today's date (e.g.
   `mar5`). The branch `autoresearch/loopclose-<tag>` must not already
   exist — this is a fresh run.
2. **Create the branch**: `git checkout -b autoresearch/loopclose-<tag>`
   from current `main`.
3. **Read the in-scope files**:
   - `dimos/mapping/loop_closure/pgo.py` — PGO + ICP + loop detection.
     This is the file you edit.
   - `dimos/mapping/loop_closure/eval.py` — the eval harness. Read it to
     understand the metric; do not modify.
   - `dimos/perception/fiducial/marker_transformer.py` — fiducial
     detector. Held fixed for fair comparison; do not modify.
4. **Verify LFS data**: the first eval run will pull
   `hk_village1..6.db` from LFS automatically via `get_data`. If LFS
   isn't configured, tell the human to run `git lfs install && git lfs pull`.
5. **Initialize `results.tsv`**: header row only. The baseline goes in
   after the first eval.
6. **Confirm and go**: confirm setup looks good.

Once you get confirmation, kick off experimentation.

## Experimentation

**What you CAN do:**
- Modify `dimos/mapping/loop_closure/pgo.py` — anything in this file is
  fair game: `PGOConfig` defaults, loop-detection logic, ICP setup,
  GTSAM noise models, keyframe selection rules, the inner classes,
  whatever.

**What you CANNOT do:**
- Modify `eval.py` or any of its dependencies (`marker_transformer.py`,
  detection types, the recordings). The eval harness is the ground
  truth metric.
- Change the **public surface** of `pgo.py`. The signatures of
  `pgo_keyframes`, `keyframes_to_corrections`, `make_interpolator`, and
  `apply_corrections` must stay the same so `eval.py` (and downstream
  consumers) keep working. You can change their internals freely.
- Add new dependencies or modify other files in the repo.

**The goal: lowest `TOTAL_SPREAD` (meters).** Secondary: lower
`WALL_TIME` (seconds) for ties. The metric is per-recording sum of
pairwise distances between PGO-corrected marker positions for the same
`marker_id`, summed across all six `hk_village*` recordings. Smaller =
tighter loop closures = the same physical marker is placed in the same
world spot every time the tracker re-acquires it.

**Simplicity criterion**: All else being equal, simpler wins. A small
spread improvement that adds 30 lines of GTSAM hackery is probably not
worth it. Deleting code and getting equal or better results is a great
outcome.

**The first run**: establish the baseline with `pgo.py` unmodified.

## Running the eval

```bash
uv run python -m dimos.mapping.loop_closure.eval > run.log 2>&1
grep "^TOTAL_\|^WALL_" run.log
```

Output trailer looks like:

```
TOTAL_PGO_TIME=37.86
TOTAL_SPREAD=48.811     ← primary metric, lower is better
TOTAL_LOOPS=42
TOTAL_LOOP_SCORE_MEAN=0.0584
TOTAL_KEYFRAMES=720
WALL_TIME=115.82        ← secondary metric, lower is better
```

## Logging results

Log every experiment to `results.tsv` (tab-separated). Do **not**
commit this file — it's per-branch scratch state, in `.gitignore`-style
spirit. The header row plus seven columns:

```
commit  spread_m  wall_s  n_loops  mean_score  n_keyframes  status  description
```

1. git commit hash (short, 7 chars)
2. `TOTAL_SPREAD` — use `0.000000` for crashes
3. `WALL_TIME` — use `0.0` for crashes
4. `TOTAL_LOOPS`
5. `TOTAL_LOOP_SCORE_MEAN` (`.4f`)
6. `TOTAL_KEYFRAMES`
7. status: `keep`, `discard`, or `crash`
8. one-line description of what this experiment tried

Example:

```
commit   spread_m   wall_s   n_loops   mean_score   n_keyframes   status    description
a1b2c3d  48.811     115.82   42        0.0584       720           keep      baseline
b2c3d4e  43.220     118.31   55        0.0501       720           keep      lower loop_score_thresh 0.1 -> 0.05
c3d4e5f  47.118     119.04   30        0.0623       720           discard   raise min_icp_inliers 50 -> 200
d4e5f6g  0.000000   0.0      0         0.0000       0             crash     drop GTSAM, use simple averaging
```

## The experiment loop

The experiment runs on a dedicated branch (e.g.
`autoresearch/loopclose-mar5` or
`autoresearch/loopclose-mar5-gpu0`).

LOOP FOREVER:

1. Look at the git state: the current branch and commit.
2. Tune `pgo.py` with an experimental idea by editing the code
   directly.
3. `git commit -am "<one-line description>"`
4. Run the eval:
   `uv run python -m dimos.mapping.loop_closure.eval > run.log 2>&1`
   (redirect everything — do NOT `tee` and don't dump the file into
   your context).
5. Read the result:
   `grep "^TOTAL_\|^WALL_" run.log`
6. If the grep output is empty, the run crashed. Run
   `tail -n 80 run.log` to see the Python traceback and either fix it
   (typo, missing import) or revert (idea fundamentally broken).
7. Record the result in `results.tsv` (keep it untracked).
8. Decision:
   - `TOTAL_SPREAD` strictly **lower** than the current branch tip →
     `keep` (advance the branch).
   - `TOTAL_SPREAD` within ±0.5% of current AND `WALL_TIME` lower →
     `keep` (compute simplification).
   - Otherwise → `discard` (`git reset --hard HEAD~1`).

You're a fully autonomous researcher. If something works, keep it. If
it doesn't, revert. The branch advances over time, and you iterate.

**Timeout**: each eval should take ~2 minutes wall. If a run exceeds
10 minutes, kill it and treat it as a failure.

**Crashes**: If a run crashes (GTSAM error, Open3D ICP failure,
import error, etc.), use judgment. Typos and missing imports → fix and
re-run. Fundamentally broken idea → log `crash` and move on.

**NEVER STOP**: Once the loop has begun, do NOT pause to ask the human
if you should continue. Don't ask "should I keep going?" or "is this a
good stopping point?". The human might be asleep or away. You are
autonomous; the loop runs until you are manually stopped. If you run
out of ideas: re-read `pgo.py` for new angles, look at the papers/code
links in its docstring, try combining previous near-misses, try more
radical changes (different loop-detection radius scaling, different
ICP estimator, swapping GTSAM optimizer, etc.).

A typical use case: human leaves you running while they sleep. At
~2 min/eval you can run ~30/hour, or roughly 240 experiments over an
8-hour night. They wake up to a `results.tsv` full of attempts and a
branch advanced to the best.
