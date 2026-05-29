# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Bayesian (TPE) hyperparameter search over the Point-LIO CONFIG — parallel and
history-aware. Same objective as search.py (SA), but driven by Optuna:

  - a TPE surrogate proposes each config from ALL past trials (sample-efficient,
    not memoryless like SA);
  - the study is persisted to optuna_study.db (sqlite), so re-running RESUMES the
    full trial history — it genuinely remembers the collected data;
  - trials run concurrently (--jobs); each writes its own trajectory via
    algo.run(out_path=...) (needs the binary's --out flag).

Shares the search space + best_config.json with search.py. Run:
    python search_optuna.py --trials 200 --jobs 6
Resume (keeps all prior trials): just run it again.
"""

import argparse
import json
import math
import os
import subprocess

import algo
import evaluate
import optuna
from search import BEST_PATH, PENALTY, SPACE  # one source of truth for the space

STUDY = "pointlio_ate"
STORAGE = f"sqlite:///{os.path.join(evaluate.HERE, 'optuna_study.db')}"


def suggest(trial):
    """Sample a CONFIG override dict from the trial, honoring each knob's kind."""
    o = {}
    for name, kind, lo, hi in SPACE:
        if kind == "log":
            o[name] = trial.suggest_float(name, lo, hi, log=True)
        elif kind == "int":
            o[name] = trial.suggest_int(name, lo, hi)
        else:
            o[name] = trial.suggest_float(name, lo, hi)
    return o


def objective(trial, timeout=None):
    overrides = suggest(trial)
    # Per-trial paths so concurrent trials don't clobber each other.
    yaml_path = os.path.join(evaluate.POINTLIO_DIR, "config", f"_trial_{trial.number}.yaml")
    out_path = os.path.join(evaluate.POINTLIO_DIR, "Log", f"trial_{trial.number}.txt")
    try:
        return algo.run(overrides, yaml_path=yaml_path, out_path=out_path, timeout=timeout)[
            "val_ate_xy"
        ]
    except subprocess.TimeoutExpired:
        return PENALTY  # finite penalty (not pruned) so TPE learns the region is bad
    except Exception:
        return PENALTY
    finally:
        for p in (yaml_path, out_path):
            try:
                os.remove(p)
            except OSError:
                pass


def main():
    ap = argparse.ArgumentParser(description="Parallel TPE (Optuna) tuning of Point-LIO CONFIG.")
    ap.add_argument("--trials", type=int, default=200, help="point_lio runs this invocation")
    ap.add_argument(
        "--jobs",
        type=int,
        default=min(6, os.cpu_count() or 1),
        help="concurrent trials (each binary also uses OpenMP threads — watch oversubscription)",
    )
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument(
        "--timeout",
        type=float,
        default=240,
        help="kill (and penalize) a trial whose binary runs longer than this; "
        "healthy runs are ~1-3 min, so the default 240s culls pathological configs early",
    )
    args = ap.parse_args()

    if not os.path.exists(evaluate.POINTLIO_BIN):
        raise SystemExit("pointlio binary not built — run ./setup.sh first.")

    study = optuna.create_study(
        study_name=STUDY,
        storage=STORAGE,
        direction="minimize",
        sampler=optuna.samplers.TPESampler(seed=args.seed),
        load_if_exists=True,  # resume the persisted history
    )
    # Warm-start a fresh study with known-good points so TPE doesn't start cold:
    # the v2_imu baseline, and the best config found so far (shared with SA).
    if not study.trials:
        study.enqueue_trial({n: algo.CONFIG[n] for n, *_ in SPACE})
    if os.path.exists(BEST_PATH):
        try:
            best = json.load(open(BEST_PATH)).get("overrides", {})
            study.enqueue_trial({n: best[n] for n, *_ in SPACE})
        except Exception:
            pass

    optuna.logging.set_verbosity(optuna.logging.WARNING)
    print(
        f">> Optuna TPE: {len(study.trials)} prior trials in study, +{args.trials} this run, {args.jobs} parallel"
    )
    print(f">> persisted study: {STORAGE}")

    done = [0]

    def cb(study, trial):
        done[0] += 1
        v = trial.value if trial.value is not None else float("nan")
        print(
            f"[{done[0]:4d}/{args.trials}] trial#{trial.number} val_ate_xy={v:10.4f}  best={study.best_value:.4f}",
            flush=True,
        )

    study.optimize(
        lambda trial: objective(trial, timeout=args.timeout),
        n_trials=args.trials,
        n_jobs=args.jobs,
        callbacks=[cb],
    )

    print("\n=== search done ===")
    print(f"total trials in study: {len(study.trials)}")
    print(f"best val_ate_xy:       {study.best_value:.6f}  (baseline ~10.97)")

    # Global best across SA + Optuna runs: only overwrite best_config.json if beaten.
    best = dict(study.best_params)
    prev = json.load(open(BEST_PATH)) if os.path.exists(BEST_PATH) else None
    if prev is None or study.best_value < prev.get("val_ate_xy", math.inf):
        with open(BEST_PATH, "w") as f:
            json.dump(
                {"val_ate_xy": study.best_value, "study": STUDY, "overrides": best}, f, indent=2
            )
        print(f"new global best — wrote {BEST_PATH}")
    else:
        print(
            f"kept existing {BEST_PATH} (stored {prev['val_ate_xy']:.4f} ≤ this run {study.best_value:.4f})"
        )

    # Re-run the best to leave viz.png + traj_ds.tsv for it.
    try:
        algo.run(best, render=True)
    except Exception as e:
        print(f"(best re-render skipped: {e})")


if __name__ == "__main__":
    main()
