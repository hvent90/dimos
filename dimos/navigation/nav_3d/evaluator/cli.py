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

"""Nav-3d evaluation CLI.

Run every suite:      python -m dimos.navigation.nav_3d.evaluator run
One dataset:          python -m dimos.navigation.nav_3d.evaluator run --dataset stairs60_a
Machine output:       python -m dimos.navigation.nav_3d.evaluator run --json report.json
Override a knob:      python -m dimos.navigation.nav_3d.evaluator run --set wall_clearance_m=0.05
"""

from __future__ import annotations

import dataclasses
import json
from pathlib import Path

import typer

from dimos.navigation.nav_3d.evaluator.cases import load_suites
from dimos.navigation.nav_3d.evaluator.config import EvalConfig
from dimos.navigation.nav_3d.evaluator.runner import Report, evaluate

app = typer.Typer(no_args_is_help=True, add_completion=False)


def _apply_overrides(cfg: EvalConfig, overrides: list[str]) -> EvalConfig:
    fields = {f.name: f.type for f in dataclasses.fields(EvalConfig)}
    for spec in overrides:
        if "=" not in spec:
            raise typer.BadParameter(f"--set expects name=value, got {spec!r}")
        name, value = spec.split("=", 1)
        if name not in fields:
            raise typer.BadParameter(f"unknown config field {name!r}")
        current = getattr(cfg, name)
        setattr(cfg, name, type(current)(value))
    return cfg


def _print_report(report: Report) -> None:
    header = f"{'case':<28} {'dataset':<22} {'attr':<8} {'spl':>5} {'len':>6} {'ref':>6} {'ms':>7}"
    print(header)
    print("-" * len(header))
    for d in report.datasets:
        for c in d.cases:
            print(
                f"{c.id:<28} {c.dataset:<22} {c.attribution:<8} "
                f"{c.online.spl:>5.2f} {c.online.length:>6.1f} {c.l_ref:>6.1f} "
                f"{c.online.plan_ms:>7.1f}"
            )
    print("-" * len(header))
    for d in report.datasets:
        print(
            f"{d.dataset}: {d.frames} frames, online {d.online_voxels} / "
            f"golden {d.golden_voxels} voxels, "
            f"false-obstacle rate {d.false_obstacle_rate:.3f}, "
            f"map build {d.map_build_ms / 1000:.1f}s"
        )
    print(
        f"\nscore {report.score:.3f} | soft {report.score_soft:.3f} | "
        f"planner {report.planner_score:.3f} | "
        f"success {report.n_success}/{report.n_cases} | "
        f"attribution {report.attribution_counts} | "
        f"plan p95 {report.plan_ms['p95']:.1f}ms"
    )


@app.command()
def run(
    manifests: list[Path] = typer.Argument(
        None, help="Suite YAMLs; defaults to every manifest under cases/"
    ),
    dataset: str = typer.Option(None, "--dataset", help="Only run suites for this dataset"),
    json_out: Path = typer.Option(None, "--json", help="Write the full report as JSON"),
    rrd_out: Path = typer.Option(None, "--rrd", help="Write a rerun recording of every case"),
    workers: int = typer.Option(1, "--workers", help="Datasets evaluated in parallel processes"),
    set_: list[str] = typer.Option(
        None, "--set", help="Repeatable EvalConfig override, e.g. wall_clearance_m=0.05"
    ),
) -> None:
    suites = load_suites(manifests or None)
    if dataset is not None:
        suites = [s for s in suites if s.dataset == dataset]
        if not suites:
            raise typer.BadParameter(f"no suite for dataset {dataset!r}")
    cfg = _apply_overrides(EvalConfig(), set_ or [])
    report = evaluate(suites, cfg, workers=workers)
    _print_report(report)
    if json_out is not None:
        json_out.write_text(json.dumps(report.to_dict(), indent=2))
        print(f"wrote {json_out}")
    if rrd_out is not None:
        from dimos.navigation.nav_3d.evaluator.viz import write_rrd

        write_rrd(report, suites, cfg, rrd_out)


@app.command("list")
def list_cases() -> None:
    for suite in load_suites():
        print(f"{suite.dataset} ({suite.path.name if suite.path else '?'})")
        for case in suite.cases:
            tags = f" [{', '.join(case.tags)}]" if case.tags else ""
            print(
                f"  {case.id}: {tuple(round(v, 2) for v in case.start)} -> "
                f"{tuple(round(v, 2) for v in case.goal)} w={case.weight:g}{tags}"
            )


if __name__ == "__main__":
    app()
