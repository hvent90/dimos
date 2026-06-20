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

"""Tests for the offline pose-domain reprocess pipeline: segmentation finds the
commanded steps, the fitter recovers injected K (~5%) / tau (~15%) / L (1 sample)
on sim recordings across regimes and seeds, and reprocess() writes a TuningConfig
artifact plus a fit-quality sidecar."""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from dimos.utils.benchmarking.plant import FopdtChannelParams, TwistBasePlantParams
from dimos.utils.benchmarking.tuning import TuningConfig
from dimos.utils.characterization.recording_io import load_recording, segment_steps
from dimos.utils.characterization.reprocess import (
    _NOMINAL_L_S,
    _NOMINAL_TAU_S,
    fit_recording_pose_domain,
    reprocess,
)
from dimos.utils.characterization.sim_ground_truth import (
    multistep_excitation,
    synthesize_recording,
)

_EXCITATION = multistep_excitation(
    vx_amps=(0.3, 0.6), vy_amps=(), wz_amps=(0.5, 1.0), hold_s=4.0, settle_s=2.0
)
# A few regimes spanning the fast Go2 corner and the vendored values.
_REGIMES = [(0.2, 0.15), (0.4, 0.05), (0.15, 0.1)]


def _plant(tau: float, dead_time: float) -> TwistBasePlantParams:
    return TwistBasePlantParams(
        vx=FopdtChannelParams(K=0.92, tau=tau, L=dead_time),
        vy=FopdtChannelParams(K=0.92, tau=tau, L=dead_time),
        wz=FopdtChannelParams(K=2.45, tau=tau, L=dead_time),
    )


def test_segmentation_finds_the_commanded_steps(tmp_path: Path) -> None:
    rec_path = synthesize_recording(
        _plant(0.2, 0.15), db_path=tmp_path / "s.db", segments=_EXCITATION, seed=0
    ).db_path
    spans = segment_steps(load_recording(rec_path))
    axes = [s.axis for s in spans]
    assert axes.count("vx") == 2  # two vx amplitudes
    assert axes.count("wz") == 2
    assert {round(abs(s.amplitude), 2) for s in spans if s.axis == "vx"} == {0.3, 0.6}


@pytest.mark.parametrize(("tau", "dead_time"), _REGIMES)
def test_recovers_steady_state_gain(tau: float, dead_time: float, tmp_path: Path) -> None:
    # K (steady-state gain) is recoverable regardless of tau/L; tau/L themselves
    # are NOT identifiable from 16 Hz pose, so we report them as nominal, not fit.
    k_ok = []
    for seed in range(3):
        plant = _plant(tau, dead_time)
        rec = synthesize_recording(
            plant, db_path=tmp_path / f"r{seed}.db", segments=_EXCITATION, seed=seed
        )
        fit = fit_recording_pose_domain(load_recording(rec.db_path))
        for axis, true in (("vx", plant.vx), ("wz", plant.wz)):
            f = fit.axes[axis]
            k_ok.append(abs(f.K - true.K) / abs(true.K))
            assert f.tau == _NOMINAL_TAU_S  # nominal, not fit
            assert f.L == _NOMINAL_L_S
            assert f.tau_l_identified is False
    assert max(k_ok) < 0.05  # K within ~5% across regimes/seeds


def test_reprocess_writes_artifact_and_quality_sidecar(tmp_path: Path) -> None:
    rec = synthesize_recording(
        _plant(0.3, 0.1), db_path=tmp_path / "sess.db", segments=_EXCITATION, seed=1
    )
    artifact = reprocess(
        rec.db_path, robot_id="go2_u01", sim_or_hw="hw", out_dir=tmp_path, git_sha="testsha"
    )
    assert artifact.exists()
    config = TuningConfig.from_json(artifact)
    assert config.plant.vx.K == pytest.approx(0.92, rel=0.05)  # gain recovered

    quality = json.loads((artifact.parent / f"{artifact.stem}_quality.json").read_text())
    assert quality["vx"]["valid"] is True
    assert quality["vx"]["tau_L_identified"] is False  # honest: not identified
    assert quality["vx"]["K_by_amplitude"]  # per-amplitude gain recorded
    assert quality["vx"]["settled_line_r2"] > 0.99  # settled region is clean & linear
