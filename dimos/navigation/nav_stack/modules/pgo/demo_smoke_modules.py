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

"""Fast shape-check: drive every eval_all algorithm through the synthetic
lockstep harness (no recording needed) and report keyframes + closures.

Proves each module deploys, starts, processes scans, and emits a non-degenerate
pose graph through the same ports the eval harness uses — without waiting on a
full recording replay. Not a pytest (demo_ prefix); run directly:

    uv run python dimos/navigation/nav_stack/modules/pgo/demo_smoke_modules.py
"""

from __future__ import annotations

from pathlib import Path
import time

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.navigation.nav_stack.modules.pgo.eval_all import ALGORITHMS
from dimos.navigation.nav_stack.modules.pgo.eval_utils.module_loading import (
    filter_config_for_module,
    load_module_class,
)
from dimos.navigation.nav_stack.modules.pgo.lockstep_harness import (
    GraphCapture,
    SyntheticLockstepReplay,
    trajectory_payload,
    trajectory_reverse_loop,
)

# A physical (un-drifted) out-and-back loop: every algorithm — position-search
# or scan-context — has a fair shot at closing it.
TRAJECTORY = trajectory_reverse_loop()

# Superset of loop-closure knobs; filtered to each module's declared fields.
SMOKE_CONFIG = {
    "use_scan_context": True,
    "key_pose_delta_trans": 0.4,
    "loop_search_radius": 3.0,
    "loop_time_thresh": 5.0,
    "loop_score_thresh": 1.0,
    "loop_submap_half_range": 5,
    "submap_resolution": 0.1,
    "min_loop_detect_duration": 1.0,
    "global_map_publish_rate": 0.0,
    "publish_global_map": False,
    "unregister_input": True,
    "scan_context_max_range_m": 30.0,
    "scan_context_match_threshold": 0.6,
    "loop_min_degeneracy": 0.0,
    "loop_min_occupancy": 0,
    "drain_stale_scans": False,
}

POLL_INTERVAL_SEC = 0.25
POST_FEED_DRAIN_SEC = 3.0


def run_one(module_path: Path, label: str) -> dict[str, object]:
    module_class = load_module_class(module_path, "PGO")
    config = filter_config_for_module(module_class, dict(SMOKE_CONFIG))
    blueprint = autoconnect(
        SyntheticLockstepReplay.blueprint(trajectory=trajectory_payload(TRAJECTORY)),
        module_class.blueprint(**config),  # type: ignore[attr-defined]
        GraphCapture.blueprint(),
    )
    coordinator = ModuleCoordinator.build(blueprint)
    try:
        replay = coordinator.get_instance(SyntheticLockstepReplay)
        capture = coordinator.get_instance(GraphCapture)
        while not replay.is_finished():
            time.sleep(POLL_INTERVAL_SEC)
        error = replay.error()
        time.sleep(POST_FEED_DRAIN_SEC)
        return {
            "label": label,
            "keyframes": capture.keyframes(),
            "closures": capture.closures(),
            "replay_error": error,
        }
    finally:
        coordinator.stop()


def main() -> None:
    results = []
    for algorithm in ALGORITHMS:
        print(f"\n========== smoke: {algorithm.name} ==========", flush=True)
        try:
            result = run_one(Path(algorithm.module_path), algorithm.name)
        except Exception as exc:
            result = {"label": algorithm.name, "error": f"{type(exc).__name__}: {exc}"}
        results.append(result)
        print(result, flush=True)

    print("\n================ SUMMARY ================")
    for result in results:
        print(result)


if __name__ == "__main__":
    main()
