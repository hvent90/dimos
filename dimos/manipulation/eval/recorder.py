# Copyright 2025-2026 Dimensional Inc.
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

"""Crash-safe episode recorder for manipulation pick-and-place benchmarks.

This records one *episode outcome* per pick-and-place attempt (success, typed
error code, per-skill durations, inferred stage flags) as a single JSON line.
It is deliberately tiny — open a file, write-and-flush one dict per episode,
close — with no session directories, SQLite, dual-clock anchoring, or
post-processing pipeline (the lessons from the heavier go2 ``session.py``).

Note on scope: this records *discrete episode outcomes*, not raw telemetry
streams.  Raw topic streams (joint states, ee pose, gripper) should be recorded
with memory2's ``Recorder`` (``dimos.memory2.module.Recorder``), which is the
standard, SQLite-backed stream store — see ``runner.py`` for where that hooks in.
"""

from __future__ import annotations

from datetime import datetime
import json
from pathlib import Path
from typing import Any

from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Order of the six pipeline stages tracked per episode.
STAGE_KEYS: tuple[str, ...] = (
    "scan",
    "grasp_gen",
    "plan_approach",
    "plan_grasp",
    "execute_pick",
    "execute_place",
)

# error_code -> failure point within the *pick* skill.
_GRASP_GEN_CODES = frozenset({"GRASP_GENERATION_FAILED", "OBJECT_NOT_DETECTED", "NO_PRIOR_POSE"})
_PLAN_CODES = frozenset(
    {"IK_FAILED", "PLANNING_FAILED", "COLLISION_AT_START", "GRASP_ATTEMPTS_EXHAUSTED"}
)
_EXEC_CODES = frozenset({"GRIPPER_FAILED", "EXECUTION_FAILED", "EXECUTION_TIMEOUT"})
# Infrastructure/precondition codes: nothing manipulation-specific was reached.
_INFRA_CODES = frozenset(
    {
        "ROBOT_NOT_FOUND",
        "INVALID_INPUT",
        "INVALID_STATE",
        "NOT_CONFIGURED",
        "WORLD_MONITOR_UNAVAILABLE",
    }
)


def _apply_pick_failure(stages: dict[str, Any], code: str | None) -> None:
    """Mark the pick stage flags for a *failed* pick given its error_code.

    A pre-PR string-returning stack has ``code is None`` (no machine-readable
    code); it degrades gracefully to "failed at the first manipulation stage".
    """
    if code in _INFRA_CODES:
        # Infra failure — leave all manipulation stages as None (unknown).
        return
    if code in _EXEC_CODES:
        stages["grasp_gen"] = True
        stages["plan_approach"] = True
        stages["plan_grasp"] = True
        stages["execute_pick"] = False
        return
    if code in _PLAN_CODES:
        stages["grasp_gen"] = True
        stages["plan_approach"] = False
        return
    # _GRASP_GEN_CODES, None (pre-PR), or any unknown code: grasp generation
    # is the earliest manipulation-specific stage, so attribute the failure there.
    stages["grasp_gen"] = False


def infer_stages(
    pick_result: dict[str, Any] | None,
    place_result: dict[str, Any] | None,
    scan_result: dict[str, Any] | None = None,
) -> dict[str, bool | None]:
    """Infer the six-stage tri-state flags from normalized skill results.

    Each stage is ``True`` (reached + passed), ``False`` (reached + the failure
    point), or ``None`` (not reached / unknown).  ``scan_result`` is optional so
    the two-argument form still works; pass it for accurate ``scan`` attribution.

    The error_code is read from whichever result carries it, so the *same* code
    attributes to different stages depending on which skill failed.
    """
    stages: dict[str, bool | None] = {k: None for k in STAGE_KEYS}

    # --- scan ---
    if scan_result is not None:
        stages["scan"] = bool(scan_result.get("success"))
        if not stages["scan"]:
            return stages  # nothing downstream was attempted

    # --- pick ---
    if pick_result is None:
        return stages  # pick never attempted
    # Reaching pick implies scan succeeded, even when no scan_result was passed.
    if stages["scan"] is None:
        stages["scan"] = True

    if pick_result.get("success"):
        stages["grasp_gen"] = True
        stages["plan_approach"] = True
        stages["plan_grasp"] = True
        stages["execute_pick"] = True
    else:
        _apply_pick_failure(stages, pick_result.get("error_code"))
        return stages

    # --- place (only reached when pick succeeded) ---
    if place_result is None:
        return stages  # place not attempted; execute_place stays None
    stages["execute_place"] = bool(place_result.get("success"))
    return stages


class EpisodeRecorder:
    """Append-only JSONL recorder — one line per episode, flushed on each write.

    Usage::

        with EpisodeRecorder(hardware="sim") as rec:
            rec.record(episode_dict)   # episode_id is assigned by the recorder

    Each instance owns one file ``eval_<hardware>_<YYYYMMDD_HHMMSS>.jsonl`` and a
    1-based episode counter (the single source of truth for ``episode_id``).
    """

    def __init__(
        self,
        output_dir: str | Path = "~/.dimos/eval_runs",
        hardware: str = "sim",
        timestamp: datetime | None = None,
    ) -> None:
        self._hardware = hardware
        self._counter = 0
        ts = timestamp if timestamp is not None else datetime.now()
        self._dir = Path(output_dir).expanduser()
        self._dir.mkdir(parents=True, exist_ok=True)
        self._path = self._dir / f"eval_{hardware}_{ts:%Y%m%d_%H%M%S}.jsonl"
        # Line-buffered UTF-8; we also flush() explicitly after every record.
        self._fh = self._path.open("w", encoding="utf-8")
        logger.info("EpisodeRecorder writing to %s", self._path)

    @property
    def path(self) -> Path:
        """Absolute path of the JSONL file being written."""
        return self._path

    @property
    def count(self) -> int:
        """Number of episodes recorded so far."""
        return self._counter

    def record(self, episode: dict[str, Any]) -> dict[str, Any]:
        """Assign an ``episode_id``, write one JSON line, flush, return the record.

        The returned dict is the exact object written (with ``episode_id`` first),
        so callers can use it directly without re-reading the file.
        """
        if self._fh.closed:
            raise RuntimeError("EpisodeRecorder is closed; cannot record more episodes")
        self._counter += 1
        episode_id = f"ep_{self._counter:04d}"
        record: dict[str, Any] = {"episode_id": episode_id}
        for key, value in episode.items():
            if key != "episode_id":
                record[key] = value
        self._fh.write(json.dumps(record, ensure_ascii=False) + "\n")
        self._fh.flush()
        return record

    def close(self) -> None:
        """Close the file handle. Idempotent."""
        if not self._fh.closed:
            self._fh.close()

    def __enter__(self) -> EpisodeRecorder:
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()
