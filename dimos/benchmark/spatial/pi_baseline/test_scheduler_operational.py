import json
from pathlib import Path
from threading import Event, Thread

import pytest

from .scheduler_models import (
    AttemptContext,
    OperationalCount,
    OperationalFailure,
    OperationalSnapshot,
    TerminalOutcome,
)
from .scheduler_operational import (
    OperationalObservationError,
    collect_operational_snapshot,
)
from .test_scheduler_runtime import FakeExecutor, make_runtime


def test_collect_operational_snapshot_reconstructs_plan_counts(tmp_path: Path) -> None:
    runtime = make_runtime(tmp_path, FakeExecutor(), workers=1, case_count=2)
    snapshot = collect_operational_snapshot(runtime.store)
    assert snapshot.observation == "reconciled"
    assert snapshot.jobs == 2
    assert snapshot.counts.pending == 2
    assert snapshot.active == 0


def test_operational_snapshot_rejects_inconsistent_counts() -> None:
    with pytest.raises(ValueError):
        OperationalSnapshot(
            experiment_id="experiment",
            workers=1,
            observation="reconciled",
            counts=OperationalCount(
                pending=1, running=0, succeeded=0, failed=0, interrupted=0, cancelled=0
            ),
            jobs=2,
            active=0,
            failures=(),
        )


def test_operational_observation_error_has_no_detail_leak() -> None:
    error = OperationalObservationError()
    assert str(error) == "operational observation unavailable"
    assert repr(error) == "OperationalObservationError('operational observation unavailable')"
    assert error.args == ("operational observation unavailable",)


def test_operational_snapshot_dump_is_fixed_schema() -> None:
    snapshot = OperationalSnapshot(
        experiment_id="experiment",
        workers=2,
        observation="busy_read_only",
        counts=OperationalCount(
            pending=0, running=0, succeeded=1, failed=1, interrupted=0, cancelled=0
        ),
        jobs=2,
        active=0,
        failures=(OperationalFailure(job_id="job-1", state="failed", reason="executor_failed"),),
    )
    assert snapshot.model_dump(mode="json") == {
        "record_type": "pi-operational-snapshot",
        "schema_version": "1.0",
        "experiment_id": "experiment",
        "workers": 2,
        "observation": "busy_read_only",
        "counts": {
            "pending": 0,
            "running": 0,
            "succeeded": 1,
            "failed": 1,
            "interrupted": 0,
            "cancelled": 0,
        },
        "jobs": 2,
        "active": 0,
        "failures": [
            {"job_id": "job-1", "state": "failed", "reason": "executor_failed"}
        ],
    }


@pytest.mark.parametrize(
    ("status", "reason", "expected"),
    [
        ("failed", "unknown", "executor_failed"),
        ("interrupted", "unknown", "executor_interrupted"),
        ("cancelled", "unknown", "executor_cancelled"),
        ("failed", "executor_interrupted", "executor_failed"),
        ("interrupted", "executor_failed", "executor_interrupted"),
        ("cancelled", "coordinator_restart", "executor_cancelled"),
        ("cancelled", "coordinator_cancelled", "coordinator_cancelled"),
        ("interrupted", "coordinator_restart", "coordinator_restart"),
        ("interrupted", "missing_terminal_outcome", "missing_terminal_outcome"),
        ("failed", "container_cleanup_failed", "container_cleanup_failed"),
    ],
)
def test_failure_reason_mapping(status: str, reason: str, expected: str) -> None:
    from .scheduler_operational import _safe_reason

    assert _safe_reason(TerminalOutcome(status=status, reason=reason)) == expected


def _busy_snapshot(runtime, callback=None):
    entered = Event()
    release = Event()

    def hold_lease() -> None:
        with runtime.store.coordinator_lease():
            entered.set()
            release.wait(2)

    holder = Thread(target=hold_lease)
    holder.start()
    assert entered.wait(2)
    try:
        return collect_operational_snapshot(runtime.store)
    finally:
        release.set()
        holder.join(2)


def test_idle_acceptance_matrix_ignores_forged_cache_and_uses_attempt_artifacts(tmp_path: Path) -> None:
    runtime = make_runtime(tmp_path, FakeExecutor(), workers=1, case_count=1)
    completed = runtime.run()[0]
    forged = completed.model_copy(
        update={
            "state": "failed",
            "outcome": TerminalOutcome(status="failed", reason="forged_private_path"),
        }
    )
    with runtime.store.coordinator_lease():
        runtime.store.write_summary(forged)
        (runtime.store.root / "jobs" / f"{completed.identity.job_id}.json").unlink()
    recovered = runtime.recover()[0]
    assert recovered.state == "succeeded"
    assert recovered.outcome is not None and recovered.outcome.reason == "completed"


def test_idle_incomplete_attempt_publishes_one_fixed_outcome_and_event(tmp_path: Path) -> None:
    runtime = make_runtime(tmp_path, FakeExecutor(), workers=1, case_count=1)
    identity = runtime.summaries()[0].identity
    context = AttemptContext(
        identity=identity,
        attempt_id="attempt-1",
        attempt_number=1,
        directory_name="attempt-1",
        manifest_digest=runtime._manifest_digest(),
    )
    with runtime.store.coordinator_lease():
        runtime.store.create_attempt(context, runtime.plan.cases[0], runtime.plan.conditions[0])
    first = collect_operational_snapshot(runtime.store)
    second = collect_operational_snapshot(runtime.store)
    assert first.counts.interrupted == second.counts.interrupted == 1
    outcome = runtime.store.read_outcome(context)
    assert outcome == TerminalOutcome(status="interrupted", reason="missing_terminal_outcome")
    events_path = (
        runtime.store.root / "attempts" / identity.job_id / "attempt-1" / "events.jsonl"
    )
    events = events_path.read_text()
    assert events.count("missing_terminal_outcome") == 1
    assert second.model_dump(mode="json") == first.model_dump(mode="json")


def test_busy_acceptance_matrix_is_read_only_and_classifies_running(tmp_path: Path) -> None:
    runtime = make_runtime(tmp_path, FakeExecutor(), workers=1, case_count=1)
    identity = runtime.summaries()[0].identity
    context = AttemptContext(
        identity=identity,
        attempt_id="attempt-1",
        attempt_number=1,
        directory_name="attempt-1",
        manifest_digest=runtime._manifest_digest(),
    )
    with runtime.store.coordinator_lease():
        runtime.store.create_attempt(context, runtime.plan.cases[0], runtime.plan.conditions[0])
    before = sorted(path.relative_to(runtime.store.root).as_posix() for path in runtime.store.root.rglob("*"))
    snapshot = _busy_snapshot(runtime)
    after = sorted(path.relative_to(runtime.store.root).as_posix() for path in runtime.store.root.rglob("*"))
    assert snapshot.observation == "busy_read_only"
    assert snapshot.counts.pending == 0
    assert snapshot.counts.running == 1
    assert before == after


def test_busy_symlink_is_rejected_without_touching_target(tmp_path: Path) -> None:
    runtime = make_runtime(tmp_path, FakeExecutor(), workers=1, case_count=1)
    identity = runtime.summaries()[0].identity
    target = tmp_path / "outside"
    target.mkdir()
    marker = target / "marker"
    marker.write_text("untouched")
    job_root = runtime.store.root / "attempts" / identity.job_id
    job_root.mkdir()
    link = job_root / "attempt-9"
    link.symlink_to(target, target_is_directory=True)
    with pytest.raises(OperationalObservationError):
        _busy_snapshot(runtime)
    assert marker.read_text() == "untouched"


def test_busy_observer_excludes_context_with_inflated_attempt_number(tmp_path: Path) -> None:
    runtime = make_runtime(tmp_path, FakeExecutor(), workers=1, case_count=1)
    identity = runtime.summaries()[0].identity
    with runtime.store.coordinator_lease():
        for number in (1, 2):
            context = AttemptContext(
                identity=identity,
                attempt_id=f"attempt-{number}",
                attempt_number=number,
                directory_name=f"attempt-{number}",
                manifest_digest=runtime._manifest_digest(),
            )
            runtime.store.create_attempt(context, runtime.plan.cases[0], runtime.plan.conditions[0])
    context_path = runtime.store.root / "attempts" / identity.job_id / "attempt-1" / "context.json"
    context_payload = json.loads(context_path.read_text())
    context_payload["attempt_number"] = 999
    context_path.write_text(json.dumps(context_payload) + "\n")
    observed = runtime.store.observe_attempts_read_only("experiment", identity.job_id)
    assert [attempt.context.attempt_id for attempt in observed] == ["attempt-2"]


def test_missing_outcome_event_retries_after_append_failure(tmp_path: Path, monkeypatch) -> None:
    runtime = make_runtime(tmp_path, FakeExecutor(), workers=1, case_count=1)
    identity = runtime.summaries()[0].identity
    context = AttemptContext(
        identity=identity,
        attempt_id="attempt-1",
        attempt_number=1,
        directory_name="attempt-1",
        manifest_digest=runtime._manifest_digest(),
    )
    with runtime.store.coordinator_lease():
        runtime.store.create_attempt(context, runtime.plan.cases[0], runtime.plan.conditions[0])
    original_append = runtime.store.append_event
    failed_once = True

    def fail_once(attempt, event):
        nonlocal failed_once
        if failed_once and event.message == "missing_terminal_outcome":
            failed_once = False
            raise OSError("injected append failure")
        return original_append(attempt, event)

    monkeypatch.setattr(runtime.store, "append_event", fail_once)
    with pytest.raises(OperationalObservationError):
        collect_operational_snapshot(runtime.store)
    snapshot = collect_operational_snapshot(runtime.store)
    assert snapshot.counts.interrupted == 1
    events = (
        runtime.store.root / "attempts" / identity.job_id / "attempt-1" / "events.jsonl"
    ).read_text()
    assert events.count("missing_terminal_outcome") == 1


@pytest.mark.parametrize(
    "failure",
    [
        ("failed", "executor_interrupted"),
        ("interrupted", "executor_failed"),
        ("cancelled", "executor_cancelled"),
    ],
)
def test_snapshot_rejects_unsafe_failure_relationships(failure) -> None:
    state, reason = failure
    with pytest.raises(ValueError):
        OperationalSnapshot(
            experiment_id="experiment",
            workers=1,
            observation="reconciled",
            counts=OperationalCount(
                pending=0, running=0, succeeded=0, failed=1, interrupted=0, cancelled=0
            ),
            jobs=1,
            active=0,
            failures=(OperationalFailure(job_id="job-1", state=state, reason=reason),),
        )


def test_snapshot_rejects_duplicate_or_zero_count_failures() -> None:
    base = dict(
        experiment_id="experiment",
        workers=1,
        observation="reconciled",
        counts=OperationalCount(
            pending=0, running=0, succeeded=0, failed=2, interrupted=0, cancelled=0
        ),
        jobs=2,
        active=0,
    )
    failure = OperationalFailure(job_id="job-1", state="failed", reason="executor_failed")
    with pytest.raises(ValueError):
        OperationalSnapshot(**base, failures=(failure, failure))
    with pytest.raises(ValueError):
        OperationalSnapshot(
            **base,
            failures=(
                OperationalFailure(
                    job_id="job-1", state="interrupted", reason="executor_interrupted"
                ),
            ),
        )
