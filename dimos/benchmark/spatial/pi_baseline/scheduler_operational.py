"""Authoritative, public operational reconstruction for Slice3."""

from __future__ import annotations

from collections import Counter
from datetime import datetime, timezone
from typing import Literal

from .scheduler_models import (
    JobIdentity,
    JobSummary,
    OperationalCount,
    OperationalEvent,
    OperationalFailure,
    OperationalSnapshot,
    TerminalOutcome,
)
from .scheduler_plan import job_id
from .scheduler_store import (
    CoordinatorLockError,
    FilesystemExperimentStore,
    LoadedDefinition,
    RecoveredAttempt,
)


class OperationalObservationError(RuntimeError):
    """A safe operational observation could not be produced."""

    _MESSAGE = "operational observation unavailable"

    def __init__(self) -> None:
        super().__init__(self._MESSAGE)

    def __str__(self) -> str:
        return self._MESSAGE


_REASONS = {
    "executor_failed": "executor_failed",
    "executor_interrupted": "executor_interrupted",
    "executor_cancelled": "executor_cancelled",
    "container_cleanup_failed": "container_cleanup_failed",
    "coordinator_cancelled": "coordinator_cancelled",
    "coordinator_restart": "coordinator_restart",
    "missing_terminal_outcome": "missing_terminal_outcome",
}


_STATUS = Literal["failed", "interrupted", "cancelled"]
_COMPATIBLE = {
    "executor_failed": "failed",
    "executor_interrupted": "interrupted",
    "executor_cancelled": "cancelled",
    "container_cleanup_failed": "failed",
    "coordinator_cancelled": "cancelled",
    "coordinator_restart": "interrupted",
    "missing_terminal_outcome": "interrupted",
}


def _safe_reason(outcome: TerminalOutcome) -> str:
    status = outcome.status
    reason = _REASONS.get(outcome.reason)
    if reason is not None and (
        _COMPATIBLE.get(reason) == status
    ):
        return reason
    return {
        "failed": "executor_failed",
        "interrupted": "executor_interrupted",
        "cancelled": "executor_cancelled",
        "succeeded": "executor_failed",
    }[status]


def _ensure_missing_outcome_event(store: FilesystemExperimentStore, context, outcome: TerminalOutcome) -> None:
    expected = OperationalEvent(
        kind="finished",
        occurred_at=datetime.now(timezone.utc),
        message="missing_terminal_outcome",
        payload={"status": outcome.status},
    )
    if any(
        event.kind == expected.kind
        and event.message == expected.message
        and event.payload == expected.payload
        for event in store.events(context)
    ):
        return
    store.append_event(context, expected)


def _snapshot(
    definition: LoadedDefinition,
    observation: Literal["reconciled", "busy_read_only"],
    attempts: dict[str, tuple[RecoveredAttempt, ...]],
) -> OperationalSnapshot:
    # LoadedDefinition is intentionally duck-typed here to keep this pure
    # reconstruction helper independent of the filesystem store.
    states: Counter[str] = Counter()
    failures: list[OperationalFailure] = []
    for planned in definition.plan.jobs:
        case = next(item for item in definition.plan.cases if item.case_id == planned.case_id)
        condition = next(item for item in definition.plan.conditions if item.name == planned.condition_name)
        identifier = job_id(definition.plan, case, condition)
        records = attempts.get(identifier, ())
        latest = max(records, key=lambda item: item.context.attempt_number, default=None)
        if latest is None:
            state = "pending"
        elif latest.outcome is None:
            state = "running"
        else:
            state = latest.outcome.status
            if state in {"failed", "interrupted", "cancelled"} and len(failures) < 3:
                failures.append(
                    OperationalFailure(
                        job_id=identifier,
                        state=state,
                        reason=_safe_reason(latest.outcome),
                    )
                )
        states[state] += 1
    counts = OperationalCount(**{name: states[name] for name in ("pending", "running", "succeeded", "failed", "interrupted", "cancelled")})
    return OperationalSnapshot(
        experiment_id=definition.manifest.experiment_id,
        workers=definition.manifest.workers,
        observation=observation,
        counts=counts,
        jobs=sum(counts.model_dump().values()),
        active=counts.running,
        failures=tuple(sorted(failures, key=lambda failure: failure.job_id)),
    )


def collect_operational_snapshot(store: FilesystemExperimentStore) -> OperationalSnapshot:
    """Reconcile immutable attempts under the lease, or observe read-only when busy."""
    if getattr(store._lease_state, "depth", 0):
        return _busy_snapshot(store)
    try:
        with store.coordinator_lease():
            definition, _, attempts = reconcile_idle_locked(store)
            return _snapshot(definition, "reconciled", attempts)
    except CoordinatorLockError:
        return _busy_snapshot(store)
    except Exception as error:
        raise OperationalObservationError from error


def _busy_snapshot(store: FilesystemExperimentStore) -> OperationalSnapshot:
    """Observe a busy store through descriptor-pinned, non-mutating reads only."""
    try:
        definition, attempts = store.observe_experiment_read_only(
            None, None
        )
        return _snapshot(definition, "busy_read_only", attempts)
    except Exception as error:
        raise OperationalObservationError from error


def reconcile_idle_locked(
    store: FilesystemExperimentStore,
) -> tuple[LoadedDefinition, dict[str, JobSummary], dict[str, tuple[RecoveredAttempt, ...]]]:
    """Reconstruct state from the immutable definition and attempt records."""
    definition = store.load_definition()
    store.recover_all_attempts(definition.manifest.experiment_id)
    summaries: dict[str, JobSummary] = {}
    attempts_by_job: dict[str, tuple[RecoveredAttempt, ...]] = {}
    for planned in definition.plan.jobs:
        case = next(item for item in definition.plan.cases if item.case_id == planned.case_id)
        condition = next(item for item in definition.plan.conditions if item.name == planned.condition_name)
        identifier = job_id(definition.plan, case, condition)
        identity = JobIdentity(
            experiment_id=definition.manifest.experiment_id,
            case_id=case.case_id,
            condition_name=condition.name,
            job_id=identifier,
        )
        records = tuple(
            record
            for record in store.recover_attempts(definition.manifest.experiment_id, identifier)
            if record.context.attempt_number > 0
        )
        latest = max(records, key=lambda record: record.context.attempt_number, default=None)
        if latest is None:
            summary = JobSummary(identity=identity, state="pending")
        else:
            outcome = latest.outcome
            if outcome is None:
                outcome = TerminalOutcome(status="interrupted", reason="missing_terminal_outcome")
                try:
                    store.write_outcome(latest.context, outcome)
                except FileExistsError:
                    outcome = store.read_outcome(latest.context)
                if outcome is None:
                    raise OperationalObservationError
            if outcome.reason == "missing_terminal_outcome":
                _ensure_missing_outcome_event(store, latest.context, outcome)
                records = tuple(store.recover_attempts(definition.manifest.experiment_id, identifier))
                latest = max(
                    (record for record in records if record.context.attempt_number > 0),
                    key=lambda record: record.context.attempt_number,
                )
            summary = JobSummary(
                identity=identity,
                state=outcome.status,
                latest_attempt_id=latest.context.attempt_id,
                outcome=outcome,
            )
        store.write_summary(summary)
        summaries[identifier] = summary
        attempts_by_job[identifier] = records
    return definition, summaries, attempts_by_job
