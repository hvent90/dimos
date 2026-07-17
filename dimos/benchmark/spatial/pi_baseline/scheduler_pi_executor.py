"""Pi adapter for the agent-neutral scheduler executor boundary."""

from __future__ import annotations

from collections.abc import Callable
import hashlib
import threading

from dimos.benchmark.spatial.models import SpatialModel
from dimos.benchmark.spatial.pi_baseline.broker import PolicyViolationError
from dimos.benchmark.spatial.pi_baseline.config import (
    PiBaselineConfig,
    PromptMode,
    PublicSelection,
)
from dimos.benchmark.spatial.pi_baseline.controller import AdapterCleanupError
from dimos.benchmark.spatial.pi_baseline.podman import ContainerCleanupError
from dimos.benchmark.spatial.pi_baseline.runner import ConditionRun, run_condition
from dimos.benchmark.spatial.pi_baseline.scheduler_executor import (
    EventSink,
    ExecutionInterrupted,
    Executor,
)
from dimos.benchmark.spatial.pi_baseline.scheduler_models import (
    AttemptContext,
    ExecutorArtifactEvent,
    ExecutorProgressEvent,
    ExpandedCase,
    NamedCondition,
    TerminalOutcome,
)
from dimos.benchmark.spatial.pi_baseline.scheduler_pi_binding import (
    PiExecutionSnapshot,
    PiRuntimeBindings,
    reconstruct_config,
    verify_public_inputs,
    verify_runtime_artifacts,
    verify_snapshot_artifacts,
)


class PiCasePayload(SpatialModel):
    """Validated scheduler payload required to execute a Pi case."""

    selection: PublicSelection


class PiConditionPayload(SpatialModel):
    """Validated Pi condition payload."""

    prompt_mode: PromptMode


ConditionRunner = Callable[
    [PiBaselineConfig, PromptMode, threading.Event, threading.Lock], ConditionRun
]


def _default_condition_runner(
    config: PiBaselineConfig,
    mode: PromptMode,
    cancel_requested: threading.Event,
    publication_lock: threading.Lock,
) -> ConditionRun:
    return run_condition(
        config,
        mode=mode,
        cancel_requested=cancel_requested,
        publication_lock=publication_lock,
    )


class PiSchedulerExecutor(Executor):
    """Adapt one scheduler job into one isolated Pi condition execution."""

    def __init__(
        self,
        snapshot: PiExecutionSnapshot,
        runtime_bindings: PiRuntimeBindings,
        *,
        manifest_executor_fingerprint: str,
        manifest_selected_inputs_digest: str | None = None,
        condition_runner: ConditionRunner = _default_condition_runner,
    ) -> None:
        if snapshot.canonical_digest() != manifest_executor_fingerprint:
            raise ValueError("Pi execution snapshot does not match executor fingerprint")
        verify_snapshot_artifacts(snapshot)
        if manifest_selected_inputs_digest is not None:
            if snapshot.pi_selected_inputs_digest != manifest_selected_inputs_digest:
                raise ValueError("Pi selected inputs do not match Pi binding")
        self._snapshot = snapshot
        self._runtime_bindings = runtime_bindings
        self._condition_runner = condition_runner

    def run(
        self,
        case: ExpandedCase,
        condition: NamedCondition,
        context: AttemptContext,
        emit: EventSink,
        cancel_requested: threading.Event,
        publication_lock: threading.Lock,
    ) -> TerminalOutcome:
        """Validate Pi payloads, execute one condition, and normalize its result."""
        _check_cancel(cancel_requested)
        parsed_case = PiCasePayload.model_validate(case.payload)
        parsed_condition = PiConditionPayload.model_validate(condition.payload)
        # All immutable material and public inputs are checked before any
        # lifecycle event or adapter process can be created.
        if self._snapshot.selected_inputs:
            _check_cancel(cancel_requested)
            verify_runtime_artifacts(self._snapshot, self._snapshot.node_adapter_command)
        if self._snapshot.selected_inputs:
            _check_cancel(cancel_requested)
            recorded = {
                item.case_id: item for item in self._snapshot.selected_inputs
            }.get(case.case_id)
            if recorded is None or recorded.selection != parsed_case.selection:
                raise ValueError("Pi case is not one of the immutable selected inputs")
            verify_public_inputs(self._snapshot, self._runtime_bindings.corpus_root)
        _check_cancel(cancel_requested)
        emit(ExecutorProgressEvent(kind="progress", code="executor_progress"))
        condition_run_id = _condition_run_id(context)
        attempt_public_root = (
            self._runtime_bindings.public_root
            / context.identity.experiment_id
            / context.identity.job_id
            / context.attempt_id
            / "public"
        )
        attempt_private_root = (
            self._runtime_bindings.private_root
            / context.identity.experiment_id
            / context.identity.job_id
            / context.attempt_id
            / "private"
        )
        bindings = self._runtime_bindings.model_copy(
            update={
                "public_root": attempt_public_root,
                "private_root": attempt_private_root,
            }
        )
        config = reconstruct_config(
            self._snapshot,
            bindings,
            selection=parsed_case.selection,
            mode=parsed_condition.prompt_mode,
            case_id=case.case_id,
            run_id=condition_run_id,
        )
        try:
            _check_cancel(cancel_requested)
            if self._snapshot.selected_inputs:
                verify_runtime_artifacts(self._snapshot, tuple(config.node_adapter_command))
            result = _invoke_condition_runner(
                self._condition_runner,
                config,
                parsed_condition.prompt_mode,
                cancel_requested,
                publication_lock,
            )
        except ExecutionInterrupted:
            raise
        except Exception as error:
            outcome = TerminalOutcome(
                status="failed",
                reason=_failure_reason(error),
            )
            return outcome

        _check_cancel(cancel_requested)
        emit(ExecutorArtifactEvent(
            kind="artifact",
            code="artifact_recorded",
            artifact_id=f"evidence-{context.attempt_id}",
            artifact_sha256=result.evidence.manifest_sha256,
        ))
        return TerminalOutcome(status="succeeded", reason="Pi condition completed")


def _failure_reason(error: Exception) -> str:
    if isinstance(error, (ContainerCleanupError, AdapterCleanupError)):
        return ContainerCleanupError.reason
    if isinstance(error, PolicyViolationError):
        return f"policy:{error.code}"
    return "condition execution failed"


def _check_cancel(cancel_requested: threading.Event) -> None:
    if cancel_requested.is_set():
        raise ExecutionInterrupted


def _invoke_condition_runner(
    runner: ConditionRunner,
    config: PiBaselineConfig,
    mode: PromptMode,
    cancel_requested: threading.Event,
    publication_lock: threading.Lock,
) -> ConditionRun:
    return runner(config, mode, cancel_requested, publication_lock)


def _condition_run_id(context: AttemptContext) -> str:
    """Derive an isolated, deterministic host/container identity for an attempt."""
    identity = "\x1f".join(
        (
            context.identity.experiment_id,
            context.identity.case_id,
            context.identity.condition_name,
            context.identity.job_id,
            context.attempt_id,
            str(context.attempt_number),
        )
    )
    # The runner appends a mode suffix before building the Podman request.
    # Reserve room for the longest suffix within Podman's 63-character limit.
    return f"pi-{hashlib.sha256(identity.encode('utf-8')).hexdigest()[:49]}"
