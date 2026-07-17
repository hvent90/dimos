"""The deliberately small executor boundary for scheduler-owned work."""

from __future__ import annotations

from collections.abc import Callable
import threading
from typing import Protocol

from .scheduler_models import (
    AttemptContext,
    ExecutorEvent,
    ExpandedCase,
    NamedCondition,
    TerminalOutcome,
)

EventSink = Callable[[ExecutorEvent], None]


class ExecutionInterrupted(Exception):  # noqa: N818 - public cooperative signal name is contractual
    """Cooperative signal used when an admitted execution is interrupted."""


class Executor(Protocol):
    """Execute one immutable case/condition attempt and return one outcome."""

    def run(
        self,
        case: ExpandedCase,
        condition: NamedCondition,
        context: AttemptContext,
        emit: EventSink,
        cancel_requested: threading.Event,
        publication_lock: threading.Lock,
    ) -> TerminalOutcome:
        """Emit safe progress/artifact events and return one terminal outcome."""
        ...
