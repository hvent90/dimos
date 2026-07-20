# Copyright 2026 Dimensional Inc.
"""Fail-closed configuration for the opt-in Pi spatial baseline runner."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
import json
from pathlib import Path
import re
from typing import Literal

from pydantic import Field, field_validator, model_validator

from dimos.benchmark.spatial.models import SpatialModel

PromptMode = Literal["visualization-forbidden", "visualization-encouraged"]
ThinkingLevel = Literal["medium"]
_ID_PATTERN = r"^[A-Za-z0-9][A-Za-z0-9_-]{0,127}$"
_DIGEST_PATTERN = r"^.+@sha256:[0-9a-f]{64}$"


def validate_node_adapter_command(command: Sequence[str]) -> list[str]:
    """Accept only a direct Node executable and one absolute JS entrypoint.

    This is intentionally a structural admission check, rather than a shell
    command parser: there is no supported wrapper, package manager, runtime
    flag, or additional argv in the Pi protocol.
    """
    if len(command) != 2 or any(not isinstance(argument, str) or not argument for argument in command):
        raise ValueError("Pi adapter command must be [absolute node executable, absolute adapter file]")
    executable, adapter = (Path(argument) for argument in command)
    if not executable.is_absolute() or executable.name not in {"node", "nodejs"}:
        raise ValueError("Pi adapter executable must be an approved absolute Node executable")
    if not executable.is_file() or not executable.stat().st_mode & 0o111:
        raise ValueError("Pi adapter executable must be an executable file")
    if not adapter.is_absolute() or adapter.suffix != ".js":
        raise ValueError("Pi adapter command must name one absolute .js file")
    return list(command)


class ModelConfig(SpatialModel):
    provider: Literal["openai-codex"]
    model_id: Literal["gpt-5.6-luna"]
    thinking_level: ThinkingLevel


class ResourceLimits(SpatialModel):
    cpu_cores: float = Field(gt=0)
    memory_mb: int = Field(gt=0)
    pids: int = Field(gt=0)
    timeout_seconds: int = Field(gt=0)


class AuditNetworkPolicy(SpatialModel):
    network_access: Literal["general-outbound"]
    audit: Literal["heuristic"]
    audit_limitations: Literal["cannot-prove-no-online-use"]


class PublicSelection(SpatialModel):
    scene_id: str = Field(pattern=_ID_PATTERN)
    trajectory_id: str = Field(pattern=_ID_PATTERN)
    question_id: str = Field(pattern=_ID_PATTERN)
    variant: Literal["clean", "noisy-01", "noisy-02"]
    instance_id: str = Field(pattern=_ID_PATTERN)


class Budgets(SpatialModel):
    max_turns: int = Field(ge=1, le=100)
    max_tool_calls: int = Field(ge=1, le=100)
    timeout_ms: int = Field(ge=1_000, le=900_000)


class FixedSmokeIdentity(PublicSelection):
    """The immutable case identity used by both paired prompt modes."""


class ImplementationDigests(SpatialModel):
    adapter: str = Field(pattern=_DIGEST_PATTERN)
    scorer: str = Field(pattern=_DIGEST_PATTERN)
    protocol: str = Field(pattern=_DIGEST_PATTERN)


class PiBaselineConfig(SpatialModel):
    """All host-side inputs needed before an external runner may start."""

    model: ModelConfig
    node_adapter_command: list[str] = Field(min_length=2, max_length=2)
    codex_oauth_auth_path: str = Field(min_length=1)
    runner_image: str = Field(pattern=_DIGEST_PATTERN)
    rootless_podman_required: Literal[True]
    resource_limits: ResourceLimits
    output_root: str = Field(min_length=1)
    audit_network_policy: AuditNetworkPolicy
    prompt_modes: list[PromptMode] = Field(min_length=1)
    corpus_root: str = Field(min_length=1)
    oracle_root: str = Field(min_length=1)
    private_root: str = Field(min_length=1)
    ledger_path: str = Field(min_length=1)
    selection: PublicSelection
    budgets: Budgets
    scorer_revision: str = Field(min_length=1)
    fixed_smoke_identity: FixedSmokeIdentity
    implementation_digests: ImplementationDigests
    case_id: str | None = Field(default=None, pattern=_ID_PATTERN)
    run_id: str | None = Field(default=None, pattern=_ID_PATTERN)

    @field_validator("node_adapter_command")
    @classmethod
    def validate_node_command(cls, value: list[str]) -> list[str]:
        return validate_node_adapter_command(value)

    @field_validator("codex_oauth_auth_path")
    @classmethod
    def validate_auth_path(cls, value: str) -> str:
        path = Path(value).expanduser()
        if not path.is_file():
            raise ValueError("codex OAuth auth path must point to an existing file")
        return str(path)

    @field_validator("output_root")
    @classmethod
    def validate_output_root(cls, value: str) -> str:
        path = Path(value).expanduser()
        if path.exists() and not path.is_dir():
            raise ValueError("output root must be a directory")
        return str(path)

    @field_validator("corpus_root", "oracle_root", "private_root", "ledger_path")
    @classmethod
    def validate_roots(cls, value: str) -> str:
        path = Path(value).expanduser()
        if path.exists() and not (path.is_dir() or path.parent.is_dir()):
            raise ValueError("configured root/path is not usable")
        return str(path)

    @field_validator("prompt_modes")
    @classmethod
    def validate_prompt_modes(cls, value: list[PromptMode]) -> list[PromptMode]:
        if set(value) != {"visualization-forbidden", "visualization-encouraged"}:
            raise ValueError("prompt_modes must contain both supported prompt modes")
        return value

    @field_validator("fixed_smoke_identity")
    @classmethod
    def validate_fixed_identity(cls, value: FixedSmokeIdentity) -> FixedSmokeIdentity:
        return value

    @model_validator(mode="after")
    def validate_selection_identity(self) -> PiBaselineConfig:
        if self.selection.model_dump() != self.fixed_smoke_identity.model_dump():
            raise ValueError("selection and fixed_smoke_identity must identify the same case")
        return self


BaselineConfig = PiBaselineConfig


def validate_identifier(value: str) -> str:
    """Validate a case, run, or instance identifier without executing a run."""
    if not re.fullmatch(_ID_PATTERN, value):
        raise ValueError("identifier must contain only letters, digits, '_' or '-'")
    return value


def load_config(path: str | Path) -> PiBaselineConfig:
    """Load and validate a JSON configuration; no OAuth or model call is made."""
    config_path = Path(path)
    with config_path.open(encoding="utf-8") as handle:
        payload: Mapping[str, object] = json.load(handle)
    return PiBaselineConfig.model_validate(payload)
