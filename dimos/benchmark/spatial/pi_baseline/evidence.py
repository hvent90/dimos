# Copyright 2026 Dimensional Inc.
"""Evidence manifests with strict public/private artifact separation."""

from __future__ import annotations

import hashlib
from pathlib import Path
from typing import Literal

from pydantic import Field

from dimos.benchmark.spatial.models import SpatialModel
from dimos.benchmark.spatial.pi_baseline.topology import PinnedDirectory
from dimos.benchmark.spatial.utilities import canonical_json, validate_relative_path


class EvidenceArtifact(SpatialModel):
    record_type: Literal["evidence-artifact"] = "evidence-artifact"
    path: str = Field(min_length=1)
    sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    size_bytes: int = Field(ge=0)


class EvidenceManifest(SpatialModel):
    record_type: Literal["evidence-manifest"] = "evidence-manifest"
    schema_version: Literal["1.0"] = "1.0"
    public: tuple[EvidenceArtifact, ...]
    private: tuple[EvidenceArtifact, ...]


def build_evidence_manifest(
    public_root: Path | PinnedDirectory,
    private_root: Path | PinnedDirectory,
    *,
    public_artifacts: tuple[str, ...],
    private_artifacts: tuple[str, ...],
    required_public: tuple[str, ...] = (),
    required_private: tuple[str, ...] = (),
) -> EvidenceManifest:
    public, public_owned = _as_pinned(public_root)
    private, private_owned = _as_pinned(private_root)
    try:
        records_public = _artifacts(public, public_artifacts, required_public)
        records_private = _artifacts(private, private_artifacts, required_private)
        _assert_public_safe(public, records_public)
        return EvidenceManifest(public=records_public, private=records_private)
    finally:
        if public_owned:
            public.close()
        if private_owned:
            private.close()


def write_evidence_manifest(
    directory: PinnedDirectory | Path,
    manifest: EvidenceManifest,
    name: str = "evidence-manifest.v1.json",
) -> None:
    payload = canonical_json(manifest.model_dump(mode="json")) + b"\n"
    if isinstance(directory, PinnedDirectory):
        directory.write_bytes(name, payload)
        return
    pinned = PinnedDirectory.open(directory, create=False)
    try:
        pinned.write_bytes(name, payload)
    finally:
        pinned.close()


def _artifacts(
    root: PinnedDirectory, names: tuple[str, ...], required: tuple[str, ...]
) -> tuple[EvidenceArtifact, ...]:
    selected = tuple(dict.fromkeys((*names, *required)))
    result: list[EvidenceArtifact] = []
    for name in selected:
        safe = validate_relative_path(name)
        try:
            data = root.read_relative(safe)
        except Exception as error:
            raise ValueError(f"required evidence artifact is missing: {name}") from error
        result.append(
            EvidenceArtifact(
                path=safe,
                sha256=hashlib.sha256(data).hexdigest(),
                size_bytes=len(data),
            )
        )
    return tuple(result)


def _assert_public_safe(root: PinnedDirectory, artifacts: tuple[EvidenceArtifact, ...]) -> None:
    forbidden_names = {"oracle", "score", "scores", "ledger", "answer", "answers", "override"}
    for artifact in artifacts:
        if any(part.lower() in forbidden_names for part in Path(artifact.path).parts):
            raise ValueError(f"private score/oracle artifact cannot be public evidence: {artifact.path}")
        data = root.read_relative(artifact.path)
        if b"pi-score" in data or b"oracle" in data or b"review-override" in data:
            raise ValueError(f"public artifact contains private score/oracle material: {artifact.path}")


def _as_pinned(value: Path | PinnedDirectory) -> tuple[PinnedDirectory, bool]:
    if isinstance(value, PinnedDirectory):
        return value, False
    return PinnedDirectory.open(value, create=False), True
