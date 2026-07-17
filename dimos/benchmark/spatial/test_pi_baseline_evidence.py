# Copyright 2026 Dimensional Inc.
from pathlib import Path

import pytest

from dimos.benchmark.spatial.pi_baseline.evidence import build_evidence_manifest


def test_evidence_hashes_separate_public_and_private_artifacts(tmp_path: Path) -> None:
    public = tmp_path / "public"
    private = tmp_path / "private"
    public.mkdir()
    private.mkdir()
    (public / "case.v1.json").write_text('{"record_type":"case"}')
    (private / "score.json").write_text('{"record_type":"pi-score"}')
    manifest = build_evidence_manifest(public, private, public_artifacts=("case.v1.json",), private_artifacts=("score.json",), required_public=("case.v1.json",), required_private=("score.json",))
    assert manifest.public[0].sha256 and manifest.private[0].sha256
    assert manifest.public[0].path == "case.v1.json"


def test_evidence_rejects_private_bytes_and_missing_required_files(tmp_path: Path) -> None:
    public = tmp_path / "public"
    private = tmp_path / "private"
    public.mkdir()
    private.mkdir()
    (public / "leak.json").write_text('{"record_type":"pi-score"}')
    with pytest.raises(ValueError, match="private"):
        build_evidence_manifest(public, private, public_artifacts=("leak.json",), private_artifacts=())
    with pytest.raises(ValueError, match="missing"):
        build_evidence_manifest(public, private, public_artifacts=(), private_artifacts=(), required_public=("case.v1.json",))
