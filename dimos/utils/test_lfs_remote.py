# Copyright 2026 Dimensional Inc.

from pathlib import Path
from typing import Any
from unittest.mock import Mock

import pytest

from dimos.utils import lfs_remote

OID_A = "a" * 64
OID_B = "b" * 64


def pointer(path: str, oid: str) -> lfs_remote.Pointer:
    return lfs_remote.Pointer(path, oid, 42)


def response(objects: Any) -> Mock:
    result = Mock()
    result.json.return_value = {"objects": objects}
    return result


def available(oid: str) -> dict:
    return {"oid": oid, "actions": {"download": {"href": "https://payload.invalid/archive"}}}


def run_verify(
    monkeypatch: pytest.MonkeyPatch, pointers: list[lfs_remote.Pointer], reply: Mock
) -> Mock:
    monkeypatch.setattr(lfs_remote, "discover_pointers", lambda target, repo: pointers)
    monkeypatch.setattr(
        lfs_remote, "configured_lfs_url", lambda repo: "https://configured.invalid/lfs"
    )
    post = Mock(return_value=reply)
    lfs_remote.verify(batch_size=2, post=post)
    return post


def test_success_batches_metadata_and_never_fetches_payload(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    fetch = Mock(side_effect=AssertionError("payload URL must not be fetched"))
    monkeypatch.setattr(lfs_remote.requests, "get", fetch)
    post = run_verify(monkeypatch, [pointer("data/a.tar.gz", OID_A)], response([available(OID_A)]))
    assert post.call_count == 1
    assert post.call_args.args[0] == "https://configured.invalid/lfs/objects/batch"
    assert post.call_args.kwargs["json"]["operation"] == "download"
    assert post.call_args.kwargs["json"]["objects"] == [{"oid": OID_A, "size": 42}]
    assert "payload.invalid" not in str(post.call_args.kwargs["json"])
    fetch.assert_not_called()


def test_missing_object_reports_path_and_oid(monkeypatch: pytest.MonkeyPatch) -> None:
    with pytest.raises(
        lfs_remote.LFSVerificationError, match="data/missing.tar.gz.*" + OID_A
    ) as caught:
        run_verify(monkeypatch, [pointer("data/missing.tar.gz", OID_A)], response([]))
    assert "bin/lfs_push" in str(caught.value)


def test_service_or_malformed_response_fails_closed(monkeypatch: pytest.MonkeyPatch) -> None:
    with pytest.raises(lfs_remote.LFSVerificationError, match="service/protocol failure"):
        run_verify(monkeypatch, [pointer("data/a.tar.gz", OID_A)], response({}))


def test_pointer_discovery_reads_commit_blobs_without_smudging(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    # This is the shape emitted by `git ls-tree -r -l -z`: the size is
    # right-aligned with repeated spaces before it.
    listing = b"100644 blob " + (b"1" * 40) + b"    42\tdata/a.tar.gz\0"
    pointer_blob = f"version {lfs_remote.POINTER_VERSION}\noid sha256:{OID_A}\nsize 42\n".encode()
    calls: list[list[str]] = []

    def fake_git(args: list[str], repo: Path) -> bytes:
        calls.append(args)
        return listing if args[0] == "ls-tree" else pointer_blob

    monkeypatch.setattr(lfs_remote, "_git", fake_git)
    assert lfs_remote.discover_pointers("HEAD") == [pointer("data/a.tar.gz", OID_A)]
    assert calls[0][0] == "ls-tree"
    assert all("lfs" not in arg for call in calls for arg in call)
