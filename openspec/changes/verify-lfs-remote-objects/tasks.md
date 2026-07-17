## 1. Implementation

- [x] 1.1 Add a focused verifier that discovers every Git LFS pointer in the commit under test, reads each declared OID and size, and groups requests into bounded Git LFS Batch API download calls using the repository-configured endpoint.
- [x] 1.2 Make the verifier fail closed for missing download actions, missing response objects, malformed responses, and transport or HTTP failures; report every affected path and OID with the `bin/lfs_push` remediation.
- [x] 1.3 Add a GitHub-hosted CI job that runs on all pull requests, including forks, checks out without LFS smudging, and invokes the verifier without credentials or archive downloads.

## 2. Tests and Documentation

- [x] 2.1 Add focused tests covering complete success, a missing remote object, a service/protocol failure, and path-to-OID diagnostics without downloading an archive payload.
- [x] 2.2 Update `docs/development/large_file_management.md` with the remote-availability gate, the meaning of its failure, and the `bin/lfs_push` remediation.

## 3. Verification

- [x] 3.1 Run `openspec validate verify-lfs-remote-objects`.
- [x] 3.2 Run the focused LFS verifier tests and the existing `dimos/utils/test_lfs.py` smoke tests where the self-hosted marker environment is available.
- [x] 3.3 Run the applicable documentation link validation for `docs/development/large_file_management.md`.
- [x] 3.4 Manually run the verifier in no-smudge mode against the current commit and confirm it completes through the custom LFS Batch API without downloading any LFS archive.
