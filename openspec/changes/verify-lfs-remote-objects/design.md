## Context

`bin/lfs_push` uploads LFS archives to the custom endpoint configured in `.lfsconfig`, while the pre-commit `lfs_check` hook confirms only that a local pointer exists for each data entry. A pointer can therefore be committed without its backing object being remotely available. CI currently downloads only selected archives.

`dimos/utils/test_lfs.py` demonstrates that `lfs.dimensionalos.com` supports the Git LFS Batch API `download` operation anonymously and returns presigned download actions. The new check must run safely on GitHub-hosted runners, including fork pull requests, without downloading archive contents.

## Goals / Non-Goals

**Goals:**
- Fail CI before merge when any LFS pointer in the tested commit lacks a remotely downloadable object.
- Provide a path- and OID-specific diagnostic that directs contributors to the established upload workflow.
- Keep requests small and credential-free.

**Non-Goals:**
- Upload, repair, or delete LFS objects from CI.
- Verify archive contents by downloading them.
- Replace pre-commit pointer coverage, `bin/lfs_push`, `git-lfs-guard`, or existing data integration tests.
- Change DimOS runtime data loading behavior.

## DimOS Architecture

This is CI and contributor-tooling work only. It adds a GitHub Actions job and a repository verification command or test that reads Git LFS pointer metadata and queries the configured LFS endpoint through its Batch API. No DimOS modules, streams, transports, blueprints, RPC references, Python `Spec` Protocols, skills/MCP tools, CLI commands, or generated registries are involved.

The job checks out with LFS smudging disabled, derives the complete pointer set from the commit under test, and submits OID/size metadata in bounded Batch API requests. It interprets a valid download action for each object as availability. A request failure, malformed response, missing object entry, or absent download action is a failure. Archive URLs returned by the API are not fetched.

## Decisions

- **Check every pointer in the target commit.** This detects both newly introduced and previously latent broken pointers. Changed-pointers-only checks and scheduled full audits were rejected because they can leave a mergeable commit containing an unusable historical pointer.
- **Use the Git LFS Batch API download operation.** It directly tests the actual remote availability contract without expensive downloads. `git lfs pull` was rejected because it transfers artifacts and obscures missing-object diagnostics.
- **Run as a GitHub-hosted required job for all pull requests.** The endpoint permits anonymous download Batch requests, so no secret is exposed to fork-originating code and no self-hosted infrastructure is required.
- **Fail closed for protocol uncertainty.** Server errors and unexpected Batch responses are availability-check failures, rather than skipped validation.
- **Use the existing configured LFS URL.** The verifier must respect repository configuration instead of duplicating the production endpoint as an independent constant.

## Safety / Simulation / Replay

No robot hardware, simulation, replay, or runtime control paths are affected. The job makes read-only metadata requests and does not retrieve artifact payloads.

## Risks / Trade-offs

- A temporary LFS endpoint outage blocks merges. This is intentional: a successful merge must represent a commit whose data artifacts can be downloaded. Diagnostics must distinguish service/protocol failures from confirmed missing objects.
- Very large pointer inventories may require batching. Requests must be bounded and preserve path-to-OID attribution in failures.
- GitHub-hosted runner network egress is an operational dependency. The first deployment validates it without any credentials; an outage is visible rather than silently bypassed.

## Migration / Rollout

Add the job as a required CI status check after its initial validation. Update the large-file management guide to explain the new failure and direct contributors to `bin/lfs_push`. No data migration or generated-file refresh is required. Rollback consists of removing the job if the custom endpoint is unavailable from hosted runners.

## Open Questions

None. The implementation should choose a reasonable bounded Batch API request size and preserve the specified failure behavior.
