## Why

DimOS commits can contain Git LFS pointers whose underlying objects were never uploaded to the custom LFS server. The current pre-commit check validates local pointer coverage only, so the defect reaches CI or users later as a smudge-time 404.

The custom LFS server already supports unauthenticated Git LFS Batch API download requests. CI can therefore validate remote availability for every pointer in a commit without downloading potentially large artifacts or requiring write credentials.

## What Changes

- Add a required GitHub-hosted CI gate that validates every Git LFS pointer in the commit under test against the configured LFS server.
- Fail the gate with the affected repository path and object ID when a pointer has no downloadable remote object.
- Preserve existing LFS guard, pre-commit pointer-coverage checks, and selective artifact-download tests.

## Affected DimOS Surfaces

- Modules/streams: None.
- Blueprints/CLI: None.
- Skills/MCP: None.
- Hardware/simulation/replay: None.
- Docs/generated registries: CI workflow and contributor guidance for uploading LFS artifacts.
- External protocols: Git LFS Batch API download operation on `lfs.dimensionalos.com`.

## Capabilities

### New Capabilities
- `lfs-remote-availability`: CI verification that every LFS pointer in a tested commit has a remotely downloadable object.

### Modified Capabilities
- None.

## Impact

Contributors receive an immediate, actionable CI failure instead of a later checkout or data-loading smudge error. The check makes metadata-only anonymous HTTPS requests and does not require LFS write credentials or artifact downloads. It must run for fork pull requests as a GitHub-hosted job. Tests must cover successful availability, a missing object response, and diagnostic output; development documentation must state that `bin/lfs_push` remains the required upload path.
