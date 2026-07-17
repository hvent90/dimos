## User-Facing Docs

None. This changes contributor CI feedback, not a robot-user or public runtime interface.

## Contributor Docs

Update `docs/development/large_file_management.md` to explain that pull requests validate all committed LFS pointers against the remote server, show the missing-object failure meaning, and direct contributors to `bin/lfs_push` before committing or updating an LFS pointer.

## Coding-Agent Docs

None. Existing repository agent instructions already identify LFS checks as part of pre-commit and CI behavior; this does not create a new coding-agent workflow.

## Doc Validation

Run the repository documentation link validation applicable to `docs/development/large_file_management.md` (for example, `doclinks` if available in the documented development environment).

## No Docs Needed

Not applicable: the contributor-facing failure and remediation path must be documented.
