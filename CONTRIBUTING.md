# Contributing to DimOS

DimOS welcomes community contributions. This guide explains how to start work, communicate scope, and submit a pull request that maintainers can review efficiently.

## Start in the right place

Use GitHub for public contribution coordination.

- For small, safe changes, you may open a pull request directly. Examples include typo fixes, broken links, small documentation clarifications, and clearly isolated bug fixes.
- For non-trivial changes, open or comment on a GitHub issue before implementation. This includes new features, behavior changes, new modules, new skills, new blueprints, dependency changes, and larger documentation changes.
- For core architecture changes, start with a GitHub issue or discussion and wait for maintainer guidance before implementation.

New contributors can start with issues labeled `good first issue`.

## Changes that need discussion first

Please do not open unsolicited pull requests for:

- large rewrites or broad refactors
- new framework abstractions or public APIs
- new robot or platform support
- major dependency changes
- changes to core module, stream, transport, blueprint, or agent architecture

These changes may still be useful, but they need agreement on direction before code review.

## Pull request shape

Before opening a pull request, make sure it is:

- scoped to one issue or clearly stated problem
- linked to the relevant GitHub issue or discussion when required
- focused, without unrelated cleanup or formatting changes
- clear about what changed and why
- validated with the checks relevant to the files you changed

The pull request template asks for the details maintainers need. Keep deeper design discussion in the issue or discussion that the PR links to.

## Validation

Run the relevant local checks before submitting. Pre-commit and CI will also guide you toward the required checks for the files you changed.

For test guidance, see [docs/development/testing.md](docs/development/testing.md).

## AI-assisted contributions

AI-assisted contributions are welcome when the human contributor owns the result.

If AI materially helped prepare a pull request, disclose that in the PR. You are responsible for understanding the change, checking it against the agreed scope, and running relevant validation.

If you use an AI coding agent, ask it to explain:

- what files it changed and why
- which issue or discussion scope it followed
- what checks it ran and the results
- any risks, assumptions, or follow-up work

Review that explanation before submitting the PR.

## Contributor License Agreement

Before submitting a pull request, read and approve the [Contributor License Agreement](CLA.md).
