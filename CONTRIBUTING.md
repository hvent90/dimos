# Contributing to DimOS

DimOS welcomes community contributions. This guide explains how to start work, communicate scope, and submit a pull request that maintainers can review efficiently.

## Before you code

Use GitHub for public contribution coordination.

- Small, safe changes can go straight to a pull request.
- Non-trivial changes should start with a GitHub issue.
- Core architecture changes should start with a GitHub issue or discussion before implementation.

New contributors can start with issues labeled `good first issue`.

Core architecture includes modules, streams, transports, blueprints, agents, public APIs, robot/platform support, and major dependency changes.

## Pull requests

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
