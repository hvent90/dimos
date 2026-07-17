## Why

The existing corpus-only change provides a fixed public spatial evaluation corpus, but it does not yet define a reproducible external Pi-agent baseline or a trustworthy evaluation boundary. The current `pi-baseline run-paired` smoke scaffold is too Pi-specific and cannot schedule a general experiment. The baseline needs a repo-owned, agent-neutral scheduler over exactly the staged public case data, while retaining a hardened single-mode case executor that isolates arbitrary execution and dependencies, compares visualization policies fairly, and preserves enough evidence for a human release decision without exposing correctness during the run.

## What Changes

- Add an external Pi SDK runner authenticated with Codex OAuth and configured with model `openai-codex/gpt-5.6-luna` and a separate medium thinking budget/level.
- Stage each public case exactly from the corpus public artifacts: the selected scene, trajectory, question, variant snapshot, map, and instance, with no oracle or private answer material.
- Run every case in a disposable container with read-only staged input and a writable work directory. The host exposes no Pi tools; the runner provides only custom case-bound tools.
- Permit arbitrary Python execution, general outbound network access, and `uv` dependency installation. Both prompt modes include the identical instruction not to use online information or services to solve the task; package installation is allowed.
- Preserve Pi transcripts, tool traces, and sandbox command audits for post-run review, flagging observable network-oriented commands or configuration while stating that this heuristic cannot prove that online information or services were not used.
- Run two prompt modes with identical case data, tools, and behavioral instruction: one forbids visualization and the other explicitly encourages it.
- Replace `pi-baseline run-paired` with `pi-baseline experiment create|run|resume|retry|status|report`. The scheduler uses an executor contract of immutable expanded case + named condition + attempt context to normalized lifecycle events and a terminal outcome; Pi is one executor, not the universal tools or sessions abstraction.
- Define an experiment as an immutable expanded case set crossed with named conditions. Pi modes are generic conditions, and pairing is a reporting semantic rather than a scheduler primitive.
- Persist authoritative state only on one coordinator and one host-local POSIX filesystem: immutable attempt directories, append-only events, and atomically published job summaries. Use one coordinator-owned bounded in-process thread pool with a fixed manifest worker count (default 10); retain the Pi Node subprocess and rootless Podman boundaries and add no child Python worker layer or multi-host coordination.
- Hash an immutable manifest containing the expanded plan, executor/model/prompt/tool/corpus/image/scorer/limits/worker-count fingerprints. Any plan or execution-affecting change creates a fork; resume preserves the manifest and worker count.
- Resume only pending/interrupted jobs; explicit retry creates a new attempt with a reason and never overwrites evidence. Provide deterministic expansion/hashing, preflight, cancellation, retention, retry filters, failure triage, disk/evidence budgets, deterministic sampling/sharding, and comparison/reporting.
- Provide a small noninteractive Rich UI (live operational progress, static status, and `--json`) and keep correctness and private scores out of live views. Generate private reports only through an explicit post-completion/review command.
- Expose one immutable, typed `submit_answer` tool. It accepts the typed answer and provides no correctness feedback.
- Score submissions privately on the host and record the result in a host-side ledger that is not available to the agent or container.
- Require a human final gate covering infrastructure checks, the same fixed smoke sample in both prompt modes, and review of retained exported bundles: public staging, workspace, logs, transcript, dependency manifest, run configuration, prediction, and private score. Destroy the container after export.
- Keep corpus generation, corpus labels, and corpus artifacts unchanged; this proposal consumes the existing immutable corpus only.

## Capabilities

### New Capabilities

- `pi-sdk-runner`: Run the external Pi SDK baseline with Codex OAuth and the pinned model configuration.
- `public-case-staging`: Materialize the exact public-only input bundle for each corpus case.
- `disposable-case-workspace`: Execute a case in a disposable read-only-input, writable-work container and export evidence before destruction.
- `case-bound-tools`: Provide only custom tools bound to the staged case, including the immutable typed answer submission.
- `package-index-proxy`: Audit dependency installation and observable network-oriented activity without enforcing a network destination allowlist.
- `prompt-mode-parity`: Evaluate visualization-forbidden and visualization-encouraged prompts with identical data and tool access.
- `private-score-ledger`: Score predictions host-side without feedback and retain private ledger records.
- `human-release-gate`: Gate the baseline on infrastructure checks, paired fixed-smoke runs, and reviewable exported bundles.
- `experiment-scheduler`: Schedule immutable expanded experiments through a neutral executor contract with filesystem-only recovery, bounded local concurrency, operational status, and explicit private reporting.
- `evidence-retention`: Preserve immutable attempt evidence, append-only lifecycle events, atomic summaries, and retention/budget/triage metadata across interruption and retries.

### Modified Capabilities

- `pi-sdk-runner`: Retain/refactor the hardened single-mode Pi case executor behind the neutral executor contract; `run-paired` is retired.
- `prompt-mode-parity`: Treat Pi visualization modes as named conditions and paired comparison as reporting semantics.
- `private-score-ledger`: Consume terminal outcomes and publish scores only through an explicit private report.
- `human-release-gate`: Gate experiment completion/release review rather than owning scheduling.

## Impact

- Adds an external, host-orchestrated, agent-neutral experiment scheduler around the existing immutable public spatial corpus and retains/refactors the hardened single-mode executor.
- Adds container isolation, public-case staging, dependency and network-activity auditing, custom case-bound tools, private scoring, and retained run evidence.
- Establishes fair condition comparisons between visualization-forbidden and visualization-encouraged Pi modes without changing their data or tool access; comparisons are generated after execution.
- Requires Codex OAuth credentials, the pinned Pi model, rootless container runtime support, and host-side storage for private scores, transcripts, tool traces, sandbox command audits, and review bundles.
- Does not modify corpus generation, public corpus contents, authoritative answers, existing runtime capabilities, or the corpus-only change.
