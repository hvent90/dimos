## ADDED Requirements

### Requirement: Human release gate
The baseline SHALL NOT be released until a human has reviewed infrastructure checks, successful paired runs of the same fixed smoke sample in both prompt modes, mode-specific visualization compliance, and the retained exported review bundles. Image-read evidence supports policy review only; it SHALL NOT be treated as proof of image relevance or offline use.

#### Scenario: Approve a baseline release
- **WHEN** infrastructure checks pass, both fixed-smoke prompt-mode runs complete, and a human reviews their evidence
- **THEN** the human can approve the baseline release

#### Scenario: Block an incomplete release
- **WHEN** infrastructure checks fail, either fixed-smoke mode is missing or failed, or human review is incomplete
- **THEN** the baseline is not marked released

### Requirement: Scheduler-owned completion and review boundary
The release gate SHALL review completed experiment evidence and private reports after scheduling. It SHALL NOT own job allocation, retries, worker scaling, or live correctness display; operational scheduler status SHALL remain separate from private scores and oracle-derived data.

#### Scenario: Block visualization-policy failure
- **WHEN** the encouraged smoke run lacks a successful bounded image read or the forbidden smoke run attempts an image read
- **THEN** the baseline is not marked released

### Requirement: Complete retained review bundle
Each reviewed run SHALL retain exported public staging, writable workspace contents, logs, Pi transcript, tool trace, sandbox command audit, dependency manifest, flagged network-oriented observations, run configuration, prediction, and private score. If export fails, the host SHALL retain a failed-run evidence record and any partial evidence instead. The bundle or failed-run evidence SHALL remain reviewable after the container is destroyed.

#### Scenario: Review a destroyed-container run
- **WHEN** a human reviews a completed run after container destruction
- **THEN** the exported bundle contains the public staging, workspace, logs, transcript, dependency manifest, run configuration, prediction, and private score needed for review
