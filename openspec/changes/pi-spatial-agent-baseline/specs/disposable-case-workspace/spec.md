## ADDED Requirements

### Requirement: Isolated disposable workspace
Every case SHALL execute in a disposable rootless Podman container whose staged input is read-only and whose working directory is writable. Rootless Podman availability SHALL be verified on the host, and run-time validation SHALL fail closed if the runtime, image, mounts, or isolation requirements are not satisfied. The host SHALL expose no Pi tools directly; only the case-bound tools provided for that run may be callable by the agent.

#### Scenario: Inspect the case boundary
- **WHEN** an agent executes inside a running case container
- **THEN** it can read the staged input, write its working directory, and cannot modify staged input or call host Pi tools

### Requirement: Evidence export before destruction
The system SHALL attempt to export the working directory and logs before destroying the case container, SHALL retain successful or partial failed-run evidence, and SHALL destroy the case container unconditionally in `finally` cleanup.

#### Scenario: Finish an isolated run
- **WHEN** a case run ends normally or fails
- **THEN** the system attempts to export the working-directory contents and execution logs, retains failed-run evidence if export fails, and destroys the container regardless of export outcome
