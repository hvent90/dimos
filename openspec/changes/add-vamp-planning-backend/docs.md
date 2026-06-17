## User-Facing Docs

- Update manipulation planning documentation under `docs/capabilities/` or `docs/usage/` to describe optional VAMP planning backend support.
- Document that VAMP supports initial joint-space planning only, and that pose planning requires an explicitly compatible kinematics backend.
- Add nested config examples for official artifacts:

  ```python
  world={"backend": "vamp", "artifact": {"mode": "official", "robot": "panda"}}
  planner={"backend": "vamp", "algorithm": "rrtc"}
  ```

- Add nested config examples for custom user-prepared artifacts:

  ```python
  world={"backend": "vamp", "artifact": {"mode": "custom", "path": "/path/to/artifact"}}
  planner={"backend": "vamp", "algorithm": "rrtc"}
  ```

- Document that DimOS does not generate VAMP artifacts. Users who need unsupported robots must generate/build VAMP artifacts themselves and provide the custom path.
- Document Franka Panda mock-control support as the recommended initial robot target for VAMP planning tests and benchmarks, including catalog usage, LFS-backed URDF/SRDF asset expectations, and the fact that control remains mock by default.
- Document CLI override examples using nested config paths, including world backend, artifact mode, official robot name or custom path, planner backend, and algorithm.
- Document expected failure modes: missing optional VAMP dependency, invalid world/planner pairing, invalid artifact config, unsupported obstacle/environment conversion, and incompatible kinematics.

## Contributor Docs

- Update planning backend contributor guidance if present, or add a short section to manipulation planning docs explaining the typed backend config pattern.
- Mention that backend imports should be lazy and adapter-owned, with actionable dependency errors.
- Mention that migrated flat config fields must emit `DeprecationWarning` and should be scheduled for removal.
- Mention that VAMP artifact generation is intentionally out of scope for DimOS; contributor work should focus on loading official or user-prepared artifacts.
- Mention that Franka Panda support should follow the shared `RobotConfig` catalog pattern, store URDF/SRDF resources through the repo's LFS-backed data package pattern, and should not require a real hardware adapter for tests or benchmarks.

## Coding-Agent Docs

- No AGENTS.md update is required unless implementation introduces new recurring coding-agent rules.
- If docs under `docs/coding-agents/` include manipulation planning guidance, update them with the VAMP boundaries:
  - no synthetic Jacobian support,
  - no DimOS artifact-generation pipeline,
  - VAMP config lives in typed nested world/planner config,
  - VAMP planner does not own IK.
  - Franka Panda mock support is a catalog/coordinator test target, not a VAMP-specific hardware driver.
  - Panda model resources should be `LfsPath`-referenced LFS assets, not runtime downloads.

## Doc Validation

- Run any repository doc link validation command documented for DimOS if docs are changed.
- For Markdown examples with runnable Python snippets, run the documented `md-babel-py run <doc>` command where applicable.
- If diagrams are added or changed, run the documented diagram generation command such as `bin/gen-diagrams` if required by the touched docs.
- At minimum, run targeted tests that cover any code examples embedded in documentation or keep examples clearly illustrative if they cannot run without optional VAMP artifacts.
- Validate any Panda examples against the selected LFS-backed Panda model/SRDF assets and mock coordinator joint naming.

## No Docs Needed

Documentation is needed because the change adds a user-visible optional planning backend, new nested configuration shape, dependency behavior, artifact-loading expectations, explicit unsupported-capability semantics, and a mock-control Franka Panda target for tests/benchmarks.
