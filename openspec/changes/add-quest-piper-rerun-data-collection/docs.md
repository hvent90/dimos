## User-Facing Docs

- Add a collection guide under `docs/capabilities/` describing the dedicated Quest Piper Rerun blueprint, required physical hardware, the top-button toggle and active-take discard controls, and how to inspect the Rerun view.
- Document the collection-to-LeRobot workflow: locate the SQLite session recording, configure the collection task label, run 30 Hz dataprep, and inspect the resulting dataset before training.
- Cross-link the guide from the existing hosted Quest teleoperation documentation where appropriate, while clearly separating teleoperation engagement controls from collection controls.

## Contributor Docs

- None. The change follows established blueprint registration, recorder, and Rerun composition patterns; implementation-specific test coverage belongs with the affected modules.

## Coding-Agent Docs

- None. Repository-level guidance already covers generated blueprint registration and testing requirements.

## Doc Validation

- Run the repository documentation link checker if the new guide adds internal references.
- Run any repository Markdown/documentation validation configured for changed docs.
- Manually verify command examples against the generated blueprint name and current dataprep CLI/API before merging.

## No Docs Needed

Documentation is required because this adds a hardware-facing collection workflow, a new runnable blueprint, controller interactions, and a dataset-conversion path that operators must execute correctly.
