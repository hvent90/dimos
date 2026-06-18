## User-Facing Docs

- Update `docs/capabilities/manipulation/adding_a_custom_arm.md` so custom robot descriptions are documented as normal filesystem paths rather than LFS-backed `LfsPath` values.
- Update manipulation/platform documentation that mentions in-tree or LFS robot descriptions, especially `docs/capabilities/manipulation/openarm_integration.md`, to reflect that supported built-in robot descriptions ship with DimOS.
- Update any user-facing examples that import `LfsPath` or call `get_data()` only to resolve URDF/mesh/SRDF robot descriptions.
- Keep large-data documentation focused on replay data, model weights, maps, datasets, recordings, and other non-description assets.

## Contributor Docs

- Update `docs/development/large_file_management.md` to clarify that robot descriptions are excluded from the LFS data workflow when they are built-in supported descriptions.
- Document where built-in robot descriptions live (`dimos/robot/descriptions/**`) and that new supported robot descriptions should include URDF, referenced meshes, and SRDF files as package data.
- Document that external robot descriptions are caller-provided paths and should not be represented as external repos managed by DimOS.
- Mention that files above the existing large-file threshold require narrow reviewed exceptions in `[tool.largefiles].ignore` when they are necessary meshes or description files.

## Coding-Agent Docs

- Update `AGENTS.md` or coding-agent docs if they continue to point contributors at LFS for robot URDF/xacro files.
- Add guidance that `get_data()`/`LfsPath` are not for robot descriptions; use the built-in description resolver for supported robots and plain `Path`s for external descriptions.

## Doc Validation

- Run doc link validation for changed docs, for example:
  - `python -m dimos.utils.docs.doclinks docs/`
- If edited docs contain executable code blocks, run the relevant doc-codeblock validation command from the repo docs workflow.
- Regenerate diagrams only if documentation changes touch diagram sources.

## No Docs Needed

Documentation changes are needed because existing docs currently describe LFS-backed robot-description workflows that this change intentionally removes.
