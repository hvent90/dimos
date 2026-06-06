## User-Facing Docs

- Add or update a capability guide under `docs/capabilities/` for optional semantic mapping once the runnable module or CLI surface exists.
  - Explain that v1 is a stream-first semantic evidence and object-map capability, not dense semantic SLAM.
  - Document required input streams: `color_image`, LiDAR/pointcloud geometry, and odometry/pose. Note that `depth_image` is not required for v1.
  - Document optional inputs: `camera_info` or static camera-LiDAR calibration, detector/tagger prompts, and recorded memory2 streams.
  - Show how to inspect semantic frame features, semantic object detections, persistent object-map entries, optional derived graph snapshots, and Rerun/entity markers.
  - Clarify that existing geometric maps, costmaps, relocalization, navigation, manipulation, MCP tools, and skills do not depend on semantic mapping.
- Add CLI examples wherever the implemented surface lands, likely in the capability guide and/or `docs/usage/cli.md`:
  - list live or recorded streams before running semantic mapping,
  - run semantic mapping against a bounded recorded-data QA window,
  - inspect persisted semantic outputs,
  - run the existing geometric mapping flow without semantic modules to verify independence.
- If a public blueprint is added, document it in the relevant blueprint/capability page after regenerating the blueprint registry.

## Contributor Docs

- Add implementation notes under `docs/development/` only if the change introduces new contributor-facing workflows beyond normal module development.
- Candidate contributor topics:
  - how to add or regenerate semantic mapping blueprints,
  - how to run deterministic recorded-data QA for semantic mapping,
  - how to mark GPU/model-dependent tests as optional or slow,
  - how to keep semantic mapping separate from geometric mapping contracts.
- If implementation only adds normal modules, streams, and tests, contributor docs can stay limited to links from the user-facing capability guide.

## Coding-Agent Docs

- Update `docs/coding-agents/` only if semantic mapping becomes a recurring coding-agent implementation area.
- Candidate coding-agent guidance:
  - treat semantic mapping as optional and non-blocking for geometry/control,
  - do not require `depth_image` for v1,
  - keep scene graphs derived from stable object-map entries,
  - use fake mask/embedding providers for default tests and reserve real SAM/CLIP execution for optional slow/manual QA,
  - avoid exposing semantic mapping through MCP/skills unless explicitly requested by a later change.
- No `AGENTS.md` update is required for this planning change because repo-wide coding guidance already covers modules, blueprints, streams, skills, and testing conventions.

## Doc Validation

Run validation for any docs changed during implementation:

```bash
doclinks
```

If docs include executable Python snippets, also run:

```bash
md-babel-py run <changed-doc.md>
```

If diagrams are generated rather than hand-written Mermaid, run the repository diagram generation command used by the affected docs, such as:

```bash
bin/gen-diagrams
```

For this planning artifact itself, OpenSpec validation is the relevant check:

```bash
openspec validate add-optional-semantic-map-overlay
```

## No Docs Needed

Documentation is needed once implementation starts because this change introduces a new user-visible capability and a new testing/inspection workflow. The docs should be added with the implementation surface, not deferred until after users can run the semantic mapping modules.
