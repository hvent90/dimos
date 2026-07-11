## User-Facing Docs

- Add or update `docs/usage/modules.md` with a short section explaining that DimOS can run local packaged-Python external modules through a deployment spec while preserving declared streams, lifecycle, RPCs, skills, and module refs.
- Add a focused guide under `docs/capabilities/` or `docs/usage/` for authoring a local packaged-Python external module:
  - lightweight declaration imported by the coordinator;
  - packaged runtime implementation that is a real `Module` subclass;
  - supported project layouts: `python/pyproject.toml` and optional `python/pixi.toml`;
  - supported launch behavior: `uv run python ...` or `pixi run uv run python ...`;
  - temporary launcher usage for `plan`, `prepare`, and `run`.
- Link to the new `examples/` package as the canonical runnable reference for declaration/runtime split, deployment spec references, and declared RPC calls.
- Document that the temporary launcher is not a stable public replacement for `dimos run` or a deployment registry.

## Contributor Docs

- Update `docs/development/dimos_run.md` or add a development note describing the temporary deployment launcher and how it differs from the stable DimOS CLI.
- Add contributor notes for testing local packaged-Python external modules, including expected missing-tool and missing-file failures.
- Document how to run the `examples/` package through the temporary launcher in `plan`, `prepare`, and `run` modes.
- If external worker logs are integrated with existing run logs, document where they appear and how to debug startup timeouts.

## Coding-Agent Docs

- Update `docs/coding-agents/` or `AGENTS.md` only if implementation introduces new conventions that coding agents must follow when adding external modules.
- Candidate guidance:
  - do not import heavy external runtime dependencies in coordinator-visible declarations;
  - use declared RPCs/skills/refs only;
  - do not rely on arbitrary Python object passthrough for external modules;
  - after adding public blueprints, regenerate `dimos/robot/all_blueprints.py` with the existing blueprint generation test.

## Doc Validation

- Run docs link validation if available for changed markdown files.
- For executable Python snippets in docs, run the repository's documented markdown Python validation command if those snippets are marked executable.
- Run the focused external deployment tests referenced in `tasks.md` to ensure docs examples match behavior.
- Run the `examples/` package through the temporary launcher as manual QA.

## No Docs Needed

Documentation changes are needed because this introduces a new developer-facing module authoring and deployment path, even though it does not add a stable public CLI command yet.
