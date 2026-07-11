## 1. Deployment contracts and planning

- [ ] 1.1 Add external deployment model types for coordinator-visible module declarations, deployment specs, class-keyed module deployment metadata, launch envelopes, plan results, prepare results, and runtime/session identifiers.
- [ ] 1.2 Define the local packaged-Python deployment metadata needed to locate the external project, declaration, runtime implementation, entrypoint, target, and launch mode.
- [ ] 1.3 Implement import-reference resolution for `module.path:variable_name` and reject unresolved references, non-deployment-spec objects, classes, factories, and arbitrary callables.
- [ ] 1.4 Implement plan validation that classifies normal Python modules and external packaged-Python modules without mutating package state.
- [ ] 1.5 Add unit tests for deployment reference resolution, invalid reference errors, mixed-module planning, and plan-mode no-mutation behavior.

## 2. External declaration and proxy behavior

- [ ] 2.1 Add the coordinator-visible external module declaration surface for streams, config metadata, declared RPC methods, declared skills, and declared module refs.
- [ ] 2.2 Implement an external proxy that exposes declared RPC methods through the existing RPC backend and rejects undeclared Python object access.
- [ ] 2.3 Ensure external module refs are represented by declared proxies rather than live Python objects or pickled instances.
- [ ] 2.4 Add tests proving declared RPC calls use the existing RPC backend shape and undeclared attribute access fails clearly.

## 3. Local packaged-Python preparation and launch

- [ ] 3.1 Implement local packaged-Python preparation checks for `python/pyproject.toml` and optional `python/pixi.toml`.
- [ ] 3.2 Implement launch command selection: `uv run python ...` for uv-only projects and `pixi run uv run python ...` when `python/pixi.toml` exists.
- [ ] 3.3 Add the packaged runtime entrypoint that imports the real module implementation, instantiates the runtime `Module`, and serves declared RPC handlers through existing Module machinery.
- [ ] 3.4 Implement external process/session lifecycle management for launch, bounded RPC readiness wait, stop, restart where supported, and teardown.
- [ ] 3.5 Add tests for uv-only command selection, Pixi plus uv command selection, missing project file errors, readiness success, and readiness timeout failure.

## 4. Coordinator integration

- [ ] 4.1 Integrate the external manager path into `ModuleCoordinator` so mixed deployments route normal Python modules to `WorkerManagerPython` and external modules to the external worker path.
- [ ] 4.2 Preserve existing normal Python blueprint deployment behavior for blueprints that do not use deployment specs or external modules.
- [ ] 4.3 Wire external modules through coordinator-managed lifecycle calls, stream setup, declared module refs, and declared RPC access.
- [ ] 4.4 Add an end-to-end local test deployment containing at least one normal Python module and one local packaged-Python external module.
- [ ] 4.5 Add regression tests proving existing normal Python module deployment still works without requiring the external path.

## 5. Temporary launcher and examples

- [ ] 5.1 Add the temporary integration launcher with `plan`, `prepare`, and `run` commands for deployment spec import references.
- [ ] 5.2 Ensure `plan` prints or returns the deployment plan without staging or launching external processes.
- [ ] 5.3 Ensure `prepare` stages/checks local packaged-Python projects without launching runtime processes.
- [ ] 5.4 Ensure `run` performs plan, prepare, launch, readiness, coordinator wiring, and lifecycle for the end-to-end proof.
- [ ] 5.5 Add a local example package under `examples/`, preferably `examples/external-python-module/`, showing the declaration/runtime split, local `python/` package layout, deployment spec module-level variable, and temporary launcher usage.
- [ ] 5.6 Ensure the example package demonstrates declared RPC behavior with a visible result returned from the external runtime implementation.
- [ ] 5.7 Ensure the example package demonstrates the rest of the supported declared surface where safe locally: streams, lifecycle, config metadata, skills, and module refs.
- [ ] 5.8 Add automated coverage or a smoke test that runs the example package through the same plan, prepare, run, readiness, and declared RPC path expected of user packages.

## 6. Documentation

- [ ] 6.1 Update `docs/usage/modules.md` or a nearby user-facing guide with the external packaged-Python module concept and supported declared surface area.
- [ ] 6.2 Add a focused authoring guide for local packaged-Python external modules, including declaration/runtime split, supported project layouts, and temporary launcher usage.
- [ ] 6.3 Link to the new `examples/` package as the canonical runnable reference for declared RPCs and other supported external module behavior.
- [ ] 6.4 Update contributor docs for temporary launcher behavior, testing expectations, example-package manual QA, and startup timeout debugging.
- [ ] 6.5 Update coding-agent docs or `AGENTS.md` only if new reusable conventions are introduced during implementation.

## 7. Verification

- [ ] 7.1 Run `openspec validate external-python-module-deployment --type change --strict --no-interactive`.
- [ ] 7.2 Run focused pytest targets for deployment planning, external proxy behavior, packaged-Python prepare/launch, coordinator routing, readiness timeout, and the end-to-end local packaged module.
- [ ] 7.3 Run existing focused tests for normal Python module deployment and coordinator lifecycle behavior.
- [ ] 7.4 Run `uv run mypy dimos/` or a narrower agreed type-check target if full mypy is too slow for the implementation iteration.
- [ ] 7.5 Run docs link/snippet validation for changed documentation if the repo docs tooling is available.
- [ ] 7.6 Manually QA the temporary launcher through `plan`, `prepare`, and `run` against the `examples/` local packaged-Python package.
- [ ] 7.7 Manually call the example package's declared RPC and confirm the visible result is produced by the external runtime implementation.
- [ ] 7.8 If implementation adds or renames registered blueprints, run `pytest dimos/robot/test_all_blueprints_generation.py`; otherwise confirm no generated blueprint registry update is needed.
