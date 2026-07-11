## Context

DimOS currently deploys normal Python modules through `ModuleCoordinator` and `WorkerManagerPython`. The coordinator deploys module classes into forkserver worker processes, creates coordinator-side proxies, wires streams/module refs, and calls lifecycle RPCs. Runtime declared RPC calls already use the selected LCM/Zenoh RPC backend: the proxy calls `<ModuleName>/<method>`, and the module instance serves its declared RPC handlers in the worker process.

The proposed deployment model needs a real proof before expanding to native or SSH execution. The combined PR should therefore include both the internal deployment shapes and one end-to-end local packaged-Python external module path. The first external path should reuse current Module runtime behavior where possible while keeping heavy packaged dependencies out of the coordinator process.

Relevant current surfaces include:

- `dimos/core/coordination/module_coordinator.py` for deployment, stream wiring, lifecycle, and module refs.
- `dimos/core/coordination/worker_manager_python.py` and `python_worker.py` for current normal Python worker behavior.
- `dimos/core/rpc_client.py` and `dimos/protocol/rpc/` for declared RPC proxy behavior.
- `dimos/core/module.py` for runtime `Module` lifecycle, stream declarations, declared RPCs, skills, and module refs.
- `dimos/core/coordination/blueprints.py` for blueprint shape discovery and wiring.

## Goals / Non-Goals

**Goals:**

- Provide coordinator-visible external module declarations that describe module shape without importing packaged runtime dependencies.
- Allow a deployment spec to combine normal Python modules and local packaged-Python external modules.
- Route normal Python modules to `WorkerManagerPython` and packaged-Python external modules to an external worker path through the real `ModuleCoordinator` deployment flow.
- Implement internal `plan`, `prepare`, and `run` phases for local packaged-Python deployment.
- Provide a temporary launcher for `plan`, `prepare`, and `run` using a Python import reference to a module-level deployment spec variable.
- Provide an example package under `examples/` that demonstrates the external packaged-Python module pattern end-to-end, including declared RPC behavior.
- Launch packaged-Python modules from `python/pyproject.toml` projects with `uv`, and from `python/pyproject.toml` plus `python/pixi.toml` projects with `pixi run uv run`.
- Preserve normal declared RPC semantics by using existing LCM/Zenoh RPC transport directly between coordinator proxy and external runtime module.
- Use current RPC responsiveness as the initial readiness check.

**Non-Goals:**

- No public `dimos deploy` CLI, deployment registry, prepared-plan registry, manifest registry, or `dimos run <deployment>` integration.
- No remote SSH target support.
- No native Rust/C++ prepare/build/run support.
- No migration of existing `NativeModule` wrappers.
- No Poetry, Hatch, Conda-without-Pixi, Nix, Docker, or generic environment plugin support.
- No strict lockfile enforcement for `uv.lock` or `pixi.lock`.
- No arbitrary Python object passthrough, raw `getattr`, live object identity, pickle-based refs, or PythonWorker Actor behavior across the external boundary.
- No separate health protocol beyond lifecycle RPC readiness in the first PR.

## DimOS Architecture

The external path should preserve the current separation between control-plane deployment management and runtime data/RPC transports.

```text
DeploymentSpec reference
        │
        ▼
temporary launcher ── plan / prepare / run ──▶ ModuleCoordinator
                                                   │
                             ┌─────────────────────┴─────────────────────┐
                             ▼                                           ▼
                    WorkerManagerPython                         WorkerManagerExternal
                    normal Python modules                       external declarations
                             │                                           │
                             ▼                                           ▼
                    PythonWorker process                         ExternalWorker process
                    real Module instance                         packaged real Module instance
                             │                                           │
                             └──────────── LCM/Zenoh RPC ────────────────┘
```

### External declarations and runtime implementation

The coordinator imports only the lightweight external declaration. The packaged runtime imports the real implementation, which subclasses the declaration and the runtime `Module` so it can reuse existing `Module.__init__()`, stream setup, lifecycle methods, `serve_module_rpc(...)`, `@rpc`, and `@skill` behavior.

```text
Coordinator-visible declaration:
  shape only: streams, config metadata, declared RPCs, declared skills, declared refs

External packaged implementation:
  declaration + real Module runtime
  starts the normal RPC backend and serves declared RPC handlers
```

The declaration is a public developer contract. It should be sufficient for the coordinator to inspect stream names/types, declared lifecycle/RPC/skill methods, and declared module refs without importing the heavy external implementation.

### Deployment specs and routing

`DeploymentSpec` should contain the blueprint/module graph plus class-keyed deployment metadata for external modules. During planning, the coordinator should classify each module into either the current Python worker path or the external path. Normal Python modules must continue to deploy as they do today.

The combined PR should prove mixed routing through the actual `ModuleCoordinator` path. A standalone demo that manually constructs launch envelopes or directly drives `WorkerManagerExternal` is not sufficient.

### Plan, prepare, run

Internal phases should exist now even though the temporary launcher is the only user-facing entrypoint:

- `plan`: resolve the import reference, validate declaration/runtime metadata enough to produce a plan, and print/return the plan without mutating the filesystem.
- `prepare`: perform local package preparation/staging for external modules without launching them.
- `run`: convenience mode that performs plan, prepare, launch, readiness, coordinator wiring, and lifecycle until a prepared-plan registry exists.

The launcher reference syntax should match existing DimOS registry conventions for blueprint objects: `module.path:variable_name`. The resolved object must be a `DeploymentSpec` instance, not a class, subclass, factory, or arbitrary callable.

### Packaged-Python preparation and launch

External packaged Python projects use sibling implementation conventions:

- `python/pyproject.toml` is required for uv-only packaged Python.
- `python/pyproject.toml` and `python/pixi.toml` are required for Pixi + uv packaged Python.

Launch command selection:

- If `python/pixi.toml` exists: `pixi run uv run python ...`
- Otherwise: `uv run python ...`

The first PR should keep validation shallow: check for required files and fail clearly when they are missing. It should not require lockfiles or perform deep reproducibility validation.

### Example package

The change should add a local example package under `examples/`, preferably `examples/external-python-module/`, that acts as both documentation and manual QA for the combined PR. The example should demonstrate the complete supported surface for this first external packaged-Python path:

- coordinator-visible external declaration;
- packaged runtime implementation that subclasses the declaration and `Module`;
- local packaged project layout under a `python/` directory;
- deployment spec module-level variable resolvable as `module.path:variable_name`;
- temporary launcher usage for `plan`, `prepare`, and `run`;
- at least one declared RPC call that reaches the external runtime and returns a visible result;
- declared streams, lifecycle behavior, config metadata, skills, and module refs where they can be shown safely in a local-only example.

The example should be small enough for focused tests and manual QA. It should not require robot hardware, remote SSH, native build tools, or non-local services.

### RPC, lifecycle, skills, and module refs

External RPC should follow the current DimOS declared RPC pattern:

```text
Coordinator proxy -> existing LCM/Zenoh RPC backend -> real Module runtime process
```

`ExternalWorker` must not become a per-call RPC forwarder. It owns process/session lifecycle: prepare, launch, readiness wait, stop/restart/teardown. Declared RPC calls go through the same backend as normal Python module RPC calls.

Coordinator-side external proxies should behave like `RPCClient.remote(...)`: declared `@rpc` methods are callable; arbitrary non-declared attributes are not available. Runtime identity can initially follow the current `<ModuleClassName>/<method>` pattern, using the declaration/runtime module name consistently. Multi-instance collision handling is a known future pressure but is not part of the first PR unless required by tests.

Module refs should be declared and rebindable by contract, not by live object assignment. For this PR, refs should support declared RPC proxy behavior across the coordinator/runtime boundary. Full Python object ref semantics are explicitly out of scope.

### DimOS Spec Protocols and adapter protocols

This change should not introduce Protocols just to satisfy type checking. If an existing DimOS Spec Protocol is needed for a module ref contract, use the real project type directly. Any new adapter interfaces should represent real deployment/runtime boundaries, not analyzer-only abstractions.

### CLI entrypoints and generated registries

The temporary launcher is an integration harness, not a committed public CLI contract equivalent to `dimos run`. It should not require changes to `dimos/robot/all_blueprints.py`. If implementation later adds or renames registered blueprints, regenerate with `pytest dimos/robot/test_all_blueprints_generation.py`, but that is not expected for this scoped change.

## Decisions

1. **Combine PR1 and PR2 scope.** The internal shapes should land with an end-to-end local packaged-Python proof. Skeletons without a real coordinator-routed module would not validate the architecture.

2. **Use a temporary launcher instead of public CLI integration.** This keeps the first PR focused on architecture and behavior without committing to deployment registry UX.

3. **Exercise the real coordinator route.** The first proof must route through `ModuleCoordinator` so normal Python and external packaged modules are wired together under the same orchestration path.

4. **Use import references to module-level deployment spec variables.** This follows current blueprint registry style and avoids callable/factory ambiguity.

5. **Support uv and Pixi + uv only.** This covers the intended packaged-Python cases without prematurely designing a generic environment system.

6. **Runtime implementation is a real `Module`.** This reuses existing Module lifecycle, stream, RPC, and skill machinery while keeping the coordinator dependent only on declarations.

7. **Declared RPC parity, not Python object parity.** External modules support normal Module semantics for declared surface area. They do not emulate PythonWorker raw object behavior.

8. **Readiness uses RPC responsiveness.** The external worker launches the process and waits until the lifecycle RPC endpoint responds. A separate health protocol can come later if needed.

## Safety / Simulation / Replay

This change should be validated with local test modules and should not require robot hardware. It does not change robot motion commands, hardware safety policy, simulation behavior, or replay data handling.

Manual QA should avoid robot-facing blueprints unless the packaged module under test is explicitly non-actuating or simulated. If an external module exposes skills later, those skills must follow the existing DimOS `@skill` safety expectations and system prompt guidance.

## Risks / Trade-offs

- **RPC name collisions:** Current RPC naming is class-name based. This matches current behavior but may not support multiple instances of the same external declaration in one graph. Mitigation: keep first tests single-instance and document instance-scoped names as future work if needed.
- **Coordinator/runtime shape drift:** Declaration and implementation can diverge. Mitigation: validate that runtime implementation satisfies the declared RPC/stream shape at launch or readiness time where practical.
- **Packaging environment variability:** uv and Pixi availability can differ by developer machine. Mitigation: fail early with clear missing-tool/missing-file errors and keep checks shallow.
- **Lifecycle timing:** RPC readiness as health can race with process startup. Mitigation: use bounded retry with a clear timeout and process output context on failure.
- **Boundary confusion:** Developers may expect arbitrary Python object access because normal Python workers support it in some cases. Mitigation: docs and errors should state that only declared external surface area is supported.

## Migration / Rollout

Existing normal Python module deployment must remain compatible. External deployment should be opt-in through `DeploymentSpec` and external declarations.

Rollout steps:

1. Add deployment declaration/planning types and external proxy behavior.
2. Add local external worker/session preparation and process launch.
3. Integrate routing into `ModuleCoordinator` without changing default blueprint execution.
4. Add the temporary launcher and example/test packaged module.
5. Add an `examples/` package that demonstrates the declaration/runtime split, local package layout, deployment spec reference, temporary launcher, and declared RPC behavior.
6. Add focused docs for external packaged-Python module authors.
7. Verify normal module tests still pass and add external deployment tests.

Rollback is straightforward while this remains behind the new deployment spec path: remove or stop using the temporary launcher and external deployment spec while normal `dimos run` remains unchanged.

## Open Questions

- Should the first implementation enforce a declaration/implementation name match, or allow an explicit runtime name override?
- What is the minimum useful runtime shape validation before launch versus after RPC readiness?
- Should process logs from external packaged modules be integrated into the existing run registry immediately, or kept as launcher output in the first PR?
