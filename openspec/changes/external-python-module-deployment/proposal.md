## Why

DimOS modules currently assume that Python implementations can be imported and run inside the coordinator-managed Python worker pool. That blocks modules whose runtime dependencies should remain outside the main DimOS environment, and it leaves the proposed native/remote deployment model without a runnable proof point.

This change introduces a unified external deployment path with an end-to-end local packaged-Python module first. The goal is to validate the deployment architecture with a real `ModuleCoordinator` route before expanding the same path to native modules or SSH execution.

## What Changes

- Add a coordinator-visible `ExternalModule` declaration model for module shape: streams, config metadata, declared RPCs, declared skills, and declared module refs.
- Add `DeploymentSpec` planning for blueprints that can contain both normal Python modules and local packaged-Python external modules.
- Route normal Python modules through the existing `WorkerManagerPython` and local packaged-Python external modules through a new external worker path.
- Add explicit internal `plan`, `prepare`, and `run` phases for local packaged-Python deployment.
- Add a temporary integration launcher for resolving a deployment reference and exercising `plan`, `prepare`, and `run` without introducing a public DimOS CLI registry yet.
- Add an example package under `examples/` that demonstrates the external packaged-Python module pattern end-to-end, including declared RPC behavior.
- Support local packaged-Python projects using either `python/pyproject.toml` with `uv run python ...` or `python/pyproject.toml` plus `python/pixi.toml` with `pixi run uv run python ...`.
- Preserve declared RPC behavior by using the existing LCM/Zenoh RPC transport directly between the coordinator-side proxy and the real module runtime process.
- Do not support arbitrary Python object passthrough, live instance identity, pickle-based refs, remote SSH execution, native module execution, or generic environment plugins in this change.

## Affected DimOS Surfaces

- Modules/streams: Module deployment metadata, external module declarations, module refs, stream shape discovery, lifecycle RPC calls, declared skills/RPCs.
- Blueprints/CLI: Blueprint deployment through `ModuleCoordinator`; a temporary deployment launcher for import references such as `module.path:deployment_spec`. No public `dimos deploy` or `dimos run <deployment>` CLI yet.
- Skills/MCP: Declared `@skill` metadata for external modules should remain discoverable through the declared module shape and runtime `Module` implementation.
- Hardware/simulation/replay: No direct hardware, simulation, or replay behavior change. The first proof should be safe for local test modules only.
- Docs/generated registries: New developer documentation and an `examples/` package for external packaged-Python module declarations, runtime implementation, declared RPCs, and the temporary launcher. No generated blueprint registry change is required for this first PR.

## Capabilities

### New Capabilities

- `external-module-deployment`: Defines how DimOS plans, prepares, launches, wires, and controls local packaged-Python external modules through the coordinator.

### Modified Capabilities

None.

## Impact

Developers will be able to prove a packaged Python module can run outside the main DimOS Python environment while still participating in normal coordinator-managed lifecycle, stream wiring, and declared RPC calls. The included example package should become the concrete manual QA surface and reference implementation for module authors. The compatibility risk is mostly around preserving existing normal Python module deployment behavior while adding a second manager path. Dependency scope is intentionally narrow: `uv` is required for packaged Python projects and Pixi is supported only when `python/pixi.toml` exists. Test coverage should include planning validation, local prepare/run behavior, coordinator routing, readiness timeout behavior, declared RPC calls over the existing RPC backend, and the example package path.
