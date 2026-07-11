# External Python Module Example

Plan or prepare with:

```bash
PYTHONPATH=examples/external-python-module uv run python -m dimos.core.deployment.launcher plan deployment:deployment_spec
PYTHONPATH=examples/external-python-module uv run python -m dimos.core.deployment.launcher prepare deployment:deployment_spec
```

This example includes `python/pixi.toml`, so `prepare` runs `pixi run uv sync`
inside the external package and creates/updates the external environment before
any runtime process is launched.

Run keeps the coordinator alive until interrupted:

```bash
PYTHONPATH=examples/external-python-module uv run python -m dimos.core.deployment.launcher run deployment:deployment_spec
```

To smoke-test the declared RPC path directly, including the external-only
`humanize` dependency and the normal module ref call into the external module:

```bash
PYTHONPATH=examples/external-python-module uv run python -c 'from dimos.core.coordination.module_coordinator import ModuleCoordinator; from dimos.core.deployment.ref import resolve_deployment_ref; from deployment import ExampleClient, ExampleExternalDeclaration; spec = resolve_deployment_ref("deployment:deployment_spec"); coordinator = ModuleCoordinator.build_deployment(spec); print(coordinator.get_instance(ExampleExternalDeclaration).greet("qa")); print(coordinator.get_instance(ExampleClient).call_external_dependency("qa")); print(coordinator.get_instance(ExampleClient).roundtrip_stream("stream-qa")); coordinator.stop()'
```

The declaration in `deployment.py` is coordinator-visible. The runtime in
`python/example_external/runtime.py` subclasses the declaration and `Module` and
serves the declared `greet` RPC from the packaged Python project. The example
also declares a local stream surface, config value, a normal Python
`ExampleClient` that calls the external module by declared RPC, and a module ref
to the normal Python `ExampleHelper`; `greet_with_helper` proves the external
runtime receives a declared RPC proxy for that ref rather than a live object
instance.

The external runtime imports `humanize`, which is intentionally declared only in
`python/pyproject.toml`. The coordinator-side declaration in `deployment.py` does
not import `humanize`, proving that heavy or unusual runtime dependencies stay in
the external package environment.
