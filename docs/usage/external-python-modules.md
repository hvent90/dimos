---
title: "External Python Modules"
---

# Local packaged-Python external modules

DimOS can run a local packaged-Python module outside the coordinator's Python
environment while keeping the coordinator-visible module shape lightweight. The
coordinator imports an `ExternalModule` declaration that declares streams,
config, lifecycle RPCs, other `@rpc` methods, skills, and module refs. The
packaged runtime class subclasses that declaration and `Module` and serves the
existing DimOS RPC backend.

The example package pins `humanize` only in the external `python/pyproject.toml`.
The coordinator imports the declaration/spec without importing `humanize`; the
external runtime imports it and returns a formatted value through RPC and stream
roundtrip paths. This is the intended dependency-isolation shape.

Supported package layouts are:

- `python/pyproject.toml` → launched as `uv run python ...`
- `python/pyproject.toml` plus `python/pixi.toml` → launched as
  `pixi run uv run python ...`

`prepare` materializes the external package environment without launching the
runtime. Uv-only packages run `uv sync`; Pixi+uv packages run
`pixi run uv sync`.

Use a module-level `DeploymentSpec` and the temporary launcher:

```bash
PYTHONPATH=examples/external-python-module uv run python -m dimos.core.deployment.launcher plan deployment:deployment_spec
PYTHONPATH=examples/external-python-module uv run python -m dimos.core.deployment.launcher prepare deployment:deployment_spec
PYTHONPATH=examples/external-python-module uv run python -m dimos.core.deployment.launcher run deployment:deployment_spec
```

This launcher is an integration harness, not a stable replacement for `dimos
run` or a deployment registry. The canonical runnable reference is
`examples/external-python-module/`.

External module proxies expose declared `@rpc` methods only. Arbitrary Python
object access and live instance passthrough across the package boundary are not
supported.
