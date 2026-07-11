---
title: "Developing External Python Modules"
---

# Temporary external Python deployment launcher

The `dimos.core.deployment.launcher` module is a contributor integration harness
for local packaged-Python modules. It is not a stable public replacement for
`dimos run` and does not write a deployment registry.

Useful QA commands for the example package:

```bash
PYTHONPATH=examples/external-python-module uv run python -m dimos.core.deployment.launcher plan deployment:deployment_spec
PYTHONPATH=examples/external-python-module uv run python -m dimos.core.deployment.launcher prepare deployment:deployment_spec
timeout 10s env PYTHONPATH=examples/external-python-module uv run python -m dimos.core.deployment.launcher run deployment:deployment_spec
```

`run` intentionally blocks after the coordinator reaches steady state, so a
timeout wrapper is expected for manual smoke checks.

`prepare` creates or updates the external package environment without launching
the runtime. Uv-only packages run `uv sync` from `python/`; packages with
`python/pixi.toml` run `pixi run uv sync` and then launch with
`pixi run uv run python ...`.

Troubleshooting notes:

- Missing `python/pyproject.toml` fails during prepare before launch.
- Missing `uv` or `pixi` appears as a local tool launch failure; Pixi is used
  only when `python/pixi.toml` exists.
- Startup timeout means the external process did not answer the declared
  `dimos_ready` RPC before `readiness_timeout_s`. Check the raised startup
  context, package import paths, and runtime class declaration/`Module`
  inheritance.
- Dependency-isolation examples should import runtime-only packages from the
  packaged implementation, not from the coordinator-visible declaration.
- Generated local artifacts such as `examples/external-python-module/python/.venv`,
  `uv.lock`, and `__pycache__` are test/runtime byproducts and should not be
  committed unless intentionally added later.
