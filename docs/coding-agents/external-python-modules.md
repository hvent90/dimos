---
title: "External Python Modules"
---

# Coding-agent guidance for external Python modules

When adding a local packaged-Python external module:

- Keep the coordinator-visible declaration lightweight. It should declare streams,
  config, RPCs, skills, and module refs without importing heavy runtime-only
  dependencies.
- Put the runtime implementation in the packaged `python/` project. The runtime
  class should subclass the declaration and `Module`.
- Use declared RPCs, skills, streams, and module refs only. Do not rely on
  arbitrary Python object access, live instance passthrough, or pickle-based refs
  across the external boundary.
- If the packaged runtime imports DimOS source, declare `dimos` as a local path
  dependency in the external project's `pyproject.toml`; `PYTHONPATH` alone does
  not install DimOS dependencies into the external `uv` environment.
- Demonstrate dependency isolation with a runtime-only dependency imported by the
  external implementation, not by the coordinator-visible declaration.
- Treat `prepare` as environment materialization: uv-only packages should sync
  with `uv sync`, while Pixi+uv packages should sync through `pixi run uv sync`.
- Exercise new examples through `dimos.core.deployment.launcher plan`, `prepare`,
  and a bounded `run` smoke because `run` intentionally stays alive.
