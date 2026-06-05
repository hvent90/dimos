## User-Facing Docs

- Update `docs/capabilities/manipulation/openarm_integration.md` to describe two OpenArm/DMMotor paths:
  - existing `openarm` adapter with in-tree custom CAN driver,
  - new `dm_motor_arm` adapter using the `dm_control` Python binding when that binding is already installed.
- Update the OpenArm quick-start tables after blueprint names are finalized, including the opt-in DMMotor coordinator and the distinction from existing OpenArm trajectory/planner blueprints.
- Document that this change does not install `dm_control`; users must provide the Python binding in the active environment before selecting the new adapter.
- Document adapter-level gravity-compensation behavior: it computes gravity feed-forward in-place when enabled and can be disabled with `gravity_comp=False`.
- Add hardware bring-up guidance for mock/vcan, one-motor validation, full-arm state monitor, gravity compensation, and safe shutdown.

## Contributor Docs

- Update contributor-facing manipulation hardware guidance if the adapter introduces a reusable pattern for lazy optional SDK imports or binding-backed adapters.
- If new blueprint registry entries are added, mention the required generation command in implementation notes or relevant development docs: `pytest dimos/robot/test_all_blueprints_generation.py`.
- No broader development-process documentation is expected unless dependency packaging is added in a later change.

## Coding-Agent Docs

- Update `docs/coding-agents/` only if there is an existing manipulation or hardware-adapter guide that should mention the `dm_control` binding path and gravity-compensation QA steps.
- No `AGENTS.md` update is required unless implementation reveals a new repo-wide convention.

## Doc Validation

- Run documentation link validation for changed docs if available in the project workflow.
- Run `md-babel-py run docs/capabilities/manipulation/openarm_integration.md` if executable Python blocks are added or changed.
- Run `pytest dimos/robot/test_all_blueprints_generation.py` if new runnable blueprints are added and `dimos/robot/all_blueprints.py` changes.

## No Docs Needed

Documentation is needed because this change affects real hardware bring-up, adapter selection, dependency expectations, and operator-visible gravity compensation behavior.
