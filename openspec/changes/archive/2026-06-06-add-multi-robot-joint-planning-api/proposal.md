# Add multi-robot joint planning API

## Why

DimOS manipulation planning currently supports multiple robots in one planning world, but joint-space planning is exposed and executed one robot at a time. Dual-arm setups can therefore plan for `left_arm` and `right_arm` independently, while the desired developer behavior is to plan both arms as one synchronized joint-space problem with shared collision validation.

The existing world model already assigns each robot a `robot_id` with an ordered joint slice and performs collision checks against the full scene. This change uses that existing scope directly: a list of `robot_id`s becomes an ad-hoc planning group for coordinated multi-robot joint planning.

## What Changes

- Add a public manipulation-planning API for planning multiple robots together from current joint state to per-robot joint targets.
- Add internal composite joint-state planning that concatenates per-robot starts, goals, and limits in deterministic `robot_ids` order.
- Add composite collision checking that sets every participating robot in the same scratch world context before validating configurations and edges.
- Return synchronized per-robot paths/trajectories that can be previewed and executed through existing per-robot coordinator task wiring.
- Preserve existing single-robot planning behavior and APIs.

## Affected DimOS Surfaces

- **Modules**: `ManipulationModule` RPC/library surface for multi-robot joint planning.
- **Planning protocols**: `WorldSpec` and `PlannerSpec`-adjacent APIs for composite state, limits, and collision checking.
- **World backend**: Drake planning world composite state helpers.
- **Control execution**: existing coordinator trajectory tasks receive split synchronized trajectories; no coordinator arbitration model change is required.
- **Blueprints**: existing dual-arm manipulation blueprints should exercise the new API; no new blueprint names are expected.
- **Docs**: manipulation planning docs should describe multi-robot joint planning and dual-arm usage.
- **Hardware/simulation**: robot-facing execution must remain opt-in and use the existing preview/execute workflow.

## Capabilities

### New Capabilities

- `manipulation-stack`

### Modified Capabilities

- None.

## Impact

Developers gain a direct way to plan coordinated dual-arm joint motion without introducing SRDF/group parsing in the first iteration. The main compatibility risk is trajectory timing and ordering: composite planning must return paths split by robot in the same `robot_ids` order used for planning, and execution must preserve synchronized waypoint timing. Focused tests should cover vector concatenation/splitting, composite collision checking, and the public multi-robot planning API. Manual QA should use a library/RPC driver against a mock dual-arm planner surface before any hardware attempt.
