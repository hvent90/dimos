## Context

`CartesianIKTask` and `EEFTwistTask` currently rely on the legacy
`PinocchioIK` implementation. Cartesian control receives `PoseStamped` targets;
keyboard teleoperation sends `TwistStamped` commands to the EEF-twist task.
The two tasks need one control pipeline and one default backend while retaining
an explicit compatibility route for existing Pinocchio behavior.

## Goals / Non-Goals

**Goals:**

- Make generic Pink the default control IK backend for Cartesian and EEF-twist
  tasks in `ControlCoordinator`.
- Keep legacy PinocchioIK available only through explicit
  `backend="pinocchio"` selection.
- Validate named EEF frames, controlled joints, model mappings, and prepared
  URDF/Xacro assets before task startup.
- Re-anchor every control tick to the coordinator's measured joint state.
- Clamp `dt`, update a Pink frame task, solve one differential step, integrate
  once, apply joint and velocity limits, and emit a finite bounded position
  command.
- Hold safely on expected runtime solver errors or invalid solver output.
- Migrate common helpers and every shipped Cartesian/EEF-twist blueprint to
  the Pink default without a Piper backend exception.
- Migrate Piper from its MJCF/numeric EEF path to the matching existing
  Xacro/URDF model and named `gripper_base` frame.

**Non-Goals:**

- Self-collision or world-obstacle avoidance.
- Planning-world or dynamic-obstacle avoidance in live control.
- Replacing `WorldSpec` or manipulation planning Pink/Drake behavior.
- New streams, RPCs, skills, MCP tools, CLI commands, or generated registries.

## Control Architecture

`CartesianIKTask` owns a shared target-to-command pipeline. It validates the
task model and named EEF frame, prepares a target, reads measured joints,
clamps the tick duration, performs one backend solve, validates the finite
bounded result, and builds the servo-position output. Expected runtime solver
errors produce a bounded hold rather than an invalid command.

`EEFTwistTask` subclasses `CartesianIKTask`. It prepares a short-horizon pose
target by applying the latest twist to forward kinematics computed from the
current measured joints, then delegates the solve and output path to the base
task. Pose and twist streams remain independently routed by the coordinator.

The Pink backend loads the prepared direct URDF/Xacro model, resolves the
configured joint mapping and named EEF frame, owns a Pink configuration and
frame task, applies joint/velocity limits, solves one step, and integrates the
finite velocity using the clamped `dt`. The legacy backend uses the existing
PinocchioIK path only when the typed backend setting is explicitly
`"pinocchio"`.

Model preparation is shared and deterministic: Xacro arguments and package
paths are resolved before backend construction, the resulting URDF is used by
Pink, and frame/joint mismatches fail startup with diagnostics.

## Backend and Blueprint Decisions

### Pink is the default

The typed control configuration defaults to Pink. `backend="pinocchio"` is an
explicit escape hatch for compatibility and testing; no task helper or shipped
blueprint silently selects it.

Common Cartesian and EEF-twist helpers pass the backend selection through task
configuration. All shipped task blueprints use the default Pink path. Piper is
not special-cased by backend.

### Piper model and frame migration

Piper Cartesian and EEF-twist tasks stop using the MJCF/numeric EEF path. They
use the matching existing Xacro/URDF model and the named `gripper_base` frame,
with the same model/joint mapping validation as other robots.

### Planning/control separation

Control Pink is a local differential-IK backend. Manipulation planning retains
its separate `WorldSpec` and planning Pink/Drake integration. Neither layer is
changed to provide collision behavior by this proposal.

## Runtime Safety and Rollout

Measured-state anchoring prevents command lag from accumulating a virtual
configuration. The `dt` clamp, joint and velocity limits, finite-value checks,
bounded joint-delta checks, and hold-on-error behavior remain in the shared
pipeline.

Validate the default Pink path in simulation or replay at the coordinator rate
before hardware use. Benchmark end-to-end control latency, exercise Cartesian
and twist targets across normal workspace motion, verify model/frame mapping and
runtime error holds, and confirm emergency-stop readiness. Any hardware check
must be supervised and low speed.

## Risks / Trade-offs

- Pink may add control-loop latency; benchmark the complete coordinator path and
  retain explicit Pinocchio selection for compatibility.
- URDF/Xacro frame or joint mismatches can prevent startup; validate them before
  backend construction and provide actionable diagnostics.
- Differential IK can fail near singularities or conflicting limits; preserve
  finite-output validation and bounded holds for expected runtime failures.
- Sharing the target-preparation boundary through inheritance requires focused
  Cartesian and twist lifecycle tests, including timeout and clear behavior.

## Migration / Rollout

Implement the backend seam and shared pipeline first, then switch common helpers
and shipped task blueprints to Pink by default. Migrate Piper's model path and
EEF frame to the existing Xacro/URDF and `gripper_base`. Keep Pinocchio
available only when explicitly configured. Run focused tests and simulation or
replay latency validation before supervised low-speed hardware validation.

## Open Questions

- Confirm the exact existing Piper Xacro/URDF asset and package arguments for
  each hardware and simulation blueprint.
- Select the coordinator-rate latency budget and benchmark thresholds for Pink
  versus the explicit Pinocchio compatibility path.
- Define coordinator-visible diagnostics for startup model errors and bounded
  runtime holds without changing stream contracts.
