# DimOS Planning

DimOS planning describes how robot motion-planning backends are represented, selected, and integrated into the manipulation framework.

## Language

**VAMP backend**:
An optional full planning backend in which VAMP owns the robot and environment representation used for planning.
_Avoid_: VAMP planner-only plugin, universal VAMP planner

**VAMP robot artifact**:
A prepared robot-specific VAMP bundle generated from robot description resources and compiled into an importable VAMP robot module.
_Avoid_: raw URDF, runtime robot model

**Artifact preparation**:
The offline process that turns URDF-derived robot resources into VAMP robot artifacts before planning runtime.
_Avoid_: runtime generation, dynamic URDF planning

**VAMP artifact recipe**:
A reproducible DimOS-owned description of how to generate a VAMP robot artifact from robot resources.
_Avoid_: generated artifact source, manual VAMP fork patch

**VAMP optional dependency**:
The pip-installable VAMP package used by DimOS only when the VAMP backend is selected.
_Avoid_: required manipulation dependency, vendored VAMP source

**Pinned VAMP dependency**:
A commit-pinned VAMP optional dependency used while the backend integration depends on unreleased packaging or robot artifact support.
_Avoid_: floating Git dependency, permanent fork dependency

**VAMP planner algorithm**:
The algorithm selected inside the VAMP backend after DimOS has selected the VAMP planner adapter.
_Avoid_: separate global planner name, backend name

**World backend**:
The selected planning-world implementation that owns robot registration, obstacle representation, collision checks, and synchronization for manipulation planning.
_Avoid_: hidden planner world, implicit backend

**Backend-agnostic pose planning**:
Pose planning in which DimOS converts a target pose into a goal joint state through a backend-agnostic IK solver before invoking the selected planner backend.
_Avoid_: VAMP-specific pose planner, backend-specific pose conversion

**Planner-native VAMP backend**:
A VAMP backend that exposes DimOS functions only where they are supported by VAMP's native planning and robot APIs, such as joint-space planning, path validation, collision checking, and forward/end-effector kinematics.
_Avoid_: synthetic Jacobian support, pretending unsupported planner capabilities exist

**VAMP kinematics boundary**:
The separation between VAMP-owned joint-space planning/world validity and DimOS-owned pose-to-joint kinematics. VAMP should not expose IK or Jacobian behavior unless VAMP or a compatible kinematics component naturally supports it.
_Avoid_: planner-owned IK, manufactured VAMP kinematics

**VAMP pose planning availability**:
Pose planning with VAMP is available only when DimOS has an explicitly compatible kinematics component for the VAMP world surface. It is not implied by selecting the VAMP planner.
_Avoid_: implicit VAMP pose support, fake Jacobian fallback

**Initial VAMP capability set**:
The first coherent VAMP integration surface: joint-space planning, VAMP algorithm selection, native path simplification/validation, official or user-prepared artifact loading, environment conversion, joint-configuration validity, joint limits when available, and native FK/end-effector pose queries.
_Avoid_: all-interface parity, synthetic unsupported methods

**User-prepared VAMP artifact**:
A robot-specific VAMP artifact generated outside DimOS by the user or by an upstream VAMP distribution, then loaded by DimOS at runtime.
_Avoid_: DimOS-owned artifact generation pipeline, automatic arbitrary-robot compilation

**VAMP artifact loading**:
The runtime mechanism by which DimOS selects either an official VAMP robot artifact or a user-specified custom artifact path for planning.
_Avoid_: dynamic artifact generation, implicit robot artifact synthesis

**Custom VAMP artifact path**:
A local world configuration value that points DimOS to a user-prepared VAMP artifact for robots not covered by official VAMP artifacts.
_Avoid_: hidden local artifact cache, hardcoded custom robot module

**Local world configuration**:
Backend-specific planning-world configuration attached to the world/robot planning context rather than to global `ManipulationModule` settings. VAMP artifact selection belongs here because it is part of how the local VAMP world is loaded.
_Avoid_: global VAMP artifact settings, module-wide custom artifact path

**Typed backend configuration**:
A discriminated configuration object that selects a backend with a `backend` field and carries backend-specific options in the same local config object.
_Avoid_: unrelated flat module-level backend fields, untyped string-only configuration

**Noisy config deprecation**:
A temporary compatibility path for legacy configuration that emits a visible deprecation warning when used, so future maintainers know the behavior is scheduled for removal.
_Avoid_: silent compatibility shim, permanent legacy alias

**Nested-only VAMP settings**:
New VAMP-specific settings that live only inside typed nested world/planner backend configuration. VAMP should not introduce new flat `ManipulationModuleConfig` fields; only pre-existing flat fields may receive temporary warning-backed compatibility paths during migration.
_Avoid_: `vamp_*` module fields, silent legacy aliases

**VAMP world/planner config split**:
The separation between VAMP world configuration, which owns artifact loading and world representation, and VAMP planner configuration, which owns algorithm selection and planner behavior.
_Avoid_: putting artifact paths in planner config, putting algorithm tuning in world config

**Strict VAMP world/planner pairing**:
The validation rule that VAMP world configuration must be paired with VAMP planner configuration, because VAMP planning depends on VAMP-native robot artifacts and environment representation.
_Avoid_: VAMP planner over Drake world, generic planner over VAMP world

**VAMP kinematics compatibility validation**:
The rule that VAMP pose planning is enabled only when the configured kinematics backend can operate on the VAMP world surface. Joint-space VAMP planning does not imply IK or Jacobian support.
_Avoid_: implicit VAMP pose planning, planner-owned Jacobian checks

**Unsupported world capability**:
A clear failure mode for optional world operations that a backend does not natively support, such as a VAMP world rejecting Jacobian queries instead of manufacturing synthetic Jacobian behavior.
_Avoid_: fake interface completeness, planner-owned capability probing

**Franka Panda mock support**:
A DimOS catalog/control-coordinator configuration for the Franka Panda arm that defaults to mock control while providing robot model metadata for manipulation planning tests and planner benchmarks.
_Avoid_: real hardware requirement, VAMP-only test robot

**LFS-backed robot description**:
A robot model package stored through DimOS' existing `data/.lfs/*.tar.gz` asset pattern and referenced in code through `LfsPath`.
_Avoid_: runtime URDF download, generated robot description at import time
