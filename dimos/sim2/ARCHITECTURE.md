# Sim2 Architecture

Sim2 has one physics authority per `SimModule`. The backend owns canonical dynamic
state and simulation time. `ControlCoordinator` owns policies, task arbitration,
and hardware-compatible control semantics.

Policy actions and observations use one versioned, dynamically sized shared-memory
`RobotChannel` per robot. A channel publishes complete double-buffered frames; no
consumer may infer channel identity from a model path or read independently
sequenced state fields.

Backends implement world operations and return opaque robot handles. Backend model
types and indices do not cross into adapters, IPC, or control tasks. A robot using
an existing control family adds a model binding and profile, not a new adapter.
The backend resolves and composes `WorldSpec.scene` with every robot-owned model
binding. The shared runtime never branches on robot names.

`WorldManifest` owns stable entity identity, type, shape, and asset references.
Physics publishes tick-stamped `WorldStateFrame` transforms and velocities for
those IDs. Native sensors may query the backend, while external sensors load an
appropriate `ScenePackage` representation and join manifest metadata with world
state. Collision, visual, and radiance representations may differ while sharing
entity IDs and frames.

Live execution is wall paced and latches the latest complete action at control
boundaries. Lockstep execution advances only after the coordinator commits the
action for the current observation. Both modes apply actions before physics and
publish observations afterward.

`WorldStateFrame` and `SensorReady` are the only synchronization contracts for
external sensor samples. `WorldManifest` is the discovery contract. All three LCM
wire representations are versioned and backend-neutral. An external sensor loads
static geometry from the scene package, applies dynamic entity transforms from the
frame, publishes its sample, and only then acknowledges the source episode and
physics tick. It never reads MuJoCo state or a robot policy channel. Native-only
transport requirements are attached to the sensor blueprint rather than copied into
each robot stack.

Reset, respawn, step, background advancement, and runtime authoring are serialized
by the runtime mutation lock. Respawn creates a new episode before publishing the
new pose. Optional backend protocols expose pose mutation and scene authoring;
unsupported operations fail explicitly instead of weakening the base backend
contract.

Robot-specific model names, asset loaders, spawn poses, and capabilities live in a
robot-owned `sim2_profile.py`. Blueprints choose the backend and execution mode.
Rerun remains a consumer of canonical streams and uses the existing robot URDF,
scene-package, map, and path converters; sim2 does not introduce a second viewer
schema.
