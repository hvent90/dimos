# DimOS Robotics Language

Shared vocabulary for DimOS robotics concepts. These terms define domain language, not implementation details.

## Language

**Robot Name**:
A stable planning-domain identity for a concrete robot/model instance, used in public planning group and flat joint-name scoping.
_Avoid_: World robot ID, hardware ID, namespace

**World Robot ID**:
An internal planning-world handle for a robot after it has been added to a backend world.
_Avoid_: Robot name, hardware ID, joint namespace

**Hardware ID**:
A control-layer routing identity for a hardware component. For manipulator robots, it normally matches the robot name at the coordinator boundary.
_Avoid_: Robot name when discussing planning semantics, world robot ID

**Planning Group**:
A named selectable serial kinematic chain of robot joints used as the unit of manipulation planning. After binding to a robot name, it is identified by a planning group ID and remains independent of backend world robot IDs.
_Avoid_: Move group, movegroup

**Planning Group Definition**:
The model-level declaration of a planning group before it is bound to a concrete robot in a planning world.
_Avoid_: Runtime group, robot ID

**End-Effector Association**:
Separate metadata used for pose-targeted operations. For a planning group defined by a chain, the end-effector link is the chain tip. For a planning group defined only by joints, there is no end-effector link. Documentation should use group end-effector language for this pose-target frame; operation names may use standard robotics terms such as forward kinematics and inverse kinematics when the planning-group scope is explicit in the parameters.
_Avoid_: Planning group definition, group target-frame pose

**Group Forward Kinematics**:
A group-centric query that computes the end-effector pose for one pose-targetable planning group. Only that planning group's joints affect the result; other planning-world joints do not need to be supplied or filled.
_Avoid_: Planning-world FK, collision-state projection

**Planning Group Selection**:
The set of one or more planning groups chosen for a planning request.
_Avoid_: Composite group

**Planning Target Set**:
The atomic manipulation UI/planning state built on top of a planning group selection: selected planning groups, target authoring state, combined IK joint target, whole-set feasibility, and generated plan. Per-group UI panels are views into this target set, not independent planning states.
_Avoid_: Independent group cards, per-robot plan state

**Auxiliary Planning Group**:
A planning group selected to participate in a planning target set without receiving a direct end-effector pose target in that request. It is solved, checked, planned, previewed, and executed with the whole target set; it simply has no assigned target gizmo. A planning group may be auxiliary in one request and directly targeted in another.
_Avoid_: Joint-only group, intrinsic auxiliary group

**Coordinated Planning Problem**:
A planning request over one or more selected planning groups that is solved as one combined joint-space problem with one synchronized result.
_Avoid_: Batch planning, independent planning

**Planning Group ID**:
An API-level identifier for a planning group, always written as `{robot_name}/{group_name}`. `/` is reserved as the delimiter and is not part of either component.
_Avoid_: Bare group name, robot ID

**Default Planning Group**:
The generated fallback planning group used by robot-scoped compatibility APIs when no planning group is specified explicitly. It is not inferred from arbitrary SRDF group uniqueness.
_Avoid_: Unique planning group, primary planning group

**Robot-Scoped Compatibility API**:
A convenience API that accepts a robot name for common single-robot calls but immediately delegates to planning-group APIs through the robot's default planning group. It does not define a separate planning model, storage model, or groupless execution path.
_Avoid_: Robot-scoped planner, groupless API

**Joint State**:
A joint-name-keyed robot state that can represent any set of joints and is not inherently coupled to a robot, planning group, planning group selection, or joint-name scope. At flat multi-robot or coordinator boundaries, joint names are required and are global joint names. Robot identity and local-vs-global meaning are provided by the API boundary or containing type, not by extra fields on the generic joint state.
_Avoid_: Planning-group-scoped state

**Collision Target Joint State**:
A partial joint state used as input to planning-world collision checking. Its joint names are required and must be global joint names. Unmentioned joints are filled from the current planning-world state before collision is checked on the resulting full world configuration. It has no planning-group semantics.
_Avoid_: Full state, group-scoped collision state

**Current Planning-World Joint State**:
A flat snapshot of fresh monitored controllable joint positions for all robots in the planning world. Joint names are global joint names. It is the source used to fill unmentioned joints when applying a collision target joint state; fallback/default backend states are not current planning-world joint state.
_Avoid_: Internal state, selected-group state, default model state

**Robot Model Joint Names**:
The ordered controllable joints of a robot model in the model's local namespace. This usually aligns with the model's actuated joints, but is not itself a planning group.
_Avoid_: Implicit planning group

**Local Model Joint Name**:
A joint name as it appears inside a robot model or SRDF before the model is bound to a concrete robot in a planning world.
_Avoid_: Runtime joint name, coordinator joint name

**Robot-Scoped Joint State**:
A single-robot joint state whose robot identity is explicit outside the generic joint state. Robot-scoped APIs may accept unnamed ordered joint vectors in robot model joint order; when joint names are present, they are local model joint names because the robot identity is already explicit.
_Avoid_: Namespaced local joint state, prefixed joint state

**Generated Plan**:
A flat planning result that may contain one or more robots. Joint states in a generated plan require names and use global joint names so the plan remains unambiguous across robot boundaries.
_Avoid_: Robot-scoped joint plan, local joint plan

**Group-Scoped Preview**:
A visualization request for a generated plan over a planning group or planning group selection. A visualization backend may render the whole robot when partial-group rendering is not practical, but the API scope remains the generated plan's selected planning groups.
_Avoid_: Robot-scoped preview API

**Global Joint Name**:
A boundary-level joint name that mechanically combines a robot name and local model joint name as `{robot_name}/{local_joint_name}` so it is stable and unique in flat joint-state representations, even when the local model joint name is already descriptive. `/` is reserved as the delimiter and is not part of either component.
_Avoid_: Resolved joint name, coordinator joint name, bare joint name, local joint name

**Robot Placement**:
The placement of a robot model within the planning world. Robot placement belongs in the robot model description rather than in a separate planning configuration transform.
_Avoid_: Planning base pose, config placement transform
