# Sim2

`sim2` is the parallel simulation stack. It does not share engines or robot-specific
adapters with `dimos/simulation`.

The ownership split is:

- `SimModule` owns one backend, canonical world state, simulation time, reset, and stepping.
- `ControlCoordinator` owns policies, task arbitration, and hardware-compatible semantics.
- One dynamically sized `RobotChannel` per robot carries complete actions and observations over
  shared memory.
- `WorldManifest` describes stable entities; `WorldStateFrame` carries tick-stamped dynamic state.
- Backends resolve robot models and compose `WorldSpec.scene`; control adapters never see backend
  model objects or indices.

See `ARCHITECTURE.md` for the invariants.

## Runnable stacks

```bash
dimos run unitree-go2-sim2
dimos run unitree-go2-sim2-lockstep
dimos --viewer none run unitree-go2-sim2-nav
dimos run xarm7-sim2
dimos --viewer rerun run unitree-g1-groot-sim2
```

The base Go2 stack is kinematic. `unitree-go2-sim2-nav` adds the cooked office, portable Rust
lidar, mapper, costmap, planner, and movement manager for fast navigation tests. The xArm7 and G1
stacks use MuJoCo. Lockstep uses the same coordinator and policies as live execution; only the
coordinator's tick source changes. The G1 stack uses the established Rerun robot, scene, costmap,
and path converters under `world/odom/g1`.

## Adding a robot

Put model knowledge next to the robot, for example
`dimos/robot/<vendor>/<robot>/sim2_profile.py`. A profile declares the stable robot ID, existing
control family, coordinator joint names, model path, capabilities, and backend-specific model
binding:

```python
def my_robot_sim2() -> SimRobotSpec:
    return SimRobotSpec(
        robot_id="my_robot",
        control_interface=ControlInterface.MANIPULATOR,
        dof=6,
        joint_names=tuple(make_joints("my_robot", 6)),
        model_path=MODEL_PATH,
        backend_options={"mujoco_spec": RobotSimSpec(...)},
    )
```

Use `sim_hardware(robot, sim_id="main")` in the blueprint. It selects the generic `sim` adapter
for the declared control family and validates the joint contract. A new robot using an existing
control family does not add an adapter or shared-memory layout. Add those only for a genuinely new
control family.

The blueprint owns execution policy explicitly:

```python
sim = SimConfig(
    sim_id="main",
    backend=MujocoBackend(),
    robots=(robot,),
    world=WorldSpec(revision="my-scene-v1"),
    execution=ExecutionConfig(
        mode=ExecutionMode.LIVE,
        physics_dt=0.002,
        control_decimation=5,
    ),
)
```

Scenes are resolved in the blueprint and composed by the backend:

```python
scene = resolve_scene_package("office")
world = WorldSpec(scene=scene, revision=scene.package_dir.name)
```

For MuJoCo, each `SimRobotSpec` contributes a robot-owned `mujoco_spec`; the backend attaches every
robot at its `SpawnPose` and prefixes model names. This permits multiple instances of the same
robot without actuator or body-name collisions. The kinematic backend consumes the same world and
robot contracts without loading MuJoCo.

For lockstep, set `mode=ExecutionMode.LOCKSTEP` on the simulation and `clock="sim"` with the same
`sim_id` on `ControlCoordinator`.

## Runtime outputs and scenario control

`SimModule` publishes canonical `world_manifest`, `world_state`, `odom`, `imu`, and `pointcloud`
streams. `odom` and sensor timestamps use simulation time. Every frame carries `episode_id`,
`physics_tick`, and `control_tick`, so consumers can reject stale data across reset boundaries.

Integration tests can use the `SceneControl` RPC surface: `set_agent_position`, `respawn_at`,
`add_wall`, and `publish_goal`. Respawn starts a new episode transactionally. Runtime wall
authoring is supported by backends implementing `SceneAuthoringBackend`; MuJoCo rejects it
explicitly because compiled topology cannot be changed in place.

## External sensors

External sensors consume `WorldStateFrame` and a separately configured `ScenePackage`
representation. `WorldManifest`, `WorldStateFrame`, and `SensorReady` have versioned JSON LCM
encodings so native modules can consume them without Python pickle. Each produced sample publishes
`SensorReady` with the source episode and physics tick. Tests can then call
`SimModule.step(..., await_sensors=["lidar"])` to require an exact sensor barrier.

Declare a portable raycast lidar on the robot profile and build it from that typed declaration:

```python
sensor = RaycastLidarSpec(
    sensor_id="lidar",
    implementation=SensorImplementation.PORTABLE,
    rate_hz=10.0,
    max_range=10.0,
)
config = SceneLidarConfig.for_scene(scene, sensor, scan_model="mid360")
lidar = scene_lidar_blueprint(config)
```

`scene_lidar_blueprint` carries its required typed LCM boundary with it. This is intentional: the
current native Rust module speaks LCM even when the rest of a blueprint uses Zenoh. Do not recreate
those transport mappings in robot blueprints.

Native sensors may query their backend directly, but their outputs must still identify the source
tick when deterministic tests need to await them.
