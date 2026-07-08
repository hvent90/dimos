# Concepts

This page explains general concepts.

## Table of Contents

- [Modules](/docs/usage/modules.md): The primary units of deployment in DimOS, modules run in parallel and are python classes.
- [Streams](/docs/usage/sensor_streams/README.md): How modules communicate, a Pub / Sub system.
- [Blueprints](/docs/usage/blueprints.md): a way to group modules together and define their connections to each other.
- [Runtime environments](/docs/usage/runtime_environments.md): run selected Python modules in named venv or project workers, or resolve native executable settings from named environments.
- [VGN MuJoCo Grasp Demo](/docs/usage/vgn_mujoco_grasp_demo.md): opt-in xArm7 TSDF reconstruction and grasp candidate visualization demo.
- [GPD MuJoCo Grasp Demo](/docs/usage/gpd_mujoco_grasp_demo.md): opt-in xArm7 pointcloud GPD candidate generation plus a separate manual agent-facing execution demo.
- [XArm Voxel Planning Viser Demo](/docs/usage/xarm_voxel_planning_viser_demo.md): opt-in xArm7 voxel-map to RoboPlan planning-scene sync demo with Viser.
- [RPC](/docs/usage/blueprints.md#calling-the-methods-of-other-modules): how one module can call a method on another module (arguments get serialized to JSON-like binary data).
- [Skills](/docs/usage/blueprints.md#defining-skills): An RPC function, except it can be called by an AI agent (a tool for an AI).
- Agents: AI that has an objective, access to stream data, and is capable of calling skills as tools.
