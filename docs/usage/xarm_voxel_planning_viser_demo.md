# XArm Voxel Planning Viser Demo

`xarm-voxel-planning-viser-demo` is an opt-in xArm7 MuJoCo demo for manually validating voxel-backed RoboPlan collision checking with Viser visualization.

Run it with MuJoCo simulation:

```bash
dimos --simulation run xarm-voxel-planning-viser-demo
```

The stack contains `MujocoSimModule`, `PointCloudSelfFilter`, `TfPoseSource`, `RayTracingVoxelMap`, `PickAndPlaceModule`, and the xArm coordinator. It does not include the GPD/VGN grasp demo controllers or an agent loop.

Planning map stream chain:

```text
MujocoSimModule.pointcloud
  -> PointCloudSelfFilter.filtered_pointcloud
  -> RayTracingVoxelMap.global_map
  -> PickAndPlaceModule.planning_voxel_map
```

Viser renders the planning voxel map at `/planning/voxel_map` from the latest map synchronized into the planning world before planning.

Key configuration values:

- `world_backend="roboplan"`
- `planner_name="roboplan"`
- `kinematics={"backend": "roboplan"}`
- shared voxel/planning resolution: `0.05` m
- strict world frame: mapper and planning input use `world`

Known v1 limitations:

- self-filtering uses configured primitives only;
- no Scene Registry integration;
- no required workspace crop;
- voxel resolution is configured out-of-band/static;
- target-object voxels are not semantically excluded.
