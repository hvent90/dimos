## Why

DimOS has geometric mapping, object detection, and memory components, but there is not yet a standalone stream-first semantic mapping module that can turn live or recorded robot observations into a queryable semantic object map. Starting QA with a recorded robot-dog room traversal keeps verification repeatable while preserving the same stream contracts needed for live operation.

This change introduces a narrow semantic mapping capability: consume camera RGB, LiDAR/pointcloud geometry, and odometry/pose streams; associate visual observations with pose/frame context; maintain a persistent ConceptGraphs-style semantic object map; and expose outputs that can be inspected during live operation or from recorded-data runs. The geometric SLAM/mapping stack remains independent; semantic mapping may reuse pointcloud/frame utilities but must not become a prerequisite for existing maps, costmaps, relocalization, navigation, or manipulation.

## What Changes

- Add behavior for optional standalone semantic mapping modules driven by live or recorded robot RGB, LiDAR/pointcloud, and pose streams.
- Add semantic frame features, object detections, and object-map entries that associate image/mask/crop evidence, projected pointcloud support, timestamps, frame IDs, pose context, labels, embeddings, and world coordinates when available.
- Support repeatable recorded-data verification using an existing robot-dog room-roaming recording whose available streams are validated before running semantic mapping.
- Preserve existing geometric mapping behavior: pointcloud, voxel, costmap, relocalization, and navigation contracts are not made dependent on semantic outputs.
- Preserve existing live and recorded-data behavior: memory2 recorded streams and live module streams remain input surfaces, and semantic mapping remains an optional consumer.
- No **BREAKING** public API, CLI, or hardware-safety changes are intended for this version.

## Affected DimOS Surfaces

- Modules/streams:
  - New optional semantic mapping modules consuming streams such as `color_image`, `camera_info` or static calibration, `pointcloud`, `lidar`, and pose-bearing observations such as `odom` or `fastlio_odometry` when present. `depth_image` is not required for v1.
  - Pose/frame context from live or recorded odometry, TF, or pose-stamped memory2 observations when available.
  - Optional semantic output streams for semantic frame features, semantic object detections, object/entity markers, semantic object-map snapshots, derived graph snapshots, or queryable semantic-memory records.
  - Existing geometric map streams such as `global_map`, `merged_map`, and `global_costmap` are related reference surfaces but remain independent of semantic mapping.
- Blueprints/CLI:
  - Add or update a semantic mapping blueprint or CLI flow if needed so the same optional modules can run against live streams and bounded recorded-data windows.
  - Existing `dimos --replay` and `--replay-db` flows are deterministic QA surfaces, not the architecture boundary.
  - Existing memory2/dataset summary tooling should be used to confirm recorded data exposes the required stream shapes before semantic mapping runs.
  - Any new runnable blueprint must be listed through the generated blueprint registry if introduced.
- Skills/MCP:
  - No required skill or MCP behavior in v1.
  - Future agent skills may query the semantic map, but this proposal does not require agent integration.
- Hardware/simulation/recorded-data QA:
  - Live-compatible stream contracts are primary; deterministic QA should use an existing robot-dog room-roaming recording with camera imagery, LiDAR/pointcloud geometry, and odometry/pose.
  - Hardware and simulation blueprint enablement can come after the stream contracts and backpressure behavior are validated; v1 introduces no robot actuation path.
  - No new autonomous motion behavior, robot actuation, or safety-critical control path is proposed.
- Docs/generated registries:
  - Document how to list live or recorded streams, run semantic mapping against the existing recorded-data QA window, and inspect outputs.
  - Update generated blueprint registry only if a new public blueprint is added.
  - Add user/developer notes distinguishing semantic map outputs from geometric SLAM, navigation costmaps, and object-scene registration.

## Capabilities

### New Capabilities

- `semantic-mapping`: Covers optional standalone semantic mapping from live or recorded robot RGB observations with LiDAR/pointcloud geometry and pose context, including stream-based ingestion, semantic evidence generation, persistent object-map output, optional derived graph output, and inspection/query behavior.

### Modified Capabilities

- None. There are no existing OpenSpec capability specs to delta-modify for this behavior.

## Impact

Users and developers gain a focused semantic mapping capability that can run through DimOS stream contracts and be verified repeatably from an existing robot-dog room recording before enabling broader live/sim blueprint variants. This reduces scope and risk while establishing the semantic map module’s input/output contracts.

Compatibility risk is low if the module remains optional and existing stream contracts are preserved. Main technical risks are stream availability, timestamp/frame alignment, camera/LiDAR calibration consistency, semantic backpressure under live operation, object association quality, and clear separation between semantic outputs and geometric map/costmap outputs.

Testing and QA should list the existing room-roaming recording streams, run the semantic mapping module against validated RGB, LiDAR/pointcloud, and pose inputs, confirm it produces inspectable semantic frame features, object detections, and persistent object-map entries, and verify existing live/recorded geometric mapping behavior is unchanged when the semantic module is absent.
