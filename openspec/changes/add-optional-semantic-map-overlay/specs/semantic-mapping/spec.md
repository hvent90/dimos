## ADDED Requirements

### Requirement: Optional semantic mapping input contract
DimOS MUST provide an optional semantic mapping capability that consumes live or recorded RGB, LiDAR/pointcloud, and pose streams without requiring `depth_image`.

#### Scenario: Required streams are available
- **GIVEN** a live or recorded run exposes RGB images, LiDAR or pointcloud geometry, and odometry or pose observations
- **WHEN** semantic mapping is enabled for a bounded run window
- **THEN** semantic mapping accepts those streams as sufficient input for v1 semantic evidence generation
- **AND** it does not require a `depth_image` stream to start processing

#### Scenario: Calibration is unavailable
- **GIVEN** RGB, geometry, and pose streams are available but camera-to-LiDAR calibration is unavailable or invalid
- **WHEN** semantic mapping processes the run window
- **THEN** it MUST avoid producing high-confidence 3D object detections from unverified projection geometry
- **AND** it MUST continue to preserve frame-level semantic evidence or skip geometry-dependent object proposals without failing unrelated live or recorded-data flows

### Requirement: Semantic evidence persistence
DimOS MUST persist semantic evidence with enough provenance to inspect, replay, and rebuild semantic outputs.

#### Scenario: Frame feature evidence is generated
- **GIVEN** semantic mapping selects an RGB frame for semantic processing
- **WHEN** frame-level semantic features are produced
- **THEN** the stored evidence MUST include the source timestamp, frame identifier, image reference or hash, model/preprocessing identity, embedding reference, and pose context when available
- **AND** the stored frame feature MUST remain inspectable independently of the persistent object map

#### Scenario: Object detection evidence is generated
- **GIVEN** semantic mapping produces an object proposal from an RGB mask or crop and projected LiDAR/pointcloud support
- **WHEN** object detection evidence is persisted
- **THEN** the stored evidence MUST include source provenance, mask or crop reference when available, embedding reference when available, associated point count, world-space geometry when available, confidence or quality metadata, and an evidence identifier
- **AND** later object-map updates MUST be traceable back to the evidence identifier

### Requirement: Persistent semantic object map
DimOS MUST maintain a queryable persistent semantic object map from semantic object detections while treating scene graphs as derived outputs.

#### Scenario: Detections describe a stable object over time
- **GIVEN** multiple semantic object detections refer to the same physical object across timestamps
- **WHEN** semantic mapping updates the semantic object map
- **THEN** the object map MUST preserve a persistent object entry with evidence references, observed time range, semantic labels or aliases when available, embedding references when available, and world-space geometry when available
- **AND** the object map entry MUST be inspectable without requiring a derived scene graph

#### Scenario: Derived graph output is enabled
- **GIVEN** stable object-map entries exist and graph derivation is enabled
- **WHEN** a graph snapshot is generated
- **THEN** graph nodes and relations MUST reference stable object-map entries or their evidence
- **AND** graph output MUST NOT become the canonical source of truth for object-map updates

### Requirement: Non-blocking live compatibility
DimOS MUST keep semantic mapping live-compatible and non-blocking for geometric mapping, control, and safety-critical streams.

#### Scenario: Semantic perception is slower than incoming frames
- **GIVEN** semantic model processing cannot keep up with the input stream rate
- **WHEN** semantic mapping is enabled during live-compatible operation
- **THEN** semantic mapping MUST drop, defer, or otherwise bound semantic work rather than blocking geometric mapping, robot control, or safety-critical streams
- **AND** current best-effort semantic outputs MUST remain inspectable when available

#### Scenario: Semantic mapping is absent
- **GIVEN** an existing live, simulation, or recorded-data geometric mapping flow is run without semantic mapping modules enabled
- **WHEN** the flow starts and processes its normal inputs
- **THEN** existing geometric map, costmap, relocalization, navigation, and inspection behavior MUST remain available without depending on semantic outputs

### Requirement: Semantic mapping inspection and QA
DimOS MUST provide an inspection path for semantic mapping outputs that supports deterministic recorded-data QA and live-compatible output inspection.

#### Scenario: Recorded-data QA window is processed
- **GIVEN** a recorded robot traversal exposes the required RGB, LiDAR/pointcloud, and pose streams
- **WHEN** semantic mapping processes a bounded recorded-data window
- **THEN** a user or developer MUST be able to inspect generated semantic frame evidence, semantic object detection evidence, and persistent object-map entries
- **AND** the same required stream contract MUST apply to live-compatible operation

#### Scenario: Semantic query surface is used
- **GIVEN** persisted semantic frame evidence or object-map entries exist
- **WHEN** a user or developer queries or lists semantic mapping outputs through the supported inspection surface
- **THEN** the response MUST identify matching frames or objects and provide enough provenance to inspect the source evidence
- **AND** missing optional graph outputs MUST NOT prevent frame or object-map inspection
