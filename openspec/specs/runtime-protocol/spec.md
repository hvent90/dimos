## Purpose

Define the lightweight backend-neutral protocol contract shared by DimOS runtime clients and simulator sidecars.
## Requirements
### Requirement: Shared runtime protocol package
The system SHALL provide a lightweight installable runtime protocol package that can be used by DimOS and simulator sidecars without installing the main DimOS package or any simulator backend SDK.

#### Scenario: Sidecar installs protocol without DimOS
- **WHEN** a Robosuite sidecar environment installs the runtime protocol package
- **THEN** it can import the protocol models and codecs without importing `dimos`, Robosuite-incompatible DimOS dependencies, or any DimOS hardware adapter modules

#### Scenario: DimOS imports the same protocol package
- **WHEN** the DimOS runtime client imports protocol models
- **THEN** it uses the same package and protocol version as the sidecar compatibility handshake

### Requirement: Protocol model validation
The protocol package SHALL define Pydantic models for runtime description, episode reset, step requests, step responses, robot motor surfaces, motor action frames, motor state frames, observation frames, scores, artifacts, and errors.

#### Scenario: Invalid step request is rejected
- **WHEN** a step request omits required episode identity, tick identity, or action payload fields
- **THEN** protocol validation rejects the message before backend-specific step logic runs

#### Scenario: Runtime description reports motor surface
- **WHEN** a sidecar describes a robot runtime
- **THEN** the response includes robot id, surface type, ordered motors, supported command modes, and available state fields

### Requirement: Protocol compatibility handshake
The runtime protocol SHALL include protocol version and capability metadata in the sidecar handshake so DimOS can fail fast on incompatible protocol versions or unsupported capabilities.

#### Scenario: Compatible sidecar connects
- **WHEN** DimOS connects to a sidecar using a compatible protocol version
- **THEN** the runtime client accepts the sidecar description and records the protocol version in artifacts

#### Scenario: Incompatible sidecar connects
- **WHEN** DimOS connects to a sidecar using an incompatible protocol version
- **THEN** prelaunch fails before launching the DimOS blueprint and records the incompatibility reason

### Requirement: Binary-friendly observation transport
The runtime protocol SHALL support image, depth, segmentation, and object/state observations without requiring large image tensors to be encoded as nested JSON lists.

#### Scenario: Image observation uses reference or binary payload
- **WHEN** a sidecar returns an RGB image observation
- **THEN** the observation frame includes stream name, kind, encoding, shape, dtype, and either a binary payload reference or a supported binary payload representation

#### Scenario: Referenced array payload is fetched separately
- **WHEN** an observation frame reports a raw array payload reference
- **THEN** the DimOS runtime client can fetch the referenced `.npy` bytes without embedding image arrays in the JSON step response

### Requirement: Backend-neutral protocol types
Runtime protocol models MUST NOT expose Robosuite, LIBERO-PRO, OmniGibson, DimOS hardware adapter, or simulator object types in public fields.

#### Scenario: Robosuite observation is translated
- **WHEN** Robosuite produces an `OrderedDict` observation
- **THEN** the Robosuite sidecar translates it into runtime protocol observation and motor state frames before sending it to DimOS

### Requirement: Native runtime action frames
The runtime protocol SHALL define a native runtime action frame for benchmark action surfaces that are not DimOS motor or joint command surfaces.

#### Scenario: Runtime action frame identifies semantic action surface
- **WHEN** a runtime action frame is serialized
- **THEN** it includes a discriminator, semantic action surface identifier, numeric action values, and sequence or tick identity without requiring motor names, motor command modes, gains, or joint position fields

#### Scenario: Runtime action frame validates numeric action values
- **WHEN** a runtime action frame contains non-finite values or values that cannot be parsed as a numeric vector
- **THEN** protocol validation rejects the frame before backend-specific step logic runs

### Requirement: Step request action frame union
The runtime protocol SHALL allow a step request to carry either a motor action frame or a native runtime action frame while preserving explicit frame discrimination.

#### Scenario: Motor step request remains valid
- **WHEN** an existing client sends a step request with a valid motor action frame
- **THEN** protocol validation accepts the request as a motor-frame step request

#### Scenario: Native runtime step request is valid
- **WHEN** a client sends a step request with a valid native runtime action frame
- **THEN** protocol validation accepts the request as a runtime-action step request

#### Scenario: Ambiguous action frame is rejected
- **WHEN** a step request action lacks a supported discriminator or mixes incompatible motor-frame and runtime-action-frame fields
- **THEN** protocol validation rejects the request before it reaches sidecar step logic
