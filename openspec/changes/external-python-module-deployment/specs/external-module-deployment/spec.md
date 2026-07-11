## ADDED Requirements

### Requirement: Deployment references resolve local deployment specs

DimOS SHALL provide a temporary deployment launcher that accepts a Python import reference to a module-level deployment spec variable and resolves it without invoking arbitrary factories or callables.

#### Scenario: Resolve a valid deployment reference
- **GIVEN** a deployment reference in the form `module.path:variable_name`
- **AND** the referenced variable is a deployment spec instance
- **WHEN** a developer runs the temporary launcher in plan mode with that reference
- **THEN** DimOS SHALL resolve the referenced deployment spec
- **AND** it SHALL report the planned modules and deployment targets without launching module processes.

#### Scenario: Reject an invalid deployment reference
- **GIVEN** a deployment reference that does not resolve to a deployment spec instance
- **WHEN** a developer runs the temporary launcher with that reference
- **THEN** DimOS MUST fail before preparing or launching modules
- **AND** the error MUST explain that the reference must point to a module-level deployment spec variable.

### Requirement: Plan prepare and run phases are explicit

DimOS SHALL expose explicit internal plan, prepare, and run phases for local packaged-Python external module deployment.

#### Scenario: Plan does not mutate local package state
- **GIVEN** a valid deployment spec containing a local packaged-Python external module
- **WHEN** a developer runs the temporary launcher in plan mode
- **THEN** DimOS SHALL validate and report the deployment plan
- **AND** it MUST NOT stage packages, create environments, or launch module processes.

#### Scenario: Prepare stages without launching
- **GIVEN** a valid deployment spec containing a local packaged-Python external module
- **WHEN** a developer runs the temporary launcher in prepare mode
- **THEN** DimOS SHALL perform required local package preparation for that external module
- **AND** it MUST NOT launch the external module runtime process.

#### Scenario: Run performs convenience deployment
- **GIVEN** a valid deployment spec containing normal Python modules and local packaged-Python external modules
- **WHEN** a developer runs the temporary launcher in run mode
- **THEN** DimOS SHALL plan, prepare, launch, wait for readiness, and run the deployment through the coordinator
- **AND** normal Python modules and external modules SHALL be available through their declared runtime surfaces.

### Requirement: Coordinator routes mixed module deployments

DimOS SHALL route normal Python modules and local packaged-Python external modules through their appropriate worker managers within the real coordinator deployment flow.

#### Scenario: Mixed deployment uses both worker paths
- **GIVEN** a deployment spec whose blueprint contains a normal Python module and a local packaged-Python external module
- **WHEN** DimOS runs the deployment
- **THEN** the normal Python module SHALL be deployed through the existing Python worker path
- **AND** the local packaged-Python external module SHALL be deployed through the external worker path
- **AND** coordinator-managed stream wiring, module refs, lifecycle calls, and declared RPC access SHALL operate across the mixed deployment.

#### Scenario: Existing normal Python deployment remains compatible
- **GIVEN** an existing blueprint that contains only normal Python modules
- **WHEN** the blueprint is deployed through the existing DimOS run path
- **THEN** DimOS MUST preserve the existing Python worker deployment behavior
- **AND** the new external deployment path MUST NOT be required for that blueprint.

### Requirement: Local packaged-Python projects use supported layouts

DimOS SHALL support local packaged-Python external modules that use either a uv project layout or a Pixi plus uv project layout.

#### Scenario: uv-only project is accepted
- **GIVEN** a local packaged-Python external module with `python/pyproject.toml`
- **AND** no `python/pixi.toml`
- **WHEN** DimOS prepares and launches the module
- **THEN** DimOS SHALL use the uv-based launch path
- **AND** the module runtime SHALL start from the packaged Python project.

#### Scenario: Pixi plus uv project is accepted
- **GIVEN** a local packaged-Python external module with `python/pyproject.toml`
- **AND** `python/pixi.toml` exists
- **WHEN** DimOS prepares and launches the module
- **THEN** DimOS SHALL use the Pixi plus uv launch path
- **AND** the module runtime SHALL start from the packaged Python project.

#### Scenario: Missing project file fails clearly
- **GIVEN** a local packaged-Python external module without `python/pyproject.toml`
- **WHEN** DimOS prepares the module
- **THEN** DimOS MUST fail before launching the module
- **AND** the error MUST identify the missing required project file.

### Requirement: External modules preserve declared Module semantics

DimOS SHALL support local packaged-Python external modules for declared lifecycle, streams, config metadata, RPC methods, skills, and module refs.

#### Scenario: Declared RPC call reaches external runtime
- **GIVEN** a local packaged-Python external module declaration with a declared RPC method
- **AND** the external runtime implementation serves that method
- **WHEN** coordinator-side code calls the declared RPC on the external module proxy
- **THEN** DimOS SHALL deliver the call to the external runtime through the configured RPC transport
- **AND** the caller SHALL receive the runtime method result.

#### Scenario: Undeclared Python object access is rejected
- **GIVEN** a local packaged-Python external module proxy
- **WHEN** coordinator-side code attempts to access an attribute that is not part of the declared external module surface
- **THEN** DimOS MUST reject the access
- **AND** it MUST NOT require live Python object passthrough from the external runtime process.

#### Scenario: Declared module refs are rebindable
- **GIVEN** a local packaged-Python external module declares a module ref to another module's declared surface
- **WHEN** the coordinator wires module refs during deployment
- **THEN** DimOS SHALL provide a declared proxy suitable for supported RPC calls
- **AND** it MUST NOT rely on pickled live module instances across the external boundary.

### Requirement: External runtime readiness is based on RPC responsiveness

DimOS SHALL treat a launched local packaged-Python external module as ready only after its required lifecycle RPC endpoint responds within a bounded timeout.

#### Scenario: Runtime becomes ready
- **GIVEN** an external worker has launched a local packaged-Python external module process
- **WHEN** the module runtime starts its RPC server and responds to the required lifecycle RPC
- **THEN** DimOS SHALL mark the module ready for coordinator lifecycle and wiring operations.

#### Scenario: Runtime startup times out
- **GIVEN** an external worker has launched a local packaged-Python external module process
- **WHEN** the required lifecycle RPC endpoint does not respond before the readiness timeout
- **THEN** DimOS MUST fail the deployment
- **AND** it SHALL provide enough error context for the developer to identify startup or packaging failures.

### Requirement: Example package demonstrates supported external module behavior

DimOS SHALL include a local example package under `examples/` that demonstrates the supported local packaged-Python external module workflow and declared module surface.

#### Scenario: Example package plans prepares and runs
- **GIVEN** the repository's local external packaged-Python example package
- **WHEN** a developer runs the temporary launcher in plan, prepare, and run modes against the example deployment reference
- **THEN** DimOS SHALL exercise the same deployment spec resolution, local package preparation, external launch, readiness, and coordinator routing behavior used by non-example packaged-Python modules
- **AND** the example MUST NOT require robot hardware, remote SSH, native build tools, or non-local services.

#### Scenario: Example package demonstrates declared RPC behavior
- **GIVEN** the repository's local external packaged-Python example package includes a declared RPC method
- **WHEN** the example is run through the coordinator deployment path and the declared RPC is called
- **THEN** the call SHALL reach the external runtime module through the configured RPC transport
- **AND** the example SHALL return a visible result that confirms the runtime implementation handled the call.

#### Scenario: Example package documents the supported surface
- **GIVEN** a developer reads the example package files or README
- **WHEN** they inspect how the example is structured
- **THEN** the example SHALL show the coordinator-visible declaration, packaged runtime implementation, deployment spec reference, supported package layout, and launcher commands
- **AND** it SHALL identify which declared streams, lifecycle behavior, config metadata, skills, module refs, and RPCs are demonstrated by the example.
