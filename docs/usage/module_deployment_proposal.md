# Proposal: Module Deployment for DimOS

Status: draft for review.

This proposal defines a shared deployment model for normal Python modules, packaged Python modules, native modules, and remote execution. The key idea is simple: DimOS should keep a stable module identity while deployment decides where and how the implementation runs.

## 1. Problem / Why now

DimOS has several deployment pressures that currently look separate:

- Python modules sometimes need heavy or conflicting dependencies that should not live in the coordinator environment.
- Native modules need repeatable build and runtime preparation.
- Remote deployment needs code or artifact sync, target preparation, process launch, logs, health, and cleanup.
- Weak robot computers may need prepared artifacts, cross-compilation, or runtime closures built elsewhere.
- Native and packaged Python modules need a shared way to describe config, stream topics, transports, and lifecycle handoff.

The common problem is **module deployment**.

The current local Python path works well for in-environment Python modules. But once a module has its own runtime requirement, DimOS needs an explicit deployment layer that can prepare the requirement, launch the implementation, and keep the Blueprint-facing module identity stable.

## 2. Current state

### Normal Python modules

Normal Python modules run inside the current DimOS Python worker environment.

```text
ModuleCoordinator
  -> WorkerManagerPython
    -> PythonWorker
      -> Python Module instance
```

They get the full DimOS surface: streams, RPCs, skills, module refs, lifecycle, and Blueprint wiring. This path should remain the default for local, in-environment Python modules.

### Current NativeModule

Today, a native module is a Python `NativeModule` wrapper deployed through the Python worker. The wrapper declares the DimOS-facing streams and config, then spawns an external executable.

```text
PythonWorker
  -> NativeModule wrapper
    -> native subprocess
```

The wrapper owns Blueprint integration, lifecycle, topic assignment, config serialization, logs, and process supervision. The native subprocess owns computation and direct pub/sub.

`NativeModuleConfig` already carries a proto launch recipe:

- `cwd`
- `executable`
- `build_command`
- `extra_args`
- `extra_env`
- `stdin_config`
- `auto_build`

When `stdin_config=True`, the wrapper sends a JSON payload to the native process:

```json
{
  "topics": {"input": "/topic#Type", "output": "/topic#Type"},
  "config": {"field": "value"}
}
```

That JSON is a useful starting point for a future **Module Launch Envelope**.

### Recent packaged Python exploration

Recent work on isolated Python runtime modules, including PR #2704, proves one backend: a Python module can keep a dependency-light Module Contract while the implementation runs in a prepared Python runtime project.

It adds:

- runtime environment registration,
- class-keyed runtime placement,
- deployment-time runtime reconciliation,
- runtime-specific Python worker pools,
- launch through the prepared `.venv/bin/python`,
- a runnable example package.

The broader lesson is not the exact worker implementation. The lesson is the seam: **what DimOS sees** can be separated from **where the implementation runs**.

### Native modules expose the same seam

Recent native work makes config and topics serializable, adds native transports, and places Python wrappers beside buildable native packages. Like packaged Python, it separates the DimOS-facing module contract from the implementation runtime. This proposal gives both paths one deployment model.

## 3. Proposed model

The proposal extends the existing manager/worker split instead of introducing a separate deployment stack.

```mermaid
flowchart TD
    Coordinator[ModuleCoordinator]

    Coordinator --> PythonManager[WorkerManagerPython\none per coordinator]
    PythonManager --> PythonWorkerA[PythonWorker]
    PythonManager --> PythonWorkerB[PythonWorker]
    PythonWorkerA --> ModuleA[normal Python Module]
    PythonWorkerA --> ModuleB[normal Python Module]
    PythonWorkerB --> ModuleC[normal Python Module]

    Coordinator --> ExternalManager[WorkerManagerExternal\none per coordinator]
    ExternalManager --> LocalWorker[ExternalWorker: local machine]
    ExternalManager --> RobotWorker[ExternalWorker: robot machine]
    ExternalManager --> GpuWorker[ExternalWorker: GPU machine]

    LocalWorker --> LocalHost[RuntimeHost\none per ExternalModule]
    RobotWorker --> RobotHostA[RuntimeHost]
    RobotWorker --> RobotHostB[RuntimeHost]
    GpuWorker --> GpuHost[RuntimeHost]
```

The ownership boundaries follow current DimOS:

```text
ModuleCoordinator       groups modules by deployment backend
WorkerManagerPython     schedules the normal Python worker pool
PythonWorker            controls one Python worker process
WorkerManagerExternal   coordinates external deployment across targets
ExternalWorkerClient    controls one target-side ExternalWorker
ExternalWorker          supervises all Runtime Hosts on one machine for one run
RuntimeHost             runs one external implementation
```

### 3.1 Module contract shape

`ExternalModule` is a declarative `Module` subclass. It adds an implementation reference but no build, process, watchdog, or transport behavior.

Packaged Python uses a class import reference:

```python
# Contract package: safe to import in the coordinator and control environment.
class HeavyDetector(ExternalModule):
    implementation = "heavy_detector.module:HeavyDetectorImpl"

    config: HeavyDetectorConfig
    image: In[Image]
    detections: Out[Detections]
```

```python
# python/src/heavy_detector/module.py
import heavy_dependency


class HeavyDetectorImpl(HeavyDetector):
    def start(self) -> None:
        ...
```

Runtime Host imports the class and requires:

```python
issubclass(HeavyDetectorImpl, HeavyDetector)
```

Native execution uses an executable path relative to its convention-discovered implementation folder:

```python
from pathlib import Path as FsPath


class MLSPlanner(ExternalModule):
    implementation = FsPath("target/release/mls_planner")

    config: MLSPlannerConfig
    global_map: In[PointCloud2]
    goal_pose: In[PoseStamped]
    path: Out[Path]
```

`ExternalModule.implementation` accepts `str | pathlib.Path` for ergonomic declarations. The convention-discovered sibling folder determines how DimOS interprets it: `python/` requires a class import reference, while `rust/` and `cpp/` require an executable path. The Python value type is not a runtime-kind flag.

The class location anchors package discovery. `ExternalModule` should not be sent to `PythonWorker`; it requires a Deployment Spec and the external worker path.

### 3.2 Complete Deployment Spec

A Deployment Spec references a Blueprint, defines reusable targets, and groups each module's deployment choices in one `ModuleDeployment`:

```python
go2_deployment = DeploymentSpec(
    blueprint=go2_stack,
    targets={
        "robot": SshTarget(
            host="go2",
            deployment_root="~/dimos-deployments/go2",
        ),
        "gpu": SshTarget(
            host="gpu-box",
            deployment_root="~/dimos-deployments/go2",
        ),
    },
    modules={
        MLSPlanner: ModuleDeployment(
            execution_target="robot",
            build_target="local",
            preparation=ArmCargoPreparation(),
        ),
        HeavyDetector: ModuleDeployment(
            execution_target="gpu",
        ),
    },
)
```

The v1 rules are concrete:

- `blueprint` supplies active modules, stream wiring, module refs, and config surfaces.
- `targets` maps stable string names to target definitions.
- `modules` maps Module classes to grouped deployment policy.
- `execution_target` defaults to `local`.
- `build_target` defaults to the execution target.
- omitted preparation and runtime-environment choices come from Convention Presets.
- `local` always exists as an implicit target.
- `local` uses a GlobalConfig-derived deployment root under DimOS state and cannot be redefined in `targets`.
- modules absent from `modules` use local defaults.
- an in-environment Python module, the most common module kind, cannot be assigned to a non-local target.
- a target's `deployment_root` contains DimOS-managed control environments, source snapshots, artifacts, runtime environments, run state, logs, caches, and locks.
- target probing rejects aliases that resolve to the same machine in v1.
- a class-keyed `ModuleDeployment` applies to every active Blueprint instance of that class; resolved plans and launch envelopes use unique Blueprint module-instance IDs.

### 3.3 Key model shapes

The public declaration remains small:

```python
class ExternalModule(Module):
    implementation: ClassVar[str | FsPath]
```

```python
@dataclass(frozen=True)
class DeploymentSpec:
    blueprint: Blueprint
    targets: Mapping[str, ExecutionTarget]
    modules: Mapping[type[ModuleBase], ModuleDeployment]
```

```python
@dataclass(frozen=True)
class ModuleDeployment:
    execution_target: str = "local"
    build_target: str | None = None
    preparation: Preparation | None = None
    runtime_environment: RuntimeEnvironmentSpec | None = None
```

```python
@dataclass(frozen=True)
class RuntimeEnvironmentSpec:
    implementation: str
    config: JsonObject = field(default_factory=dict)
```

```python
@dataclass(frozen=True)
class LocalTarget(ExecutionTarget):
    deployment_root: Path
```

```python
@dataclass(frozen=True)
class SshTarget(ExecutionTarget):
    host: str
    deployment_root: PurePosixPath
    expected_platform: Platform | None = None
```

`Preparation` produces and stages deployable material before ExternalWorker starts. `RuntimeEnvironment` materializes that staged material on the execution target before Runtime Host starts:

```python
class Preparation(ABC):
    async def prepare(self, context: PreparationContext) -> None: ...
    async def cleanup(self, context: PreparationContext) -> None: ...
```

```python
class RuntimeEnvironment(ABC):
    async def setup(self, context: RuntimeEnvironmentContext) -> RuntimeLaunch: ...
    async def teardown(self, context: RuntimeEnvironmentContext) -> None: ...
```

Convention Presets provide both objects for standard layouts. A `ModuleDeployment` overrides either one only when a deployment needs exceptional behavior. Preparation runs coordinator-side and may remain an ordinary local object. Runtime Environment runs remotely, so its override is a serializable `RuntimeEnvironmentSpec`: a top-level class import reference plus JSON-compatible configuration. The resolved plan never sends a live Python object over SSH.

Resolved plan, path, launch, environment-reference, and worker-route types remain internal.

The distinction is operational:

```text
DeploymentSpec   user-authored deployment intent
ModuleDeployment grouped policy for one module
ExternalModule   module-owned implementation declaration
DeploymentPlan   validated and fully resolved actions
```

### 3.4 Planning, prepare, and deploy

Deployment follows one ordered lifecycle:

```text
resolve modules, targets, and conventions
probe build and execution targets
prepare source and artifacts through Target Sessions
transfer content-addressed snapshots and outputs
bootstrap one ExternalWorker per execution machine
materialize Runtime Environments inside ExternalWorker
launch one Runtime Host per ExternalModule
wait for ready acknowledgements
```

`TargetSession` is coordinator-side target access. A local session executes commands and copies files directly. An SSH session executes remote commands, transfers files, bootstraps ExternalWorker, and tunnels its control RPC. `Preparation` may coordinate both its build and execution sessions, which supports local cross-compilation followed by remote transfer without splitting one preparation workflow into per-machine objects.

Target probing produces one resolved platform identity used by worker bootstrap, Preparation, and artifact validation. `expected_platform`, when supplied, is an assertion against those detected facts rather than a second platform declaration.

Source snapshots are transferred to content-addressed directories beneath `deployment_root` and published atomically. The snapshot contains the dependency-light contract and custom deployment extensions as well as implementation source. ExternalWorker imports a custom Runtime Environment by top-level import reference from this staged package before module dependencies exist, so the extension must remain dependency-light.

ExternalWorker itself uses a separate, versioned control environment beneath `deployment_root/control/`. TargetSession transfers a pinned environment tool and provisions the required Python and control dependencies there instead of relying on target-global Python or uv installations.

`WorkerManagerExternal` owns orchestration for the external backend:

```text
group modules by target
open or reuse Target Sessions
coordinate Preparation and artifact transfer
start or connect one ExternalWorker per execution machine
request Runtime Environment setup and Runtime Host launch
roll back successful work if another deployment fails
aggregate worker health and shutdown
```

Each `ExternalWorkerClient` sends requests over one control connection to its target-side ExternalWorker. The worker executes post-bootstrap deployment actions:

```text
materialize target-local Runtime Environments
spawn one RuntimeHost per ExternalModule
forward control requests by module ID
collect logs, readiness, health, and exit state
stop RuntimeHosts during rollback or shutdown
```

`DeploymentPlan` is immutable data consumed by the manager/worker path; it is not a separate behavioral reconciler layer.

Planning validates module declarations, target references, convention resolution, and required manifests before mutation. Preparation and Runtime Environment setup validate their generated artifacts before launch. A native executable may be a preparation output, so planning validates its destination rather than requiring it to exist in advance.

### 3.5 Resolved plan

The previous Deployment Spec should resolve to an inspectable plan:

```text
Module         Build   Execute  Preparation  Environment  Worker route
Agent          local   local    —            —            WorkerManagerPython -> PythonWorker
MLSPlanner     local   robot    Cargo cross  Native       WorkerManagerExternal -> ExternalWorker -> RuntimeHost
HeavyDetector  gpu     gpu      source sync  uv           WorkerManagerExternal -> ExternalWorker -> RuntimeHost
```

Plan, prepare, and run use the same resolution rules and plan schema. `dimos deploy prepare` persists a content-addressed plan manifest and source digest in the local run registry and target deployment roots. `dimos run <deployment>` loads that prepared manifest and verifies that the current declaration and source still match; otherwise it fails and asks the user to prepare again. `dimos deploy plan` remains a mutation-free preview and does not persist state.

### 3.6 Module Launch Envelope

After prepare and connection resolution, Runtime Host receives one unified envelope:

```python
@dataclass(frozen=True)
class ModuleLaunchEnvelope:
    module_id: str
    runtime: RuntimeLaunch
    config: ModuleConfigPayload
    topics: Mapping[str, TopicBinding]
    control: ControlEndpoint
```

```json
{
  "module_id": "mls_planner-1",
  "runtime": {
    "executable": "target/release/mls_planner"
  },
  "topics": {
    "global_map": {
      "channel": "/global_map",
      "type": "sensor_msgs.PointCloud2",
      "transport": "zenoh"
    }
  },
  "config": {
    "world_frame": "map",
    "voxel_size": 0.1
  },
  "control": {
    "endpoint": "..."
  }
}
```

This extends the current `NativeModule.stdin_config` shape. DimOS may track where fields originate internally, but Runtime Host receives one handoff containing launch metadata, module config, stream bindings, transport descriptors, and control details.

### 3.7 Process topology

```mermaid
flowchart LR
    Spec[DeploymentSpec] --> Planner[Deployment Planner]
    Modules[ExternalModule declarations] --> Planner
    Planner --> Plan[DeploymentPlan]

    Plan --> PythonManager[WorkerManagerPython]
    Plan --> ExternalManager[WorkerManagerExternal]
    Plan --> Envelope[ModuleLaunchEnvelope]

    PythonManager --> PythonWorker[PythonWorker]
    ExternalManager --> Session[TargetSession]
    Session --> Preparation[Preparation]
    Session --> ExternalWorker[ExternalWorker: one per machine]
    ExternalWorker --> PythonEnv[Python RuntimeEnvironment]
    ExternalWorker --> NativeEnv[Native RuntimeEnvironment]
    PythonEnv --> PythonHost[Python RuntimeHost]
    NativeEnv --> NativeHost[Native RuntimeHost]

    Envelope --> PythonHost
    Envelope --> NativeHost
```

## 4. Package discovery convention

The `ExternalModule` class file anchors package discovery. `ModuleDeployment` configures where that class builds and executes; it does not repeat or override implementation details.

The implementation directory is selected by a hardcoded sibling convention:

```text
python/pyproject.toml   packaged Python implementation
rust/Cargo.toml        Rust executable implementation
cpp/CMakeLists.txt     C++ executable implementation
```

Python may include Pixi or Nix metadata inside `python/`:

```text
python/
  pyproject.toml
  uv.lock
  pixi.toml
  pixi.lock
  src/...
```

The `implementation` field is interpreted using the discovered directory, regardless of whether the declaration used `str` or `Path`:

```text
python/   import a Python implementation class
rust/     launch an executable relative to rust/
cpp/      launch an executable relative to cpp/
```

### Existing native precedent

Current native modules already follow a lightweight convention:

```text
mls_planner/
  mls_planner_native.py        # NativeModule wrapper / Module Contract
  rust/
    Cargo.toml
    Cargo.lock
    src/...
```

The wrapper declares:

```python
class MLSPlannerNativeConfig(NativeModuleConfig):
    cwd = "rust"
    executable = "target/release/mls_planner"
    build_command = "cargo build --release"
    stdin_config = True
```

Deployment should generalize this convention rather than replace it with a manifest immediately.

The new API extends the design with a parallel declarative module, leaving the existing `NativeModule` untouched until migration:

```text
mls_planner/
  mls_planner_native.py        # existing NativeModule compatibility path
  mls_planner_external.py      # new ExternalModule declaration
  rust/
    Cargo.toml
    src/...
```

```python
from pathlib import Path as FsPath


class MLSPlanner(ExternalModule):
    implementation = FsPath("target/release/mls_planner")

    global_map: In[PointCloud2]
    path: Out[Path]
```

### Packaged Python mirror

Packaged Python should mirror the native shape:

```text
detector/
  detector_module.py           # ExternalModule declaration
  python/
    pyproject.toml
    uv.lock
    src/detector_runtime/
      module.py                # Runtime implementation
```

```python
class HeavyDetector(ExternalModule):
    implementation = "detector_runtime.module:HeavyDetectorImpl"

    image: In[Image]
    detections: Out[Detections]
```

V1 discovery rule:

1. Start at the `ExternalModule` class file.
2. Walk to the nearest package root.
3. Look for exactly one of `python/pyproject.toml`, `rust/Cargo.toml`, or `cpp/CMakeLists.txt`.
4. Interpret `implementation` as a Python class reference or executable path according to the matched convention.
5. Fail during planning if zero or multiple implementation directories match.

The convention can expand later, but v1 should not expose a project-root override.

### Convention presets

Users should not need to type raw `uv sync`, `pixi install`, `cargo build`, or CMake commands for every module. DimOS should ship **Convention Presets** that recognize common layouts and select default Preparation and Runtime Environment behavior.

Initial presets:

```text
python/pyproject.toml + python/uv.lock
  -> source Preparation + uv Runtime Environment

python/pixi.toml + python/pixi.lock + python/pyproject.toml
  -> source Preparation + Pixi-backed Runtime Environment

rust/Cargo.toml
  -> Cargo Preparation + native Runtime Environment

cpp/CMakeLists.txt
  -> CMake Preparation + native Runtime Environment
```

Preset matching uses the most specific convention within one implementation folder: a Python directory with Pixi metadata selects the Pixi-backed environment; otherwise `pyproject.toml` plus `uv.lock` selects uv. Automatic selection requires exactly one supported implementation folder. Zero or multiple matches fail unless `ModuleDeployment` supplies explicit Preparation and Runtime Environment behavior that selects and validates one implementation folder within the discovered package root. Overrides do not replace package-root discovery.

## 5. Worker / runtime architecture

DimOS keeps one manager per deployment backend:

```mermaid
flowchart TD
    Coordinator[ModuleCoordinator]
    Coordinator --> PythonManager[WorkerManagerPython]
    Coordinator --> ExternalManager[WorkerManagerExternal]

    PythonManager --> PythonWorker[PythonWorker pool]
    PythonWorker --> Normal[normal Python Modules]

    ExternalManager --> LocalWorker[ExternalWorker: local]
    ExternalManager --> RobotWorker[ExternalWorker: robot]
    ExternalManager --> GpuWorker[ExternalWorker: GPU]

    LocalWorker --> HostA[RuntimeHost]
    RobotWorker --> HostB[RuntimeHost]
    RobotWorker --> HostC[RuntimeHost]
    GpuWorker --> HostD[RuntimeHost]
```

There is one `WorkerManagerPython` and one `WorkerManagerExternal` per coordinator. `WorkerManagerExternal` owns one `ExternalWorkerClient` per execution machine. Each client communicates with one target-side ExternalWorker, which owns one `RuntimeHost` per deployed `ExternalModule`.

### Normal Python modules

Normal Python modules stay local and use the existing PythonWorker path.

```text
Normal Python Module -> PythonWorker
```

They are not remotely deployable in v1. If a module is assigned to a non-local target, it must be packaged Python or native.

This proposal does not replace PythonWorker. In-environment Python modules should keep using PythonWorker because its lightweight object and RPC envelope works well for normal local modules.

### Packaged Python modules

Packaged Python implementations are declared through `ExternalModule` and always use `WorkerManagerExternal`, `ExternalWorker`, and Runtime Host, even on the local machine.

```text
ExternalModule -> WorkerManagerExternal -> ExternalWorker -> Python RuntimeHost
```

The ExternalWorker spawns and supervises Runtime Host without importing user implementation code. Runtime Host imports the implementation inside the prepared Python environment, verifies that it subclasses the declared `ExternalModule`, initializes it, and sends an explicit ready acknowledgement.

### Native modules

Native implementations use the same `ExternalModule` contract and worker hierarchy for both local and remote targets.

```text
ExternalModule -> WorkerManagerExternal -> ExternalWorker -> Native RuntimeHost
```

The existing `NativeModule` remains unchanged during initial development. New native support uses `ExternalModule`; later PRs migrate existing native modules and eventually remove `NativeModule` and `NativeModuleConfig`.

### WorkerManagerExternal

`WorkerManagerExternal` parallels `WorkerManagerPython`. It owns coordinator-side external deployment state:

```text
target definitions
module deployment policies
target name -> TargetSession and ExternalWorkerClient
parallel preparation and deployment
rollback
health aggregation
shutdown
```

It resolves grouped module policies, coordinates pre-worker Preparation through Target Sessions, and then coordinates Runtime Environment setup and Runtime Host launch through ExternalWorker Clients.

### TargetSession and ExternalWorker

`TargetSession` provides coordinator-side access to one target. Local sessions execute commands and transfer files directly. SSH sessions execute commands remotely, transfer content-addressed snapshots and artifacts, bootstrap ExternalWorker, and maintain the SSH tunnel that carries worker RPC in v1.

`ExternalWorkerClient` is the coordinator-side RPC handle. `ExternalWorker` is the target-side process for one machine and deployment run:

```text
one control connection
worker ID and machine identity
deployed module IDs
RuntimeHost handles on that machine
lease and shutdown state
```

ExternalWorker materializes Runtime Environments, starts Runtime Hosts, forwards control requests by module ID, and stops hosts during rollback or shutdown. It does not build source or transfer artifacts. Those pre-worker operations belong to Preparation through Target Sessions.

V1 uses one ExternalWorker per machine per deployment run. A future persistent target agent can implement the same worker contract later.

Within one deployment run, modules share a Runtime Environment only when their source digest, execution-machine identity, and resolved `RuntimeEnvironmentSpec` fingerprint match. The run ID scopes the environment path so another deployment cannot mutate or tear it down. ExternalWorker serializes setup with a target-side lock and may rerun idempotent setup commands instead of merging preparation plans or maintaining a separate freshness database. Teardown occurs only after every dependent Runtime Host in that run stops. Cross-run reuse is limited to immutable source, artifact, and package-manager caches.

### Runtime Host

Runtime Host is the external equivalent of a module instance inside `PythonWorker`. It hosts exactly one `ExternalModule` implementation, receives one Module Launch Envelope, initializes stream/control bindings, and sends an explicit ready or failure response.

For packaged Python, Runtime Host is the prepared Python process: it imports and instantiates the implementation class in-process.

For native execution, Runtime Host is the DimOS host process around the executable: it passes the launch envelope to the child, forwards logs and control, supervises exit, and waits for the native SDK's ready acknowledgement. This extracts and generalizes the process-management responsibilities currently implemented by `NativeModule`.

## 6. Control plane vs data plane

Deployment needs two different communication paths.

### Deployment Control Plane

Control plane handles:

- spawn,
- stop,
- lifecycle,
- health,
- logs,
- status,
- method calls where supported.

For `ExternalModule` implementations, control flows through:

```text
ModuleCoordinator <-> WorkerManagerExternal <-> ExternalWorker <-> RuntimeHost
```

Remote v1 starts the target-side ExternalWorker over SSH and carries its loopback control RPC through an SSH tunnel. The detached worker survives a transient tunnel loss, but its coordinator lease continues to expire. The lease timeout is a bounded GlobalConfig value with a proposed v1 default of 30 seconds. WorkerManagerExternal may reconnect with the run's authenticated, single-worker resume token and resume heartbeats before expiry. A successful resume atomically advances the connection epoch, closes the prior control session, and rejects later RPCs or heartbeats from older epochs. Expiry is final: ExternalWorker invalidates the token and stops its Runtime Hosts. A persistent target agent is explicitly later work.

### Deployment Data Plane

Data plane handles module streams:

- images,
- point clouds,
- poses,
- paths,
- commands,
- maps.

Those streams continue to use transports such as Zenoh, DDS, ROS, LCM, or SHM where applicable. SSH never carries module stream data, and SHM remains machine-local.

V1 should report cross-target transport assumptions in `dimos deploy plan`, but it should not try to fully prove data-plane compatibility yet. Strict data-plane validation can come later. Section 3.6 defines the Module Launch Envelope that carries the resolved control and stream bindings into each Runtime Host.

## 7. End-user UX

The safe deployment flow should be explicit:

```bash
dimos deploy plan <deployment>
dimos deploy prepare <deployment>
dimos run <deployment>
```

### `dimos deploy plan`

Dry-run. No remote mutation.

It should show the resolved plan:

```text
Module           Contract        Target   Worker path
Agent            Module          local    WorkerManagerPython -> PythonWorker
MLSPlanner       ExternalModule  robot    WorkerManagerExternal -> ExternalWorker -> RuntimeHost
HeavyDetector    ExternalModule  gpu      WorkerManagerExternal -> ExternalWorker -> RuntimeHost
```

It should also show:

- build and execution targets,
- selected Preparation and Runtime Environment,
- source, artifact, and runtime paths,
- transport assumptions,
- obvious missing files or invalid config.

### `dimos deploy prepare`

Mutates targets but does not launch the stack.

It may:

- create deployment roots,
- sync content-addressed source snapshots,
- sync artifacts,
- build native artifacts,
- cross-compile elsewhere and copy outputs.

Prepare should be idempotent: run the build or sync tool as needed and rely on its cache or up-to-date checks. It does not materialize module Runtime Environments or launch ExternalWorker.

### `dimos run`

`dimos run <blueprint>` keeps today's local-default behavior.

A plain Blueprint bypasses Deployment Spec planning and uses the existing coordinator and `WorkerManagerPython` path. It cannot contain `ExternalModule` declarations because those require a Deployment Spec.

`dimos run <deployment>` verifies and loads the persisted prepared-plan manifest, bootstraps ExternalWorkers, ensures module Runtime Environments, launches Runtime Hosts, and registers the deployment with the same DimOS run lifecycle commands. If declarations or source no longer match the manifest, it fails before launch and asks the user to run `dimos deploy prepare` again.

Open question: should `dimos deploy <deployment>` become shorthand for prepare plus run later?

## 8. Lifecycle semantics

Deployment runs should participate in the existing lifecycle model.

- `dimos status` shows coordinator, managers, ExternalWorkers, and Runtime Hosts.
- `dimos stop` stops the whole deployment, not just the local coordinator process.
- `dimos restart` reruns the original command; it should not implicitly prepare unless the original command did.
- `dimos log` aggregates coordinator, ExternalWorker, Runtime Host, packaged Python, and native process logs.
- a shared Runtime Environment is torn down only after all dependent Runtime Hosts stop.
- immutable source snapshots and artifacts may remain cached beneath the deployment root.

Startup should be fail-fast:

```text
if any module fails before deployment is ready:
  stop already-started workers and runtime hosts
  mark deployment failed
```

Runtime Host death after startup should mark the deployment unhealthy. V1 should not auto-restart by default; robot safety makes restart policy an explicit later feature.

ExternalWorkers need a coordinator lease. If the coordinator disappears, each ExternalWorker should stop its Runtime Hosts instead of leaving orphaned robot processes. This lease cannot guarantee cleanup of arbitrary system changes, so safety-relevant provisioning must be explicitly bounded or self-expiring.

## 9. Implementation path / PR slicing

This should be built as a sequence of small PRs. PR #2704 should stay as a draft/reference for the packaged-runtime exploration; it should not merge as the public packaged-Python architecture because packaged Python and native implementations should share the `ExternalModule` and external worker path.

### PR 1: internal shape and skeletons

Define the internal deployment layer without adding user-facing CLI commands or public APIs.

Include:

- deployment data models and planning under `dimos/core/deployment/`,
- manager/worker skeletons beside the current worker implementation under `dimos/core/coordination/`,
- declarative `ExternalModule` base,
- internal `DeploymentSpec`, `ModuleDeployment`, and target models,
- `ModuleLaunchEnvelope` model,
- `WorkerManagerExternal`, `ExternalWorker`, and `RuntimeHost` skeletons,
- Preparation, Runtime Environment, and Convention Preset interfaces,
- isolated planner tests with fake modules and fake backends,
- a short in-tree design doc.

Exclude:

- no `dimos deploy ...` CLI,
- no `ModuleCoordinator` behavior changes,
- no packaged Python launch,
- no native migration,
- no remote execution.

### PR 2: local packaged Python on the unified path

Add the first narrow public feature: local packaged Python through a Deployment Spec.

This PR should:

- require a Deployment Spec even for local packaged Python,
- stage the local Python project,
- wire `WorkerManagerExternal` into `ModuleCoordinator`,
- launch a local Python Runtime Host through the local ExternalWorker,
- materialize its uv Runtime Environment inside ExternalWorker,
- require the imported Python implementation to subclass its `ExternalModule` contract,
- preserve Python module semantics where practical,
- avoid the runtime-specific PythonWorker pool approach explored in PR #2704.

Open question: what is the first runnable entrypoint before the full deployment CLI/registry exists?

### PR 3: local native automatic prepare/build

Add native automatic build/prepare and launch through `ExternalModule` without changing existing `NativeModule` wrapper files.

This PR should:

- use Deployment Spec,
- add a new `ExternalModule` declaration beside an existing native module,
- use Convention Presets for a native package such as `rust/Cargo.toml`,
- run prepare/build through `WorkerManagerExternal` and a local Target Session,
- install and validate the result through the native Runtime Environment,
- launch the executable through a Native Runtime Host,
- leave the existing `NativeModule` class and existing native wrappers unchanged.

Use the MLS planner package to prove the real sibling `rust/Cargo.toml` convention while keeping `MLSPlannerNative` available as the compatibility path.

### PR 4: NativeModule runtime migration

Migrate existing `NativeModule` declarations onto `ExternalModule`, ExternalWorker, and Native Runtime Host.

This PR should:

- migrate a tiny native example first,
- keep the legacy `NativeModule` path available for unmigrated modules,
- make the new path recommended for migrated modules,
- avoid a flag-day migration across all native modules.

After the path is stable, migrate `MLSPlannerNative` and then other native modules. Remove `NativeModule` and `NativeModuleConfig` only after all callers migrate.

### PR 5: SSH remote packaged Python

Add remote execution for packaged Python modules.

This PR should:

- define SSH target profiles,
- sync packaged Python source into the target deployment root,
- provision a hermetic, versioned ExternalWorker control environment,
- start one ExternalWorker for the remote machine over SSH,
- tunnel ExternalWorker RPC through SSH,
- materialize the Python Runtime Environment inside ExternalWorker,
- launch remote Python Runtime Hosts,
- keep the control contract compatible with a future persistent target agent.

### PR 6: SSH remote native modules

Add remote execution for native modules.

This PR should:

- sync content-addressed native source or prepared artifacts,
- support remote native build or cross-compile-and-sync through Target Sessions,
- materialize the native Runtime Environment inside ExternalWorker,
- reuse the machine's ExternalWorker,
- launch remote Native Runtime Hosts,
- keep packaged-Python remote and native remote separate because their prepare and failure modes differ.

## 10. Open questions

1. When should DimOS add optional `dimos.module.toml` or another manifest?
2. What is the PR 2 run entrypoint for local packaged-Python Deployment Specs before the full CLI/registry exists?
3. When should deployment definitions get YAML/TOML after Python-first API?
4. How should shared target profiles and local overlays layer secrets and personal machine details?
5. Should `dimos deploy <deployment>` ever become shorthand for prepare plus run?
6. When should strict data-plane compatibility checks become mandatory?
7. What is the minimal resolved-plan JSON schema for tooling and CI?
8. When should DimOS add bounded garbage collection for cached deployment roots?

## Suggested narrative

Use recent packaged-Python work as evidence, not as the whole story:

> Recent isolated-runtime work, including PR #2704, shows a seam between what DimOS sees and where a module implementation runs. This proposal extends that seam into a shared deployment model for native modules, packaged Python modules, and remote execution.
