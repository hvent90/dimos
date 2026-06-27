## 1. Runtime Config and Planning

- [ ] 1.1 Add `backend_options` support to `BenchmarkEpisodeConfig` without breaking existing fake and Robosuite configs.
- [ ] 1.2 Add typed LIBERO-PRO backend option validation for benchmark name, task order index, task index, init-state index, controller, cameras, horizon, and asset roots.
- [ ] 1.3 Update runtime plan resolution or demo-side validation so `libero-pro` plans fail before blueprint startup when backend, robot id, motor count, protocol version, or command modes mismatch sidecar metadata.
- [ ] 1.4 Add a LIBERO-PRO benchmark config fixture for one registered suite/task using common top-level fields plus `backend_options`.

## 2. LIBERO-PRO Sidecar Package

- [ ] 2.1 Create `packages/dimos-libero-pro-sidecar/` with package metadata, import-safe module structure, and console script entrypoints.
- [ ] 2.2 Implement lazy LIBERO-PRO imports so importing the sidecar package does not import `dimos`, `libero`, `robosuite`, or `torch`.
- [ ] 2.3 Implement sidecar configuration/state models for registered-suite task selection and runtime settings.
- [ ] 2.4 Implement `/health`, `/describe`, `/reset`, `/step`, `/score`, and `/payloads/{id}` endpoints using the existing runtime protocol models.
- [ ] 2.5 Keep the LIBERO-PRO HTTP server single-threaded for MuJoCo/Robosuite render-context safety.

## 3. LIBERO-PRO Task and Motor Runtime

- [ ] 3.1 Resolve registered LIBERO-PRO tasks via `get_benchmark(benchmark_name)(task_order_index)` and `benchmark.get_task(task_index)`.
- [ ] 3.2 Create and reset `OffScreenRenderEnv`, apply `set_init_state(init_states[init_state_index])`, and return initial protocol state/metadata.
- [ ] 3.3 Validate the selected controller/action space exposes Panda joint-position plus gripper with the expected motor order and action dimension.
- [ ] 3.4 Translate `MotorActionFrame` position commands into LIBERO-PRO environment actions and translate returned simulator state into `MotorStateFrame`.
- [ ] 3.5 Export configured camera observations as runtime observation frames with fetchable `.npy` payload references.
- [ ] 3.6 Normalize sidecar-owned score output with success, reward or score, steps, benchmark name, task name, language, and init-state index.

## 4. Asset Bootstrap and Validation

- [ ] 4.1 Add explicit asset validation that detects missing BDDL and init-state assets for the selected registered suite/task before episode reset.
- [ ] 4.2 Add an optional asset bootstrap command or demo flag that uses Hugging Face APIs where enabled and stages supported assets into the expected LIBERO-PRO layout.
- [ ] 4.3 Ensure sidecar startup and `/health` validate assets without downloading or mutating local files unless explicit bootstrap was requested.
- [ ] 4.4 Document manual/prepared asset path usage and bootstrap behavior in the sidecar or runtime-sidecar docs.

## 5. Full-Control Demo

- [ ] 5.1 Add `scripts/benchmarks/demo_libero_pro_runtime.py` modeled on the Robosuite demo flow.
- [ ] 5.2 Launch the LIBERO-PRO sidecar subprocess, wait for health, fetch runtime description, and derive the resolved runtime plan.
- [ ] 5.3 Create the SHM motor owner and benchmark runtime hardware component for `HardwareComponent(adapter_type="benchmark_runtime")`.
- [ ] 5.4 Run a scripted `ControlCoordinator` servo loop that sends Panda motor position targets, posts `/step`, writes returned motor state to SHM, and records motor/protocol traces.
- [ ] 5.5 Fetch camera payloads and publish or record at least one camera observation through the runtime observation path.
- [ ] 5.6 Request `/score`, write score/artifact outputs, and guarantee sidecar, coordinator, and SHM cleanup on success and failure.

## 6. Always-On Tests

- [ ] 6.1 Add import-boundary tests ensuring the LIBERO-PRO sidecar package imports without loading `dimos`, `libero`, `robosuite`, or `torch`.
- [ ] 6.2 Add backend-options validation tests for valid registered-suite config and missing/invalid LIBERO-PRO option failures.
- [ ] 6.3 Add stubbed sidecar endpoint tests for `/describe`, `/reset`, `/step`, `/score`, and camera payload behavior without real LIBERO-PRO dependencies.
- [ ] 6.4 Add action-surface validation tests covering compatible Panda joint-position plus gripper metadata and incompatible OSC/action-dimension failures.
- [ ] 6.5 Add config/plan tests confirming existing fake and Robosuite configs still resolve after introducing `backend_options`.

## 7. Optional Real Integration

- [ ] 7.1 Add optional/manual test marker coverage for launching a real LIBERO-PRO sidecar with prepared assets.
- [ ] 7.2 Verify the full ControlCoordinator + SHM demo path against one registered LIBERO-PRO suite/task in a prepared environment.
- [ ] 7.3 Record manual verification instructions, required environment assumptions, asset preparation steps, and expected artifacts.
