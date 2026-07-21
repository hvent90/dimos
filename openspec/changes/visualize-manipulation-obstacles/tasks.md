## 1. Visualization contract and Viser scene

- [x] 1.1 Define the internal obstacle-visualization adapter contract for accepted add/remove events without expanding `WorldSpec`, adding streams, or exposing skills/MCP tools.
- [x] 1.2 Extend the Viser scene to create planner-parity box, sphere, cylinder, and mesh entities from `Obstacle` pose, dimensions, mesh path, and valid RGBA appearance, with a consistent appearance fallback for unusable color data.
- [x] 1.3 Track obstacle entities by world obstacle ID under the local `manipulation.obstacles` namespace and remove the matching entity, proxy, and label together.
- [x] 1.4 Add the single `manipulation.obstacles` visibility control, default it to visible, and ensure toggling visibility preserves existing handles/render state and applies the current state to newly added obstacles.
- [x] 1.5 Implement accepted-mesh rendering failure feedback using a local proxy at the accepted pose and a user-visible failure label, without raising a planner-world failure or silently dropping the obstacle.
- [x] 1.6 Add focused fake-server/scene tests for primitive geometry parity, appearance fallback, mesh success/failure, removal cleanup, default visibility, and hidden-state persistence across additions.

## 2. Native planning-world mutation hook

- [x] 2.1 Add a narrowly typed direct hook facility to the concrete planning-world mutation path, preserving the original `DrakeWorld`/`RoboPlanWorld` object identity and avoiding a proxy world, queue, or polling loop.
- [x] 2.2 Invoke the add callback only after a real native obstacle addition and bookkeeping succeed, forwarding the accepted obstacle and returned ID exactly once; do not forward rejected, duplicate/no-op additions.
- [x] 2.3 Invoke the remove callback only after a real native removal succeeds, forwarding the matching obstacle ID exactly once; leave visualization unchanged for rejected or missing-ID removals.
- [x] 2.4 Preserve existing world locks and lifecycle behavior while making direct callback invocation safe for RPC and obstacle-monitor mutation threads; isolate visualization callback failures after native world state commits.
- [x] 2.5 Add backend-independent and concrete-world tests covering successful/rejected add/remove forwarding, exact-once behavior, no-op behavior, native world identity, and callback teardown/lifecycle safety.

## 3. Enabled startup and blueprint wiring

- [x] 3.1 Reorder manipulation startup so enabled Viser is initialized and its scene is ready after robot metadata is available but before the floor or any obstacle mutation.
- [x] 3.2 Install the direct hook on that existing concrete world before adding the floor, and route floor, RPC, and perception add/remove mutations through the same hook without reconstructing obstacles by polling.
- [x] 3.3 Ensure disabled visualization installs no hook, starts no Viser runtime, and leaves planning, obstacle outcomes, and actuation behavior unchanged.
- [x] 3.4 Wire the optional Viser configuration into the xArm6 planner-only blueprint using existing configuration/dependency conventions, without adding CLI, stream, skill, MCP, or generated-registry surfaces.
- [x] 3.5 Add startup-order and disabled-path tests proving the floor is visible on initial readiness, the first accepted obstacle needs no retry, and the concrete world identity is preserved.

## 4. Documentation and generated artifacts

- [x] 4.1 Confirm from `docs.md` that no user-facing, contributor, coding-agent, or general visualization documentation updates are required; do not modify documentation files.
- [x] 4.2 Confirm no blueprint registry regeneration is required because no blueprint name or generated registry input changes; do not modify generated registry artifacts.

## 5. Verification

- [x] 5.1 Run `openspec validate visualize-manipulation-obstacles`.
- [x] 5.2 Run focused pytest targets covering the Viser scene/visualizer tests, manipulation world/monitor tests, visualization factory tests, and startup integration tests.
- [x] 5.3 Run focused mypy validation for the changed manipulation planning and Viser visualization modules, resolving type errors introduced by the hook and adapter contracts.
- [x] 5.4 Manually run the xArm6 planner-only surface with Viser enabled and use the existing RPC API to add box, sphere, cylinder, and mesh obstacles, remove accepted and missing IDs, and verify exact visual parity and no visual update for rejected mutations.
- [x] 5.5 In the same xArm6 smoke test, verify the `manipulation.obstacles` checkbox is visible and enabled by default and persists when hidden across add/remove operations. Verify the accepted-mesh renderer-failure proxy with the focused fake-Viser test because Drake rejects malformed mesh assets before the live visualization hook runs.
