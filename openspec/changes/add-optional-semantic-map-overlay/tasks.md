## 1. Semantic data contracts and storage

- [ ] 1.1 Add `dimos/perception/semantic_mapping/` package with typed records for `SemanticFrameFeature`, `SemanticObjectDetection`, `SemanticObjectMapEntry`, `SemanticSceneGraphSnapshot`, and `SemanticRelation`, including source evidence IDs, timestamps, frame IDs, TF lookup metadata, projection quality, and resolved-frame geometry fields from `design.md`.
- [ ] 1.2 Add serialization/codecs or memory2-compatible payload support for semantic records so `SqliteStore` can persist and reload semantic frame features, object detections, object-map entries, and optional graph snapshots.
- [ ] 1.3 Add semantic stream/store naming helpers for `semantic_frame_feature`, `semantic_object_detection`, `semantic_object_map_entry`, `semantic_object_map_snapshot`, and `semantic_scene_graph_snapshot` so producers and query tools use consistent stream names.
- [ ] 1.4 Add tests that persist and reload each semantic record type through memory2 with evidence anchors, resolved spatial anchors, embedding references, and tags intact.

## 2. Frame feature generation

- [ ] 2.1 Implement a configurable `SemanticFrameFeatureModule` that consumes `color_image` and pose/TF context, selects bounded keyframes, computes or accepts image embedding references, and persists `SemanticFrameFeature` evidence without requiring `depth_image`.
- [ ] 2.2 Add a lightweight embedding-provider interface with a fake deterministic provider for default tests and an adapter point for CLIP/OpenCLIP/SigLIP-style providers without making heavyweight model execution required by normal pytest.
- [ ] 2.3 Implement live backpressure/keyframe controls for frame features, including maximum frame rate, minimum motion/novelty settings, queue bounds, and drop/defer behavior that cannot block geometry or control streams.
- [ ] 2.4 Add focused tests for keyframe selection, provenance fields, model/preprocess IDs, image hash/reference storage, pose context handling, and non-blocking behavior under a slow fake embedding provider.

## 3. Object proposal and 2D-to-3D projection

- [ ] 3.1 Implement a mask/proposal-provider interface for SAM/SAM2 or detector-prompted masks with a fake deterministic provider for default tests and optional real-model wiring for manual or slow QA.
- [ ] 3.2 Implement `SemanticObjectProposalModule` to align RGB frames with LiDAR/pointcloud and pose observations by timestamp using bounded tolerances, query TF at the image timestamp, and persist explicit TF frame-pair/tolerance provenance.
- [ ] 3.3 Reuse or factor the `Detection3DPC.from_2d()` projection pattern so mask/crop proposals keep only pointcloud support that projects inside the mask or bounding region, then compute support point count, 3D bbox, centroid, covariance when available, and projection-quality metadata.
- [ ] 3.4 Enforce conservative proposal filtering for invalid calibration, missing TF, low mask confidence, insufficient point count, excessive mask area, and poor projection quality; preserve frame-level evidence or low-confidence candidates without producing authoritative stable 3D markers.
- [ ] 3.5 Add focused tests for successful projection, missing calibration, missing or late TF, stream alignment tolerance failures, image-only candidates, and evidence IDs traceable from detections back to source observations.

## 4. Object-map fusion

- [ ] 4.1 Implement `SemanticObjectMapFusionModule` with idempotent evidence-ID processing, timestamp-ordered updates, candidate/stable/merged/pruned/stale statuses, and persistent object-map entries stored in a single resolved frame per map version.
- [ ] 4.2 Implement ConceptGraphs-style association using spatial similarity, visual similarity, `(1 + phys_bias) * spatial_sim + (1 - phys_bias) * visual_sim`, configurable thresholds, and best-match-or-new-candidate behavior.
- [ ] 4.3 Implement merge/update behavior that appends and downsamples or denoises support pointclouds, recomputes bbox/centroid, increments `num_detections`, preserves evidence IDs, and weighted-averages normalized CLIP/text embeddings by detection count.
- [ ] 4.4 Implement periodic denoise/filter/duplicate-merge passes and stable-object promotion thresholds for minimum point count, minimum detections, confidence, and projection quality.
- [ ] 4.5 Add focused tests for new-object creation, repeated-detection merge, duplicate merge, overmerge rejection, idempotent replay of the same evidence ID, delayed inference ordering, and stable-object promotion.

## 5. Derived graph, query, and inspection surfaces

- [ ] 5.1 Implement optional `SemanticSceneGraphDerivationModule` that derives graph nodes and relations from stable object-map entries only, persists rebuildable graph snapshots, and never feeds graph output back into object-map updates.
- [ ] 5.2 Implement a `SemanticMapSpec` Protocol or equivalent module RPC for `query(text)`, `get_object(object_id)`, `snapshot()`, and optional `get_graph(snapshot_id)` if RPC is chosen as the public inspection surface.
- [ ] 5.3 Add CLI inspection commands, likely under `dimos mem` or a new semantic-map command group, to list required streams, run semantic mapping over a bounded recorded-data window, list/show frame evidence, list/show object-map entries, query text over frames/objects, and show evidence provenance.
- [ ] 5.4 Publish optional `EntityMarkers` or Rerun inspection output from persisted object-map entries without recomputing or silently reinterpreting object poses.
- [ ] 5.5 Add focused tests or CLI runner tests for list/show/query behavior, missing optional graph outputs, provenance display, and visualization payload generation from persisted object-map entries.

## 6. Blueprint and integration wiring

- [ ] 6.1 Add an optional semantic mapping blueprint or CLI flow that composes existing Go2 live or recorded stream sources with `SemanticFrameFeatureModule`, `SemanticObjectProposalModule`, and `SemanticObjectMapFusionModule`; keep `SemanticSceneGraphDerivationModule` optional.
- [ ] 6.2 Ensure semantic modules are optional consumers only: existing `global_map`, `global_costmap`, relocalization, navigation, manipulation, MCP, and skill flows must run unchanged when semantic modules are absent.
- [ ] 6.3 Add configuration for stream names, map version/resolved frame, alignment tolerances, TF forward tolerance, keyframe/backpressure limits, mask/proposal limits, projection thresholds, association thresholds, and model/preprocess IDs.
- [ ] 6.4 If a public blueprint or module registry input changes, run `pytest dimos/robot/test_all_blueprints_generation.py` and include the regenerated `dimos/robot/all_blueprints.py` output.
- [ ] 6.5 Add integration tests using fake providers and synthetic RGB/pointcloud/pose streams to verify live-like incremental execution and deterministic recorded-data rebuild produce object-map entries within configured spatial tolerance.

## 7. Documentation

- [ ] 7.1 Add or update a user-facing semantic mapping capability guide under `docs/capabilities/` explaining v1 semantic evidence/object maps, required RGB + LiDAR/pointcloud + pose streams, optional calibration/model inputs, and that `depth_image` is not required.
- [ ] 7.2 Document CLI or blueprint examples for listing live/recorded streams, running a bounded recorded-data QA window, inspecting frame features, inspecting object detections, inspecting object-map entries, querying semantic outputs, and viewing optional markers or graph snapshots.
- [ ] 7.3 Document that semantic mapping is optional and independent from geometric maps, costmaps, relocalization, navigation, manipulation, MCP tools, and skills.
- [ ] 7.4 Add contributor notes only if implementation introduces new workflow beyond normal module development, such as semantic blueprint regeneration, deterministic recorded-data QA, or optional slow GPU/model tests.
- [ ] 7.5 Update coding-agent docs only if semantic mapping becomes a recurring implementation area; otherwise confirm existing repo guidance is sufficient.

## 8. Verification and manual QA

- [ ] 8.1 Run `openspec validate add-optional-semantic-map-overlay`.
- [ ] 8.2 Run focused pytest targets for changed semantic mapping, memory2 persistence, projection, fusion, CLI, and blueprint integration code.
- [ ] 8.3 Run `pytest dimos/robot/test_all_blueprints_generation.py` if any public blueprint or module registry input changes.
- [ ] 8.4 Run `uv run mypy dimos/` or the repository's focused type-check target for the changed typed semantic mapping surfaces.
- [ ] 8.5 Run docs validation for changed docs with `doclinks`; run `md-babel-py run <changed-doc.md>` for executable Python snippets and `bin/gen-diagrams` if generated diagrams are touched.
- [ ] 8.6 Manually QA the user surface by listing streams for the target recorded robot traversal, running semantic mapping over a bounded recorded-data window, confirming persisted frame features, object detections, stable object-map entries, provenance inspection, and optional graph/marker outputs.
- [ ] 8.7 Manually QA live-compatibility behavior with fake slow model providers or a live-like stream driver to confirm semantic work drops or defers frames without blocking geometry/control streams.
- [ ] 8.8 Manually QA independence by running an existing live, simulation, or recorded geometric mapping flow without semantic modules and confirming map/costmap/relocalization/navigation inspection still works.
