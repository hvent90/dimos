# Scene packages

A **scene package** is a self-contained directory holding everything any
DimOS simulator needs to load a 3D scene: visual mesh, collision mesh,
per-object semantic table, MuJoCo wrapper, and a list of dynamic
interactables. One source asset → one package → consumed by both
**pimsim** (browser Havok) and **MuJoCo** with the same metadata.

```text
~/.cache/dimos/scene_packages/<name>/
├── scene.meta.json                manifest — all paths relative to package root
├── browser/
│   ├── visual.glb                 gltfpack-optimised, static parts only
│   ├── collision.glb              decimated trimesh for browser raycasts
│   └── objects.json               per-prim semantic table (id, path, AABB)
├── entities/
│   └── <safe_id>/visual.glb       per-entity GLB in entity-local frame
└── mujoco/
    └── <key>/
        ├── wrapper.xml            self-contained MJCF (all paths relative)
        ├── compiled.mjb           mj_saveModel binary, fast load
        ├── robot/                 copy of robot MJCF + sibling includes
        ├── *.STL / *.png          robot mesh + texture assets (symlinked
        │                          by default; copies with
        │                          DIMOS_PACKAGE_COPY_ASSETS=1)
        └── *.obj                  cooked scene collision meshes
```

Packages are content-hash keyed: change the source mesh, the sidecar,
or the robot MJCF and you get a new package. Existing packages are
preserved.

## Authoring a scene

Drop two files next to your source mesh:

```text
my_scene.glb                       # the source asset (USD/GLB/OBJ/PLY)
my_scene.cook.json                 # the cook sidecar (optional but recommended)
```

A minimal `cook.json`:

```json
{
  "$schema": "https://dimensional.dev/schemas/dimos/scene-cook-sidecar.v1.json",
  "collision": {
    "prim_overrides": {
      "Floor": {"type": "plane"}
    }
  },
  "interactables": []
}
```

Without a sidecar the cooker still works — you get an entity-free
package with auto-fit collision.

## Interactables

Two flavours, both expressed in `interactables[]`:

### Extracted from the source mesh

For chairs, fridges, printers — anything already modelled in the
source asset that you want as a dynamic body. The cooker matches
`source_prim_paths` with `fnmatch` against USD prim paths and GLB node
names, splits the matched prims out of the static bake, and emits a
per-entity GLB.

```json
{
  "id": "office_chair_001",
  "source_prim_paths": ["Chair.001_*"],
  "kind": "dynamic",
  "mass": 8.0,
  "physics": {"shape": "mesh"},
  "tags": ["chair", "office", "movable"]
}
```

### Synthetic

For props that aren't in the source mesh — manipulation cubes, test
markers, placed obstacles. No prim matching; geometry comes from
`physics.shape` + `physics.extents`, pose is explicit.

```json
{
  "id": "manip_cube",
  "pose": {"x": 0.0, "y": 0.75, "z": 0.69},
  "kind": "dynamic",
  "mass": 0.15,
  "physics": {
    "shape": "box",
    "extents": [0.08, 0.08, 0.08],
    "friction": [1.0, 0.1, 0.001]
  },
  "visual": {"rgba": [0.85, 0.20, 0.20, 1.0]},
  "tags": ["manipulation"]
}
```

### Field reference

| Field | Required | Notes |
|---|---|---|
| `id` | yes | Stable, used as `entity:<id>` MuJoCo body name |
| `source_prim_paths` | one of | `fnmatch` globs against USD paths / GLB nodes — extracted entity |
| `pose` | one of | `{x, y, z, qw, qx, qy, qz}` — synthetic entity |
| `kind` | – | `"dynamic"` (freejoint+mass), `"kinematic"` (RPC-driven), `"static"` (welded). Default `"dynamic"` |
| `mass` | – | kg. 0 forces kinematic. Default 1.0 |
| `physics.shape` | – | `"mesh"` \| `"box"` \| `"sphere"` \| `"cylinder"`. Default `"box"` |
| `physics.extents` | for synthetic | Half-extents triple for box, `[radius]` for sphere, `[radius, half_height]` for cylinder |
| `physics.friction` | – | Scalar sliding or `[slide, torsional, rolling]`. Default `[0.3, 0.05, 0.001]` |
| `visual.rgba` | – | `[r, g, b, a]` 0–1 — colour in both pimsim and MuJoCo |
| `remove_from_static` | – | Strip matched prims from the static bake. Default true |
| `spawn` | – | `"initial"` (in world at boot) or `"manual"` (only via RPC). Default `"initial"` |
| `tags` | – | Free-form labels — semantic queries |

### Collision policy overrides

The `collision` block under the sidecar is the same schema as the
older `<scene>.collision.json` file. Most useful keys:

```json
{
  "collision": {
    "default": "auto",
    "prim_overrides": {
      "Floor":           {"type": "plane"},
      "Wall_*":          {"type": "box"},
      "Curtain_*":       {"type": "skip"},
      "Stairs_*":        {"type": "decompose", "max_hulls": 16},
      "FlatPanel_*":     {"type": "hull"}
    }
  }
}
```

Per-pattern `type` values: `"auto"` | `"box"` | `"sphere"` |
`"cylinder"` | `"capsule"` | `"plane"` | `"hull"` | `"mesh"` |
`"decompose"` | `"skip"`. See `dimos/simulation/mujoco/collision_spec.py`
for the full reference.

## Cooking

```bash
python -m dimos.simulation.scene_assets.cook \
    path/to/my_scene.glb \
    --robot-mjcf=data/mujoco_sim/g1_gear_wbc.xml \
    --meshdir=path/to/menagerie/unitree_g1/assets \
    --output-dir=~/.cache/dimos/scene_packages/my_scene
```

Without `--robot-mjcf` you get a browser-only package (no MuJoCo
artifacts). `--rebake` ignores all caches and rebuilds from scratch.

The cooker auto-discovers `<scene>.cook.json` next to the source. It's
content-hash cached on source-bytes + sidecar JSON + robot MJCF bytes
+ meshdir — bumping any of those gives a new cache key without
clobbering the previous one.

## Dropping in a new robot

To make a scene package work with a robot other than G1:

1. Have a self-contained robot MJCF and its mesh/asset directory. Most
   `mujoco_menagerie` robots fit: `<robot>.xml` + `assets/*.STL`.
2. Cook with the right `--robot-mjcf` and `--meshdir`:

   ```bash
   python -m dimos.simulation.scene_assets.cook \
       my_scene.glb \
       --robot-mjcf=path/to/go2.xml \
       --meshdir=path/to/go2/assets
   ```

3. The cooker bundles the robot MJCF, every XML sibling next to it
   (so any `<include>`s resolve), and every `.STL` / `.png` /
   `.jpg` / etc. from `meshdir` into the package under
   `mujoco/<key>/`. The emitted `wrapper.xml` uses **only relative
   paths** — the package is movable across machines (`tar -h …` will
   dereference the symlinks if you want one tarball).
4. To produce a tarball-portable package without dereferencing,
   pre-set `DIMOS_PACKAGE_COPY_ASSETS=1` and the cooker will copy
   the asset files instead of symlinking them.
5. At runtime, point your blueprint at the cooked package via
   `--scene <name>` (if you've added it to
   `dimos/simulation/scenes/catalog.py`) or by pointing
   `resolve_scene_package(...)` at an absolute path to the
   package directory or its `scene.meta.json`.

### What the robot MJCF must not have

- **No scene geometry.** No floor planes, walls, furniture. The scene
  package owns that.
- **No manipulation rigs.** No hardcoded `manip_*` bodies or tables.
  Author those as synthetic interactables in the scene's `cook.json`
  instead — they then spawn into pimsim and MuJoCo from one source.
- **No external `<include>`s.** Everything the robot needs must live
  inside the same directory as the robot MJCF (so the cooker can copy
  the sibling tree and the include resolves locally).

## Consuming a package at runtime

### From Python (G1 blueprint, MuJoCo backend)

```python
from dimos.simulation.scenes.catalog import resolve_scene_package
from dimos.simulation.mujoco.entity_scene import compose_entity_model

pkg = resolve_scene_package("dimos-office", robot_mjcf_path=...)
# pkg.entities is the list MuJoCoSimModule + BabylonSceneViewerModule consume.
mjb_path = compose_entity_model(pkg)  # static wrapper.mjb extended with entities
```

### Topics published by either simulator

| Topic | Type | Producer | Notes |
|---|---|---|---|
| `/odom` | `PoseStamped` | sim | robot base pose |
| `/cmd_vel` | `Twist` | controller | drive command (sim subscribes) |
| `/coordinator/joint_state` | `JointState` | coordinator | sim writes via SHM, coordinator publishes |
| `/entity_state_batch` | `EntityStateBatch` | physics authority | **same wire format from MuJoCo and pimsim** — consumers don't care |
| `/lidar` | `PointCloud2` | rust `scene_lidar` | raycast over `browser/collision.glb` + dynamic entities |

The "physics authority" is whichever sim is wired in the blueprint —
MuJoCo for G1 whole-body, pimsim Havok for navigation-only flows. The
*other* one mirrors the published states as kinematic bodies for
rendering.

## What a `scene.meta.json` entity entry looks like

```json
{
  "id": "office_chair_001",
  "tags": ["chair", "office", "movable"],
  "source_prim_paths": ["Chair.001_*"],
  "matched_prim_paths": ["Chair.001_Mesh.030", "Chair.001_Mesh.030_1"],
  "visual_node_patterns": ["Chair.001"],
  "remove_from_static": true,
  "spawn": "initial",
  "synthetic": false,
  "aabb": {"min": [...], "max": [...]},
  "initial_pose": {"x": -3.74, "y": -0.82, "z": 0.49,
                   "qw": 1.0, "qx": 0.0, "qy": 0.0, "qz": 0.0},
  "visual_path": "entities/office_chair_001/visual.glb",
  "descriptor": {
    "entity_id": "office_chair_001",
    "kind": "dynamic",
    "shape_hint": "mesh",
    "extents": [],
    "mass": 8.0,
    "mesh_ref": "entities/office_chair_001/visual.glb"
  },
  "physics": {"shape": "mesh"},
  "visual": {}
}
```

`descriptor` is the runtime-facing subset (matches `EntityDescriptor`
on the wire); the rest is provenance for debuggability.

## Bundled scenes

| Scene name | Source | Use case |
|---|---|---|
| `dimos-office` | `data/dimos_office_mesh/dimos_office_mesh.glb` | Default — office with 17 chairs, 1 printer, manip rig |
| `street-lite` | Sketchfab street scene | Outdoor nav |
| `lowpoly-tdm` | Sketchfab lowpoly map | Lightweight nav stress test |
| `mall-babylon-nolights` | Generated mall | Large indoor nav |

Aliases live in `dimos/simulation/scenes/catalog.py`. Add a new entry
to `_PACKAGE_DIRS` and `_ALIASES` if you cook a new scene you want
the rest of the stack to find by name.

## Reference

- `dimos/simulation/scene_assets/sidecar.py` — `<scene>.cook.json` schema
- `dimos/simulation/scene_assets/plan.py` — sidecar → resolved cook plan
- `dimos/simulation/scene_assets/cook.py` — top-level cook entry point + CLI
- `dimos/simulation/scene_assets/spec.py` — `ScenePackage` dataclass + on-disk JSON shape
- `dimos/simulation/scene_assets/browser_collision.py` — fused collision GLB + `objects.json` sidecar
- `dimos/simulation/scene_assets/visual_blender.py` — Blender pass to extract per-entity GLBs
- `dimos/simulation/mujoco/scene_mesh_to_mjcf.py` — MJCF wrapper + portable robot bundling
- `dimos/simulation/mujoco/entity_scene.py` — runtime composition of entities into the cooked MJCF
- `dimos/simulation/scenes/catalog.py` — `resolve_scene_package(name | path)` registry
