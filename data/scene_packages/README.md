# Scene Packages

Scene packages are robot-agnostic, cooked scene artifacts. A package contains a
portable `scene.meta.json` sidecar plus the browser and MuJoCo assets referenced
from that sidecar. Robot MJCFs are attached later at runtime; they are not cooked
into scene packages.

Large scene packages should not be committed as loose files. Store them through
the existing DimOS data workflow as `data/.lfs/scene_packages.tar.gz`, which
extracts to `data/scene_packages/`.

Use `get_data("scene_packages/<package-name>")` to pull and unpack a package
from LFS when it is not already present locally.

Expected package contents:

- `scene.meta.json`: the runtime contract. It records package-relative artifact
  paths, entity initial poses, dynamic/static metadata, masses, friction, visual
  paths, and cooked MuJoCo collision paths.
- `browser/visual.glb`: optimized browser visual scene.
- `browser/collision.glb`: browser raycast/physics collision scene.
- `browser/objects.json`: browser semantic sidecar for objects in the collision
  scene.
- `mujoco/<cook-key>/wrapper.xml`: scene-only MuJoCo wrapper for static
  collision geometry.
- `mujoco/<cook-key>/*.obj`: static scene collision hulls referenced by
  `wrapper.xml`.
- `entities/<entity-id>/visual.glb`: per-entity visual mesh.
- `entities/<entity-id>/mujoco_collision/*.obj`: cooked CoACD collision hulls for
  dynamic mesh entities.
- `entities/<entity-id>/mujoco_collision/manifest.json`: hull manifest for that
  entity's cooked collision assets.
