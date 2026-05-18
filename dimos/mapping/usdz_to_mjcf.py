# Copyright 2025-2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Bake a USDZ/GLB/OBJ scene mesh into an MJCF wrapper around a robot MJCF.

MuJoCo only reads ``.stl`` / ``.obj`` / ``.msh`` meshes — not USD.  This
module loads a scene mesh through the existing ``mesh_scene`` loader,
writes the welded geometry out as OBJ, and emits a tiny MJCF that
``<include>``s the robot MJCF and adds the scene mesh as a single
static collidable body.

Pipeline:

  1. ``load_scene_mesh()`` → ``open3d.geometry.TriangleMesh`` in dimos
     world frame (Z-up, meters), with all per-prim transforms baked in.
  2. ``open3d.io.write_triangle_mesh()`` → OBJ on disk.
  3. Wrapper MJCF references both the robot MJCF (via absolute-path
     ``<include>``) and the scene OBJ, declares one body with one mesh
     geom (``contype=1 conaffinity=1`` — collides with anything that's
     also enabled).  ``meshdir`` / ``texturedir`` in the wrapper's
     ``<compiler>`` are pinned to the robot MJCF's directory so the
     robot's STLs still resolve through the include.

Output is cached at ``~/.cache/dimos/scene_meshes/<hash>/`` keyed on
the SHA256 of (source mesh, robot MJCF, alignment params).  Repeat
runs with the same inputs reuse the conversion.
"""

from __future__ import annotations

from dataclasses import asdict
import hashlib
from pathlib import Path

import numpy as np
import open3d as o3d  # type: ignore[import-untyped]

from dimos.simulation.scene_assets.mesh_scene import SceneMeshAlignment
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


CACHE_DIR = Path.home() / ".cache" / "dimos" / "scene_meshes"


_WRAPPER_TEMPLATE = """\
<mujoco model="{model_name}">
  <compiler angle="radian" meshdir="{meshdir}" texturedir="{meshdir}"/>
  <include file="{robot_mjcf_abs}"/>
  <asset>
{asset_meshes}
  </asset>
  <worldbody>
    <body name="dimos_scene" pos="0 0 0">
{scene_geoms}
    </body>
  </worldbody>
</mujoco>
"""

_ASSET_LINE = '    <mesh name="{name}" file="{file}"/>'
_GEOM_LINE = (
    '      <geom name="{name}" type="mesh" mesh="{mesh}" '
    'contype="1" conaffinity="1" group="3" rgba="0.6 0.6 0.6 1"/>'
)


def _menagerie_g1_assets_dir() -> Path | None:
    """Locate the bundled ``mujoco_menagerie/unitree_g1/assets`` directory.

    Returns ``None`` if ``mujoco_playground`` isn't installed (its
    ``__init__`` is heavy — we use ``importlib.util.find_spec`` to find
    the package on disk without executing it).
    """
    import importlib.util

    spec = importlib.util.find_spec("mujoco_playground")
    if spec is None or not spec.submodule_search_locations:
        return None
    candidate = (
        Path(next(iter(spec.submodule_search_locations)))
        / "external_deps"
        / "mujoco_menagerie"
        / "unitree_g1"
        / "assets"
    )
    return candidate if candidate.exists() else None


def bake_scene_mjcf(
    scene_mesh_path: str | Path,
    robot_mjcf_path: str | Path,
    alignment: SceneMeshAlignment | None = None,
    meshdir: str | Path | None = None,
    cache_root: Path | None = None,
) -> Path:
    """Convert ``scene_mesh_path`` to OBJ and emit a wrapped MJCF.

    Args:
        scene_mesh_path: ``.usdz`` / ``.glb`` / ``.obj`` etc. — anything
            ``mesh_scene.load_scene_mesh`` accepts.
        robot_mjcf_path: the base robot MJCF the wrapper will
            ``<include>``.
        alignment: scale / translation / rotation / y-up swap to bake
            into the OBJ before MuJoCo sees it.  Authoritative for all
            three views (MuJoCo physics, viser, mesh camera) — the
            blueprint passes the same ``SceneMeshAlignment`` to each
            so the world frames agree to the millimeter.
        meshdir: directory MuJoCo should resolve unqualified mesh
            filenames against.  For the G1 GR00T MJCF this is the
            bundled ``mujoco_menagerie/unitree_g1/assets`` directory
            (where ``pelvis_contour_link.STL`` etc. actually live).
            ``None`` falls back to auto-detecting the menagerie path.
        cache_root: override for the cache directory (defaults to
            ``~/.cache/dimos/scene_meshes``).

    Returns:
        Path to the wrapper MJCF.  Pass this to ``MujocoSimModule``
        instead of the raw robot MJCF.
    """
    scene_mesh_path = Path(scene_mesh_path).expanduser().resolve()
    robot_mjcf_path = Path(robot_mjcf_path).expanduser().resolve()
    align = alignment or SceneMeshAlignment()

    if not scene_mesh_path.exists():
        raise FileNotFoundError(f"scene mesh not found: {scene_mesh_path}")
    if not robot_mjcf_path.exists():
        raise FileNotFoundError(f"robot MJCF not found: {robot_mjcf_path}")

    if meshdir is None:
        meshdir = _menagerie_g1_assets_dir()
        if meshdir is None:
            raise RuntimeError(
                "bake_scene_mjcf: could not locate mujoco_menagerie/unitree_g1/assets; "
                "pass meshdir= explicitly"
            )
    meshdir = Path(meshdir).expanduser().resolve()

    # Cache key — invalidates when any input changes.
    h = hashlib.sha256()
    h.update(scene_mesh_path.read_bytes())
    h.update(robot_mjcf_path.read_bytes())
    h.update(repr(sorted(asdict(align).items())).encode())
    h.update(str(meshdir).encode())
    cache_key = h.hexdigest()[:12]

    root = (cache_root or CACHE_DIR).expanduser()
    cache_dir = root / cache_key
    wrapper_path = cache_dir / "wrapper.xml"

    # Cache hit: wrapper exists + at least one prim OBJ next to it.
    if wrapper_path.exists() and any(cache_dir.glob("*.obj")):
        logger.info(f"bake_scene_mjcf: cache hit at {cache_dir}")
        return wrapper_path

    cache_dir.mkdir(parents=True, exist_ok=True)

    # Per-prim split: MuJoCo replaces each colliding mesh with its
    # convex hull, so the welded scene mesh's hull is just the room
    # bounding box (robot is "inside" the box → 5 m penetration).  Each
    # USD prim individually is approximately convex (a crate, a wall
    # panel, a barrel), so the hull-of-each is a faithful collider.
    from dimos.simulation.scene_assets.mesh_scene import load_scene_prims

    logger.info(f"bake_scene_mjcf: loading + aligning {scene_mesh_path} (per-prim)")
    prims = load_scene_prims(scene_mesh_path, alignment=align)
    logger.info(f"bake_scene_mjcf: {len(prims)} prims to bake")

    # Per-prim convex hull.  Per-prim splitting + one-hull-per-prim is
    # enough for a static scene populated by furniture-sized objects
    # (chair, sofa, wall, table-top): the robot bumps into the bounding
    # convex shape, which is what we want — fine-grained concavity
    # decomposition (VHACD/CoACD per prim) costs minutes-to-hours on
    # large multi-prim scenes (1000+ prims) and only buys the ability to
    # navigate *inside* a chair's frame, which we don't need.  For
    # large structural prims (one prim covering an entire room shell)
    # the user should split them in the source asset before export.
    import trimesh

    asset_lines: list[str] = []
    geom_lines: list[str] = []
    total_tris = 0
    skipped_degenerate = 0
    n_hulls = 0
    # 1 mm — anything thinner is coplanar for qhull's purposes.
    _DEGENERATE_EPS = 1e-3
    # Single convex hull is fine for furniture-scale prims (chair, desk,
    # crate) but disastrous for architectural shells (wall mesh, ceiling,
    # floor): those are *concave* — wrapping them in one hull produces a
    # room-sized solid block that swallows the robot's spawn point and
    # MuJoCo resolves the penetration by slamming the pelvis down to the
    # floor.  Detect "shell" prims by hull volume; only the architectural
    # bits exceed ~2 m³ (a sofa is ~0.5 m³, a desk ~0.3 m³, a wall is 100+).
    # For those, run VHACD — it produces ~64 thin slab hulls following the
    # actual wall surfaces.  VHACD on these few large prims is fast (~0.2 s
    # each) because they have very few triangles (the artist welded each
    # wall as a flat plane).
    _SHELL_VOLUME_M3 = 2.0
    n_decomposed = 0
    logger.info(f"bake_scene_mjcf: per-prim convex-hulling {len(prims)} prims (one-time)…")
    for prim in prims:
        tm = trimesh.Trimesh(
            vertices=prim.vertices.astype(np.float64),
            faces=prim.triangles,
            process=False,
        )
        try:
            single_hull = tm.convex_hull
        except Exception as e:
            logger.warning(f"  convex_hull failed for {prim.name}: {e}; skipping")
            continue

        if float(single_hull.volume) > _SHELL_VOLUME_M3:
            try:
                parts = tm.convex_decomposition(maxConvexHulls=64, resolution=200_000)
                if not isinstance(parts, list):
                    parts = [parts]
                hulls = parts
                n_decomposed += 1
                logger.info(
                    f"  {prim.name}: VHACD decomposed "
                    f"({single_hull.volume:.1f} m³ shell → {len(parts)} sub-hulls)"
                )
            except Exception as e:
                logger.warning(
                    f"  VHACD failed for {prim.name}: {e}; "
                    f"using single hull (will swallow robot spawn area)"
                )
                hulls = [single_hull]
        else:
            hulls = [single_hull]

        for j, hull in enumerate(hulls):
            v = np.asarray(hull.vertices, dtype=np.float32)
            f = np.asarray(hull.faces, dtype=np.int32)
            if len(v) < 4 or len(f) < 4:
                skipped_degenerate += 1
                continue
            extent = v.max(axis=0) - v.min(axis=0)
            if (extent < _DEGENERATE_EPS).any():
                skipped_degenerate += 1
                continue
            # qhull's coplanarity tolerance scales with the hull's max
            # extent — a 7 mm thinness in a 380 mm-wide hull (e.g. CoACD's
            # split of an "ultrawide monitor" prim) reads as coplanar and
            # MuJoCo's mj_loadXML aborts with QH6154.  Skip pre-emptively
            # any hull whose smallest axis is < 5% of the largest, *or*
            # whose absolute thinness is < 5 mm — caught both failing
            # examples in the dimos_office bake.
            min_ext = float(extent.min())
            max_ext = float(extent.max())
            if max_ext > 0 and (min_ext / max_ext) < 0.05:
                skipped_degenerate += 1
                continue
            if min_ext < 5e-3:
                skipped_degenerate += 1
                continue
            # Belt-and-suspenders: try the same qhull call MuJoCo will
            # do at load time.  If scipy can't build a 3D hull from
            # these points, mj_loadXML can't either — skip rather than
            # take the whole sim down on adapter-connect timeout.
            try:
                from scipy.spatial import ConvexHull, QhullError

                ConvexHull(v, qhull_options="Qt")
            except (QhullError, ValueError):
                skipped_degenerate += 1
                continue

            asset_name = f"{prim.name}_h{j:03d}"
            obj_file = cache_dir / f"{asset_name}.obj"
            o3d_mesh = o3d.geometry.TriangleMesh()
            o3d_mesh.vertices = o3d.utility.Vector3dVector(v.astype(np.float64))
            o3d_mesh.triangles = o3d.utility.Vector3iVector(f)
            o3d_mesh.compute_vertex_normals()
            if not o3d.io.write_triangle_mesh(
                str(obj_file),
                o3d_mesh,
                write_vertex_normals=True,
                write_vertex_colors=False,
            ):
                raise RuntimeError(f"open3d failed to write OBJ: {obj_file}")

            total_tris += len(f)
            n_hulls += 1
            asset_lines.append(_ASSET_LINE.format(name=asset_name, file=str(obj_file)))
            geom_lines.append(_GEOM_LINE.format(name=f"{asset_name}_geom", mesh=asset_name))

    if not asset_lines:
        raise RuntimeError(
            "bake_scene_mjcf: every hull came out degenerate; nothing left to collide against"
        )
    logger.info(
        f"bake_scene_mjcf: baked {n_hulls} convex hulls from {len(prims)} prims "
        f"({total_tris} tris total), VHACD-decomposed {n_decomposed} shell prims, "
        f"skipped {skipped_degenerate} degenerate hulls"
    )

    wrapper_xml = _WRAPPER_TEMPLATE.format(
        model_name=f"g1_with_scene_{cache_key}",
        meshdir=str(meshdir),
        robot_mjcf_abs=str(robot_mjcf_path),
        asset_meshes="\n".join(asset_lines),
        scene_geoms="\n".join(geom_lines),
    )
    wrapper_path.write_text(wrapper_xml)
    logger.info(f"bake_scene_mjcf: wrote wrapper {wrapper_path}")
    return wrapper_path


def cli_main() -> None:
    """``python -m dimos.mapping.usdz_to_mjcf <scene_path> <robot_mjcf> [scale] [--view]``.

    Bake the wrapper, verify it loads, optionally open MuJoCo's native
    viewer for visual inspection.  ``--view`` works on macOS without
    ``mjpython`` because we're invoking ``mujoco.viewer.launch`` from
    the main thread of a fresh process — the issue dimos hits in
    workers is that ``launch_passive`` in a *non-main* thread requires
    mjpython.
    """
    import sys

    args = list(sys.argv[1:])
    view = False
    if "--view" in args:
        view = True
        args.remove("--view")
    if len(args) < 2:
        print(
            "usage: python -m dimos.mapping.usdz_to_mjcf <scene_path> <robot_mjcf> [scale] [--view]"
        )
        sys.exit(2)
    scene = Path(args[0])
    robot = Path(args[1])
    scale = float(args[2]) if len(args) >= 3 else 0.05
    align = SceneMeshAlignment(scale=scale)
    wrapper = bake_scene_mjcf(scene, robot, alignment=align)
    print(f"wrapper: {wrapper}")

    import mujoco  # type: ignore[import-untyped]

    model = mujoco.MjModel.from_xml_path(str(wrapper))
    print(f"loaded:  {model.nbody} bodies, {model.ngeom} geoms, {model.nmesh} meshes")
    print(f"joints:  {model.njnt}, dof:  {model.nv}")

    if view:
        import mujoco.viewer  # type: ignore[import-untyped]

        # ``launch`` runs MuJoCo's interactive viewer with its own
        # internal physics loop.  Blocks until the user closes it.
        # Press F1 in the viewer for the keyboard cheatsheet; ``Tab``
        # toggles the rendering panel where you can switch geom groups
        # (group 3 = our scene collision hulls, group 1 = robot
        # visual mesh, group 0 = robot collision mesh).
        print("\n→ launching MuJoCo viewer (press Esc / close window to exit)")
        mujoco.viewer.launch(model)


if __name__ == "__main__":
    cli_main()


__all__ = ["bake_scene_mjcf"]
