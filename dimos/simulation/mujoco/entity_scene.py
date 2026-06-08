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

"""Compose scene-package entities into a MuJoCo model via ``MjSpec``.

The cook step removes entity prims (chairs, props) from the static
scene bake and writes their per-entity GLBs and metadata to the
package's ``entities/`` directory. At runtime, ``MujocoSimModule``
calls :func:`add_entities_to_spec` on its scene spec **before** the
robot attach, so the cooked entities become first-class bodies in the
composed model.

Entities with ``kind == "dynamic"`` and positive mass receive a
freejoint (robot can push/grasp them); anything else is welded static.
Collision: primitive shapes (box/sphere/cylinder) use descriptor extents;
mesh entities consume package-authored ``collision_paths`` emitted by the
offline scene cooker. Runtime loading does not run CoACD or write geometry
caches.

A spawn-contact audit can be run on the compiled model to report
entities that start in deep penetration with the static scene; see
:func:`spawn_penetrators`.

Body naming: ``entity:<entity_id>`` - consumers map MuJoCo bodies back
to entity ids through this prefix.
"""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING, Any

from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    import mujoco

logger = setup_logger()

ENTITY_BODY_PREFIX = "entity:"

_MIN_HALF_EXTENT = 0.01
# Sliding friction 0.3 is furniture that scoots when bumped. Entity geoms
# carry priority=1 so this wins the contact pair outright (MuJoCo's
# default combine rule is element-wise max, which would let the mu=1.0
# floor override it). Graspable props override via ``physics.friction``.
_DEFAULT_FRICTION = (0.3, 0.05, 0.001)
_DEFAULT_RGBA = (0.62, 0.62, 0.68, 1.0)
# Same geom group as the baked static scene so depth-based lidar renders
# (which hide robot groups 0/1) still see entities.
_ENTITY_GEOM_GROUP = 3
# Spawn-contact audit: deeper penetration than this at t=0 is reported
# by ``spawn_penetrators`` for targeted pose/collision fixes.
_SPAWN_PENETRATION_LIMIT_M = 0.02


def entity_body_name(entity_id: str) -> str:
    return f"{ENTITY_BODY_PREFIX}{entity_id}"


def _initial_entities(entities: list[dict[str, Any]]) -> list[dict[str, Any]]:
    return [e for e in entities if e.get("spawn", "initial") == "initial"]


def _box_size_and_offset(entity: dict[str, Any]) -> tuple[list[float], list[float]] | None:
    """Half-extents + geom offset (body frame) for an entity's collision box."""
    descriptor = entity.get("descriptor", {})
    shape = descriptor.get("shape_hint", "mesh")
    extents = [float(x) for x in descriptor.get("extents", [])]

    if shape == "box" and len(extents) == 3:
        half = [max(x / 2.0, _MIN_HALF_EXTENT) for x in extents]
        return half, [0.0, 0.0, 0.0]
    if shape == "sphere" and len(extents) == 1:
        r = max(extents[0], _MIN_HALF_EXTENT)
        return [r, r, r], [0.0, 0.0, 0.0]
    if shape == "cylinder" and len(extents) == 2:
        r = max(extents[0], _MIN_HALF_EXTENT)
        h = max(extents[1] / 2.0, _MIN_HALF_EXTENT)
        return [r, r, h], [0.0, 0.0, 0.0]

    # Mesh entities: box from the cooked world-frame AABB. Cook poses are
    # identity-rotation, so the AABB is axis-aligned in the body frame too.
    aabb = entity.get("aabb")
    pose = entity.get("initial_pose")
    if not aabb or not pose:
        return None
    lo = [float(x) for x in aabb["min"]]
    hi = [float(x) for x in aabb["max"]]
    half = [max((h - low) / 2.0, _MIN_HALF_EXTENT) for low, h in zip(lo, hi, strict=True)]
    center = [(h + low) / 2.0 for low, h in zip(lo, hi, strict=True)]
    origin = [float(pose.get(k, 0.0)) for k in ("x", "y", "z")]
    offset = [c - o for c, o in zip(center, origin, strict=True)]
    return half, offset


def _entity_collision_paths(entity: dict[str, Any]) -> list[Path]:
    raw = entity.get("collision_paths")
    if raw is None:
        raw = entity.get("collision_path")
    if isinstance(raw, str):
        raw_paths: list[Any] = [raw]
    elif isinstance(raw, list | tuple):
        raw_paths = list(raw)
    else:
        return []

    entity_id = str(entity.get("id", "unknown"))
    paths: list[Path] = []
    missing: list[str] = []
    for value in raw_paths:
        if not isinstance(value, str):
            continue
        path = Path(value).expanduser()
        if path.exists():
            paths.append(path)
        else:
            missing.append(str(path))
    if missing:
        logger.warning(
            "entity %s: missing cooked MuJoCo collision paths: %s",
            entity_id,
            ", ".join(missing),
        )
    return paths


def _entity_friction(entity: dict[str, Any]) -> tuple[float, float, float]:
    """``physics.friction`` from entity metadata (scalar sliding or full
    [sliding, torsional, rolling] triple), else the scoot-able default."""
    raw = entity.get("physics", {}).get("friction")
    sliding, torsional, rolling = _DEFAULT_FRICTION
    if isinstance(raw, int | float):
        sliding = float(raw)
    elif isinstance(raw, list | tuple) and len(raw) == 3:
        sliding, torsional, rolling = (float(v) for v in raw)
    return sliding, torsional, rolling


def _entity_rgba(descriptor: dict[str, Any]) -> tuple[float, float, float, float]:
    raw = descriptor.get("rgba")
    if isinstance(raw, list | tuple) and len(raw) == 4:
        return tuple(float(v) for v in raw)  # type: ignore[return-value]
    return _DEFAULT_RGBA


def add_entities_to_spec(
    spec: mujoco.MjSpec,
    entities: list[dict[str, Any]],
) -> None:
    """Append scene-package entities as bodies on ``spec.worldbody``.

    Call before attaching the robot. Each ``spawn=="initial"`` entity
    becomes one body named ``entity:<id>`` with the descriptor's geom
    shape and friction; dynamic entities also receive a freejoint named
    ``entity:<id>:free``.

    """
    import mujoco

    for entity in _initial_entities(entities):
        descriptor = entity.get("descriptor", {})
        entity_id = descriptor.get("entity_id") or entity.get("id")
        pose = entity.get("initial_pose")
        if not entity_id or not pose:
            continue
        entity_id = str(entity_id)

        kind = descriptor.get("kind", "kinematic")
        mass = float(descriptor.get("mass", 0.0))
        dynamic = kind == "dynamic" and mass > 0.0

        body = spec.worldbody.add_body(
            name=entity_body_name(entity_id),
            pos=[
                float(pose.get("x", 0.0)),
                float(pose.get("y", 0.0)),
                float(pose.get("z", 0.0)),
            ],
            quat=[
                float(pose.get("qw", 1.0)),
                float(pose.get("qx", 0.0)),
                float(pose.get("qy", 0.0)),
                float(pose.get("qz", 0.0)),
            ],
        )
        if dynamic:
            body.add_freejoint(name=f"{entity_body_name(entity_id)}:free")

        rgba = _entity_rgba(descriptor)
        friction = _entity_friction(entity)
        geom_kwargs: dict[str, Any] = dict(
            name=f"{entity_body_name(entity_id)}:geom",
            rgba=list(rgba),
            friction=list(friction),
            group=_ENTITY_GEOM_GROUP,
            # priority=1: contact friction comes from the entity geom alone.
            # MuJoCo's default combine rule (element-wise max across the
            # pair) would otherwise let the mu=1.0 floor override every
            # entity's friction.
            priority=1,
        )
        if dynamic:
            geom_kwargs["mass"] = mass

        shape = descriptor.get("shape_hint", "mesh")
        hull_paths = _entity_collision_paths(entity) if shape == "mesh" else []
        if hull_paths:
            base_name = entity_body_name(entity_id)
            if dynamic:
                # MuJoCo derives per-geom mass from a body-level mass split
                # across geoms by volume. Setting mass on each geom would
                # double-count. Move mass to the body and drop it from the
                # per-geom kwargs.
                body.mass = mass
                geom_kwargs.pop("mass", None)
            for i, hull_obj in enumerate(hull_paths):
                mesh_name = f"{base_name}:hull{i:03d}"
                spec.add_mesh(name=mesh_name, file=str(hull_obj))
                # Name geoms uniquely so MuJoCo's compile doesn't reject
                # duplicate-name collisions across multi-hull entities.
                gk = dict(geom_kwargs)
                gk["name"] = f"{base_name}:geom{i:03d}"
                body.add_geom(
                    type=mujoco.mjtGeom.mjGEOM_MESH,
                    meshname=mesh_name,
                    **gk,
                )
            continue

        box = _box_size_and_offset(entity)
        if box is None:
            logger.warning("entity %s has no usable collision shape; skipping", entity_id)
            continue
        half, offset = box
        geom_kwargs["pos"] = offset
        if shape == "sphere":
            geom_type = mujoco.mjtGeom.mjGEOM_SPHERE
            size = [half[0], 0.0, 0.0]
        elif shape == "cylinder":
            geom_type = mujoco.mjtGeom.mjGEOM_CYLINDER
            size = [half[0], half[2], 0.0]
        else:
            geom_type = mujoco.mjtGeom.mjGEOM_BOX
            size = half
        body.add_geom(type=geom_type, size=size, **geom_kwargs)


def spawn_penetrators(model: mujoco.MjModel) -> frozenset[str]:
    """Entity ids whose geoms start in deep contact at the spawn pose.

    Run after ``spec.compile()`` and before stepping. This is a diagnostic
    helper only; callers should fix poses/collision geometry rather than
    silently changing dynamic assets to static.
    """
    import mujoco

    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    bad: set[str] = set()
    for c in range(data.ncon):
        contact = data.contact[c]
        if contact.dist >= -_SPAWN_PENETRATION_LIMIT_M:
            continue
        for geom_id in (contact.geom1, contact.geom2):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(geom_id)) or ""
            if not name.startswith(ENTITY_BODY_PREFIX):
                continue
            suffix = name[len(ENTITY_BODY_PREFIX) :]
            if ":geom" in suffix:
                bad.add(suffix.split(":geom", 1)[0])
    return frozenset(bad)


__all__ = [
    "ENTITY_BODY_PREFIX",
    "add_entities_to_spec",
    "entity_body_name",
    "spawn_penetrators",
]
