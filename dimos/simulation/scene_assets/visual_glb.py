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

"""Cook browser visual assets while preserving authored materials."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import shutil
import subprocess
import tempfile
from typing import Any

from dimos.simulation.scene_assets.inspect import inspect_scene_asset
from dimos.simulation.scene_assets.spec import BrowserVisualSpec
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_BLENDER_SCRIPT = r"""
import pathlib
import sys

import bpy

source = pathlib.Path(sys.argv[-2])
target = pathlib.Path(sys.argv[-1])
suffix = source.suffix.lower()

bpy.ops.object.select_all(action="SELECT")
bpy.ops.object.delete()

if suffix in {".usd", ".usda", ".usdc", ".usdz"}:
    bpy.ops.wm.usd_import(filepath=str(source))
elif suffix in {".gltf", ".glb"}:
    bpy.ops.import_scene.gltf(filepath=str(source))
elif suffix == ".obj":
    bpy.ops.wm.obj_import(filepath=str(source))
elif suffix == ".stl":
    bpy.ops.wm.stl_import(filepath=str(source))
elif suffix == ".ply":
    bpy.ops.wm.ply_import(filepath=str(source))
else:
    raise RuntimeError(f"unsupported visual source suffix: {suffix}")

bpy.ops.export_scene.gltf(
    filepath=str(target),
    export_format="GLB",
    export_yup=True,
    export_apply=True,
)
"""


@dataclass(frozen=True)
class BrowserVisualCookResult:
    path: Path
    stats: dict[str, Any]
    tool: str


def cook_browser_visual(
    source_path: str | Path,
    output_dir: str | Path,
    *,
    spec: BrowserVisualSpec | None = None,
    rebake: bool = False,
) -> BrowserVisualCookResult | None:
    """Write the browser visual GLB for a scene package.

    GLB inputs are copied byte-for-byte so texture/material fidelity is not
    accidentally reduced.  Non-GLB sources are exported through Blender when
    available because Blender preserves authoring-tool material bindings far
    better than geometry-only Python loaders.
    """
    visual_spec = spec or BrowserVisualSpec()
    if not visual_spec.enabled:
        return None

    source = Path(source_path).expanduser().resolve()
    out_dir = Path(output_dir).expanduser().resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / visual_spec.output_name
    if out_path.exists() and not rebake:
        return BrowserVisualCookResult(
            path=out_path,
            stats=inspect_scene_asset(out_path).to_json_dict(),
            tool="cache",
        )

    suffix = source.suffix.lower()
    if suffix == ".glb":
        shutil.copy2(source, out_path)
        tool = "copy"
    else:
        _export_with_blender(source, out_path)
        tool = "blender"

    stats = inspect_scene_asset(out_path).to_json_dict()
    warnings = _budget_warnings(stats, visual_spec)
    if warnings:
        stats["warnings"] = warnings
        for warning in warnings:
            logger.warning("browser visual budget: %s", warning)
    return BrowserVisualCookResult(path=out_path, stats=stats, tool=tool)


def _export_with_blender(source: Path, target: Path) -> None:
    blender = shutil.which("blender")
    if blender is None:
        raise RuntimeError(
            f"{source.suffix} visual export requires Blender on PATH. "
            "Install Blender or provide a GLB source asset."
        )

    with tempfile.NamedTemporaryFile("w", suffix=".py", delete=False) as script:
        script.write(_BLENDER_SCRIPT)
        script_path = Path(script.name)
    try:
        subprocess.run(
            [
                blender,
                "--background",
                "--factory-startup",
                "--python",
                str(script_path),
                "--",
                str(source),
                str(target),
            ],
            check=True,
            text=True,
        )
    finally:
        script_path.unlink(missing_ok=True)


def _budget_warnings(stats: dict[str, Any], spec: BrowserVisualSpec) -> list[str]:
    warnings: list[str] = []
    mesh_count = int(stats.get("node_count") or stats.get("mesh_count") or 0)
    material_count = int(stats.get("material_count") or 0)
    if mesh_count > spec.max_meshes:
        warnings.append(f"{mesh_count} render nodes exceeds target {spec.max_meshes}")
    if material_count > spec.max_materials:
        warnings.append(f"{material_count} materials exceeds target {spec.max_materials}")
    return warnings


__all__ = ["BrowserVisualCookResult", "cook_browser_visual"]
