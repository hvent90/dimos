# Copyright 2026 Dimensional Inc.
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

"""Built-in robot description asset paths."""

from __future__ import annotations

from pathlib import Path

_DESCRIPTION_ROOT = Path(__file__).resolve().parent / "descriptions"


def robot_description_path(name: str) -> Path:
    """Return the package-owned path for a built-in robot description.

    Args:
        name: Directory name under ``dimos/robot/descriptions``.

    Raises:
        ValueError: If ``name`` contains path separators or traversal.
        FileNotFoundError: If the requested built-in description is not shipped.
    """
    if not name or Path(name).name != name:
        raise ValueError(f"Robot description name must be a single directory name: {name!r}")

    path = _DESCRIPTION_ROOT / name
    if not path.is_dir():
        available = sorted(child.name for child in _DESCRIPTION_ROOT.iterdir() if child.is_dir())
        raise FileNotFoundError(
            f"Built-in robot description not found: {name!r}. Available descriptions: {available}"
        )
    return path


__all__ = ["robot_description_path"]
