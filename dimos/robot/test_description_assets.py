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

from __future__ import annotations

import importlib.util
from pathlib import Path
import re
import sys

import pytest

from dimos.robot.catalog.a750 import A750_FK_MODEL, a750
from dimos.robot.catalog.openarm import OPENARM_V10_FK_MODEL, openarm_arm, openarm_single
from dimos.robot.catalog.piper import PIPER_FK_MODEL, piper
from dimos.robot.catalog.ufactory import XARM6_FK_MODEL, XARM7_FK_MODEL, xarm6, xarm7
from dimos.robot.config import RobotConfig
from dimos.robot.description_assets import robot_description_path
from dimos.robot.model_parser import parse_model
from dimos.robot.unitree.g1.config import G1

if sys.version_info >= (3, 11):
    import tomllib
else:
    import tomli as tomllib


REPO_ROOT = Path(__file__).resolve().parents[2]
DESCRIPTION_NAMES = (
    "xarm_description",
    "piper_description",
    "a750_description",
    "openarm_description",
)
DESCRIPTION_ARCHIVES = tuple(f"{name}.tar.gz" for name in DESCRIPTION_NAMES)
DESCRIPTION_CATALOGS = (
    REPO_ROOT / "dimos/robot/catalog/ufactory.py",
    REPO_ROOT / "dimos/robot/catalog/piper.py",
    REPO_ROOT / "dimos/robot/catalog/a750.py",
    REPO_ROOT / "dimos/robot/catalog/openarm.py",
    REPO_ROOT / "dimos/hardware/manipulators/openarm/adapter.py",
)
DESCRIPTION_SOURCE_PATTERN = re.compile(
    r"(?:LfsPath|get_data)\(\s*['\"](?:xarm_description|piper_description|a750_description|openarm_description)"
)


@pytest.mark.parametrize("name", DESCRIPTION_NAMES)
def test_robot_description_path_returns_existing_normal_path(name: str) -> None:
    path = robot_description_path(name)

    assert path.__class__.__module__ == "pathlib"
    assert path.is_dir()
    assert path.name == name


def test_robot_description_path_rejects_unknown_or_nested_names() -> None:
    with pytest.raises(FileNotFoundError, match="Built-in robot description not found"):
        robot_description_path("missing_description")

    with pytest.raises(ValueError, match="single directory name"):
        robot_description_path("../xarm_description")


@pytest.mark.parametrize(
    ("model_path", "package_root"),
    (
        (XARM6_FK_MODEL, robot_description_path("xarm_description")),
        (XARM7_FK_MODEL, robot_description_path("xarm_description")),
        (PIPER_FK_MODEL, robot_description_path("piper_description")),
        (A750_FK_MODEL, robot_description_path("a750_description")),
        (OPENARM_V10_FK_MODEL, robot_description_path("openarm_description")),
    ),
)
def test_catalog_fk_models_exist(model_path: Path, package_root: Path) -> None:
    assert model_path.exists()
    assert package_root in model_path.parents


@pytest.mark.parametrize(
    "robot_config",
    (
        xarm6(),
        xarm7(),
        piper(),
        a750(),
        openarm_arm("left"),
        openarm_arm("right"),
        openarm_single(),
        G1,
    ),
)
def test_builtin_robot_config_paths_exist(robot_config: RobotConfig) -> None:
    if robot_config.model_path is not None:
        assert robot_config.model_path.exists()

    for package_path in robot_config.package_paths.values():
        assert package_path.is_dir()


@pytest.mark.parametrize(
    "robot_config",
    (
        xarm6(),
        xarm7(),
        piper(),
        a750(),
        openarm_arm("left"),
        openarm_arm("right"),
        openarm_single(),
        G1,
    ),
)
def test_builtin_robot_configs_parse_without_git_lfs(robot_config: RobotConfig) -> None:
    if robot_config.model_path is not None and robot_config.model_path.suffix == ".xacro":
        if importlib.util.find_spec("xacro") is None:
            pytest.skip("xacro is not installed")

    description = robot_config.model_description

    assert description.links
    assert robot_config.resolved_joint_names


@pytest.mark.parametrize(
    ("model_path", "package_paths"),
    (
        (XARM7_FK_MODEL, {"xarm_description": robot_description_path("xarm_description")}),
        (PIPER_FK_MODEL, {"piper_description": robot_description_path("piper_description")}),
        (A750_FK_MODEL, {"a750_description": robot_description_path("a750_description")}),
        (
            OPENARM_V10_FK_MODEL,
            {"openarm_description": robot_description_path("openarm_description")},
        ),
        (G1.model_path, G1.package_paths),
    ),
)
def test_supported_builtin_models_parse(
    model_path: Path | None, package_paths: dict[str, Path]
) -> None:
    assert model_path is not None

    description = parse_model(model_path, package_paths)

    assert description.links
    assert description.actuated_joint_names


def test_description_package_data_config_includes_full_tree() -> None:
    pyproject = tomllib.loads((REPO_ROOT / "pyproject.toml").read_text())
    package_data = pyproject["tool"]["setuptools"]["package-data"]["dimos"]
    manifest = (REPO_ROOT / "MANIFEST.in").read_text()

    assert "robot/descriptions/**" in package_data
    assert "recursive-include dimos/robot/descriptions *" in manifest


def test_description_lfs_archives_are_not_runtime_sources() -> None:
    lfs_dir = REPO_ROOT / "data/.lfs"

    for archive in DESCRIPTION_ARCHIVES:
        assert not (lfs_dir / archive).exists()


@pytest.mark.parametrize("source_path", DESCRIPTION_CATALOGS)
def test_builtin_description_sources_do_not_use_lfs_loader(source_path: Path) -> None:
    assert not DESCRIPTION_SOURCE_PATTERN.search(source_path.read_text())
