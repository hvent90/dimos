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

import os
from pathlib import Path
import subprocess

import pytest

from dimos.robot.assets.declarations import ROBOT_ASSETS
from dimos.robot.assets.git_cache import GitAssetCache
from dimos.robot.assets.manager import (
    ArtifactRole,
    RobotAssetDeclaration,
    RobotAssetError,
    RobotAssetManager,
    RobotAssetPackagePath,
    RobotAssetPath,
    robot_asset_package_paths,
    robot_asset_xacro_args,
    set_default_robot_asset_manager,
)


def _git(cwd: Path, *args: str) -> str:
    return subprocess.run(
        ["git", *args],
        cwd=cwd,
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()


@pytest.fixture()
def asset_manager(tmp_path: Path) -> RobotAssetManager:
    repo = tmp_path / "robot_assets"
    repo.mkdir()
    _git(repo, "init", "-b", "main")
    _git(repo, "config", "user.email", "test@example.com")
    _git(repo, "config", "user.name", "Test User")
    (repo / "robots" / "testbot").mkdir(parents=True)
    (repo / "robots" / "testbot" / "model.urdf").write_text("<robot name='testbot'/>")
    (repo / "packages" / "testbot_description").mkdir(parents=True)
    (repo / "packages" / "testbot_description" / "package.xml").write_text("<package/>")
    _git(repo, "add", ".")
    _git(repo, "commit", "-m", "assets")

    declaration = RobotAssetDeclaration(
        model="testbot",
        repo_url=str(repo),
        ref="main",
        artifacts={"urdf": "robots/testbot/model.urdf"},
        package_roots={"testbot_description": "packages/testbot_description"},
    )
    return RobotAssetManager(
        {"testbot": declaration},
        git_cache=GitAssetCache(tmp_path / "cache"),
    )


def test_resolves_artifact_paths_and_package_roots(asset_manager: RobotAssetManager) -> None:
    artifact = asset_manager.resolve_artifact("testbot", ArtifactRole.URDF)
    package_root = asset_manager.resolve_package_root("testbot", "testbot_description")

    assert artifact.name == "model.urdf"
    assert artifact.read_text() == "<robot name='testbot'/>"
    assert package_root.name == "testbot_description"
    assert (package_root / "package.xml").exists()


def test_unknown_model_and_undeclared_artifact_role_raise(asset_manager: RobotAssetManager) -> None:
    with pytest.raises(RobotAssetError, match="Unknown robot asset model 'missing'"):
        asset_manager.resolve_artifact("missing", ArtifactRole.URDF)

    with pytest.raises(RobotAssetError, match="does not declare artifact role 'mjcf'"):
        asset_manager.resolve_artifact("testbot", ArtifactRole.MJCF)


def test_lazy_asset_paths_resolve_only_on_path_operations(asset_manager: RobotAssetManager) -> None:
    artifact_path = RobotAssetPath("testbot", ArtifactRole.URDF, manager=asset_manager)
    package_path = RobotAssetPackagePath("testbot", "testbot_description", manager=asset_manager)

    assert object.__getattribute__(artifact_path, "_robot_asset_resolved_cache") is None
    assert object.__getattribute__(package_path, "_robot_asset_resolved_cache") is None

    artifact_string = str(artifact_path)
    assert artifact_string.endswith("robots/testbot/model.urdf")
    assert object.__getattribute__(artifact_path, "_robot_asset_resolved_cache") is not None

    assert os.fspath(package_path).endswith("packages/testbot_description")
    assert object.__getattribute__(package_path, "_robot_asset_resolved_cache") is not None

    assert artifact_path.exists()
    assert (package_path / "package.xml").exists()


def test_default_manager_can_be_injected(asset_manager: RobotAssetManager) -> None:
    set_default_robot_asset_manager(asset_manager)
    try:
        assert robot_asset_package_paths("testbot")["testbot_description"].exists()
        assert robot_asset_xacro_args("testbot") == {}
    finally:
        set_default_robot_asset_manager(None)


def test_robot_asset_declarations_are_static_and_consistent() -> None:
    known_roles = {role.value for role in ArtifactRole} | {"urdf_ik"}

    for key, declaration in ROBOT_ASSETS.items():
        assert key == declaration.model
        assert declaration.repo_url
        assert declaration.ref
        assert declaration.artifacts

        for role, relative_path in declaration.artifacts.items():
            assert role in known_roles
            assert relative_path
            assert not Path(relative_path).is_absolute()

        for package_name, relative_path in declaration.package_roots.items():
            assert package_name
            assert relative_path
            assert not Path(relative_path).is_absolute()
