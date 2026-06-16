# Robot Assets

`dimos.robot.assets` resolves robot description sources into local filesystem paths.
It is the home for Git-backed robot model assets, package-root resolution, and
generic URDF rendering helpers.

This directory is intentionally self-contained so it can be extracted later. Do
not add compatibility wrappers outside this module for new code. Import directly
from the source modules, for example:

```python
from dimos.robot.assets.manager import RobotAssetPath, robot_asset_package_paths
```

There is no `__init__.py` on purpose: DimOS disallows package `__init__.py` files
except at the root package to avoid accidental import side effects.

## Cache behavior

Assets live under:

```text
~/.cache/dimos/robot_assets/
├── sources/                 # Git checkouts by source identity
├── locks/                   # per-source file locks
└── derived/
    ├── rendered_urdfs/      # generic rendered URDF cache
    └── drake_urdfs/         # Drake-specific prepared URDF cache
```

`GitAssetCache` uses the “fresh-when-safe” policy:

- clone when the source is missing;
- update clean cached repos before use;
- warn and keep cached content if update fails;
- warn and skip update for dirty cached repos, preserving local edits.

## Declaring a robot asset

Add declarations in `declarations.py`:

```python
from dimos.robot.assets.manager import RobotAssetDeclaration

ROBOT_ASSETS["myarm"] = RobotAssetDeclaration(
    model="myarm",
    repo_url="https://github.com/example/myarm_description",
    ref="main",  # branch, tag, or commit
    artifacts={
        "urdf": "urdf/myarm.urdf.xacro",
        "mesh_dir": "meshes",
    },
    package_roots={"myarm_description": "."},
    xacro_args={"limited": "true"},
)
```

Artifact role keys are strings. Common roles are `urdf`, `mjcf`, `srdf`, and
`mesh_dir`; extra flat roles such as `urdf_ik` are allowed when a catalog needs
an additional model variant.

`package_roots` maps ROS package names to directories inside the checkout. These
roots are used for `package://...` URIs and Xacro `$(find package_name)`.

## Using assets in catalogs

Catalogs should stay lazy at import time:

```python
from dimos.robot.assets.manager import RobotAssetPath, robot_asset_package_paths

model_path = RobotAssetPath("myarm", "urdf")
package_paths = robot_asset_package_paths("myarm")
```

`RobotAssetPath` and `RobotAssetPackagePath` defer clone/update/path validation
until path operations such as `str(path)`, `path.resolve()`, or `path.exists()`.

## Rendering URDFs

Use `processing.py` for generic robot-description rendering:

```python
from dimos.robot.assets.processing import render_urdf

rendered_path = render_urdf(
    model_path,
    package_paths,
    xacro_args={"limited": "true"},
    package_uri_mode="preserve",  # or "absolute"
)
```

Keep consumer-specific processing outside this module. For example, Drake-specific
cleanup still belongs in `dimos/manipulation/planning/utils/mesh_utils.py`.
