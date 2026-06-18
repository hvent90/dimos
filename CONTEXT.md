# DimOS

DimOS is an agentic operating system for robotics. This glossary defines project-specific domain language used when discussing robot runtime assets and packaging.

## Language

**Robot assets**:
Small static files required to instantiate supported robot models at runtime: URDF files, mesh files referenced by those URDFs, and SRDF files. Robot assets exclude replay data, perception model weights, learned policies, maps, recordings, and benchmark datasets.
_Avoid_: robot artifacts, data files, LFS files

**Robot description**:
A general model of a robot's kinematic structure, geometry, and semantic planning metadata, represented for now by URDF files, referenced meshes, and SRDF files.
_Avoid_: simulation asset, xacro package, robot artifact

**External robot description**:
A robot description supplied by a user or integrator from the local filesystem. External robot descriptions are referenced by normal paths, not by Git repositories or DimOS-managed downloads.
_Avoid_: external repo, robot package loader, remote robot asset

**Built-in robot description**:
A robot description for a robot DimOS supports directly. Built-in robot descriptions are shipped with the DimOS Python package and are stored in their canonical, runtime-ready form.
_Avoid_: LFS robot description, bundled repo checkout, robot artifact
