{
  description = "Point-LIO C++ build toolchain for the LIO autoresearch experiment (cmake, eigen, pcl, yaml-cpp, boost, zstd).";

  # STANDALONE on purpose. This flake does NOT reference the root dimos flake.
  # The dimos worktree is ~100 G and usually dirty, and a flake that lives inside
  # that repo (or takes it as an input) makes `nix develop` copy the whole tree
  # into the store. By keeping this independent — only nixpkgs as an input — and
  # invoking it with `path:` (see ../setup.sh), `nix develop` copies just this
  # point_lio directory, not the repo. A bare `nix develop` (no `path:`) would
  # still trigger the repo copy, so always use the `path:` form.
  #
  # Scope is the C++ build only. Python (numpy, matplotlib, the dimos package for
  # get_data) comes from the dimos .venv, which runs fine without any nix env —
  # so after building, you "drop into" the dimos venv to run evaluate.py/algo.py.
  inputs = {
    # Pinned to the same rev as the root dimos flake, so pcl/boost/etc. resolve
    # to store paths the dimos dev shell already realized (no extra downloads).
    nixpkgs.url = "github:NixOS/nixpkgs/d233902339c02a9c334e7e593de68855ad26c4cb";
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs = { self, nixpkgs, flake-utils, ... }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs { inherit system; };
      in {
        devShells.default = pkgs.mkShell {
          nativeBuildInputs = [ pkgs.cmake pkgs.pkg-config ];
          buildInputs = [
            pkgs.eigen
            pkgs.pcl
            pkgs.yaml-cpp
            pkgs.boost
            pkgs.zstd
          ];
          # nixpkgs lays PCL headers under include/pcl-<major.minor>/, which
          # point_lio's hand-rolled find_path (HINTS only /usr/include/pcl*)
          # can't see. Put that dir on CMAKE_INCLUDE_PATH, which find_path
          # honors — keeps the vendored substrate untouched. Other deps resolve
          # via the cmake setup hook (they're buildInputs).
          shellHook = ''
            for d in ${pkgs.pcl}/include/pcl-*; do
              export CMAKE_INCLUDE_PATH="$d''${CMAKE_INCLUDE_PATH:+:$CMAKE_INCLUDE_PATH}"
            done
          '';
        };
      });
}
