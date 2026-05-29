{
  description = "LIO autoresearch: Point-LIO build deps (pcl, yaml-cpp, boost) layered onto the dimos dev shell";

  inputs = {
    # The root dimos flake, layered under this one to inherit the full dev shell.
    #
    # GOTCHA: the rev pin below is load-bearing. To evaluate a flake input, Nix
    # copies that flake's source into the store, and the dimos worktree is ~100 G
    # (≈20 G tracked with smudged LFS, plus untracked junk) and usually dirty:
    #   - `path:../../../../..`        -> copies the whole 100 G tree. No.
    #   - bare `git+file:` (dirty tree) -> copies the smudged ~20 G working tree. No.
    #   - `git+file:...?rev=<sha>`      -> exports from git OBJECTS at that commit:
    #                                      LFS files are ~130 B pointers, untracked
    #                                      files don't exist. ~16 M closure, locks
    #                                      in <1 s. <- this.
    # Bump <sha> only when the root flake's dev shell changes (it rarely does).
    # Also note: this flake.nix must be `git add`ed — Nix won't see an untracked
    # flake file that lives inside a git repo.
    dimos.url = "git+file:../../../../..?rev=6c24579da002fb050231a9486d2b008cc0d7cada&shallow=1";
    nixpkgs.follows = "dimos/nixpkgs";
    flake-utils.follows = "dimos/flake-utils";
  };

  outputs = { self, dimos, nixpkgs, flake-utils, ... }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs { inherit system; };
      in {
        # The full dimos dev shell + the three native deps Point-LIO needs that
        # the root flake doesn't already carry (eigen and cmake come from root).
        devShells.default =
          dimos.devShells.${system}.default.overrideAttrs (old: {
            buildInputs = (old.buildInputs or []) ++ [
              pkgs.pcl
              pkgs.yaml-cpp
              pkgs.boost
              pkgs.zstd  # MCAP zstd chunk decompression (vendored mcap C++ reader)
            ];
            # nixpkgs lays PCL headers under include/pcl-<major.minor>/, which
            # point_lio's hand-rolled find_path (HINTS only /usr/include/pcl*)
            # can't see. Put that dir on CMAKE_INCLUDE_PATH, which find_path
            # honors — keeps the vendored substrate untouched. Libs resolve via
            # CMAKE_PREFIX_PATH (pcl is in buildInputs).
            shellHook = (old.shellHook or "") + ''
              for d in ${pkgs.pcl}/include/pcl-*; do
                export CMAKE_INCLUDE_PATH="$d''${CMAKE_INCLUDE_PATH:+:$CMAKE_INCLUDE_PATH}"
              done
            '';
          });
      });
}
