{
  description = "Native Rust FAST-LIO2 LiDAR-inertial odometry module for dimos";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs = { self, nixpkgs, flake-utils, ... }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs { inherit system; };

        # The crate depends on sibling crates higher up the dimos tree via cargo
        # path deps (../../../../../../native/rust/{dimos-module,dimos-module-macros}).
        # Reassemble that tree at the same relative depth so the paths resolve
        # inside the build sandbox.
        combinedSrc = pkgs.runCommandLocal "rustlio2-src" { } ''
          crate=$out/dimos/hardware/sensors/lidar/rustlio2/rust
          mkdir -p "$crate" $out/native/rust
          cp -r ${./.}/. "$crate"
          cp -r ${../../../../../../native/rust/dimos-module} $out/native/rust/dimos-module
          cp -r ${../../../../../../native/rust/dimos-module-macros} $out/native/rust/dimos-module-macros
          chmod -R u+w $out
          rm -rf "$crate/target" "$crate/result"
        '';

        # dimos-lcm and lcm-msgs are the same git source at the same rev.
        dimosLcmHash = "sha256-4DWFTf7Xqnx6pd2jXA/MVpRmZiFr6HqTSp9Qo9ZjToA=";

        rustlio2_native = pkgs.rustPlatform.buildRustPackage {
          pname = "rustlio2_native";
          version = "0.1.0";

          src = combinedSrc;
          sourceRoot = "rustlio2-src/dimos/hardware/sensors/lidar/rustlio2/rust";

          cargoLock = {
            lockFile = ./Cargo.lock;
            outputHashes = {
              "dimos-lcm-0.1.0" = dimosLcmHash;
              "lcm-msgs-0.1.0" = dimosLcmHash;
            };
          };

          doCheck = false;
        };
      in {
        packages = {
          default = rustlio2_native;
          inherit rustlio2_native;
        };
      });
}
