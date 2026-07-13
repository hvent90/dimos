{
  description = "dimos-repulsive-field: native Rust repulsive-field local planner";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    # Relative git+file: will be deprecated (nix#12281) but there's no
    # viable alternative for reaching local path deps outside the flake dir currently
    # presumably an alternative will be added before this is removed.
    # This crate is 5 dirs below repo root
    # (dimos/navigation/nav_3d/repulsive_local_planner/rust), so go up 5.
    # Track this feature branch: its dimos-module differs from main and is the
    # version this crate compiles against.
    dimos-repo = { url = "git+file:../../../../..?ref=jeff/feat/local_plan"; flake = false; };
  };

  outputs = { self, nixpkgs, flake-utils, dimos-repo }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs { inherit system; };

        src = pkgs.runCommand "dimos-repulsive-field-src" {} ''
          mkdir -p $out/dimos/navigation/nav_3d/repulsive_local_planner/rust
          cp -r ${./src} $out/dimos/navigation/nav_3d/repulsive_local_planner/rust/src
          cp ${./Cargo.toml} $out/dimos/navigation/nav_3d/repulsive_local_planner/rust/Cargo.toml
          cp ${./Cargo.lock} $out/dimos/navigation/nav_3d/repulsive_local_planner/rust/Cargo.lock

          mkdir -p $out/native/rust
          cp -r ${dimos-repo}/native/rust/dimos-module $out/native/rust/dimos-module
          cp -r ${dimos-repo}/native/rust/dimos-module-macros $out/native/rust/dimos-module-macros
        '';
      in {
        packages.default = pkgs.rustPlatform.buildRustPackage {
          pname = "dimos-repulsive-field";
          version = "0.1.0";

          inherit src;
          cargoRoot = "dimos/navigation/nav_3d/repulsive_local_planner/rust";
          buildAndTestSubdir = "dimos/navigation/nav_3d/repulsive_local_planner/rust";

          # Binary-only build: just the repulsive_field bin (native feature is
          # the default, and the bin requires it). No lib/wasm/web, no tests.
          cargoBuildFlags = [ "--bin" "repulsive_field" ];
          doCheck = false;

          cargoHash = "sha256-2g1oWdr4RyMFoujGo+QPd52661oNt6hAsuHBwzGNOdQ=";

          # Keep the output light: only $out/bin/repulsive_field.
          postInstall = ''
            rm -rf $out/lib
          '';

          meta.mainProgram = "repulsive_field";
        };
      });
}
