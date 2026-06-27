{
  description = "Manifold Tech Odin1 (lidar + camera + onboard odometry) native module for DimOS";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    # Path input to the in-repo rust SDK crates (mirrors virtual_mid360).
    dimos-rust = { url = "path:../../../../native/rust"; flake = false; };
  };

  outputs = { self, nixpkgs, flake-utils, dimos-rust }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs { inherit system; };
        sub = "dimos/hardware/sensors/odin1";

        # PREREQUISITES before this builds:
        #   1. Generate Cargo.lock: `cargo generate-lockfile` in this dir.
        #   2. Fill the odin1 + odin1-sys git hashes below (nix prints the
        #      expected value on the first failing build).
        # The Manifold SDK blob is vendored inside the odin1-rs git dep, not here.
        src = pkgs.runCommand "odin1-src" {} ''
          mkdir -p $out/${sub}
          cp ${./Cargo.toml} $out/${sub}/Cargo.toml
          cp ${./Cargo.lock} $out/${sub}/Cargo.lock
          cp -r ${./src} $out/${sub}/src

          mkdir -p $out/native/rust
          cp -r ${dimos-rust}/dimos-module $out/native/rust/dimos-module
          cp -r ${dimos-rust}/dimos-module-macros $out/native/rust/dimos-module-macros
        '';
      in {
        packages.default = pkgs.rustPlatform.buildRustPackage {
          pname = "odin1-module";
          version = "0.1.0";

          inherit src;
          cargoRoot = sub;
          buildAndTestSubdir = sub;

          # bindgenHook sets LIBCLANG_PATH for odin1-sys (built as a git dep);
          # libusb/openssl are the closed lib's link deps (libstdc++ comes via
          # the cc toolchain).
          nativeBuildInputs = [ pkgs.rustPlatform.bindgenHook ];
          buildInputs = [ pkgs.libusb1 pkgs.openssl ];

          cargoLock = {
            lockFile = ./Cargo.lock;
            outputHashes = {
              "dimos-lcm-0.1.0" = "sha256-4DWFTf7Xqnx6pd2jXA/MVpRmZiFr6HqTSp9Qo9ZjToA=";
              # Placeholders: replace with the hash nix prints on first build.
              "odin1-0.1.0" = "sha256-AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=";
              "odin1-sys-0.1.0" = "sha256-AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=";
            };
          };

          meta.mainProgram = "odin1_module";
        };
      });
}
