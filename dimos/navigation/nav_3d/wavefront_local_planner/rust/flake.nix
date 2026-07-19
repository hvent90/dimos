{
  description = "dimos-wavefront: native Rust wavefront local planner";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    rust-overlay.url = "github:oxalica/rust-overlay";
    rust-overlay.inputs.nixpkgs.follows = "nixpkgs";
    flake-utils.url = "github:numtide/flake-utils";
    # Relative git+file: will be deprecated (nix#12281) but there's no
    # viable alternative for reaching local path deps outside the flake dir currently
    # presumably an alternative will be added before this is removed.
    # This crate is 5 dirs below repo root
    # (dimos/navigation/nav_3d/wavefront_local_planner/rust), so go up 5.
    # Track this feature branch: its dimos-module differs from main and is the
    # version this crate compiles against.
    dimos-repo = { url = "git+file:../../../../..?ref=jeff/feat/local_plan"; flake = false; };
  };

  outputs = { self, nixpkgs, rust-overlay, flake-utils, dimos-repo }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        overlays = [ (import rust-overlay) ];
        pkgs = import nixpkgs { inherit system overlays; };

        cargoRoot = "dimos/navigation/nav_3d/wavefront_local_planner/rust";

        # Assemble a source tree that also contains the two local path deps
        # (dimos-module + dimos-module-macros) the crate references via
        # ../../../../../native/rust. Shared by native + cross builds.
        src = pkgs.runCommand "dimos-wavefront-src" {} ''
          mkdir -p $out/${cargoRoot}
          cp -r ${./src} $out/${cargoRoot}/src
          cp ${./Cargo.toml} $out/${cargoRoot}/Cargo.toml
          cp ${./Cargo.lock} $out/${cargoRoot}/Cargo.lock

          mkdir -p $out/native/rust
          cp -r ${dimos-repo}/native/rust/dimos-module $out/native/rust/dimos-module
          cp -r ${dimos-repo}/native/rust/dimos-module-macros $out/native/rust/dimos-module-macros
        '';

        # Pinned stable toolchain with the aarch64-musl cross-target bundled.
        # Used as the compiler in the cross build.
        rustToolchain = pkgs.rust-bin.stable.latest.default.override {
          targets = [
            "aarch64-unknown-linux-musl"
          ];
        };

        rustPlatform = pkgs.makeRustPlatform {
          cargo = rustToolchain;
          rustc = rustToolchain;
        };

        # Cross pkgs set — used only to pull in the musl GCC cross-linker.
        pkgsCrossArm64 = import nixpkgs {
          inherit system;
          crossSystem.config = "aarch64-unknown-linux-musl";
        };

        commonArgs = {
          pname = "dimos-wavefront";
          version = "0.1.0";
          inherit src;
          inherit cargoRoot;
          buildAndTestSubdir = cargoRoot;
          cargoHash = "sha256-sRzHsjVWzmL7HE9TOsg+/v49psLkfSzFEAckIcban+4=";
          meta.mainProgram = "wavefront";
        };

        # ── native build (host system) ──────────────────────────────────────
        # Binary-only: just the wavefront bin (native feature is the
        # default, and the bin requires it). No lib/wasm/web, no tests.
        buildNative = pkgs.rustPlatform.buildRustPackage (commonArgs // {
          cargoBuildFlags = [ "--bin" "wavefront" ];
          doCheck = false;
          postInstall = ''
            rm -rf $out/lib
          '';
        });

        # ── generic cross build ─────────────────────────────────────────────
        # Overrides build/install so cargo targets the foreign triple and the
        # binary is picked up from the target-specific output directory.
        buildCross = rustTarget: ccPkgs:
          let
            targetSnake = builtins.replaceStrings ["-"] ["_"] rustTarget;
            targetUpper = pkgs.lib.toUpper targetSnake;
            ccBinDir    = "${ccPkgs.stdenv.cc}/bin";
            ccPrefix    = ccPkgs.stdenv.cc.targetPrefix;
          in
          rustPlatform.buildRustPackage (commonArgs // {
            pname = "dimos-wavefront-${rustTarget}";
            doCheck = false;

            buildPhase = ''
              runHook preBuild
              ( cd ${cargoRoot} && cargo build --release --target ${rustTarget} --bin wavefront )
              runHook postBuild
            '';

            installPhase = ''
              runHook preInstall
              mkdir -p $out/bin
              install -m755 ${cargoRoot}/target/${rustTarget}/release/wavefront $out/bin/wavefront
              # Keep the output binary-only.
              rm -rf $out/lib
              runHook postInstall
            '';

            # Tell cargo which linker to use for the foreign target.
            "CARGO_TARGET_${targetUpper}_LINKER" = "${ccBinDir}/${ccPrefix}cc";

            # Tell cc-rs (build scripts compiling any C deps) which cross
            # toolchain to use, so it doesn't fall back to host clang with a
            # wrong sysroot.
            "CC_${targetSnake}"  = "${ccBinDir}/${ccPrefix}cc";
            "CXX_${targetSnake}" = "${ccBinDir}/${ccPrefix}c++";
            "AR_${targetSnake}"  = "${ccBinDir}/${ccPrefix}ar";
          });

      in {
        packages = {
          default = buildNative;
          wavefront-aarch64 =
            buildCross "aarch64-unknown-linux-musl" pkgsCrossArm64;
        };
      });
}
