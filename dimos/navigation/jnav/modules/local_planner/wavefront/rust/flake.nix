{
  # This crate lives inside a larger git repo and depends on the in-repo
  # `dimos-module` crate by path, so `nix build` of the crate alone can't
  # resolve it. Build via the dev shell (what module.py's build_command runs):
  #
  #   cd dimos/navigation/jnav/modules/local_planner/wavefront/rust
  #   nix develop path:. --command cargo build --release
  description = "dimos-wavefront: native Rust wavefront local planner";

  # Pin mirrors the sibling gsc_pgo_rs flake (nixpkgs 549bd84) so the rust
  # toolchain is shared in the nix store — no extra download.
  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/549bd84d6279f9852cae6225e372cc67fb91a4c1";
    flake-utils.url = "github:numtide/flake-utils/11707dc2f618dd54ca8739b309ec4fc024de578b";
  };

  outputs = { self, nixpkgs, flake-utils, ... }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs { inherit system; };
      in {
        # Toolchain only — cargo resolves crates from ~/.cargo + git as usual.
        # openssl/git/cacert cover the git-sourced lcm-msgs dependency.
        devShells.default = pkgs.mkShell {
          packages = [
            pkgs.cargo
            pkgs.rustc
            pkgs.pkg-config
            pkgs.openssl
            pkgs.git
            pkgs.cacert
          ];
          env.SSL_CERT_FILE = "${pkgs.cacert}/etc/ssl/certs/ca-bundle.crt";
        };
      });
}
