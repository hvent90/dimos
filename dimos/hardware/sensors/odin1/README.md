# Odin1 sensor module

A dimos native source for the Manifold Tech Odin1 (dTOF lidar + global-shutter
RGB camera + IMU, factory pre-calibrated, onboard SLAM). It publishes lidar point
clouds, the RGB image, and onboard odometry. Because the device runs SLAM
onboard, no host-side LIO/VIO is required.

## Layout

```
odin1/                 (this dir, in the dimos tree — source only, no SDK blob)
  module.py            Python NativeModule wrapper (Out ports, config, perception specs)
  blueprints.py        demo wiring (Odin1 -> rerun)
  flake.nix            nix build of the Rust binary
  Cargo.toml           the odin1_module binary package
  src/main.rs          #[derive(Module)] source: connect, pump frames, publish
  src/convert.rs       Frame -> PointCloud2/Image/Odometry
```

The vendor wrapper and the proprietary SDK blob live in a separate repo,
**github.com/aclauer/odin1-rs** (`odin1-sys` + `odin1`), pulled in as a git
dependency. dimos's tree carries no binary blobs.

## Key steps to implement (in order)

1. **Vendor the SDK** in the `odin1-rs` repo — drop Manifold's headers + static
   libs into `odin1-sys/vendor/`. Confirm with Manifold that the prebuilt
   `liblydHostApi` blob is redistributable before committing it there.

2. **FFI device lifecycle + frame decode** in `odin1-rs` (`odin1/src/lib.rs`):
   the `system_init -> create -> register_cb -> open -> set_mode -> start_stream`
   sequence, the `extern "C"` callbacks, and the per-stream plane decoding. This
   is the core FFI work. Tag a release.

3. **Pin the dependency** here: set the `odin1` git tag/rev in `Cargo.toml`,
   generate `Cargo.lock`, and fill the `odin1`/`odin1-sys` hashes in `flake.nix`
   (nix prints the expected value on first build).

4. **Conversion polish** in `src/convert.rs`: confidence/offset_time PointFields,
   NV12->BGR decode, CameraInfo from `calib.yaml` (upgrades the wrapper to
   `perception.Camera`).

5. **Bring-up test**: `nix build .#default`, run `odin1_module` standalone,
   confirm per-stream frame counts, then wire `blueprints.py` and view in rerun.

During FFI bring-up, develop against a local checkout of odin1-rs with a cargo
patch to skip the push/pin loop:

```toml
[patch."https://github.com/aclauer/odin1-rs.git"]
odin1 = { path = "../../../../../odin1-rs/odin1" }
```

## Open questions to resolve on the bench

- `calib.yaml` schema (camera intrinsics/distortion, lidar<->camera extrinsics).
- Cross-stream timestamp sync (device clock vs host; the SDK exposes NTP/PTP).
- Whether SLAM mode delivers raw dtof + RGB + odometry simultaneously as assumed.
