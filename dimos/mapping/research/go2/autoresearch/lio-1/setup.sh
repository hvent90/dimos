#!/usr/bin/env bash
# Environment prep for the LIO autoresearch experiment. Run once before the loop.
#
# Just run `./setup.sh` — it bootstraps the point_lio build shell itself
# (point_lio/flake.nix: a standalone flake with cmake/eigen/pcl/yaml-cpp/boost/
# zstd), so you do NOT need to `nix develop` first. Python (numpy, matplotlib,
# dimos.get_data) comes from the dimos .venv, which runs without any nix env.
set -euo pipefail

# Resolve our own absolute path before any cd, so the re-exec below is robust.
SELF="$(cd "$(dirname "$0")" && pwd)/$(basename "$0")"
HERE="$(dirname "$SELF")"

# --- bootstrap: re-exec inside the point_lio build shell if not already there.
# The `path:` ref is load-bearing — point_lio/flake.nix lives inside the ~100 G
# dimos repo, and a bare `nix develop` would resolve it as git+file and copy the
# whole tree into the store. `path:` copies just point_lio/. Guard var stops
# infinite recursion. ---
if [ -z "${LIO_NIX_SHELL:-}" ]; then
  command -v nix >/dev/null 2>&1 || {
    echo "ERROR: nix not found. Install nix (flakes enabled), or provide cmake + eigen/pcl/yaml-cpp/boost/zstd yourself."; exit 1; }
  echo ">> entering point_lio build shell (path:$HERE/point_lio)..."
  exec env LIO_NIX_SHELL=1 nix develop "path:$HERE/point_lio" --command bash "$SELF" "$@"
fi

cd "$HERE"

# --- build tools come from the nix shell ---
command -v cmake >/dev/null || { echo "ERROR: cmake missing (build shell didn't load?)"; exit 1; }

# --- dimos venv: numpy/matplotlib + dimos.get_data all live here ---
echo ">> checking dimos venv"
if ! python -c "import dimos, numpy, matplotlib" 2>/dev/null; then
  root=$(git rev-parse --show-toplevel 2>/dev/null || echo "")
  # shellcheck disable=SC1091
  [ -n "$root" ] && [ -f "$root/.venv/bin/activate" ] && . "$root/.venv/bin/activate"
fi
python -c "import dimos, numpy, matplotlib" || {
  echo "ERROR: dimos venv not available. Activate the dimos .venv (numpy, matplotlib, dimos)."; exit 1; }

# --- build the Point-LIO substrate (fixed; not edited by the agent) ---
echo ">> building point_lio"
cmake -S point_lio -B point_lio/build -DCMAKE_BUILD_TYPE=Release >/dev/null
cmake --build point_lio/build -j"$(nproc 2>/dev/null || echo 4)"

# --- sanity check (pulls the data via get_data on first run) ---
echo ">> data + harness check"
python evaluate.py
echo ">> setup done. Run a baseline with:  python algo.py > run.log 2>&1"
