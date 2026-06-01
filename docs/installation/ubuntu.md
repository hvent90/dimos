## System Dependencies Install (Ubuntu 22.04 or 24.04)

```sh skip
sudo apt-get update
# Required system libraries. libturbojpeg + portaudio19-dev cover image/audio; libgl1 +
# libegl1 are the OpenGL runtime for open3d and rerun-sdk (both always-installed core deps).
# Without libgl1/libegl1 the visualizer fails at runtime with
# "libGL.so.1: cannot open shared object file" (e.g. on minimal/headless/Docker installs).
sudo apt-get install -y curl g++ portaudio19-dev git-lfs libturbojpeg libgl1 libegl1 python3-dev

# install uv
curl -LsSf https://astral.sh/uv/install.sh | sh && export PATH="$HOME/.local/bin:$PATH"
# NOTE: the `export` above only affects the current shell. Open a new terminal (or run
# `source ~/.bashrc`) so `uv` is on PATH in later sessions — the installer also adds it there.
```

## Using DimOS as a library

```sh skip
mkdir myproject && cd myproject

uv venv --python 3.12
source .venv/bin/activate

# install everything (depending on your use case you might not need all extras,
# check your respective platform guides)
uv pip install 'dimos[misc,sim,visualization,agents,web,perception,unitree,manipulation,cpu]'
```

## Developing on DimOS

```sh skip
# this allows getting large files on-demand (and not pulling all immediately)
export GIT_LFS_SKIP_SMUDGE=1
git clone https://github.com/dimensionalOS/dimos.git
cd dimos

# Install all dependency groups (tests, lint, …) so mypy + pytest are
# both available. For self-hosted tests, see docs/development/testing.md.
uv sync --all-groups

# type check
uv run mypy dimos

# tests (around a minute to run)
uv run pytest --numprocesses=auto dimos
```
