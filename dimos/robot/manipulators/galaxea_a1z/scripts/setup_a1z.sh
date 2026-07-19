#!/usr/bin/env bash
# Install/verify the pinned Galaxea A1Z SDK as the normal user, then run the
# privileged Linux SocketCAN setup. This is the one-command A1Z host setup.
set -euo pipefail

SDK_REPOSITORY="https://github.com/userguide-galaxea/GALAXEA-A1Z.git"
# Known-working revision from the vendor's gripper branch. Pinning the commit
# prevents a moving vendor branch from silently changing hackathon machines.
SDK_REVISION="e931ecd0e25ad35df251097ba42921b3d2fa7224"
SDK_REQUIREMENT="a1z @ git+${SDK_REPOSITORY}@${SDK_REVISION}"

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPOSITORY_ROOT="$(cd -- "$SCRIPT_DIR/../../../../.." && pwd)"
CAN_SETUP_SCRIPT="$SCRIPT_DIR/setup_a1z_can.sh"

usage() {
    cat <<EOF
Usage: $0 [--sdk-only]

Install or verify the pinned Galaxea A1Z gripper SDK in the DimOS virtual
environment, then configure and test the Linux SocketCAN adapter.

  --sdk-only  Install/verify the SDK without running privileged CAN setup.

Run this command as your normal user. It requests sudo only for SocketCAN.
EOF
}

sdk_only=false
case "${1:-}" in
    "") ;;
    --sdk-only) sdk_only=true ;;
    -h|--help)
        usage
        exit 0
        ;;
    *)
        usage >&2
        exit 2
        ;;
esac

if ((EUID == 0)); then
    cat >&2 <<EOF
Do not run this wrapper with sudo.

It installs the Python SDK into the DimOS virtual environment as your normal
user and requests sudo itself only for the SocketCAN setup:

  $0
EOF
    exit 1
fi

python_bin="${DIMOS_PYTHON:-$REPOSITORY_ROOT/.venv/bin/python}"
if [[ ! -x "$python_bin" ]]; then
    cat >&2 <<EOF
DimOS virtual-environment Python was not found at:
  $python_bin

Create the project environment first, then rerun this setup:
  cd "$REPOSITORY_ROOT"
  uv sync
  "$0"

Set DIMOS_PYTHON=/path/to/python if DimOS uses a different environment.
EOF
    exit 1
fi

verify_sdk() {
    "$python_bin" - <<'PY'
import inspect
import sys

try:
    import a1z
    from a1z.robots.get_robot import get_a1z_robot
except Exception as exc:
    print(f"cannot import a1z: {exc}", file=sys.stderr)
    raise SystemExit(1) from None

try:
    parameters = inspect.signature(get_a1z_robot).parameters
except (TypeError, ValueError) as exc:
    print(f"cannot inspect a1z get_a1z_robot(): {exc}", file=sys.stderr)
    raise SystemExit(1) from None

if "with_gripper" not in parameters:
    print(
        "installed a1z SDK does not expose get_a1z_robot(with_gripper=...); "
        "the vendor gripper branch is required",
        file=sys.stderr,
    )
    raise SystemExit(1)

print(a1z.__file__)
PY
}

sdk_path=""
if sdk_path="$(verify_sdk 2>/dev/null)"; then
    echo "A1Z vendor SDK check passed: G1Z gripper support is available."
    echo "SDK package: $sdk_path"
else
    uv_bin="${A1Z_UV_BIN:-$(command -v uv || true)}"
    if [[ -z "$uv_bin" ]]; then
        cat >&2 <<EOF
The Galaxea A1Z SDK is missing or does not support the G1Z gripper, and 'uv'
was not found. Install uv, then rerun this command.

SDK source:   $SDK_REPOSITORY
Pinned commit: $SDK_REVISION
EOF
        exit 1
    fi

    echo "Installing the pinned Galaxea A1Z gripper SDK into: $python_bin"
    echo "SDK commit: $SDK_REVISION"
    if ! "$uv_bin" pip install --python "$python_bin" "$SDK_REQUIREMENT"; then
        cat >&2 <<EOF

Failed to install the Galaxea A1Z SDK from GitHub.

Check internet/GitHub access, then retry this exact command:
  "$uv_bin" pip install --python "$python_bin" \
    "$SDK_REQUIREMENT"
EOF
        exit 1
    fi

    if ! sdk_path="$(verify_sdk)"; then
        cat >&2 <<EOF

The SDK installation completed, but the installed package still lacks G1Z
gripper support. Do not start the robot. Remove any conflicting 'a1z' package
and rerun this setup.
EOF
        exit 1
    fi
    echo "A1Z vendor SDK installed and verified: $sdk_path"
fi

if [[ "$sdk_only" == true ]]; then
    exit 0
fi

case "$(uname -s)" in
    Linux)
        if ! command -v sudo >/dev/null 2>&1; then
            echo "ERROR: sudo is required for Linux SocketCAN setup." >&2
            exit 1
        fi
        exec sudo "$CAN_SETUP_SCRIPT"
        ;;
    Darwin)
        echo "macOS uses the A1Z userspace USB-CAN transport; SocketCAN setup is not required."
        ;;
    *)
        echo "ERROR: A1Z host setup supports Linux and macOS only." >&2
        exit 1
        ;;
esac
