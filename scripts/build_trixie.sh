#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(git -C "$SCRIPT_DIR" rev-parse --show-toplevel)"

if (($# == 0)); then
    echo "Usage: $0 <command> [arguments...]" >&2
    exit 2
fi

docker run --rm -it \
    -v "$REPO_ROOT:/workspace" \
    -w /workspace \
    wpilib/debian-base:trixie \
    bash -c '
        set -euo pipefail

        apt-get update
        apt-get install -y \
            ninja-build \
            build-essential \
            libtbb-dev=2022.1.0-1+deb13u1 \
            nodejs \
            npm

        npm install -g pnpm

        exec "$@"
    ' bash "$@"