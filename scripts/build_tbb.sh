#!/usr/bin/env bash
# Usage: install_tbb.sh <version> [prefix]

# We need this because we need a specific version of TBB (2022.1.0-1) for the thirdparty GTSAM to work

# Claude slop

set -euo pipefail

VERSION="$1"
PREFIX="${2:-/usr/local}"
TAG="${VERSION#v}"
TAG="v$TAG"

DIR="$(mktemp -d)"
trap 'rm -rf "$DIR"' EXIT

git clone --branch "$TAG" --depth 1 https://github.com/uxlfoundation/oneTBB.git "$DIR/src"
cmake -S "$DIR/src" -B "$DIR/build" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$PREFIX" -DTBB_TEST=OFF
cmake --build "$DIR/build" -j"$(nproc)"
cmake --install "$DIR/build"
ldconfig || true
