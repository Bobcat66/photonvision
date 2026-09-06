#!/usr/bin/env bash
# Usage: install_tbb.sh <version> [prefix]
# Pulls the prebuilt Linux binary from the GitHub *release* for <version>
# (not a git branch/tag clone) and extracts it into <prefix>.

# Claude slop

set -euo pipefail

VERSION="${1#v}"
PREFIX="${2:-/opt/tbb}"
URL="https://github.com/uxlfoundation/oneTBB/releases/download/v${VERSION}/oneapi-tbb-${VERSION}-lin.tgz"

mkdir -p "$PREFIX"
curl -fsSL "$URL" | tar xz -C "$PREFIX" --strip-components=1

LIBDIR=/opt/tbb/lib/intel64/gcc4.8    # the actual arch/compiler subfolder, not lib/ itself

echo "$LIBDIR" | tee /etc/ld.so.conf.d/tbb.conf
sudo ldconfig
