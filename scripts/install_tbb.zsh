
#!/usr/bin/env zsh
# Usage: install_tbb.zsh <version> [prefix]
# Pulls the prebuilt macOS binary from the GitHub *release* for <version>
# (not a git branch/tag clone) and extracts it into <prefix>.

# More claude slop

set -euo pipefail

VERSION="${1#v}"
PREFIX="${2:-/opt/tbb}"
URL="https://github.com/uxlfoundation/oneTBB/releases/download/v${VERSION}/oneapi-tbb-${VERSION}-mac.tgz"

mkdir -p "$PREFIX"
curl -fsSL "$URL" | tar xz -C "$PREFIX" --strip-components=1
