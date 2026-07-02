#!/usr/bin/env bash
# Builds and installs educelab/smgl and educelab/libcore from source.
# Neither is packaged for apt or bundled in ci-docker, and both are required
# by volume-cartographer's find_package() checks. Run before configuring.
# Requires: git, cmake, ninja, and a system nlohmann_json (e.g. nlohmann-json3-dev).
set -euo pipefail

SMGL_VERSION="${SMGL_VERSION:-v0.11.0-rc.2}"
LIBCORE_VERSION="${LIBCORE_VERSION:-v0.3.0-rc.1}"
PREFIX="${1:-/usr/local}"
BUILD_TYPE="${CI_INSTALL_BUILD_TYPE:-Release}"

workdir="$(mktemp -d)"
trap 'rm -rf "$workdir"' EXIT

git clone --quiet https://github.com/educelab/smgl.git "$workdir/smgl"
git -C "$workdir/smgl" checkout --quiet "$SMGL_VERSION"
cmake -S "$workdir/smgl" -B "$workdir/smgl/build" -GNinja \
    -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
    -DCMAKE_INSTALL_PREFIX="$PREFIX" \
    -DSMGL_BUILD_JSON=OFF -DSMGL_USE_BOOSTFS=OFF \
    -DSMGL_BUILD_TESTS=OFF -DSMGL_BUILD_DOCS=OFF
cmake --build "$workdir/smgl/build"
cmake --install "$workdir/smgl/build"

git clone --quiet https://github.com/educelab/libcore.git "$workdir/libcore"
git -C "$workdir/libcore" checkout --quiet "$LIBCORE_VERSION"
cmake -S "$workdir/libcore" -B "$workdir/libcore/build" -GNinja \
    -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
    -DCMAKE_INSTALL_PREFIX="$PREFIX" \
    -DEDUCE_CORE_BUILD_TESTS=OFF -DEDUCE_CORE_BUILD_DOCS=OFF \
    -DEDUCE_CORE_BUILD_EXAMPLES=OFF
cmake --build "$workdir/libcore/build"
cmake --install "$workdir/libcore/build"
