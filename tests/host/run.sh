#!/usr/bin/env bash
# Host-side tests for the pure va_core control-plane (no ESPHome/esp-idf
# needed). CI calls exactly this script — keep the path + exit-code contract.
set -euo pipefail
cd "$(dirname "$0")"

CXX="${CXX:-c++}"
# Build outside the tree so no artifacts land in the repo.
BUILD_DIR="${BUILD_DIR:-$(mktemp -d)}"
trap 'rm -rf "$BUILD_DIR"' EXIT
mkdir -p "$BUILD_DIR"

# vendor/ holds the amalgamated ArduinoJson single header matching the
# version the component pins in __init__.py (7.4.2); -isystem keeps vendor
# code out of our -Werror surface.
"$CXX" -std=c++17 -Wall -Wextra -Werror -isystem vendor \
  -o "$BUILD_DIR/test_va_core" test_va_core.cpp
"$CXX" -std=c++17 -O2 -Wall -Wextra -Werror -isystem vendor \
  -o "$BUILD_DIR/fuzz_va_core" fuzz_va_core.cpp
"$CXX" -std=c++17 -O2 -Wall -Wextra -Werror \
  -o "$BUILD_DIR/test_bus_arbiter" test_bus_arbiter.cpp

"$BUILD_DIR/test_va_core"
# Randomized invariant run (seeded, deterministic). FUZZ_SEEDS / FUZZ_STEPS
# scale it; a failure prints the seed and the event trace to reproduce.
"$BUILD_DIR/fuzz_va_core"
# Shared mic/speaker bus arbitration (Atom Echo S3R): example + randomized.
"$BUILD_DIR/test_bus_arbiter"
