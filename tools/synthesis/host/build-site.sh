#!/bin/bash
# Build everything the host serves: the patched Synthesis web app (site/) and the relay (glueball).
# Needs: git, git-lfs, bun (https://bun.sh), Rust/cargo (https://rustup.rs). Takes ~5 minutes.
#   ./build-site.sh            -> ./site and ./bin/glueball next to this script
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
KIT="$HERE/.."                                    # tools/synthesis
SYNTHESIS_COMMIT="9f227a2a68457ac93fab60b00b3d04146944648a"   # Autodesk/synthesis dev, 2026-09-22
WORK="$HERE/.work"

# Fetch only the pinned commit: the full Synthesis history is several gigabytes.
if [ ! -d "$WORK/synthesis/.git" ]; then
    git init -q "$WORK/synthesis"
    git -C "$WORK/synthesis" remote add origin https://github.com/Autodesk/synthesis.git
fi
cd "$WORK/synthesis"
git fetch -q --depth 1 origin "$SYNTHESIS_COMMIT"
git checkout -q -f "$SYNTHESIS_COMMIT"
git apply "$KIT/fission.patch"
mkdir -p fission/src/dev
cp "$KIT/SwerveCodeSim.ts" fission/src/dev/
cp "$KIT/urdf/out/sphinx_urdf.zip" fission/public/

# Field and robot models (the 2026 field is in Synthesis's asset pack).
git lfs pull --include fission/public/assetpack.zip
rm -rf fission/public/Downloadables && unzip -q -o fission/public/assetpack.zip -d fission/public/

(cd fission && bun install --frozen-lockfile && bunx vite build)
rm -rf "$HERE/site" && cp -R fission/dist "$HERE/site" && rm -f "$HERE/site/assetpack.zip"

(cd glueball && cargo build --release -q)
mkdir -p "$HERE/bin" && cp glueball/target/release/glueball "$HERE/bin/"

git checkout -q -f "$SYNTHESIS_COMMIT"             # leave the clone clean for the next build
echo "Built $HERE/site and $HERE/bin/glueball"
