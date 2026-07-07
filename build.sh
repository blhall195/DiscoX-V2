#!/usr/bin/env bash
set -euo pipefail

REPO="$(git rev-parse --show-toplevel)"
V2_DIR="$REPO/PCB_V2"
UF2CONV="$REPO/tools/uf2conv.py"
BUILD="$REPO/build"

echo "=== Building PCB V2 ==="
pio run --project-dir "$V2_DIR" -e pcb_v2
mkdir -p "$BUILD"
python3 "$UF2CONV" \
    "$V2_DIR/.pio/build/pcb_v2/firmware.hex" \
    --family 0xada52840 --convert \
    --output "$BUILD/v2.uf2"
echo "-> $BUILD/v2.uf2"
