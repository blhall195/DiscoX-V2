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

# Flash-on-build, as V1's SAMD path had: uf2conv auto-deploys to a mounted
# UF2 bootloader drive, but only in non---convert mode (hex input needs
# --convert), so deploy explicitly. No drive mounted (e.g. CI) -> skip.
if [ -n "$(python3 "$UF2CONV" --list)" ]; then
    echo "=== Flashing (UF2 drive) ==="
    python3 "$UF2CONV" "$BUILD/v2.uf2" --deploy
else
    echo "(no UF2 drive mounted — skipped flash)"
fi
