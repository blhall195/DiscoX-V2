build:
    ./build.sh

lsp:
    cd "PCB_V2" && pio run -t compiledb

test:
    cd "PCB_V2" && pio test -e native

format:
    ./format.sh

# Uses the PlatformIO artefact, not build/v2.uf2: that one is only refreshed
# by build.sh, so it goes stale after a bare `pio run`. firmware.uf2 is
# rewritten by every compile, so it always matches the last build.
# Flash the last build to a mounted UF2 drive; waits for a board
flash:
    @test -f "PCB_V2/.pio/build/pcb_v2/firmware.uf2" || { echo "No firmware.uf2 — run 'just build' first"; exit 1; }
    python3 tools/uf2conv.py "PCB_V2/.pio/build/pcb_v2/firmware.uf2" --deploy --wait
