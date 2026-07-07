build:
    ./build.sh

lsp:
    cd "PCB_V2" && pio run -t compiledb

test:
    cd "PCB_V2" && pio test -e native

format:
    ./format.sh
