# Post-build: emit firmware.uf2 alongside firmware.hex.
#
# UF2 is the end-user update path (double-tap reset -> drag onto the
# bootloader drive). The nrfutil DFU upload PlatformIO uses for development
# doesn't need it, so nothing else in the build produces one.
#
# The image covers only the application region (0x26000-0xCD000 per
# include/flash_layout.h), so flashing it preserves the FAT settings
# partition and the LittleFS store holding config.json.

import os
import subprocess
import sys

Import("env")  # noqa: F821 — injected by PlatformIO

UF2_FAMILY = "0xADA52840"  # nRF52840


def make_uf2(source, target, env):
    build_dir = env.subst("$BUILD_DIR")
    hex_path = os.path.join(build_dir, "firmware.hex")
    uf2_path = os.path.join(build_dir, "firmware.uf2")

    framework = env.PioPlatform().get_package_dir("framework-arduinoadafruitnrf52")
    if not framework:
        print("make_uf2: adafruit nrf52 framework package not found — skipping")
        return
    tool = os.path.join(framework, "tools", "uf2conv", "uf2conv.py")
    if not os.path.isfile(tool) or not os.path.isfile(hex_path):
        print("make_uf2: uf2conv.py or firmware.hex missing — skipping")
        return

    result = subprocess.run(
        [sys.executable, tool, "-f", UF2_FAMILY, "-c", "-o", uf2_path, hex_path],
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        print("make_uf2: FAILED\n" + result.stdout + result.stderr)
        return
    print("UF2 image: %s (%.1f KB)" % (uf2_path, os.path.getsize(uf2_path) / 1024.0))


# Hook "checkprogsize" rather than the firmware.hex node: SCons skips a
# post-action on an up-to-date target, which would leave a stale or missing
# .uf2 behind on an incremental build. checkprogsize runs at the end of
# every build, so the image is always regenerated.
env.AddPostAction("checkprogsize", make_uf2)  # noqa: F821
