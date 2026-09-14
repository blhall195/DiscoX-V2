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
    elf_path = os.path.join(build_dir, "firmware.elf")
    uf2_path = os.path.join(build_dir, "firmware.uf2")
    # Our own intermediate, NOT the build's firmware.hex — see the note below
    # on why that one cannot be trusted here.
    tmp_hex = os.path.join(build_dir, "firmware.uf2.hex")

    framework = env.PioPlatform().get_package_dir("framework-arduinoadafruitnrf52")
    if not framework:
        print("make_uf2: adafruit nrf52 framework package not found — skipping")
        return
    tool = os.path.join(framework, "tools", "uf2conv", "uf2conv.py")
    if not os.path.isfile(tool):
        print("make_uf2: uf2conv.py not found — skipping")
        return
    if not os.path.isfile(elf_path):
        print("make_uf2: firmware.elf missing — skipping")
        return

    # Derive the hex from the ELF that was just linked. SCons builds
    # firmware.hex AFTER checkprogsize runs, so reading that file here either
    # finds it missing (fresh build -> no .uf2 at all) or one build stale
    # (incremental -> a .uf2 of the PREVIOUS firmware). Both shipped the wrong
    # image; going straight from the ELF removes the ordering assumption.
    objcopy = env.subst("$OBJCOPY") or "arm-none-eabi-objcopy"
    result = subprocess.run(
        [objcopy, "-O", "ihex", elf_path, tmp_hex], capture_output=True, text=True
    )
    if result.returncode != 0:
        print("make_uf2: objcopy FAILED\n" + result.stdout + result.stderr)
        return

    result = subprocess.run(
        [sys.executable, tool, "-f", UF2_FAMILY, "-c", "-o", uf2_path, tmp_hex],
        capture_output=True,
        text=True,
    )
    try:
        os.remove(tmp_hex)
    except OSError:
        pass
    if result.returncode != 0:
        print("make_uf2: FAILED\n" + result.stdout + result.stderr)
        return
    print("UF2 image: %s (%.1f KB)" % (uf2_path, os.path.getsize(uf2_path) / 1024.0))


# Hook "checkprogsize" rather than the firmware.hex node: SCons skips a
# post-action on an up-to-date target, which would leave a stale or missing
# .uf2 behind on an incremental build. checkprogsize runs on every build, so
# the image is always regenerated — but it runs BEFORE firmware.hex is
# written, which is why make_uf2 converts from firmware.elf via its own
# objcopy rather than reading firmware.hex.
env.AddPostAction("checkprogsize", make_uf2)  # noqa: F821
