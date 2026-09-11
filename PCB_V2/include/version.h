#pragma once

// ── Firmware version ─────────────────────────────────────────────────
// Shown on the boot splash under the device name, and printed once over
// serial at start-up so a board's build can be identified without opening
// the case. This is the FIRMWARE version and is deliberately independent of
// the PCB revision — bump it here, in this one place, on every release.
//
// Keep it short: the splash line is size 1 (6 px per character) centred on a
// 128 px screen, so anything past 21 characters is clipped.
constexpr char FIRMWARE_VERSION[] = "v2.0.0";
