#pragma once

#include <stdint.h>

// User settings, kept in the full firmware's own /config.json on LittleFS.
//
// This firmware shares the device with ../PCB_V2 (a user can go back and
// forth by UF2), so it is a polite guest in that file:
//   - it reads and writes only the keys below, under their full-firmware
//     names and units, so a timeout set here is still set after going back;
//   - a save is read-modify-write: every other key in the file (BLE name,
//     splays, charge limit, ...) is carried over untouched;
//   - it never formats the filesystem. If LittleFS won't mount, the device
//     runs on defaults and menu changes are not saved — the calibration and
//     bonds in there are the full firmware's to repair, not ours to wipe.
struct Settings {
    uint32_t autoShutdownTimeout; // s — "auto_shutdown_timeout"
    uint32_t laserTimeout;        // s — "laser_timeout"
    uint8_t screenBrightness;     // 0-255 — "screen_brightness" (read only)
    bool measureFromFront;        // "measure_from_front" (read only)
};

namespace SettingsStore {

/// Mount LittleFS (never formats). False = no storage this session.
bool begin();
bool available();

/// Defaults, overlaid with whatever /config.json holds. Out-of-range
/// values fall back to the default.
void load(Settings &s);

/// Write the two timeouts back into /config.json, keeping every other key.
bool save(const Settings &s);

} // namespace SettingsStore
