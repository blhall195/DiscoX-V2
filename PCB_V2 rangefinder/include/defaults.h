#pragma once

#include <stdint.h>

// Values shared with ../PCB_V2/include/defaults.h carry the same names and
// (apart from laserSpreadLimitMm) numbers, so the reused files (laser_manager, button_manager) build
// unchanged and a shot here is validated exactly as a survey shot is.
// Only what this firmware uses is kept.
namespace Defaults {
// Laser shot validation — see ../PCB_V2/include/defaults.h for the tuning
// history (SQ is HIGHER = stronger, contrary to the module's manual).
constexpr uint8_t laserShotsMax = 5;
constexpr uint8_t laserShots = 3;
constexpr uint16_t laserSqLimit = 20;
// 0 = no agreement check between the shots (removed 2026-11-27 at
// Brendan's request: no SPREAD error). The median of the three is still
// what's reported, and the SQ floor above still rejects weak shots. The
// full firmware keeps 25 mm.
constexpr uint16_t laserSpreadLimitMm = 0;
constexpr uint32_t laserMinDistanceMm = 30;
constexpr uint32_t laserMaxDistanceMm = 100000;

constexpr float laserDistanceOffset = 0.12f; // m added when measuring from the back
constexpr float laserFrontOffset = 0.02f;    // m subtracted when measuring from the front
constexpr bool measureFromFront = false;

// User settings (config.json keys auto_shutdown_timeout / laser_timeout /
// screen_brightness / measure_from_front, shared with the full firmware)
constexpr uint32_t autoShutdownTimeout = 1800; // s (30 min)
constexpr uint32_t laserTimeout = 120;         // s (2 min)
constexpr uint8_t screenBrightness = 255;
} // namespace Defaults

namespace Timing {
constexpr uint32_t BUTTON_DEBOUNCE_MS = 10;
constexpr uint32_t BATTERY_CHECK_MS = 30000;
constexpr uint32_t AUTO_SHUTOFF_CHECK_MS = 5000;
constexpr uint32_t LASER_TIMEOUT_CHECK_MS = 1000;
constexpr uint32_t DISPLAY_REFRESH_MS = 250;
constexpr uint32_t LOOP_INTERVAL_MS = 10;
constexpr uint32_t POWER_HOLD_OFF_MS = 1500;  // hold this long (bar) to power off
constexpr uint32_t POWER_BAR_DELAY_MS = 250;  // bar only once a press is clearly a hold
constexpr uint32_t POWER_HOLD_ON_MS = 1000;   // cold boot: keep holding this long
constexpr uint32_t POWER_TAP_NOTICE_MS = 2000;
constexpr uint32_t ERROR_SCREEN_MS = 2500;    // laser error screen before the readings return
constexpr float BATTERY_SHUTDOWN_PCT = 5.0f;
} // namespace Timing
