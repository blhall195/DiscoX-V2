#pragma once

#include <stdint.h>

namespace Defaults {
constexpr float magTolerance = 10.0f;  // degrees
constexpr float gravTolerance = 10.0f; // degrees
constexpr float dipTolerance = 10.0f;  // degrees
constexpr bool anomalyDetection = true;
constexpr float stabilityTolerance = 0.5f;    // degrees
constexpr float quickShotStabilityTol = 8.0f; // degrees (wider for splay shots)
constexpr uint8_t stabilityBufferLength = 5;
constexpr float emaAlphaStable = 0.15f;         // stationary alpha — lightened for the low-noise
                                                // SCA3300/RM3100 (V1's noisy IMU needed 0.05)
constexpr float emaAlphaMoving = 0.6f;          // high alpha when moving (responsive)
constexpr float emaJumpThreshold = 8.0f;        // degrees — snap EMA when error exceeds this
constexpr float legAngleTolerance = 1.7f;       // degrees
constexpr float cartesianTolerance = 10.0f;     // cm (BCRA grade 5)
constexpr float cartesianToleranceMin = 1.0f;   // cm
constexpr float cartesianToleranceMax = 200.0f; // cm
constexpr float laserDistanceOffset = 0.162f;   // meters added when measuring from back
constexpr float laserFrontOffset = 0.03f;       // meters subtracted when measuring from front
constexpr float calMagConsistency = 0.5f;       // degrees (angular consistency window for mag)
constexpr float calGravConsistency = 0.4f;      // degrees (angular consistency window for gravity)
constexpr uint8_t calBufferLength = 5;          // samples for calibration consistency window
constexpr uint16_t calSettleMs = 250;           // ms device must stay stable before accepting
constexpr float calEmaAlpha = 0.3f;             // EMA pre-filter (lower = smoother, 0.3 = ~3 sample lag)
constexpr uint16_t calTimeoutMs = 4000;         // ms max wait for stability before forcing acceptance
constexpr uint32_t autoShutdownTimeout = 1800;  // seconds (30 min)
constexpr uint32_t laserTimeout = 120;          // seconds (2 min)
constexpr bool laserWibble = true;              // blink laser on leg detect
// Laser shot validation (dark/specular targets return plausible-but-wrong
// distances — see discox-sq-rejection-brief.md). A measurement is N low-speed
// shots; a shot survives if its frame validates and its signal quality (SQ,
// HIGHER = stronger — the manual claims the inverse but bench testing
// 2026-09-06 disproved it) is at least laserSqLimit. Median of survivors is
// returned only if enough survive and they agree within laserSpreadLimitMm.
constexpr uint8_t laserShotsMax = 5;  // buffer size / settings clamp
constexpr uint8_t laserShots = 1;     // default: single shot, SQ-gated — fast
constexpr uint16_t laserSqLimit = 0;      // reject shots with SQ BELOW this; 0 = gate disabled
constexpr uint16_t laserSpreadLimitMm = 25;
constexpr uint32_t laserMinDistanceMm = 30;     // module rated minimum
constexpr uint32_t laserMaxDistanceMm = 100000; // rated max at reflectivity 1.0
constexpr bool measureFromFront = false;        // false = Back (add offset), true = Front (raw laser)
constexpr bool splaysEnabled = false;           // enable splay shots on button 2 short press
constexpr uint8_t screenBrightness = 255;       // OLED contrast 0-255
constexpr char bleName[] = "SAP6_DiscoX";
constexpr uint8_t bleNameMaxLen = 20; // max chars for BLE name
} // namespace Defaults

namespace Timing {
constexpr uint32_t BUTTON_DEBOUNCE_MS = 10;
constexpr uint32_t SENSOR_POLL_MS = 10;          // ~90 Hz (matches RM3100 CC300 rate)
constexpr uint32_t SENSOR_MEASURE_POLL_MS = 100; // 10 Hz measuring
constexpr uint32_t BUTTON_POLL_MS = 50;
constexpr uint32_t BATTERY_CHECK_MS = 30000; // 30 sec
constexpr uint32_t BLE_PIN_CHECK_MS = 300;
constexpr uint32_t BLE_UART_POLL_MS = 10;
constexpr uint32_t AUTO_SHUTOFF_CHECK_MS = 5000;
constexpr uint32_t LASER_TIMEOUT_CHECK_MS = 1000;
constexpr uint32_t DISPLAY_REFRESH_MS = 250; // 4 Hz display updates
constexpr uint32_t LOOP_INTERVAL_MS = 10;    // main loop pace
constexpr float BATTERY_SHUTDOWN_PCT = 5.0f;
constexpr uint32_t CALIB_HOLD_MS = 1000;    // 1s hold for menu
constexpr uint32_t DISCO_HOLD_MS = 1500;    // 2s hold for disco toggle
constexpr uint32_t SNAKE_HOLD_MS = 5000;    // 5s hold for snake
constexpr uint32_t MEASURE_GUARD_MS = 1500; // post-measure lockout
} // namespace Timing

namespace Disco {
constexpr float BASE_BRIGHTNESS = 0.3f;
constexpr float WILD_BRIGHTNESS = 1.0f;
constexpr float SHAKE_THRESHOLD = 11.0f; // m/s²
constexpr uint32_t SHAKE_TIMEOUT_MS = 5000;
constexpr uint32_t BASE_SLEEP_MS = 200;
constexpr uint32_t WILD_SLEEP_MS = 100;
constexpr uint8_t COLOR_STEP_BASE = 20; // hue increment per frame
} // namespace Disco
