#pragma once

#include "config.h"
#include <Adafruit_SH110X.h>
#include <Arduino.h>

/// How the battery icon is drawn (same three modes as the full firmware)
enum class BatteryMode : uint8_t { NORMAL, CHARGING, FULL };

/// What the line under the distance says
enum class LaserHint : uint8_t {
    OFF,       // nothing shown
    AIMING,    // "FIRE to measure" (inverted bar: beam is on)
    MEASURING, // "Measuring..."
};

// SH1107 screens for the rangefinder. The battery icon, power prompt and
// error screen are lifted from ../PCB_V2/src/display_manager.cpp (same
// geometry), so the shared screens look the same on both firmwares.
class RfDisplay {
  public:
    bool begin();
    void setBrightness(uint8_t level);

    // ── Main screen (drawn from cached state by refresh()) ──
    void setDistance(float metres) { _distance = metres; } // 0 = none yet
    void setHint(LaserHint h) { _hint = h; }
    void setBattery(float pct, BatteryMode mode);
    void refresh();

    /// Full-panel error: size-3 headline (size 2 if over 7 chars), size-2
    /// remedy wrapped to 3 lines of 10 ('\n' forces a break). Draws and pushes.
    void showError(const char *headline, const char *detail);

    /// Centred size-2 lines, optional progress bar (0..1) and size-1 hint
    /// ('\n'-separated). Draws and pushes.
    void showPowerPrompt(const char *const *lines, uint8_t count, float progress = -1.0f,
                         const char *hint = nullptr);

    /// Charging screen: battery icon, big %, status, size-1 hint lines
    void showCharging(float pct, bool havePct, const char *status, const char *hint1, const char *hint2);

    void blank();
    void setSleep(bool sleep);

    Adafruit_SH1107 &raw() { return _display; }

  private:
    Adafruit_SH1107 _display{SH1107_WIDTH, SH1107_HEIGHT, &Wire, -1, 400000, 400000};
    bool _ok = false;
    float _distance = 0.0f;
    LaserHint _hint = LaserHint::OFF;
    float _battery = 0.0f;
    BatteryMode _batMode = BatteryMode::NORMAL;

    void drawBattery();
    void centred(const char *text, uint8_t size, int16_t y);
};
