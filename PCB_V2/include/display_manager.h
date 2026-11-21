#pragma once

#include "config.h"
#include <Adafruit_SH110X.h>
#include <Arduino.h>

/// How the battery icon is drawn (top-right, on the main and charging screens)
enum class BatteryMode : uint8_t {
    NORMAL,   // fill = charge level
    CHARGING, // charged part solid, the rest filling up in steps
    FULL,     // charge complete: solid at the level, no animation
};

class DisplayManager {
  public:
    /// Initialize the SH1107 display. Returns true on success.
    bool begin();

    /// Draw the main measurement screen from cached values
    void initScreen();

    /// Update all three reading labels at once
    void updateSensorReadings(float distance, float azimuth, float inclination);

    /// Update distance label (meters). 0 = blank.
    void updateDistance(float distance);

    /// Update distance label with arbitrary text (e.g. an error headline).
    /// `detail` is an optional second line drawn small underneath it — use it
    /// for the remedy ("Dim target: use card"). Max 7 chars show at the
    /// headline's size 3, 21 at the detail's size 1. Passing no detail clears
    /// any previous one; the pair is hidden again by any distance update.
    void updateDistanceText(const char *text, const char *detail = nullptr);

    /// Update azimuth label (degrees)
    void updateAzimuth(float azimuth);

    /// Update inclination label (degrees)
    void updateInclination(float inclination);

    /// Update battery bar (0-100%)
    void updateBattery(float percentage);

    /// Charging animation on the battery icon (redrawn at each refresh)
    void setBatteryMode(BatteryMode mode) { _batMode = mode; }
    BatteryMode batteryMode() const { return _batMode; }

    /// Charging screen: the battery icon where the main screen has it, the
    /// gauge reading to 2 d.p. (or "--.--%" without a gauge), a status line,
    /// a size-1 note between the icon and the % (charge limit / paused), and
    /// up to CHARGING_HINTS size-1 lines at the bottom (nullptr = none).
    /// Draws and pushes.
    static constexpr uint8_t CHARGING_HINTS = 3;
    void showChargingScreen(float pct, bool havePct, const char *status, const char *note,
                            const char *const *hints);

    /// Intro played as the charging screen opens: a bolt strikes, a big
    /// battery pops in and fills to pct while the % counts up, then it
    /// shrinks into the icon's corner and the status types in. Frame t (ms)
    /// is drawn and pushed; from CHARGING_INTRO_MS on it is exactly
    /// showChargingScreen() with the same arguments, so the hand-off is seamless.
    static constexpr uint16_t CHARGING_INTRO_MS = 2200;
    void showChargingIntro(uint32_t t, float pct, bool havePct, const char *status, const char *note,
                           const char *const *hints);

    /// Show/hide BT label
    void updateBTLabel(bool connected);

    /// Show pending BLE reading count (0 = hidden)
    void updateBTNumber(uint16_t pending);

    /// Update measure-from indicator (true = Front, false = Back)
    void updateMeasureFrom(bool front);

    /// Take over the whole panel with an error: a size-3 headline (max 7
    /// chars) over a size-2 remedy wrapped to at most 3 lines of 10 columns.
    /// `detail` may use '\n' to choose its own breaks. Stays up — including
    /// across refresh() — until clearErrorScreen(); the caller owns how long
    /// that is. Use instead of updateDistanceText when the readings behind
    /// the error are stale and not worth the space.
    void showErrorScreen(const char *headline, const char *detail);

    /// Return to the normal readings screen after showErrorScreen().
    void clearErrorScreen();

    /// Clear display to black
    void blankScreen();

    /// Full-screen power prompt: up to 4 centred size-2 lines, plus a
    /// progress bar underneath when progress >= 0 (0..1) and/or size-1
    /// hint text ('\n' between lines, max 21 chars each). Used for the hold-to-power-on/off bars,
    /// the standby notice and the "hold the button" notice.
    void showPowerPrompt(const char *const *lines, uint8_t count, float progress = -1.0f,
                         const char *hint = nullptr);

    /// Panel off (standby) / back on. The buffer is kept, so the next
    /// display() after waking shows whatever is drawn then.
    void setSleep(bool sleep);

    /// Show "Starting Menu" message
    void showStartingMenu();

    /// Show "Device Initialising" message
    void showInitialisingMessage();

    /// Play the boot splash animation (Splash::Style, blocking ~1.5-2.7 s),
    /// leaving its final frame — name, BLE suffix, version — on screen
    void playSplash(uint8_t style, const char *nameSuffix = nullptr);

    /// Set OLED contrast/brightness (0-255)
    void setBrightness(uint8_t level);

    /// Push buffer to OLED (call after updates)
    void refresh();

    /// Direct access to the underlying display (used by MenuManager)
    Adafruit_SH1107 &getDisplay() { return _display; }

  private:
    // Keep I2C at 400 kHz before AND after display transactions
    Adafruit_SH1107 _display{SH1107_WIDTH, SH1107_HEIGHT, &Wire, -1, 400000, 400000};
    bool _initialized = false;

    // Cached display state (set by updateXxx, drawn by refresh)
    float _distance = 0.0f;
    char _distText[16] = "";
    char _distDetail[24] = "";
    bool _distIsText = false;
    bool _errorScreen = false;
    float _azimuth = 0.0f;
    float _inclination = 0.0f;
    float _battery = 0.0f;
    BatteryMode _batMode = BatteryMode::NORMAL;
    bool _btConnected = false;
    uint16_t _btPending = 0;
    bool _measureFromFront = false;

    void drawMainScreen();
    void drawErrorScreen();
    void drawBattery(float pct);
    void drawDegreeSymbol(int16_t y);
    // Charging screen text; statusChars limits how much of the status is
    // drawn (the intro types it in), hints only when showHints
    void drawChargingText(float pct, bool havePct, const char *status, size_t statusChars,
                          const char *note, const char *const *hints, bool showHints);
    // Battery outline + fill at any size (the intro's big battery and its
    // flight into the corner). wave = amplitude of the fill's wavy edge, px.
    void drawBatteryAt(float x, float y, float w, float h, float frac, float wave, uint32_t t);
};
