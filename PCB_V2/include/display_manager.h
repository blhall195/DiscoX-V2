#pragma once

#include "config.h"
#include <Adafruit_SH110X.h>
#include <Arduino.h>

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

    /// Show "Starting Menu" message
    void showStartingMenu();

    /// Show "Device Initialising" message
    void showInitialisingMessage();

    /// Show boot splash with laser beam on or off
    /// nameSuffix: if non-null, shown instead of "Initialising..."
    void showSplash(bool laserOn, const char *nameSuffix = nullptr);

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
    bool _btConnected = false;
    uint16_t _btPending = 0;
    bool _measureFromFront = false;

    void drawMainScreen();
    void drawErrorScreen();
    void drawBattery(float pct);
    void drawDegreeSymbol(int16_t y);
};
