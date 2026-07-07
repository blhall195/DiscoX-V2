#pragma once

#include <Arduino.h>

#include "drivers/buzzer.h"
#include "drivers/ldj100.h"

// ── Laser error codes (kept from the V1 LaserEgismos driver) ─────────
// Call sites in main.cpp / calibration_mode.cpp were written against this
// enum; the LDJ100 status codes are mapped onto it in laser_manager.cpp.
enum class LaserError : uint8_t {
    OK = 0,
    TIMEOUT,        // No response within timeout period
    COMMAND_FAILED, // Bad frame, module error, or unexpected response
    TOO_DIM,        // laser spot too dim / signal too weak
    TOO_BRIGHT,     // too much ambient light or too close
    BAD_READING,    // unable to measure (e.g. target out of range)
};

// ── Laser manager — V2 adapter ───────────────────────────────────────
// Presents the V1 LaserEgismos surface over the Meskernel LDJ-100RED
// (drivers/ldj100, register protocol @115200 on Serial1) plus the on-board
// piezo (drivers/buzzer). Two V1 behaviours change meaning here:
//   - Beeps came from the Egismos module's buzzer command; V2 has its own
//     4 kHz piezo, so all beeps go through the Buzzer driver instead.
//   - setBuzzer(true) was a stateful "buzzer on"; the piezo driver is a
//     blocking bit-bang, so setBuzzer(true) emits one short beep and
//     setBuzzer(false) is a no-op. The on/delay/off call sites still
//     produce the intended audible feedback.
class LaserManager {
  public:
    // serial must already be configured (setPins + RX INPUT_PULLUP + begin
    // at LASER_UART_BAUD_LDJ100) — see initLaser() in main.cpp.
    // Returns true if the module answers a status read.
    bool begin(Stream &serial, Buzzer &buzzer);

    LaserError setLaser(bool on);
    LaserError setBuzzer(bool on);
    LaserError stopMeasuring();

    void singleBeep();
    void doubleBeep();
    void failureBeep();
    void wibble();

    // Single measurement. Distance returned in mm via out param.
    LaserError measure(int32_t &distanceMm);

    LaserError lastError() const { return _lastError; }

    static const char *errorString(LaserError err);

  private:
    // Map an LDJ100 error-frame status (signed int16) to LaserError.
    static LaserError mapStatus(uint16_t status);

    LDJ100 _ldj;
    Buzzer *_buzzer = nullptr;
    LaserError _lastError = LaserError::OK;
};
