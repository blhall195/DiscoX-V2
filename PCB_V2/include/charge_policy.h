#pragma once

#include <stdint.h>

// ── Charge policy (pure logic, unit-tested in test/test_charge_policy) ──
// Decides, from the charger pins and the fuel gauge, what state the charge
// is in and whether the firmware should hold charging off to honour the
// user's charge limit (config.json "charge_limit": 80, 90 or 100).
//
// Hardware (see "Charging" in CLAUDE.md): boards from the 2026-10-18 PCB
// revision wire the BQ24074's CHG (P0.13), PGOOD (P0.15) and CE (P0.17) to
// the nRF, with R23 holding CE low (charging on) whenever the nRF is not
// driving it. Older boards wire none of them: they always charge to the
// charger's own 4.2 V and "complete" can only be guessed from the gauge.
//
// 100% leaves CE alone: the BQ24074 charges to its fixed 4.2 V and stops by
// itself when the current tapers off. 80% / 90% stop at that gauge reading
// (or at kCapVolts, whichever comes first) and resume kResumeGap points
// below it. This only works while the firmware runs; with the device fully
// off, R23 turns charging on and the cell charges to 4.2 V.
namespace ChargePolicy {

enum class State : uint8_t {
    UNPLUGGED, // no USB
    CHARGING,  // charging current flowing
    PAUSED,    // USB present, below the limit, charger idle and not by our
               // choice: the thermistor (board < 0 or > ~51 C) or the
               // safety timer
    COMPLETE,  // limit reached (firmware hold) or the charger's own finish
};

/// LED colour for a state (DiscoManager draws it)
enum class Led : uint8_t { NONE, RED, ORANGE, GREEN, ORANGE_PULSE };

constexpr uint8_t kLimitDefault = 100;
constexpr float kResumeGap = 5.0f;     // resume this many points below the limit
constexpr float kCapVolts = 4.10f;     // 80/90%: never charge past this, whatever the gauge says
constexpr float kResumeMaxVolts = 4.05f; // ...and don't resume above this
constexpr float kOrangeFrom = 80.0f;   // LED: red below this, orange from here until complete
constexpr float kFullVolts = 4.05f;    // charger idle at or above this = finished, not paused
constexpr float kFullSoc = 98.0f;      // ...likewise for the gauge

struct Inputs {
    bool usb = false;        // VBUS present
    bool hasControl = false; // CHG/CE wired to the MCU (new boards)
    bool chgLow = false;     // CHG pin LOW = charger is charging (new boards only)
    bool gaugeOk = false;    // soc/volts are real readings
    float soc = 0.0f;        // MAX17048 state of charge, % (can read > 100)
    float volts = 0.0f;      // MAX17048 cell voltage
    uint8_t limit = kLimitDefault;
};

/// Persists between steps (one per device)
struct Memory {
    bool held = false; // firmware is holding charging off (limit reached)
};

struct Output {
    State state = State::UNPLUGGED;
    bool disableCharging = false; // drive CE high
};

/// 80, 90 or 100; anything else becomes 100 (a hand-edited config.json)
uint8_t sanitiseLimit(int limit);

/// One decision. Call every few seconds while on USB and on every plug/unplug.
Output step(Memory &mem, const Inputs &in);

/// LED for a state at a given charge
Led ledFor(State state, float soc);

/// Gauge reading for display: clamped to 0..100 (the gauge can read > 100%)
float displayPercent(float soc);

} // namespace ChargePolicy
