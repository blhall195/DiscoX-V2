#pragma once

#include <stdint.h>

// ── UI sound vocabulary ──────────────────────────────────────────────
// Every noise the device makes is defined in src/sounds.cpp — one blocking
// function per event, so tweaking a tune means editing exactly one place.
// All sounds are blocking (the buzzer driver is a bit-bang); the longest
// (legComplete) is ~0.6 s.
//
// The piezo is a 4 kHz resonant element: it is LOUD at 4 kHz and gets
// quieter the further a note sits from resonance. Success sounds therefore
// end at/near 4 kHz; sad sounds fall away from it.

class Buzzer;

namespace Sounds {

// Must be called once (after Buzzer::begin) before any sound plays.
// Sounds are silently skipped until then.
void begin(Buzzer &buzzer);

void shotStart(); // measurement started / laser fired: short crisp click
void click();     // generic short confirmation blip (calibration points, etc.)
void readingOk(); // reading captured: single LOUD bleep at resonance
void legComplete(); // 3 consistent shots: rising three-note fanfare
void warning();     // soft refusal (e.g. splays disabled): two mid blips
void error();       // measurement/system error: sad falling womp

// ── Snake game ──
void snakeStart(); // game begins: quick rising "ready" chirp
void snakeEat(
    uint16_t score); // food eaten: pickup blip that climbs with the score
void snakeCrash(
    uint8_t n); // game over: falling womp per red flash (n = 0,1,2…)

} // namespace Sounds
