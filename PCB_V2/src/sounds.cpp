#include "sounds.h"

#include "drivers/buzzer.h"

// Tuning notes for experimentation:
//   - buzzer->tone(freqHz, ms)          fixed-pitch note
//   - buzzer->sweep(fromHz, toHz, ms)   glissando (rising = happy, falling = sad)
//   - delay(ms)                         gap between notes
// Loudness is set by distance from the element's 4 kHz resonance, not by
// duty cycle — there is no volume control. Keep frequencies roughly within
// 1–5 kHz; the element is nearly inaudible far outside that band.

namespace Sounds {

static Buzzer *buzzer = nullptr;

void begin(Buzzer &b) { buzzer = &b; }

void shotStart() {
    if (!buzzer) return;
    buzzer->tone(4000, 20);
}

void click() {
    if (!buzzer) return;
    buzzer->tone(4000, 40);
}

void readingOk() {
    if (!buzzer) return;
    // The headline "reading taken" bleep. Deliberately BELOW the 4 kHz
    // resonance: dead on the peak, bit-bang timing jitter modulates the
    // amplitude and the tone warbles ("chirpy"). 3.5 kHz sits on the flatter
    // shoulder of the response — slightly quieter but a clean steady bleep.
    buzzer->tone(3500, 250);
}

void legComplete() {
    if (!buzzer) return;
    // Rising major-ish fanfare that lands on resonance: unmistakably "done".
    buzzer->tone(2400, 90);
    delay(40);
    buzzer->tone(3200, 90);
    delay(40);
    buzzer->sweep(3600, 4000, 60);
    buzzer->tone(4000, 220);
}

void warning() {
    if (!buzzer) return;
    buzzer->tone(2200, 60);
    delay(70);
    buzzer->tone(2200, 60);
}

void error() {
    if (!buzzer) return;
    // Sad trombone-ish double fall — clearly distinct from every success sound.
    buzzer->sweep(3000, 2200, 150);
    delay(60);
    buzzer->sweep(2200, 1300, 280);
}

// ── Snake game ──────────────────────────────────────────────────────

void snakeStart() {
    if (!buzzer) return;
    // Quick rising "get ready" chirp landing on resonance.
    buzzer->tone(2000, 45);
    buzzer->tone(2800, 45);
    buzzer->sweep(3200, 4000, 80);
}

void snakeEat(uint16_t score) {
    if (!buzzer) return;
    // Two-note pickup blip. Pitch climbs with the score toward resonance,
    // so eating gets louder and shriller as the snake grows. ~70 ms total —
    // must stay well under the 200 ms move tick.
    uint32_t base = 2200 + 120u * (score > 10 ? 10 : score); // caps at 3400
    buzzer->tone(base, 25);
    buzzer->tone(base + 500, 45);
}

void snakeCrash(uint8_t n) {
    if (!buzzer) return;
    // One falling womp per red game-over flash, each lower than the last.
    uint32_t top = 2600 - 400u * (n > 3 ? 3 : n);
    buzzer->sweep(top, top / 2, 220);
}

} // namespace Sounds
