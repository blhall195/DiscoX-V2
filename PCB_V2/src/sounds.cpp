#include "sounds.h"

#include "drivers/buzzer.h"

// Tuning notes for experimentation:
//   - buzzer->tone(freqHz, ms)          fixed-pitch note
//   - buzzer->sweep(fromHz, toHz, ms)   glissando (rising = happy, falling =
//   sad)
//   - delay(ms)                         gap between notes
// Loudness is set by distance from the element's 4 kHz resonance, not by
// duty cycle — there is no volume control. Keep frequencies roughly within
// 1–5 kHz; the element is nearly inaudible far outside that band.

namespace Sounds {

static Buzzer *buzzer = nullptr;

void begin(Buzzer &b) { buzzer = &b; }

void shotStart() {
    if (!buzzer) {
        return;
    }
    buzzer->tone(4000, 20);
}

void click() {
    if (!buzzer) {
        return;
    }
    buzzer->tone(4000, 40);
}

void readingOk() {
    if (!buzzer) {
        return;
    }
    // The headline "reading taken" bleep. Deliberately BELOW the 4 kHz
    // resonance: dead on the peak, bit-bang timing jitter modulates the
    // amplitude and the tone warbles ("chirpy"). 3.5 kHz sits on the flatter
    // shoulder of the response — slightly quieter but a clean steady bleep.
    buzzer->tone(3500, 250);
}

void legComplete() {
    if (!buzzer) {
        return;
    }
    // Rising major-ish fanfare that lands on resonance: unmistakably "done".
    buzzer->tone(2400, 90);
    delay(40);
    buzzer->tone(3200, 90);
    delay(40);
    buzzer->sweep(3600, 4000, 60);
    buzzer->tone(4000, 220);
}

void warning() {
    if (!buzzer) {
        return;
    }
    buzzer->tone(2200, 60);
    delay(70);
    buzzer->tone(2200, 60);
}

void error() {
    if (!buzzer) {
        return;
    }
    // Sad trombone-ish double fall — clearly distinct from every success sound.
    buzzer->sweep(3000, 2200, 150);
    delay(60);
    buzzer->sweep(2200, 1300, 280);
}

// ── Snake game ──────────────────────────────────────────────────────

void snakeStart() {
    if (!buzzer) {
        return;
    }
    // Quick rising "get ready" chirp landing on resonance.
    buzzer->tone(2000, 45);
    buzzer->tone(2800, 45);
    buzzer->sweep(3200, 4000, 80);
}

void snakeEat(uint16_t score) {
    if (!buzzer) {
        return;
    }
    // Two-note pickup blip. Pitch climbs with the score toward resonance,
    // so eating gets louder and shriller as the snake grows. ~70 ms total —
    // must stay well under the 200 ms move tick.
    uint32_t base = 2200 + 120u * (score > 10 ? 10 : score); // caps at 3400
    buzzer->tone(base, 25);
    buzzer->tone(base + 500, 45);
}

void snakeCrash(uint8_t n) {
    if (!buzzer) {
        return;
    }
    // One falling womp per red game-over flash, each lower than the last.
    uint32_t top = 2600 - 400u * (n > 3 ? 3 : n);
    buzzer->sweep(top, top / 2, 220);
}


// ── Mario theme (disco easter egg) ──────────────────────────────────
// Transposed up two octaves from concert pitch. The tune sits around
// 300-800 Hz where it is normally played, and this element is "nearly
// inaudible" that far below its 4 kHz resonance (see the header note) — at
// written pitch it is a barely-there buzz. Shifted to 1.3-3.5 kHz it lands
// in the band the piezo actually reproduces, which is why the numbers below
// look high for Mario.
namespace {

struct Note {
    uint16_t freq; // Hz, 0 = rest
    uint16_t ms;   // total slot length including the articulation gap
};

// Note names are the sounded (transposed) octave. Everything lands between
// 1.3 and 4.2 kHz, i.e. either side of the element's resonance.
constexpr uint16_t E6 = 1319, GS6 = 1661, A6 = 1760, AS6 = 1865, B6 = 1976;
constexpr uint16_t G6 = 1568;
constexpr uint16_t C7 = 2093, D7 = 2349, DS7 = 2489, E7 = 2637, F7 = 2794;
constexpr uint16_t FS7 = 2960, G7 = 3136, A7 = 3520;
constexpr uint16_t C8 = 4186;
constexpr uint16_t R = 0;

// The full overworld theme: intro, main phrase twice, bridge twice, closing
// — about 30 s, then it loops. Rests are their own entries so the written
// note lengths stay readable.
constexpr Note kMario[] = {
    // ── Intro ────────────────────────────────────────────────────────
    {E7, 150},  {E7, 150},  {R, 150},   {E7, 150},  {R, 150},   {C7, 150},
    {E7, 150},  {R, 150},   {G7, 150},  {R, 450},   {G6, 150},  {R, 450},

    // ── Main phrase ──────────────────────────────────────────────────
    {C7, 150},  {R, 300},   {G6, 150},  {R, 300},   {E6, 150},  {R, 300},
    {A6, 150},  {R, 150},   {B6, 150},  {R, 150},   {AS6, 150}, {A6, 150},
    {R, 150},   {G6, 200},  {E7, 200},  {G7, 200},  {A7, 150},  {R, 150},
    {F7, 150},  {G7, 150},  {R, 150},   {E7, 150},  {R, 150},   {C7, 150},
    {D7, 150},  {B6, 150},  {R, 450},

    // ── Main phrase again ────────────────────────────────────────────
    {C7, 150},  {R, 300},   {G6, 150},  {R, 300},   {E6, 150},  {R, 300},
    {A6, 150},  {R, 150},   {B6, 150},  {R, 150},   {AS6, 150}, {A6, 150},
    {R, 150},   {G6, 200},  {E7, 200},  {G7, 200},  {A7, 150},  {R, 150},
    {F7, 150},  {G7, 150},  {R, 150},   {E7, 150},  {R, 150},   {C7, 150},
    {D7, 150},  {B6, 150},  {R, 450},

    // ── Bridge ───────────────────────────────────────────────────────
    {R, 150},   {G7, 150},  {FS7, 150}, {F7, 150},  {DS7, 150}, {R, 150},
    {E7, 150},  {R, 150},   {GS6, 150}, {A6, 150},  {C7, 150},  {R, 150},
    {A6, 150},  {C7, 150},  {D7, 150},  {R, 300},
    {G7, 150},  {FS7, 150}, {F7, 150},  {DS7, 150}, {R, 150},   {E7, 150},
    {R, 150},   {C8, 150},  {R, 150},   {C8, 150},  {C8, 150},  {R, 450},
    {R, 150},   {G7, 150},  {FS7, 150}, {F7, 150},  {DS7, 150}, {R, 150},
    {E7, 150},  {R, 150},   {GS6, 150}, {A6, 150},  {C7, 150},  {R, 150},
    {A6, 150},  {C7, 150},  {D7, 150},  {R, 300},
    {DS7, 150}, {R, 300},   {D7, 150},  {R, 300},   {C7, 150},  {R, 600},

    // ── Bridge again ─────────────────────────────────────────────────
    {R, 150},   {G7, 150},  {FS7, 150}, {F7, 150},  {DS7, 150}, {R, 150},
    {E7, 150},  {R, 150},   {GS6, 150}, {A6, 150},  {C7, 150},  {R, 150},
    {A6, 150},  {C7, 150},  {D7, 150},  {R, 300},
    {G7, 150},  {FS7, 150}, {F7, 150},  {DS7, 150}, {R, 150},   {E7, 150},
    {R, 150},   {C8, 150},  {R, 150},   {C8, 150},  {C8, 150},  {R, 450},
    {R, 150},   {G7, 150},  {FS7, 150}, {F7, 150},  {DS7, 150}, {R, 150},
    {E7, 150},  {R, 150},   {GS6, 150}, {A6, 150},  {C7, 150},  {R, 150},
    {A6, 150},  {C7, 150},  {D7, 150},  {R, 300},
    {DS7, 150}, {R, 300},   {D7, 150},  {R, 300},   {C7, 150},  {R, 600},

    // ── Closing ──────────────────────────────────────────────────────
    {C7, 150},  {C7, 150},  {R, 150},   {C7, 150},  {R, 150},   {C7, 150},
    {D7, 150},  {R, 150},   {E7, 150},  {C7, 150},  {R, 150},   {A6, 150},
    {G6, 150},  {R, 450},
    {C7, 150},  {C7, 150},  {R, 150},   {C7, 150},  {R, 150},   {C7, 150},
    {D7, 150},  {E7, 150},  {R, 600},
};
constexpr uint16_t kMarioCount = sizeof(kMario) / sizeof(kMario[0]);

// Silence at the tail of every note so repeated same-pitch notes (the
// opening E-E-E) are heard as separate notes rather than one long tone.
constexpr uint16_t kGapMs = 30;

const Note *melody = nullptr;
uint16_t melodyCount = 0;
uint16_t melodyIndex = 0;
uint32_t noteStart = 0;
bool melodyOn = false;
bool inGap = false;

void beginNote() {
    if (!buzzer) {
        return;
    }
    const Note &n = melody[melodyIndex];
    inGap = false;
    noteStart = millis();
    if (n.freq == 0) {
        buzzer->stopTone();
    } else {
        buzzer->startTone(n.freq);
    }
}

} // namespace

void startMelody() {
    if (!buzzer) {
        return;
    }
    melody = kMario;
    melodyCount = kMarioCount;
    melodyIndex = 0;
    melodyOn = true;
    beginNote();
}

void stopMelody() {
    if (!melodyOn) {
        return;
    }
    melodyOn = false;
    inGap = false;
    if (buzzer) {
        buzzer->stopTone();
    }
}

bool melodyPlaying() { return melodyOn; }

void updateMelody() {
    if (!melodyOn || !buzzer) {
        return;
    }
    const Note &n = melody[melodyIndex];
    uint32_t elapsed = millis() - noteStart;

    // Two phases per slot: the note sounds, then a short silence.
    if (!inGap) {
        uint16_t sounding = (n.ms > kGapMs) ? (uint16_t)(n.ms - kGapMs) : n.ms;
        if (elapsed < sounding) {
            return;
        }
        buzzer->stopTone();
        inGap = true;
        return;
    }

    if (elapsed < n.ms) {
        return;
    }

    melodyIndex++;
    if (melodyIndex >= melodyCount) {
        melodyIndex = 0; // loop for as long as the disco is on
    }
    beginNote();
}

} // namespace Sounds
