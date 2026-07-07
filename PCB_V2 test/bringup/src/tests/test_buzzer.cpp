// PCB V2 bring-up test: piezo buzzer (BUZZ1 P0.20 / BUZZ2 P0.24, antiphase)
//
// Output-only device — nothing to read back, so like the WS2812 test this is a
// "confirm by ear" test. The boot self-test only proves the driver initialises
// the two GPIOs (silence means both pins idle LOW, no DC across the element).
// After the report it plays an audible sequence: a frequency sweep that should
// peak in loudness at ~4 kHz (the element's resonance), then a short beep
// pattern. No sound at all points at the BUZZ nets, the ENA-gated rail, or a
// dead element; loudest away from 4 kHz means the wrong resonant part is fitted.
//
// Pins (pins_v2.h): BUZZ1 P0.20, BUZZ2 P0.24 — driven in antiphase for max
// voltage swing across the element.

#include <Arduino.h>

#include "drivers/buzzer.h"
#include "pins_v2.h"

static Buzzer buzzer;

static int passCount = 0;
static int failCount = 0;

static void report(const char *name, bool ok, const char *detail) {
    if (ok) {
        passCount++;
    } else {
        failCount++;
    }
    Serial.print(ok ? "[PASS] " : "[FAIL] ");
    Serial.print(name);
    if (detail && detail[0]) {
        Serial.print(" — ");
        Serial.print(detail);
    }
    Serial.println();
}

static void runTests() {
    char detail[96];

    // 1. Driver init — pins become outputs and idle LOW (silent). digitalRead of
    //    an output pin reads back the driven level on the nRF52, so this also
    //    confirms both pins actually latched LOW.
    buzzer.begin(PIN_BUZZER_A, PIN_BUZZER_B);
    bool idleLow = (digitalRead(PIN_BUZZER_A) == LOW) && (digitalRead(PIN_BUZZER_B) == LOW);
    snprintf(detail, sizeof(detail), "BUZZ1 P0.20 / BUZZ2 P0.24 idle %s",
             idleLow ? "LOW (silent)" : "not LOW (stuck driven?)");
    report("driver init / idle silent", idleLow, detail);

    Serial.println();
    Serial.print("Result: ");
    Serial.print(passCount);
    Serial.print(" passed, ");
    Serial.print(failCount);
    Serial.println(" failed");
    Serial.println(">>> Buzzer electrical check done — CONFIRM SOUND BY EAR <<<");
    Serial.println();
    Serial.println("Playing: frequency sweep (should be loudest near 4 kHz),");
    Serial.println("then a triple beep. No sound = BUZZ net / rail / dead element.");
    Serial.println("Send any character to re-run.");
    Serial.println();
}

// Play the audible sequence once. Returns true if a key arrived mid-sequence so
// the caller can re-run the boot self-test.
static bool playSequence() {
    static const uint32_t SWEEP[] = {1000, 2000, 3000, 4000, 5000, 6000};
    static const size_t NUM_SWEEP = sizeof(SWEEP) / sizeof(SWEEP[0]);

    Serial.println("  sweep:");
    for (size_t i = 0; i < NUM_SWEEP; i++) {
        Serial.print("    ");
        Serial.print(SWEEP[i]);
        Serial.println(" Hz");
        buzzer.tone(SWEEP[i], 250);
        // gap between tones; bail out early on a keypress
        uint32_t start = millis();
        while (millis() - start < 150) {
            if (Serial.available()) {
                buzzer.off();
                return true;
            }
            delay(5);
        }
    }

    Serial.println("  triple beep at 4 kHz");
    for (int i = 0; i < 3; i++) {
        buzzer.beep(120);
        uint32_t start = millis();
        while (millis() - start < 120) {
            if (Serial.available()) {
                buzzer.off();
                return true;
            }
            delay(5);
        }
    }

    // pause before repeating
    uint32_t start = millis();
    while (millis() - start < 1000) {
        if (Serial.available()) {
            return true;
        }
        delay(10);
    }
    return false;
}

void setup() {
    Serial.begin(115200);
    uint32_t start = millis();
    while (!Serial && millis() - start < 5000) {
        delay(10); // wait for USB host, but don't block forever
    }

    Serial.println();
    Serial.println("=== Mr Zappy PCB V2 — buzzer test ===");
    Serial.println();

    runTests();
}

void loop() {
    // Any keypress re-runs the full test sequence
    if (Serial.available()) {
        while (Serial.available()) {
            Serial.read();
        }
        passCount = failCount = 0;
        buzzer.off();
        Serial.println();
        Serial.println("--- re-running test sequence ---");
        runTests();
        return;
    }

    playSequence();
}
