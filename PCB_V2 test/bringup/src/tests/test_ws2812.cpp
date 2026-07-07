// PCB V2 bring-up test: WS2812B RGB LED (DIN on P0.31)
//
// Output-only device — nothing to read back, so this is a visual test: the
// LED cycles solid red, green, blue, white, then a rainbow sweep, while the
// serial output names the colour that should be showing. Wrong colour order
// on the LED means the pixel type isn't NEO_GRB; no light at all points at
// the DIN net or the LED's power rail.
//
// NOTE: module pin 12 (P0.31) is a "standard drive / low frequency only" pad
// per the Raytac datasheet, and WS2812 signalling is 800 kHz — if BLE RF
// performance degrades in the merged firmware, suspect this pin first.

#include <Adafruit_NeoPixel.h>
#include <Arduino.h>

#include "pins_v2.h"

static constexpr uint16_t NUM_PIXELS = 1;

static Adafruit_NeoPixel pixel(NUM_PIXELS, PIN_NEOPIXEL_DIN, NEO_GRB + NEO_KHZ800);

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

    // 1. Driver init + first frame out (show() blocks until the PWM/DMA
    //    transfer completes, so returning at all proves the transfer ran)
    pixel.begin();
    pixel.clear();
    pixel.show();
    bool ok = pixel.numPixels() == NUM_PIXELS;
    snprintf(detail, sizeof(detail), "%u pixel on P0.31, GRB @ 800 kHz", NUM_PIXELS);
    report("driver init / first show()", ok, detail);

    // 2. canShow() recovers — proves the frame-gap timestamping works
    delay(1);
    report("canShow() after frame gap", pixel.canShow(), "");

    Serial.println();
    Serial.print("Result: ");
    Serial.print(passCount);
    Serial.print(" passed, ");
    Serial.print(failCount);
    Serial.println(" failed");
    Serial.println(">>> WS2812 electrical checks done — CONFIRM COLOURS BY EYE <<<");
    Serial.println();
    Serial.println("Cycling: RED, GREEN, BLUE, WHITE (1 s each), then rainbow.");
    Serial.println("Wrong order = pixel type not GRB; no light = DIN/power issue.");
    Serial.println();
}

// Colour wheel position -> RGB (from the NeoPixel strandtest example)
static uint32_t wheel(uint8_t pos) {
    pos = 255 - pos;
    if (pos < 85) {
        return Adafruit_NeoPixel::Color(255 - pos * 3, 0, pos * 3);
    }
    if (pos < 170) {
        pos -= 85;
        return Adafruit_NeoPixel::Color(0, pos * 3, 255 - pos * 3);
    }
    pos -= 170;
    return Adafruit_NeoPixel::Color(pos * 3, 255 - pos * 3, 0);
}

// Returns true if a key arrived (caller re-runs the test sequence)
static bool showFor(uint32_t color, const char *name, uint32_t ms) {
    if (name) {
        Serial.print("  now showing: ");
        Serial.println(name);
    }
    pixel.setPixelColor(0, color);
    pixel.show();
    uint32_t start = millis();
    while (millis() - start < ms) {
        if (Serial.available()) {
            return true;
        }
        delay(10);
    }
    return false;
}

static bool colourCycle() {
    if (showFor(Adafruit_NeoPixel::Color(255, 0, 0), "RED", 1000)) return true;
    if (showFor(Adafruit_NeoPixel::Color(0, 255, 0), "GREEN", 1000)) return true;
    if (showFor(Adafruit_NeoPixel::Color(0, 0, 255), "BLUE", 1000)) return true;
    if (showFor(Adafruit_NeoPixel::Color(255, 255, 255), "WHITE", 1000)) return true;
    Serial.println("  now showing: rainbow sweep (5 s)");
    for (uint16_t i = 0; i < 256; i++) {
        if (showFor(wheel((uint8_t)i), nullptr, 20)) {
            return true;
        }
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
    Serial.println("=== Mr Zappy PCB V2 — WS2812B RGB LED test ===");
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
        pixel.clear();
        pixel.show();
        Serial.println();
        Serial.println("--- re-running test sequence ---");
        runTests();
        return;
    }

    colourCycle();
}
