// PCB V2 bring-up test: SH1107 128x128 OLED (CN2, I2C 0x3C)
//
// I2C on the V2 pins (SDA=P0.26, SCL=P0.05). OLEDPOWER is ENA-gated, so the
// board must be powered on via the power button for the panel to light. The
// controller ACK + begin() init are genuine electrical checks; the rest is a
// "confirm by eye" visual test (like the WS2812) that cycles through patterns:
// all-pixels-on, an X-in-a-border, a checkerboard, then text + shapes.
//
// Reading the result: begin() passes but screen stays dark -> check the
// OLEDPOWER rail / the CN2 ribbon seating. Image shifted, wrapped, or split ->
// wrong controller geometry or a flaky ribbon. Upside-down text -> rotation
// (setRotation(2) matches the device's mounting, same as V1).

#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <Arduino.h>
#include <Wire.h>

#include "pins_v2.h"

// This panel straps SA0 for 0x3C (verified by the I2C scan on hardware
// 2026-07-07); the V1 board's SH1107 was at 0x3D.
static constexpr uint8_t OLED_ADDR = 0x3C;
static constexpr uint8_t OLED_W = 128;
static constexpr uint8_t OLED_H = 128;

// Same constructor args as the V1 DisplayManager.
static Adafruit_SH1107 display(OLED_W, OLED_H, &Wire, -1, 400000, 400000);

static int passCount = 0;
static int failCount = 0;
static bool displayOk = false;

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

// Probe every 7-bit address and print responders. Decisive when the OLED
// NAKs: the RM3100 (0x20) and MAX17048 (0x36) share this bus, so if they
// answer but 0x3C/0x3D don't, the bus/rail is fine and the fault is
// OLED-specific (CN2 ribbon, address, or the OLEDPOWER sub-rail). If nothing
// answers at all, the shared I2C rail is down (board not powered via button).
static void scanBus() {
    Serial.println("I2C scan (SDA=P0.26 SCL=P0.05):");
    int found = 0;
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            found++;
            Serial.print("  found 0x");
            Serial.print(addr, HEX);
            if (addr == 0x20) Serial.print("  (RM3100)");
            else if (addr == 0x36) Serial.print("  (MAX17048)");
            else if (addr == 0x3C || addr == 0x3D) Serial.print("  (OLED)");
            Serial.println();
        }
    }
    if (found == 0) {
        Serial.println("  (no devices — shared I2C rail down? power the board via the button)");
    }
    Serial.println();
}

static void runTests() {
    char detail[80];

    scanBus();

    // 1. Device ACKs on the bus
    Wire.beginTransmission(OLED_ADDR);
    bool ack = Wire.endTransmission() == 0;
    snprintf(detail, sizeof(detail), "addr 0x%02X on SDA=P0.26 SCL=P0.05", OLED_ADDR);
    report("I2C ACK", ack, detail);
    if (!ack) {
        Serial.println("  (board powered via the button? OLEDPOWER is ENA-gated; check CN2 ribbon)");
        return;
    }

    // 2. Controller init (sends the SH1107 init sequence over I2C)
    displayOk = display.begin(OLED_ADDR, true);
    snprintf(detail, sizeof(detail), "%ux%u, rotation 2 (device mounting)", OLED_W, OLED_H);
    report("begin / driver init", displayOk, detail);
    if (!displayOk) {
        return;
    }
    display.setRotation(2); // 180° to match the panel's mounting (as in V1)
    display.clearDisplay();
    display.display();

    Serial.println();
    Serial.print("Result: ");
    Serial.print(passCount);
    Serial.print(" passed, ");
    Serial.print(failCount);
    Serial.println(" failed");
    Serial.println(">>> OLED electrical checks done — CONFIRM IMAGE BY EYE <<<");
    Serial.println();
    Serial.println("Cycling: all-on, border+X, checkerboard, text+shapes.");
    Serial.println("Dark screen = OLEDPOWER/CN2; shifted/split = geometry/ribbon.");
    Serial.println("Send any character to re-run.");
    Serial.println();
}

// Show the current framebuffer for ms, printing what should be visible.
// Returns true if a key arrived (caller re-runs the test sequence).
static bool holdFrame(const char *name, uint32_t ms) {
    Serial.print("  now showing: ");
    Serial.println(name);
    uint32_t start = millis();
    while (millis() - start < ms) {
        if (Serial.available()) {
            return true;
        }
        delay(10);
    }
    return false;
}

static bool patternCycle() {
    // 1. Every pixel on — dead rows/columns or dim patches show up here.
    display.clearDisplay();
    display.fillRect(0, 0, OLED_W, OLED_H, SH110X_WHITE);
    display.display();
    if (holdFrame("all pixels ON (look for dead rows/columns)", 2000)) return true;

    // 2. 1px border + corner-to-corner X — proves the full extent is addressed
    //    and edges are flush; a shifted panel clips the border.
    display.clearDisplay();
    display.drawRect(0, 0, OLED_W, OLED_H, SH110X_WHITE);
    display.drawLine(0, 0, OLED_W - 1, OLED_H - 1, SH110X_WHITE);
    display.drawLine(0, OLED_H - 1, OLED_W - 1, 0, SH110X_WHITE);
    display.display();
    if (holdFrame("border + X (edges flush, X centred?)", 2000)) return true;

    // 3. 8x8 checkerboard — a uniform grid confirms addressing across the panel.
    display.clearDisplay();
    for (int y = 0; y < OLED_H; y += 8) {
        for (int x = 0; x < OLED_W; x += 8) {
            if (((x / 8) + (y / 8)) & 1) {
                display.fillRect(x, y, 8, 8, SH110X_WHITE);
            }
        }
    }
    display.display();
    if (holdFrame("checkerboard (uniform 8x8 grid?)", 2000)) return true;

    // 4. Text + shapes — confirms GFX rendering and readable orientation.
    display.clearDisplay();
    display.setTextColor(SH110X_WHITE);
    display.setTextSize(2);
    display.setCursor(6, 6);
    display.println("Mr Zappy");
    display.setTextSize(1);
    display.setCursor(6, 28);
    display.println("SH1107 128x128 OK");
    display.fillCircle(64, 82, 18, SH110X_WHITE);
    display.drawCircle(64, 82, 26, SH110X_WHITE);
    display.display();
    if (holdFrame("text + circles (text upright & readable?)", 2500)) return true;

    return false;
}

void setup() {
    Serial.begin(115200);
    uint32_t start = millis();
    while (!Serial && millis() - start < 5000) {
        delay(10); // wait for USB host, but don't block forever
    }

    Serial.println();
    Serial.println("=== Mr Zappy PCB V2 — SH1107 OLED test ===");
    Serial.println();

    Wire.setPins(PIN_I2C_SDA, PIN_I2C_SCL); // V2 routing, not the variant default
    Wire.begin();
    Wire.setClock(400000);

    runTests();
}

void loop() {
    // Any keypress re-runs the full test sequence
    if (Serial.available()) {
        while (Serial.available()) {
            Serial.read();
        }
        passCount = failCount = 0;
        Serial.println();
        Serial.println("--- re-running test sequence ---");
        runTests();
        return;
    }

    if (!displayOk) {
        delay(1000);
        return;
    }

    patternCycle();
}
