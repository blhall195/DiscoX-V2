// PCB V2 bring-up test: MAX17048 LiPo fuel gauge (U3, I2C 0x36)
//
// I2C on the V2 pins (SDA=P0.26, SCL=P0.05). Reports cell voltage, state of
// charge, and charge/discharge rate, then streams them at 1 Hz so you can watch
// SOC/voltage respond (e.g. plug/unplug USB to see the charge rate flip sign).
//
// The gauge senses BAT+_RAW (the raw battery input at CN1), so attach a LiPo
// for a meaningful reading — on USB with no battery, voltage/SOC are a
// voltage-only guess and may sit out of the plausible-cell range.
//
// V1 lesson baked into the driver (max17048.h): we skip the Adafruit begin()'s
// reset() so the IC keeps its learned ModelGauge state across reboots.

#include <Arduino.h>
#include <Wire.h>

#include "drivers/max17048.h"
#include "pins_v2.h"

static constexpr uint8_t MAX17048_ADDR = 0x36;

static MAX17048_Persistent battery;

static int passCount = 0;
static int failCount = 0;
static bool sensorOk = false;

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

    // 1. Device ACKs on the bus
    Wire.beginTransmission(MAX17048_ADDR);
    bool ack = Wire.endTransmission() == 0;
    snprintf(detail, sizeof(detail), "addr 0x%02X on SDA=P0.26 SCL=P0.05", MAX17048_ADDR);
    report("I2C ACK", ack, detail);
    if (!ack) {
        Serial.println("  (is the board powered via the power button? the gauge's rail is ENA-gated)");
        return;
    }

    // 2. Driver init — no-reset begin() preserves the ModelGauge state
    sensorOk = battery.begin(&Wire);
    report("begin / driver init (no reset)", sensorOk, "ModelGauge state preserved");
    if (!sensorOk) {
        return;
    }
    delay(100); // let the SOC register settle after wake()

    // 3. Cell voltage in a plausible LiPo range (fails if no battery attached)
    float v = battery.cellVoltage();
    bool vOk = v > 2.5f && v < 4.5f;
    snprintf(detail, sizeof(detail), "%.3f V (expect ~3.0-4.2 with a LiPo attached)", v);
    report("cell voltage", vOk, detail);

    // 4. State of charge within 0-100%
    float pct = battery.cellPercent();
    bool pctOk = pct >= 0.0f && pct <= 100.0f;
    snprintf(detail, sizeof(detail), "%.1f%% SOC", pct);
    report("state of charge", pctOk, detail);

    // Charge rate is informational — sign shows charging vs discharging
    float rate = battery.chargeRate();
    Serial.print("[info] charge rate ");
    Serial.print(rate, 2);
    Serial.println(" %/hr (positive = charging)");

    Serial.println();
    Serial.print("Result: ");
    Serial.print(passCount);
    Serial.print(" passed, ");
    Serial.print(failCount);
    Serial.println(" failed");
    Serial.println(failCount == 0 ? ">>> MAX17048 OK <<<" : ">>> MAX17048 CHECK FAILED <<<");
    if (pctOk == false) {
        Serial.println("  (SOC out of range on a fresh cell is a ModelGauge seeding");
        Serial.println("   artifact, not a wiring fault — press 'q' to quickStart it)");
    }
}

void setup() {
    Serial.begin(115200);
    uint32_t start = millis();
    while (!Serial && millis() - start < 5000) {
        delay(10); // wait for USB host, but don't block forever
    }

    Serial.println();
    Serial.println("=== Mr Zappy PCB V2 — MAX17048 battery gauge test ===");
    Serial.println();

    Wire.setPins(PIN_I2C_SDA, PIN_I2C_SCL); // V2 routing, not the variant default
    Wire.begin();
    Wire.setClock(400000);

    runTests();

    if (sensorOk) {
        Serial.println();
        Serial.println("Streaming at 1 Hz (voltage, SOC, charge rate):");
    }
}

void loop() {
    // Keypress: 'q' quickStarts the gauge (re-anchors SOC to the present
    // voltage); any other key re-runs the full test sequence.
    if (Serial.available()) {
        int c = Serial.read();
        while (Serial.available()) {
            Serial.read();
        }
        if ((c == 'q' || c == 'Q') && sensorOk) {
            Serial.println();
            Serial.print("quickStart: SOC ");
            Serial.print(battery.cellPercent(), 1);
            Serial.print("% -> ");
            battery.quickStart();
            delay(200); // let SOC recompute from the current cell voltage
            Serial.print(battery.cellPercent(), 1);
            Serial.println("%");
        } else {
            passCount = failCount = 0;
            Serial.println();
            Serial.println("--- re-running test sequence ---");
            runTests();
            if (sensorOk) {
                Serial.println();
                Serial.println("Streaming at 1 Hz (voltage, SOC, charge rate):");
            }
        }
    }

    if (!sensorOk) {
        delay(1000);
        return;
    }

    char line[96];
    snprintf(line, sizeof(line), "V %.3f  SOC %5.1f%%  rate %+6.2f %%/hr",
             battery.cellVoltage(), battery.cellPercent(), battery.chargeRate());
    Serial.println(line);

    delay(1000);
}
