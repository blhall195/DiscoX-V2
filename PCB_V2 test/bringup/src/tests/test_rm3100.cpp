// PCB V2 bring-up test: RM3100 magnetometer (U14)
//
// I2C on the V2 pins (SDA=P0.26, SCL=P0.05), DRDY on P1.15. Runs a one-shot
// check sequence, then streams field readings at 10 Hz. Expected away from
// magnets: |B| roughly 30-60 uT (Earth's field), readings change on rotation.

#include <Arduino.h>
#include <Wire.h>

#include "drivers/rm3100.h"
#include "pins_v2.h"

static constexpr uint8_t RM3100_ADDR = 0x20;
static constexpr uint16_t CYCLE_COUNT = 400; // same as V1 firmware

static RM3100 mag;

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
    char detail[80];

    // 1. Device ACKs on the bus
    Wire.beginTransmission(RM3100_ADDR);
    bool ack = Wire.endTransmission() == 0;
    snprintf(detail, sizeof(detail), "addr 0x%02X on SDA=P0.26 SCL=P0.05", RM3100_ADDR);
    report("I2C ACK", ack, detail);
    if (!ack) {
        Serial.println("  (is the board powered via the power button? RM3100 LDO hangs off ENA)");
        return;
    }

    // 2. Driver init (verifies device, sets cycle counts)
    sensorOk = mag.begin(Wire, RM3100_ADDR, CYCLE_COUNT, PIN_MAG_DRDY);
    snprintf(detail, sizeof(detail), "cycle count %u, DRDY on P1.15", CYCLE_COUNT);
    report("begin / driver init", sensorOk, detail);
    if (!sensorOk) {
        return;
    }

    // 3. Single-shot reading completes (exercises DRDY / status polling)
    RM3100::Reading raw = mag.readSingle();
    bool nonZero = raw.x != 0 || raw.y != 0 || raw.z != 0;
    snprintf(detail, sizeof(detail), "raw counts X %ld Y %ld Z %ld", (long)raw.x, (long)raw.y, (long)raw.z);
    report("single-shot reading", nonZero, detail);

    // 4. Field magnitude plausible for Earth's field (fails near magnets/steel)
    float ux, uy, uz;
    mag.toMicroTesla(raw, ux, uy, uz);
    float magnitude = sqrtf(ux * ux + uy * uy + uz * uz);
    bool fieldOk = magnitude > 15.0f && magnitude < 90.0f;
    snprintf(detail, sizeof(detail), "|B| = %.1f uT (expect ~30-60 away from magnets)", magnitude);
    report("field magnitude", fieldOk, detail);

    Serial.println();
    Serial.print("Result: ");
    Serial.print(passCount);
    Serial.print(" passed, ");
    Serial.print(failCount);
    Serial.println(" failed");
    Serial.println(failCount == 0 ? ">>> RM3100 OK <<<" : ">>> RM3100 CHECK FAILED <<<");

    if (sensorOk) {
        mag.startContinuousReading(150.0f);
    }
}

void setup() {
    Serial.begin(115200);
    uint32_t start = millis();
    while (!Serial && millis() - start < 5000) {
        delay(10); // wait for USB host, but don't block forever
    }

    Serial.println();
    Serial.println("=== Mr Zappy PCB V2 — RM3100 magnetometer test ===");
    Serial.println();

    Wire.setPins(PIN_I2C_SDA, PIN_I2C_SCL); // V2 routing, not the variant default
    Wire.begin();
    Wire.setClock(400000);

    runTests();

    if (sensorOk) {
        Serial.println();
        Serial.println("Streaming at 10 Hz (x/y/z in uT, magnitude, rough heading):");
    }
}

void loop() {
    // Any keypress re-runs the full test sequence
    if (Serial.available()) {
        while (Serial.available()) {
            Serial.read();
        }
        passCount = failCount = 0;
        mag.stop();
        Serial.println();
        Serial.println("--- re-running test sequence ---");
        runTests();
        if (sensorOk) {
            Serial.println();
            Serial.println("Streaming at 10 Hz (x/y/z in uT, magnitude, rough heading):");
        }
    }

    if (!sensorOk) {
        delay(1000);
        return;
    }

    RM3100::Reading raw = mag.getLastReading();
    float ux, uy, uz;
    mag.toMicroTesla(raw, ux, uy, uz);
    float magnitude = sqrtf(ux * ux + uy * uy + uz * uz);
    // Uncalibrated heading, just to show the reading responds to rotation
    float heading = atan2f(uy, ux) * 180.0f / PI;
    if (heading < 0) {
        heading += 360.0f;
    }

    char line[96];
    snprintf(line, sizeof(line), "X %+8.2f  Y %+8.2f  Z %+8.2f uT | |B| %6.2f | heading %5.1f",
             ux, uy, uz, magnitude, heading);
    Serial.println(line);

    delay(100);
}
