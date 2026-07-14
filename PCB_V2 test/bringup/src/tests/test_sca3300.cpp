// PCB V2 bring-up test: SCA3300 accelerometer (U1)
//
// Runs a one-shot check sequence over USB serial, then streams live
// readings at 10 Hz. Expected with the board sitting still: WHOAMI 0x51,
// clean status, |g| within 5% of 1.0, STO well inside ±800 LSB (mode 1).

#include <Arduino.h>
#include <SPI.h>

#include "drivers/sca3300.h"
#include "pins_v2.h"

// Dedicated SPIM instance on the PCB V2 accelerometer pins (SPIM3 is
// reserved by the core's default SPI object, SPIM0/1 by Wire/Serial)
static SPIClass scaSpi(NRF_SPIM2, PIN_SCA3300_MISO, PIN_SCA3300_SCK,
                       PIN_SCA3300_MOSI);
static SCA3300 sca;

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

static const char *errorName(SCA3300::Error e) {
  switch (e) {
  case SCA3300::Error::NONE:
    return "none";
  case SCA3300::Error::CRC:
    return "CRC mismatch (check wiring/SCK)";
  case SCA3300::Error::STARTUP:
    return "startup in progress";
  case SCA3300::Error::SENSOR_FLAG:
    return "sensor error flag (see STATUS)";
  case SCA3300::Error::WHOAMI:
    return "WHOAMI mismatch";
  }
  return "?";
}

static void printStatusBits(uint16_t status) {
  if (status == 0) {
    Serial.println("  status clear");
    return;
  }
  if (status & SCA3300::STATUS_DIGI1)
    Serial.println("  DIGI1: digital block error 1");
  if (status & SCA3300::STATUS_DIGI2)
    Serial.println("  DIGI2: digital block error 2");
  if (status & SCA3300::STATUS_CLK)
    Serial.println("  CLK: clock error");
  if (status & SCA3300::STATUS_SAT)
    Serial.println("  SAT: signal saturated");
  if (status & SCA3300::STATUS_TEMP_SAT)
    Serial.println("  TEMP_SAT: temp saturated");
  if (status & SCA3300::STATUS_PWR)
    Serial.println("  PWR: startup/voltage flag");
  if (status & SCA3300::STATUS_MEM)
    Serial.println("  MEM: NVM error");
  if (status & SCA3300::STATUS_PD)
    Serial.println("  PD: power down");
  if (status & SCA3300::STATUS_MODE_CHANGE)
    Serial.println("  MODE_CHANGE");
  if (status & SCA3300::STATUS_PIN_CONTINUITY)
    Serial.println("  PIN_CONTINUITY: internal connection error");
}

static void runTests() {
  char detail[64];

  // 1. Start-up sequence (SW reset, mode set, status clear, WHOAMI)
  sensorOk = sca.begin(scaSpi, PIN_SCA3300_CS, SCA3300::Mode::MODE_1);
  snprintf(
      detail, sizeof(detail), "mode 1 (±3g, 70Hz)%s%s",
      sensorOk ? "" : ", error: ", sensorOk ? "" : errorName(sca.lastError()));
  report("begin / start-up sequence", sensorOk, detail);
  if (!sensorOk) {
    uint16_t status;
    if (sca.readStatus(status)) {
      Serial.print("  STATUS = 0x");
      Serial.println(status, HEX);
      printStatusBits(status);
    }
    return;
  }

  // 2. WHOAMI
  uint8_t who = sca.readWhoAmI();
  snprintf(detail, sizeof(detail), "read 0x%02X, expect 0x51", who);
  report("WHOAMI", who == SCA3300::WHOAMI_VALUE, detail);

  // 3. Status summary clean after start-up
  uint16_t status = 0xFFFF;
  bool stOk = sca.readStatus(status) && status == 0;
  snprintf(detail, sizeof(detail), "STATUS = 0x%04X", status);
  report("status summary clear", stOk, detail);
  if (!stOk) {
    printStatusBits(status);
  }

  // 4. Serial number (also exercises bank switching)
  char serialNo[16] = {0};
  bool snOk = sca.readSerialNumber(serialNo, sizeof(serialNo));
  report("serial number", snOk, snOk ? serialNo : "read failed");

  // 5. Self-test output at rest (±800 LSB threshold in mode 1)
  int16_t sto = 0;
  bool stoRead = sca.readSelfTest(sto);
  bool stoOk = stoRead && sto > -800 && sto < 800;
  snprintf(detail, sizeof(detail), "STO = %d LSB (limit ±800)", sto);
  report("self-test output", stoOk, stoRead ? detail : "read failed");

  // 6. Temperature plausible
  float temp = 0;
  bool tOk = sca.readTemperature(temp) && temp > -20.0f && temp < 60.0f;
  snprintf(detail, sizeof(detail), "%.1f degC", temp);
  report("temperature", tOk, detail);

  // 7. Gravity magnitude: 200 samples, mean |g| should be 1.0 ±0.05
  double sum = 0, sumSq = 0;
  int n = 0;
  for (int i = 0; i < 200; i++) {
    float x, y, z;
    if (sca.readAcceleration(x, y, z)) {
      double mag = sqrt((double)x * x + (double)y * y + (double)z * z);
      sum += mag;
      sumSq += mag * mag;
      n++;
    }
    delay(2); // ~ODR/4, keeps register reads fresh
  }
  if (n > 150) {
    float mean = sum / n;
    // fmax: in float, sumSq/n - mean^2 can cancel to <0 and sqrt -> nan
    float rms = sqrt(fmax(0.0, sumSq / n - (double)mean * mean));
    bool gOk = mean > 0.95f && mean < 1.05f;
    snprintf(detail, sizeof(detail),
             "|g| mean %.4f, rms noise %.4f g (%d/200 samples)", mean, rms, n);
    report("gravity magnitude", gOk, detail);
  } else {
    snprintf(detail, sizeof(detail), "only %d/200 samples read OK", n);
    report("gravity magnitude", false, detail);
  }

  Serial.println();
  Serial.print("Result: ");
  Serial.print(passCount);
  Serial.print(" passed, ");
  Serial.print(failCount);
  Serial.println(" failed");
  Serial.println(failCount == 0 ? ">>> SCA3300 OK <<<"
                                : ">>> SCA3300 CHECK FAILED <<<");
}

void setup() {
  Serial.begin(115200);
  uint32_t start = millis();
  while (!Serial && millis() - start < 5000) {
    delay(10); // wait for USB host, but don't block forever
  }

  Serial.println();
  Serial.println("=== Mr Zappy PCB V2 — SCA3300 accelerometer test ===");
  Serial.println("Pins: CS=P0.14 SCK=P0.21 MOSI=P0.19 MISO=P0.16 (SPIM2)");
  Serial.println();

  runTests();

  if (sensorOk) {
    Serial.println();
    Serial.println("Streaming at 10 Hz (x/y/z in g, pitch/roll in deg, temp):");
  }
}

void loop() {
  // Any keypress re-runs the full test sequence (the boot-time run is easy
  // to miss while the USB port re-enumerates after flashing)
  if (Serial.available()) {
    while (Serial.available()) {
      Serial.read();
    }
    passCount = failCount = 0;
    Serial.println();
    Serial.println("--- re-running test sequence ---");
    runTests();
    if (sensorOk) {
      Serial.println();
      Serial.println(
          "Streaming at 10 Hz (x/y/z in g, pitch/roll in deg, temp):");
    }
  }

  if (!sensorOk) {
    delay(1000);
    return;
  }

  float x, y, z;
  if (!sca.readAcceleration(x, y, z)) {
    Serial.print("read error: ");
    Serial.println(errorName(sca.lastError()));
    uint16_t status;
    if (sca.lastError() == SCA3300::Error::SENSOR_FLAG &&
        sca.readStatus(status)) {
      printStatusBits(status);
    }
    delay(500);
    return;
  }

  // Tilt from gravity — the quantity this sensor is on the board for
  float pitch = atan2f(-x, sqrtf(y * y + z * z)) * 180.0f / PI;
  float roll = atan2f(y, z) * 180.0f / PI;

  float temp = 0;
  sca.readTemperature(temp);

  char line[96];
  snprintf(
      line, sizeof(line),
      "X %+7.4f  Y %+7.4f  Z %+7.4f g | pitch %+6.1f  roll %+6.1f | %.1f C", x,
      y, z, pitch, roll, temp);
  Serial.println(line);

  delay(100);
}
