// PCB V2 bring-up test: Meskernel LDJ-100RED laser rangefinder (U5 JST)
//
// UART on nRF TX=P1.00 / RX=P0.25 (the netlist TX/RX nets are named from the
// laser's side — verified on hardware), module default 115200 8N1. The laser
// rail (LZR_PWR_3V) is LDO-gated by ENA, so the module powers up with the
// board — its 2.5 s auto-baud window is normally long gone and it sits at the
// fixed default 115200. The connect scan still tries the 0x55 handshake,
// other bauds, and swapped TX/RX, and reports whichever combination answered.
//
// After the checks: laser dot for aiming, one measurement, then continuous
// streaming. Any keypress re-runs the full sequence.

#include <Arduino.h>

#include "drivers/ldj100.h"
#include "pins_v2.h"

static LDJ100 laser;

static int passCount = 0;
static int failCount = 0;
static bool sensorOk = false;

// Filled in by the connect scan
static uint32_t linkBaud = 0;
static bool linkSwapped = false;
static bool linkHandshake = false;

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

static void beginLaserUart(uint32_t baud, bool swapped) {
  static bool uartBegun = false;
  // Uart::end() on a never-begun UART spins forever waiting for
  // TXSTOPPED/RXTO events that can't fire while the peripheral is disabled
  if (uartBegun) {
    Serial1.end();
  }
  uartBegun = true;
  uint8_t rx = swapped ? PIN_LASER_TX : PIN_LASER_RX;
  uint8_t tx = swapped ? PIN_LASER_RX : PIN_LASER_TX;
  Serial1.setPins(rx, tx);
  pinMode(rx, INPUT_PULLUP); // module TXD is open drain
  Serial1.begin(baud);
  delay(20);
  laser.begin(Serial1);
}

static bool tryLink(uint32_t baud, bool swapped) {
  beginLaserUart(baud, swapped);
  laser.stopContinuous(); // in case a previous run left it streaming
  linkHandshake = false;
  if (laser.ping(300)) {
    return true;
  }
  uint8_t moduleAddr;
  if (laser.autoBaudHandshake(moduleAddr, 200) && laser.ping(300)) {
    linkHandshake = true;
    return true;
  }
  return false;
}

static bool connectScan() {
  static const uint32_t BAUDS[] = {115200, 9600, 19200, 38400};
  for (int swapped = 0; swapped <= 1; swapped++) {
    for (uint32_t baud : BAUDS) {
      if (tryLink(baud, swapped != 0)) {
        linkBaud = baud;
        linkSwapped = swapped != 0;
        return true;
      }
    }
  }
  return false;
}

static void runTests() {
  char detail[96];

  // 1. Module answers on the UART (scans baud rates and TX/RX swap)
  sensorOk = connectScan();
  if (sensorOk) {
    snprintf(detail, sizeof(detail), "%lu baud, pins %s%s",
             (unsigned long)linkBaud,
             linkSwapped ? "SWAPPED (pins_v2.h wrong?)"
                         : "as pins_v2.h (TX=P1.00 RX=P0.25)",
             linkHandshake ? ", via 0x55 handshake" : "");
  } else {
    snprintf(detail, sizeof(detail),
             "no response at any baud, either pin order");
  }
  report("UART link", sensorOk, detail);
  if (!sensorOk) {
    Serial.println(
        "  (laser plugged into U5 JST? LZR_PWR_3V rail is ENA-gated —");
    Serial.println("   board must be powered via the power button)");
    return;
  }

  // 2. Status register reports no error
  uint16_t status = 0xFFFF;
  bool ok = laser.readStatus(status);
  snprintf(detail, sizeof(detail), "0x%04X (%s)", status,
           LDJ100::statusText(status));
  report("status register", ok && status == 0x0000, detail);

  // 3. Identity registers respond
  uint16_t hwVer = 0, swVer = 0;
  uint32_t serialNo = 0;
  ok = laser.readHardwareVersion(hwVer) && laser.readSoftwareVersion(swVer) &&
       laser.readSerialNumber(serialNo);
  snprintf(detail, sizeof(detail), "HW 0x%04X, SW 0x%04X, SN 0x%08lX", hwVer,
           swVer, (unsigned long)serialNo);
  report("version / serial readout", ok, detail);

  // 4. Input voltage plausible for the 3.3 V laser rail
  uint16_t mv = 0;
  ok = laser.readInputVoltageMv(mv);
  snprintf(detail, sizeof(detail), "%u mV (rail is LZR_PWR_3V via LDO1)", mv);
  report("input voltage", ok && mv > 2500 && mv < 3600, detail);

  // 5. Laser diode control — dot should be visible while this runs
  bool onOk = laser.setLaser(true);
  delay(1500); // aiming time: red dot on target
  bool offOk = laser.setLaser(false);
  snprintf(detail, sizeof(detail), "on %s, off %s (red dot for 1.5 s)",
           onOk ? "ok" : "FAILED", offOk ? "ok" : "FAILED");
  report("laser on/off", onOk && offOk, detail);

  // 6. Single automatic measurement (3 attempts — transient errors happen)
  LDJ100::Measurement m;
  ok = false;
  for (int attempt = 0; attempt < 3 && !ok; attempt++) {
    ok = laser.measure(m, LDJ100::SINGLE_AUTO, 5000);
    if (!ok) {
      delay(300);
    }
  }
  if (ok) {
    snprintf(detail, sizeof(detail), "%lu mm (%.3f m), signal quality %u",
             (unsigned long)m.distanceMm, m.distanceMm / 1000.0,
             m.signalQuality);
  } else if (laser.lastStatus() != 0) {
    snprintf(detail, sizeof(detail), "module error %d (%s)",
             (int16_t)laser.lastStatus(),
             LDJ100::statusText(laser.lastStatus()));
  } else {
    snprintf(detail, sizeof(detail), "timeout waiting for result frame");
  }
  report("single measurement",
         ok && m.distanceMm >= 30 && m.distanceMm <= 200000, detail);
  if (!ok && laser.lastStatus() != 0) {
    Serial.println(
        "  (aim the laser at a matte surface 0.03-100 m away, then press");
    Serial.println(
        "   any key to re-run — the module reports out-of-range/weak-signal");
    Serial.println("   errors when it has no valid target)");
  }

  Serial.println();
  Serial.print("Result: ");
  Serial.print(passCount);
  Serial.print(" passed, ");
  Serial.print(failCount);
  Serial.println(" failed");
  Serial.println(failCount == 0 ? ">>> LDJ-100 OK <<<"
                                : ">>> LDJ-100 CHECK FAILED <<<");

  if (sensorOk) {
    laser.startContinuous(LDJ100::CONT_AUTO);
    Serial.println();
    Serial.println(
        "Streaming continuous measurements (distance, signal quality):");
  }
}

void setup() {
  Serial.begin(115200);
  uint32_t start = millis();
  while (!Serial && millis() - start < 5000) {
    delay(10); // wait for USB host, but don't block forever
  }

  Serial.println();
  Serial.println("=== Mr Zappy PCB V2 — Meskernel LDJ-100RED laser test ===");
  Serial.println();

  runTests();
}

void loop() {
  // Any keypress re-runs the full test sequence
  if (Serial.available()) {
    while (Serial.available()) {
      Serial.read();
    }
    if (sensorOk) {
      laser.stopContinuous();
    }
    passCount = failCount = 0;
    Serial.println();
    Serial.println("--- re-running test sequence ---");
    runTests();
  }

  if (!sensorOk) {
    delay(1000);
    return;
  }

  static uint16_t lastPrintedError = 0;
  LDJ100::Measurement m;
  if (laser.poll(m, 500)) {
    lastPrintedError = 0;
    char line[64];
    snprintf(line, sizeof(line), "%8.3f m | SQ %5u", m.distanceMm / 1000.0,
             m.signalQuality);
    Serial.println(line);
  } else if (laser.lastStatus() != 0 &&
             laser.lastStatus() != lastPrintedError) {
    // Only print error changes — no-target errors stream at ~20 Hz
    lastPrintedError = laser.lastStatus();
    char line[80];
    snprintf(line, sizeof(line), "module error %d (%s)",
             (int16_t)laser.lastStatus(),
             LDJ100::statusText(laser.lastStatus()));
    Serial.println(line);
  }
}
