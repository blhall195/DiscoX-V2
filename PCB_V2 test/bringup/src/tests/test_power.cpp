// PCB V2 bring-up test: power management — power button turns the device OFF
// (LTC2954-1 U9: INT P0.06 / KILL P0.04, plus BQ24074 PGOOD P0.13).
//
// The boot self-test proves the controller is in the expected idle state:
// INT idles HIGH (button released) and KILL idles HIGH (rail latched on while
// our GPIO is high-impedance). It also reports charger PGOOD for reference.
//
// Then it runs live: hold the power button for ~1 s and the device powers off
// (a short buzzer beep confirms the shutdown path ran, then KILL is asserted
// LOW and every rail — including the nRF — drops). Press the button again to
// power back on, which reboots and re-runs this test.
//
// Boot-hold guard: right after power-on the user is still pressing the button,
// so INT is LOW. We refuse to arm shutdown until INT has been seen HIGH once
// (button released) — otherwise the board would instantly power itself back
// off.
//
// Verified on hardware 2026-07-07: INT does NOT stay LOW while the button is
// held — it pulses LOW (<1 s) per press, so a "hold to power off" check can
// never complete. We therefore power off on the first armed LOW edge, after a
// short 20 ms confirm so line noise can't shut the device down. This matches
// V1, which shut down on the INT falling edge via interrupt.

#include <Arduino.h>

#include "drivers/buzzer.h"
#include "drivers/power.h"
#include "pins_v2.h"

static constexpr uint32_t INT_CONFIRM_MS = 20; // INT must stay LOW this long

static PowerControl power;
static Buzzer buzzer;

static int passCount = 0;
static int failCount = 0;

// Shutdown is armed only after the power button has been seen released, so the
// press that powered the board on doesn't immediately power it back off.
static bool armed = false;

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

  power.begin(PIN_KILL, PIN_PB_INT, PIN_PGOOD);
  buzzer.begin(PIN_BUZZER_A, PIN_BUZZER_B);
  delay(5); // let the pullups settle before sampling

  // 1. INT must idle HIGH — the button is released. This also arms shutdown:
  //    if it's LOW here (still held from power-on), we stay disarmed.
  bool intHigh = (digitalRead(PIN_PB_INT) == HIGH);
  snprintf(detail, sizeof(detail), "INT P0.06 %s",
           intHigh ? "HIGH (released)" : "LOW (still held / shorted?)");
  report("INT idles HIGH", intHigh, detail);
  armed = intHigh;

  // 2. KILL must idle HIGH via its external pullup while our GPIO is high-Z.
  //    A LOW here means the pullup is missing or KILL is shorted — the board
  //    would never stay powered.
  bool killHigh = (digitalRead(PIN_KILL) == HIGH);
  snprintf(detail, sizeof(detail),
           "KILL P0.04 %s (high-Z, pull-up holds rail on)",
           killHigh ? "HIGH" : "LOW (no pull-up / short?)");
  report("KILL idles HIGH", killHigh, detail);

  // Charger power-good is informational (depends on USB being plugged in).
  bool pg = power.powerGood();
  Serial.print("[info] PGOOD P0.13 ");
  Serial.println(pg ? "LOW — USB input present (charging source good)"
                    : "HIGH — no USB input (running on battery)");

  Serial.println();
  Serial.print("Result: ");
  Serial.print(passCount);
  Serial.print(" passed, ");
  Serial.print(failCount);
  Serial.println(" failed");
  Serial.println(">>> Power-off is LIVE <<<");
  Serial.println(
      "Press the power button to power OFF (beep confirms, then rails drop).");
  if (!armed) {
    Serial.println("(shutdown disarmed until the button is released once)");
  }
  Serial.println("Send any character to re-run the self-test.");
  Serial.println();
}

// Monitor the power button: on an armed INT LOW edge (confirmed for
// INT_CONFIRM_MS so noise can't trigger it), power the device off.
static void pollPower() {
  bool down = power.buttonPressed(); // INT LOW == pressed

  if (!armed) {
    // Wait for the power-on press to be released before allowing shutdown.
    if (!down) {
      armed = true;
      Serial.println("  power button released — shutdown armed");
    }
    return;
  }

  if (!down) {
    return;
  }

  // Confirm the LOW isn't a glitch. INT's minimum assertion is well above
  // 20 ms, so a real press always passes; if it deasserts mid-confirm we
  // still honour it as a press only if it stayed LOW the whole window.
  uint32_t start = millis();
  while (millis() - start < INT_CONFIRM_MS) {
    if (!power.buttonPressed()) {
      return; // glitch — ignore
    }
    delay(1);
  }

  Serial.println("  power button pressed — powering off");
  Serial.flush();
  buzzer.beep(200); // audible confirmation the shutdown path ran
  power.powerOff(); // asserts KILL LOW — never returns
}

void setup() {
  Serial.begin(115200);
  uint32_t start = millis();
  while (!Serial && millis() - start < 5000) {
    delay(10); // wait for USB host, but don't block forever
  }

  Serial.println();
  Serial.println("=== Mr Zappy PCB V2 — power management test ===");
  Serial.println();

  runTests();
}

void loop() {
  // Any keypress re-runs the self-test (never powers off — that's button-only)
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

  pollPower();
  delay(5);
}
