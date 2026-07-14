// PCB V2 bring-up test: buttons 1-4 (H3 header, active LOW, INPUT_PULLUP)
//
// Passive GPIO inputs — no bus, no driver. The boot self-test only proves the
// pins idle HIGH through their internal pull-ups (a stuck-LOW pin means a short
// to GND or a held button). Actually confirming each button is wired to the
// GPIO it's supposed to be needs a human, so after the report this drops into a
// live mode that names every button as it's pressed and released.
//
// Pins (pins_v2.h): BUTTON1 P0.27, BUTTON2 P1.03, BUTTON3 P1.05, BUTTON4 P1.07.
// All active LOW: released reads HIGH, pressed reads LOW.

#include <Arduino.h>

#include "pins_v2.h"

struct Button {
  const char *name;
  uint8_t pin;
};

static const Button BUTTONS[] = {
    {"BUTTON1", PIN_BUTTON1},
    {"BUTTON2", PIN_BUTTON2},
    {"BUTTON3", PIN_BUTTON3},
    {"BUTTON4", PIN_BUTTON4},
};
static constexpr size_t NUM_BUTTONS = sizeof(BUTTONS) / sizeof(BUTTONS[0]);

static int passCount = 0;
static int failCount = 0;

// last debounced logical state per button (true = pressed), for live mode
static bool pressed[NUM_BUTTONS] = {false};

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

  for (size_t i = 0; i < NUM_BUTTONS; i++) {
    pinMode(BUTTONS[i].pin, INPUT_PULLUP);
  }
  // let the pull-ups settle before sampling
  delay(5);

  // Each pin must idle HIGH through its pull-up. A LOW here means the button
  // is being held, or the line is shorted to GND / mis-wired.
  for (size_t i = 0; i < NUM_BUTTONS; i++) {
    int level = digitalRead(BUTTONS[i].pin);
    bool ok = (level == HIGH);
    snprintf(detail, sizeof(detail), "P-pin %u idles %s", BUTTONS[i].pin,
             level == HIGH ? "HIGH (released)" : "LOW (held/short?)");
    report(BUTTONS[i].name, ok, detail);
    pressed[i] = (level == LOW);
  }

  Serial.println();
  Serial.print("Result: ");
  Serial.print(passCount);
  Serial.print(" passed, ");
  Serial.print(failCount);
  Serial.println(" failed");
  Serial.println(
      ">>> Button pull-up checks done — CONFIRM WIRING BY PRESSING <<<");
  Serial.println();
  Serial.println("Live mode: press each button; it should report by name.");
  Serial.println("Send any character to re-run the boot self-test.");
  Serial.println();
}

// Poll all buttons; print on any debounced edge. Simple 20 ms hold debounce.
static void pollButtons() {
  static uint32_t lastChange[NUM_BUTTONS] = {0};
  static bool rawLast[NUM_BUTTONS] = {false};

  for (size_t i = 0; i < NUM_BUTTONS; i++) {
    bool rawPressed = (digitalRead(BUTTONS[i].pin) == LOW);
    if (rawPressed != rawLast[i]) {
      rawLast[i] = rawPressed;
      lastChange[i] = millis();
    } else if (rawPressed != pressed[i] && millis() - lastChange[i] >= 20) {
      pressed[i] = rawPressed;
      Serial.print("  ");
      Serial.print(BUTTONS[i].name);
      Serial.println(rawPressed ? " pressed" : " released");
    }
  }
}

void setup() {
  Serial.begin(115200);
  uint32_t start = millis();
  while (!Serial && millis() - start < 5000) {
    delay(10); // wait for USB host, but don't block forever
  }

  Serial.println();
  Serial.println("=== Mr Zappy PCB V2 — buttons 1-4 test ===");
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
    Serial.println();
    Serial.println("--- re-running test sequence ---");
    runTests();
    return;
  }

  pollButtons();
  delay(2);
}
