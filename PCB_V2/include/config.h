#pragma once

// <Arduino.h> MUST come before "pins_v2.h": the pca10056 board variant we
// build against #defines PIN_BUTTON1..4 for the DK's own on-board buttons
// (pins 11/12/24/25). pins_v2.h re-#defines those names to THIS board's
// button pins (27/35/37/39) and must be processed last, or button_manager.cpp
// (which pulls this header before any Arduino.h) silently reads the DK's pins.
#include "defaults.h"
#include <Arduino.h>
#include "pins_v2.h"

// PCB V2 — single nRF52840 (Raytac MDBT50Q-U1MV2). All physical pins come
// from pins_v2.h (canonical, cross-checked against
// "../PCB_V2 test/hardware/NETLIST.md"). This file maps them onto the
// firmware's functional names and holds the non-pin constants.

// ── Button pins (all active LOW with internal pull-ups) ─────────────
// V2 has 4 GPIO buttons + a dedicated hardware power toggle (LTC2954).
constexpr uint8_t PIN_BTN_FIRE = PIN_BUTTON1;     // B1 — take measurement
constexpr uint8_t PIN_BTN_UP_DISCO = PIN_BUTTON2; // B2 — menu up / disco toggle
constexpr uint8_t PIN_BTN_DOWN = PIN_BUTTON3;     // B3 — menu down
constexpr uint8_t PIN_BTN_MENU = PIN_BUTTON4;     // B4 — menu open / select

// ── Power control (LTC2954 pushbutton controller) ───────────────────
// PIN_KILL / PIN_PB_INT / PIN_PGOOD come from pins_v2.h.
// KILL stays high-Z (INPUT) until shutdown — drive LOW to power off.
// PB_INT pulses LOW (<1 s) on a power-button press (edge-triggered).

// ── Laser UART (Meskernel LDJ-100RED on Serial1) ────────────────────
// PIN_LASER_TX / PIN_LASER_RX from pins_v2.h. RX needs INPUT_PULLUP
// (module TXD is open drain). Rail is ENA-gated — no GPIO power control.
constexpr uint32_t LASER_UART_BAUD_LDJ100 = 115200;

// ── Magnetometer RM3100 ─────────────────────────────────────────────
// PIN_MAG_DRDY comes from pins_v2.h (P1.15).
constexpr uint8_t RM3100_I2C_ADDR = 0x20;
constexpr uint16_t RM3100_CYCLE_COUNT = 400;

// ── I2C addresses ───────────────────────────────────────────────────
constexpr uint8_t MAX17048_ADDR = 0x36; // Battery gauge
constexpr uint8_t SH1107_ADDR = 0x3C;   // OLED display (V2 panel straps SA0 low; V1 was 0x3D — verified on hardware 2026-07-07)
constexpr uint8_t SH1107_WIDTH = 128;
constexpr uint8_t SH1107_HEIGHT = 128;

// ── WS2812 RGB LED ──────────────────────────────────────────────────
// Data pin = PIN_NEOPIXEL_DIN (pins_v2.h). No power-gate pin on V2 —
// the LED rail is hardware-gated by ENA.
constexpr uint8_t NEOPIXEL_COUNT = 1;

// ── Calibration axis mappings ───────────────────────────────────────
// ⚠ PLACEHOLDERS — V1 PCB values. The V2 board mounts an SCA3300 (not the
// ISM330DHCX) and the RM3100 sits in a new position/orientation. Determine
// the real mappings from streamed raw data during commissioning, then run a
// full calibration (56-pt ellipsoid + 24-pt alignment + F/B check).
constexpr char MAG_AXES[] = "-X-Y-Z";
constexpr char GRAV_AXES[] = "-Y-X+Z";

// ── System power-off hook ───────────────────────────────────────────
// Defined in main.cpp: syncs pending data, beeps, then drives KILL LOW via
// the LTC2954 (never returns). Used by menu/snake auto-shutdown paths that
// used to write V1's PIN_POWER directly.
void systemPowerOff();

// ── Unit conversion ─────────────────────────────────────────────────
// The SCA3300 reports acceleration in g; the whole V1 pipeline (calibration
// fitting, shake detection, field-strength stats) was built on the
// ISM330DHCX's m/s² — convert at the read site to keep everything unchanged.
constexpr float GRAVITY_MS2 = 9.80665f;

// ── Sensor fusion constants ─────────────────────────────────────────
// (Gravity filter params are in sensor_manager.h as class constants)
// EMA alpha and stability buffer length come from Defaults:: (defaults.h)
