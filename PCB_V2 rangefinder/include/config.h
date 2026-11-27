#pragma once

// <Arduino.h> MUST come before "pins_v2.h": the pca10056 board variant
// #defines PIN_BUTTON1..4 for the DK's own buttons, and pins_v2.h re-#defines
// them to this board's pins. Same rule as ../PCB_V2/include/config.h.
#include <Arduino.h>
#include "defaults.h"
#include "pins_v2.h"

// Functional pin names — identical to ../PCB_V2/include/config.h (the
// 2026-08-05 button remap). Change both together.
constexpr uint8_t PIN_BTN_FIRE = PIN_BUTTON4;     // P1.07 — laser on / measure / menu select
constexpr uint8_t PIN_BTN_UP_DISCO = PIN_BUTTON2; // P1.03 — menu up
constexpr uint8_t PIN_BTN_DOWN = PIN_BUTTON1;     // P0.27 — menu down / laser off
constexpr uint8_t PIN_BTN_MENU = PIN_BUTTON3;     // P1.05 — menu open / select

constexpr uint32_t LASER_UART_BAUD_LDJ100 = 115200;

constexpr uint8_t MAX17048_ADDR = 0x36; // battery gauge
constexpr uint8_t SH1107_ADDR = 0x3C;   // OLED
constexpr uint8_t SH1107_WIDTH = 128;
constexpr uint8_t SH1107_HEIGHT = 128;

constexpr uint8_t NEOPIXEL_COUNT = 1;
