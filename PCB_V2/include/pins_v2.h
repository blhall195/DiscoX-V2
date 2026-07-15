#pragma once

// PCB V2 pin map — Raytac MDBT50Q-U1MV2 (nRF52840) module, single-MCU design.
// Derived from the netlist + module datasheet; the full decoded pin/net
// table lives in ../hardware/NETLIST.md.
//
// Arduino pin numbers use the pca10056 variant identity mapping:
//   P0.xx -> xx, P1.yy -> 32 + yy

// --- SCA3300 accelerometer (U1, SPI, own LDO on 3V3SCA) ---
#define PIN_SCA3300_CS 14   // module pin 36, P0.14, net CSB
#define PIN_SCA3300_MISO 16 // module pin 38, P0.16, net MISO
#define PIN_SCA3300_MOSI 19 // module pin 42, P0.19, net MOSI
#define PIN_SCA3300_SCK 21  // module pin 43, P0.21, net SCK

// --- I2C bus (RM3100 U14, MAX17048 U3, OLED CN2; 4.7k pullups to 3V3NRF) ---
#define PIN_I2C_SDA 26 // module pin 19, P0.26, net SDA
#define PIN_I2C_SCL 5  // module pin 21, P0.05, net SCL

// --- RM3100 magnetometer ---
#define PIN_MAG_DRDY 47 // module pin 8, P1.15, net DRRDY

// --- Laser rangefinder UART (U5 JST) ---
// Laser power rail (LZR_PWR_3V) is gated by ENA via LDO1 — no GPIO control.
// The netlist net names TX/RX are from the LASER's perspective — verified on
// hardware 2026-07-06 (LDJ-100 only answers with this orientation).
#define PIN_LASER_TX 32 // nRF TX -> laser RXD, module pin 47, P1.00, net RX
#define PIN_LASER_RX 25 // nRF RX <- laser TXD, module pin 49, P0.25, net TX

// --- Buttons (active LOW, use INPUT_PULLUP) ---
// The pca10056 board variant already #defines PIN_BUTTON1..4 for the DK's
// on-board buttons (pins 11/12/24/25) — undef so our board's pins win. This
// header must be included AFTER <Arduino.h> for the undef to take effect.
#undef PIN_BUTTON1
#undef PIN_BUTTON2
#undef PIN_BUTTON3
#undef PIN_BUTTON4
#define PIN_BUTTON1 27 // module pin 16, P0.27, net BUTTON1
#define PIN_BUTTON2 35 // module pin 60, P1.03, net BUTTON2
#define PIN_BUTTON3 37 // module pin 59, P1.05, net BUTTON3
#define PIN_BUTTON4 39 // module pin 58, P1.07, net BUTTON4

// --- Power management (LTC2954 pushbutton controller U9, BQ24074 charger U2)
// ---
#define PIN_KILL 4   // module pin 20, P0.04, net KILL  — drive LOW to power off
#define PIN_PB_INT 6 // module pin 22, P0.06, net INT   — LOW on power-button press
#define PIN_PGOOD 13 // module pin 37, P0.13, net PGOOD — charger power good (open drain)

// --- Buzzer (driven push-pull across two GPIOs) ---
#define PIN_BUZZER_A 20 // module pin 44, P0.20, net BUZZ1
#define PIN_BUZZER_B 24 // module pin 48, P0.24, net BUZZ2

// --- WS2812B RGB LED ---
// NOTE: module pin 12 is a "standard drive / low frequency only" pad per the
// Raytac datasheet; WS2812 signalling is 800 kHz — watch for RF degradation.
#define PIN_NEOPIXEL_DIN 31 // module pin 12, P0.31, net DIN
