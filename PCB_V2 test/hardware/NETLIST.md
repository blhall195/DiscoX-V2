# PCB V2 — decoded netlist reference

Decoded from `PCB_V2-PCB_2026-07-06.json` (EasyEDA PCB export) cross-checked
against the Raytac MDBT50Q-U1MV2 datasheet pin assignment. Anchors verified:
32 kHz crystal on module pins 17/18, USB on 32/34/35, SWD on 51/53, VDD/VDDH
on 28/30 — all match, so this mapping is trusted.

Arduino pin numbers assume the pca10056 variant identity mapping
(P0.xx = xx, P1.yy = 32 + yy). Canonical defines: `bringup/include/pins_v2.h`.

## nRF module (U7, MDBT50Q-U1MV2) — used pins

| Module pin | nRF GPIO | Arduino pin | Net | Function |
|-----------|----------|-------------|---------|----------|
| 8  | P1.15 | 47 | DRRDY   | RM3100 data ready |
| 12 | P0.31 | 31 | DIN     | WS2812B data (low-freq-only pad — RF caveat) |
| 16 | P0.27 | 27 | BUTTON1 | Button, active LOW |
| 17 | P0.00 | —  | XL1     | 32.768 kHz crystal |
| 18 | P0.01 | —  | XL2     | 32.768 kHz crystal |
| 19 | P0.26 | 26 | SDA     | I2C data (4.7k pullup to 3V3NRF) |
| 20 | P0.04 | 4  | KILL    | LTC2954 KILL — drive LOW to power off (10k pullup) |
| 21 | P0.05 | 5  | SCL     | I2C clock (4.7k pullup to 3V3NRF) |
| 22 | P0.06 | 6  | INT     | LTC2954 INT — LOW on power-button press (10k pullup) |
| 28 | VDD   | —  | 3V3NRF  | Module supply |
| 30 | VDDH  | —  | 3V3NRF  | High-voltage supply (tied to VDD) |
| 32 | VBUS  | —  | USB     | USB 5V |
| 34 | D-    | —  | D-      | USB (27R series) |
| 35 | D+    | —  | D+      | USB (27R series) |
| 36 | P0.14 | 14 | CSB     | SCA3300 chip select |
| 37 | P0.13 | 13 | PGOOD   | BQ24074 power good (open drain — needs internal pullup) |
| 38 | P0.16 | 16 | MISO    | SCA3300 SPI |
| 40 | P0.18 | —  | (SW1)   | nRESET — reset button to GND |
| 42 | P0.19 | 19 | MOSI    | SCA3300 SPI |
| 43 | P0.21 | 21 | SCK     | SCA3300 SPI |
| 44 | P0.20 | 20 | BUZZ1   | Buzzer leg A (push-pull pair) |
| 47 | P1.00 | 32 | RX      | Laser UART RX |
| 48 | P0.24 | 24 | BUZZ2   | Buzzer leg B (push-pull pair) |
| 49 | P0.25 | 25 | TX      | Laser UART TX |
| 51 | SWDIO | —  | SWDIO   | SWD header H2 pin 5 |
| 53 | SWDCLK| —  | SWDCLK  | SWD header H2 pin 4 |
| 58 | P1.07 | 39 | BUTTON4 | Button, active LOW |
| 59 | P1.05 | 37 | BUTTON3 | Button, active LOW |
| 60 | P1.03 | 35 | BUTTON2 | Button, active LOW |

## ICs and connectors

| Ref | Part | Bus / pins |
|-----|------|-----------|
| U1  | SCA3300-D01-1 accelerometer | SPI: CSB/MISO/MOSI/SCK above; powered from 3V3SCA |
| U14 | RM3100 magnetometer (module) | I2C addr 0x20; DRDY→DRRDY; powered from 3V3RM3100 |
| U3  | MAX17048 battery gauge | I2C addr 0x36; senses BAT+_RAW |
| CN2 | OLED connector (BM04B-SRSS) | I2C: SCL, SDA, OLEDPOWER, GND |
| U5  | Laser JST connector | LZR_PWR_3V, RX, TX, buzzer-enable (U5_4 via 220R from ENA), GND |
| U2  | BQ24074 LiPo charger | USB in, BAT+ out; PGOOD to MCU |
| U9  | LTC2954-1 pushbutton power controller | INT/KILL/ENA; PB on H3 pin 4 (PWR_TOG) |
| LED1| WS2812B-MINI | DIN from P0.31; power gated by Q1/Q2 from ENA |
| BUZZ| 4 kHz buzzer | BUZZ1/BUZZ2 across two GPIOs |
| H2  | SWD header | GND, 3V3NRF, GND, SWDCLK, SWDIO |
| H3  | Button header 1x6 | GND, BUTTON4, BUTTON3, PWR_TOG, BUTTON2, BUTTON1 |
| CN1 | Battery JST | BAT+_RAW, GND (+2 unused) |
| USB2| USB-C | 5.1k CC pulldowns (proper USB-C sink) |

## Power rails

All 3.3 V rails are enabled by `ENA` from the LTC2954 (i.e. the device must
be "powered on" via the power button for anything downstream to work):

| Rail | LDO | Feeds |
|------|-----|-------|
| 3V3NRF    | U10 LP5907  | nRF module (VDD+VDDH), I2C pullups, KILL/INT pullups |
| 3V3SCA    | U4 LP5907   | SCA3300 only (clean analog supply) |
| 3V3RM3100 | U12 LP5907  | RM3100 only |
| OLEDPOWER | LDO2 ME6211 | OLED via CN2 |
| LZR_PWR_3V| LDO1 ME6211 | Laser module |
| BAT+      | BQ24074 SYS | LDO inputs, WS2812 (via Q1) |

Note: 3V3NRF is also ENA-gated (VBUS only powers the nRF's USB PHY, not the
core), and ENA has a 100k pullup to BAT+ (R21). Bench observation 2026-07-06:
the module enumerates and flashes over USB with just the battery/USB attached,
so ENA sits high in the default state.
