# Mr Zappy PCB V2 — production firmware (single nRF52840)

Merged port of the V1 two-board system onto the Raytac MDBT50Q-U1MV2 module:
everything from `../Main board C++/` (sensors, display, laser, survey logic,
calibration) plus the SAP6 BLE protocol from `../DiscoX C++ BLE/` now runs
in-process on one MCU. The V1 UART bridge (SERCOM1, DRDY handshake,
`COMPASS:`/`ALIVE`/`NAME:` lines, READY/ACK strings) is gone.

Hardware bring-up tests, the decoded netlist, and datasheets live in
**`../PCB_V2 test/`** — its CLAUDE.md has the per-IC verification status.
The V1 projects remain the reference implementation this was ported from.

## Build / flash / monitor

```bash
cd PCB_V2
pio run -t upload          # env pcb_v2 is the default — USB DFU, port auto-detected
pio device monitor -b 115200
```

- Board enumerates as **VID 239A** (Adafruit bootloader); COM number drifts
  between flashes — auto-detect, never hard-code.
- Upload fails with PermissionError if anything holds the COM port (close
  serial monitors, check for stale `python` processes).
- `board = nrf52840_dk_adafruit` (pca10056 variant): Arduino pin N maps 1:1
  to nRF GPIO. `include/pins_v2.h` (copied verbatim from the bring-up
  project) relies on this — **never** switch to an ItsyBitsy-based variant.
- `pio test -e native` runs the Unity unit tests (button_manager,
  leg_checker, math_utils) — needs a host g++, which this PC does not have
  installed; the env is carried over from V1 for machines that do.

## Architecture (what changed from V1)

Cooperative 10 ms super-loop in `src/main.cpp`, same poller structure as the
V1 main board. Modes (menu / calibration / snake) short-circuit the loop.

| Seam | V1 | V2 |
|------|----|----|
| BLE | UART bridge to DiscoX (`ble_manager` on SERCOM1) | `src/ble_manager.cpp` wraps `src/drivers/sap6_ble.*` in-process; same `BleCommand` dispatch in `pollBLECommands()`. Connection monitor + Coded-PHY dance live in `BleManager::update()` |
| Accelerometer | ISM330DHCX (I2C, accel+gyro, m/s²) | SCA3300 (SPI on dedicated `SPIClass(NRF_SPIM2, …)`, MODE_1 ±3 g, reports g → ×9.80665 at the read sites). **No gyro**: motion for the adaptive EMA + display freeze is derived from accel-vs-EMA deviation (`ACCEL_MOTION_THRESHOLD` in main.cpp — tune on hardware) |
| Laser | Egismos @ 9600 (`laser_egismos`) | Meskernel LDJ-100RED @ 115200: `src/laser_manager.cpp` presents the old `LaserError` surface over `drivers/ldj100`; **beeps go to the on-board piezo** (`drivers/buzzer`), not the laser module. `setBuzzer(true)` = one short blocking beep, `setBuzzer(false)` = no-op |
| Storage | QSPI flash + FAT (SdFat) + USB-MSC drive mode | **Internal flash + LittleFS** (`InternalFileSystem`). Same file set: `/config.json`, `/calibration.{bin,json}`, `/cal_metrics.bin`, `/pending.txt`, `/flags/*`. USB drive mode is **gone** (no QSPI chip); firmware update still via UF2 bootloader (`enterUf2Dfu()`), storage recovery via menu → Settings → Reformat Storage |
| Buttons | 5 GPIO buttons | 4 GPIO buttons + hardware power toggle (LTC2954). Enum: `FIRE, UP_DISCO, DOWN, MENU` |
| Power off | SHUTDOWN button GPIO + LTC2952 PIN_POWER | LTC2954: `pollPowerButton()` watches PB_INT (edge-triggered, 20 ms confirm, boot-hold guard), `doShutdown()` → `power.powerOff()` drives KILL LOW. `systemPowerOff()` (declared in config.h) is the hook for menu/snake timeout paths |
| RGB LED | NeoPixel + power-gate pin | WS2812 on P0.31, rail hardware-gated by ENA (no power pin) |
| BLE name | `NAME:` UART sync w/ retry | Set locally at boot from `config.json` (`ble_name` key, `SAP6_` prefix auto-applied); menu rename restarts advertising |

## Button roles per mode

| Mode | FIRE (B1) | UP_DISCO (B2) | DOWN (B3) | MENU (B4) |
|------|-----------|---------------|-----------|-----------|
| Normal | take shot / wake laser | hold: disco toggle; short: splay shot | — | enter settings menu |
| Menu | select | up | down | select |
| Calibration | record point / F-B shot | finish F/B early; hold combos: save (FIRE+UP) / discard (UP) | any-button advance | undo last point |
| Snake | turn left | turn right | exit | exit |

Power on/off = the dedicated hardware button into the LTC2954 (H3 pin 4,
PWR_TOG). Holding it long enough hard-kills via the LTC2954 itself even if
firmware hangs.

## Gotchas (inherited + new)

- **`PIN_BUTTON1..4` collide with the pca10056 variant** (the DK's on-board
  buttons at pins 11/12/24/25). `config.h` therefore includes `<Arduino.h>`
  **before** `pins_v2.h`, and `pins_v2.h` `#undef`s the four names before
  redefining them to this board's pins (27/35/37/39). Do not reorder those
  includes — with `pins_v2.h` first, the variant's values win in whatever
  translation unit pulls `config.h` before any Arduino header (this silently
  made `button_manager.cpp` read the DK's buttons; fixed 2026-07-07).
- **I2C pins are not variant defaults**: `Wire.setPins(PIN_I2C_SDA, PIN_I2C_SCL)`
  runs before `Wire.begin()` in setup() — don't reorder.
- **Coded PHY needs the patched global Bluefruit library**
  (`BLEConnection.cpp` ~line 392 answers PHY update requests with
  `BLE_GAP_PHY_CODED`, see "BLE Long Range" in `../DiscoX C++ BLE/CLAUDE.md`).
  A platform/framework update reverts it silently — if a previously-Coded
  phone reports 1 Mbps, check there FIRST.
- `Bluefruit.configPrphConn(…, 24, …)` must run **before** `Bluefruit.begin()`
  (done inside `BleManager::begin()`); event length < 24 makes the PHY update
  fail with 0x0013 NRF_ERROR_RESOURCES.
- **`Uart::end()` hangs if the UART was never begun** — initLaser() only ever
  begins Serial1 once; keep it that way.
- **Laser probe must be retried at boot.** On a cold power-button boot the
  ENA-gated LDJ-100 powers up with the MCU and spends ~2.5 s in an auto-baud
  window where it won't answer a fixed-baud read. `initLaser()` retries the
  probe (~15×) instead of a single early ping — a single ping latches
  `laserOk=false` for the whole session, and the symptom is FIRE only ever
  printing "getting ready for a shot" and never measuring. (Warm USB-reflash
  resets hide this because the module is already awake.) Don't revert to a
  one-shot ping.
- **FIRE is a two-press trigger**: first press wakes the laser to aim
  (`prepareForShot`, `laserEnabled=false→true`), second press measures
  (`startShot`). Entering the menu turns the laser off, so the first FIRE
  after exiting the menu is always a wake press.
- Laser UART: RX (P0.25) needs `INPUT_PULLUP` (module TXD is open drain);
  TX/RX net names in the netlist are from the laser's perspective.
- `<ArduinoEigenDense.h>`, never `<ArduinoEigen.h>` (SparseLU macro clash).
- LittleFS `FILE_O_WRITE` does **not** truncate — remove the file first
  (config_manager does this everywhere).
- Flash writes are deferred to IDLE (RAM-buffered pending readings) — a
  SoftDevice flash op can block for a few ms; don't write mid-measurement.
- KILL (P0.04) stays high-Z (INPUT) until shutdown; driving it LOW is
  irreversible until the next power-button press.
- MAX17048 SOC can read >100% on a fresh cell (ModelGauge seeding) — V1's
  no-reset `MAX17048_Persistent` (drivers/max17048.h) is used; consider
  clamping the displayed percentage.
- WS2812 DIN is on a "low-frequency-only" module pad (P0.31) — watch for RF
  degradation while BLE is active.

## ⚠ Commissioning still required (firmware builds, hardware unproven)

1. `MAG_AXES` / `GRAV_AXES` in `include/config.h` are **V1 placeholder
   values** — the SCA3300 replaces the ISM330DHCX and sensor orientations
   differ. Determine real mappings from streamed raw data, then run a full
   on-device calibration (56-pt ellipsoid + 24-pt alignment + F/B check).
   The embedded `CALIBRATION_JSON` fallback in main.cpp is V1 data.
2. OLED electrically verified on hardware 2026-07-07 (I2C ACK + `begin()`,
   pattern cycle running) — this exposed that the V2 panel is at **0x3C**,
   not V1's 0x3D, so `SH1107_ADDR` (config.h) was corrected. Still needs a
   visual by-eye confirm of the image, and the buttons bring-up test is
   still unrun on hardware (`../PCB_V2 test/` status table).
3. Accel motion threshold (`ACCEL_MOTION_THRESHOLD`, main.cpp) and the EMA
   alphas need field tuning — V1 gated these on the gyro, which V2 lacks.
4. BLE needs a phone run (SexyTopo / nRF Connect): 17-byte legs, ACK/seq
   bit, commands, Coded PHY on Android (iOS = 1 Mbps, normal).
5. SCA3300 runs in MODE_1 (±3 g) so disco shake detection (11 m/s²) doesn't
   clip; revisit MODE_4 (low noise) if shake detection is retired.

## Status

- 2026-07-07: initial merge complete — builds clean (RAM 8.7%, flash 45.2%).
- 2026-07-07: **first hardware bring-up of the merged firmware PASSED the core
  spine.** Flashed via USB DFU; serial shows the loop running with live,
  stable sensor fusion (`>azimuth:176.8 >inclination:0.5` at ~4 Hz — RM3100
  over I2C + SCA3300 over SPIM2, converted to m/s², through the ported
  calibration pipeline) and the battery gauge (`BAT: 4.145V 76.0%`). OLED
  init succeeds at 0x3C, so `dispOk` is true.
- 2026-07-07: **buttons, laser measurement, and menu verified on hardware**
  after fixing the `PIN_BUTTON1..4` variant collision (see Gotchas). FIRE
  runs the full measurement pipeline (laser returns real distances, e.g.
  187/725/778 mm); MENU opens/exits the settings menu.
- 2026-07-07: fixed a **cold-boot laser init failure** (single early ping →
  `laserOk=false` → FIRE never measures; now retried past the module's
  auto-baud window — see Gotchas). Verified on a cold power-button boot: full
  two-press measurement `MEAS OK: AZ=180.6 INC=0.2 DIST=0.32`, and the
  offline path queued the leg to flash (BLE not connected). Measurements may
  report `anomaly: MagErr` — expected, the calibration is still the V1
  placeholder (commissioning item #1), not a firmware fault.
- Still unexercised on hardware: buzzer beeps (audible), disco animation,
  calibration/snake UI flows, and the BLE phone link — plus the full
  commissioning checklist above (real axis mappings + calibration is the
  big one).
