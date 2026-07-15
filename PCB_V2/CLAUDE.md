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
  leg_checker, math_utils) — needs a host C++ compiler. Each test-suite dir
  has an `arduino_stubs_build.cpp` shim pulling in the shared
  `test/support/arduino_stubs.cpp` (PlatformIO only compiles sources inside
  the suite's own folder — without the shim every suite fails to link).

## Architecture (what changed from V1)

Cooperative 10 ms super-loop in `src/main.cpp`, same poller structure as the
V1 main board. Modes (menu / calibration / snake) short-circuit the loop.

| Seam | V1 | V2 |
|------|----|----|
| BLE | UART bridge to DiscoX (`ble_manager` on SERCOM1) | `src/ble_manager.cpp` wraps `src/drivers/sap6_ble.*` in-process; same `BleCommand` dispatch in `pollBLECommands()`. Connection monitor + Coded-PHY dance live in `BleManager::update()` |
| Accelerometer | ISM330DHCX (I2C, accel+gyro, m/s²) | SCA3300 (SPI on dedicated `SPIClass(NRF_SPIM2, …)`, MODE_1 ±3 g, reports g → ×9.80665 at the read sites). **No gyro**: motion for the adaptive EMA + display freeze is derived from accel-vs-EMA deviation (`ACCEL_MOTION_THRESHOLD` in main.cpp — tune on hardware) |
| Laser | Egismos @ 9600 (`laser_egismos`) | Meskernel LDJ-100RED @ 115200: `src/laser_manager.cpp` presents the old `LaserError` surface over `drivers/ldj100`. Beeps are no longer the laser's job — **all UI sounds live in `src/sounds.cpp`** (per-event vocabulary: shot click, loud 4 kHz reading bleep, rising leg-complete fanfare, falling error womp; power on/off is deliberately silent) over `drivers/buzzer` (`tone` + `sweep` primitives, blocking bit-bang, piezo loudest at its 4 kHz resonance). `LaserManager::setBuzzer(true)` survives as a V1-compat shim (→ `Sounds::click()`) for calibration_mode; `setBuzzer(false)` = no-op |
| Storage | QSPI flash + FAT (SdFat) + USB-MSC drive mode | **Internal flash + LittleFS** (`InternalFileSystem`). Same file set: `/config.json`, `/calibration.{bin,json}`, `/cal_metrics.bin`, `/pending.txt`, `/flags/*`. USB drive mode is **back** (2026-07-09) via a 128 KB FAT12 partition carved out of internal flash — see "USB drive mode" section. Firmware update still via UF2 bootloader (double-tap-reset magic, see Gotchas), storage recovery via menu → Settings → Reformat Storage |
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

## USB drive mode (settings/calibration/readings on a PC)

Added 2026-07-09, exercised on hardware 2026-07-10 (the first run exposed a
USB re-enumeration race — `UsbDrive::beginMsc` now forces a detach/attach,
see the comment there). V1's QSPI USB drive is reproduced on internal
flash: a **128 KB FAT12 partition** at `0xCD000–0xED000`, exposed over
TinyUSB MSC as a drive named `MRZAPPY`.

Flash map (see `include/flash_layout.h`, enforced by
`linker/nrf52840_s140_v6_usbfat.ld` via `board_build.ldscript`):

```
0x00000-0x26000  SoftDevice S140
0x26000-0xCD000  application (668 KB cap — LINK FAILS if outgrown; 57.8% used)
0xCD000-0xED000  FAT12 USB partition (128 KB, survives UF2/DFU flashes)
0xED000-0xF4000  InternalFS LittleFS (core-fixed)
0xF4000-…        UF2 bootloader
```

Design: **LittleFS stays authoritative**; the FAT volume is a staging area
synced only at explicit points (avoids V1's FAT-as-primary fragility):

- **Entry**: menu → Settings → USB Drive Mode (sets `usb_drive` flag,
  reboots), or **hold DOWN (B3) at power-on** — the recovery route, needs no
  flash writes, lets a PC reformat a corrupt partition. On entry the MSC
  interface is registered before USB enumerates, then LittleFS → FAT staging:
  `CONFIG.JSON`, `CALIBRATION.JSON`, `READINGS.CSV` (pending legs,
  export-only), `README.TXT`.
- **Exit**: eject on the PC, press MENU → validated import → reboot. Plain
  power-off also works: the `usb_import` flag makes the next boot run the
  same import before `loadConfig()`/`initCalibration()`.
- **Import validation**: file must parse as JSON (calibration additionally
  needs `mag` + `grav` objects); a bad file is rejected and the old LittleFS
  copy kept. A calibration import also deletes `/calibration.bin` — the
  binary is preferred at boot and would silently shadow the imported JSON.

Implementation: `src/usb_drive.cpp` + `include/usb_drive.h`;
FatFs R0.15a vendored in `lib/fatfs` (`ffconf.h`: FAT12 mkfs, LFN, tiny,
no-RTC). Sector I/O for both FatFs and the MSC callbacks goes through the
core's SoftDevice-safe `flash_nrf5x` HAL (same one LittleFS uses) — its
single 4 KB page cache is shared, which is why host access is gated
(`setHostAccess`) and exit waits 500 ms before importing: MSC callbacks run
on the USB task and must never interleave with loop-task filesystem writes.

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
- **Boot-button roles differ**: FIRE held at power-on = serial-debug wait +
  I2C scan (pre-existing); **DOWN held at power-on = USB drive mode**. Don't
  reassign either without updating the drive-mode recovery docs.
- **Menu → Update Firmware must NOT use `enterUf2Dfu()`**: that GPREGRET
  (0x57) path gives USB only 3 s to enumerate before the bootloader falls
  back into the app — macOS's "Allow accessory to connect?" prompt outlasts
  it, so the device appeared to just restart. main.cpp instead writes the
  bootloader's double-tap-reset magic (`0x5A1AD5` → RAM `0x20007F7C`) and
  resets — the no-timeout DFU branch, waits until firmware arrives or the
  power button hard-kills. (Fixed 2026-07-15.)
- The linker script is project-local (`linker/nrf52840_s140_v6_usbfat.ld`).
  Removing the `board_build.ldscript` line silently lets the app grow over
  the FAT partition once it passes 668 KB — keep script, `flash_layout.h`,
  and `board_upload.maximum_size` in sync.

## ⚠ Commissioning still required

1. ~~Determine real `MAG_AXES`/`GRAV_AXES`~~ **done 2026-07-10**: V2
   mappings measured empirically (mag `+Y-X+Z`, grav `+Y-X-Z`) via raw-axis
   snapshots in three poses; config.h and the embedded `CALIBRATION_JSON`
   axes both updated. Still required: full on-device calibration (56-pt
   ellipsoid + 24-pt alignment + F/B check) — the embedded transform/centre
   data is still V1's, so `MagErr` and absolute-azimuth error persist until
   then.
   **Serial debug commands** (normal mode only — the menu/cal/snake loops
   short-circuit before the handler): `r` prints one `RAW mag … | acc …`
   line (chip-frame values before `Axes::fixAxes`, for reading the mapping
   off known poses); `s` toggles the `>azimuth`/`>inclination` teleplot
   stream; `c` dumps the stored `/config.json`; `f` resets filter tuning
   (EMA alphas + stability buffer) to firmware defaults and saves — needed
   because a stored config.json otherwise shadows new defaults forever.
   Note the axes strings are ALSO stored inside saved calibrations
   (`/calibration.{bin,json}`) — they override config.h at load. Boot serial
   prints `Mag axes:`/`Grav axes:` showing what was actually loaded; if a
   stale stored calibration shadows the new strings, re-calibrate
   (`calibration_mode.cpp` constructs from `MAG_AXES`/`GRAV_AXES`) or delete
   the stored files.
2. OLED electrically verified on hardware 2026-07-07 (I2C ACK + `begin()`,
   pattern cycle running) — this exposed that the V2 panel is at **0x3C**,
   not V1's 0x3D, so `SH1107_ADDR` (config.h) was corrected. Image since
   confirmed by eye (menus and live readings used throughout the 2026-07-07
   → 07-10 test passes). The per-IC buttons bring-up test in
   `../PCB_V2 test/` remains unrun — moot for the production firmware,
   whose buttons are verified.
3. Accel motion threshold (`ACCEL_MOTION_THRESHOLD`, main.cpp) and the EMA
   alphas need field tuning — V1 gated these on the gyro, which V2 lacks.
   First pass done on hardware: `emaAlphaStable` raised 0.05 → 0.15 (the
   SCA3300/RM3100 pair is much quieter than V1's IMU) and V1's motion-gated
   display freeze/anchor clamp removed in favour of a plain 0.10° deadband
   (`updateDisplay`, main.cpp). Confirm underground with real survey legs.
4. BLE needs a phone run (SexyTopo / nRF Connect): 17-byte legs, ACK/seq
   bit, commands, Coded PHY on Android (iOS = 1 Mbps, normal).
5. SCA3300 runs in MODE_1 (±3 g) so disco shake detection (11 m/s²) doesn't
   clip; revisit MODE_4 (low noise) if shake detection is retired.
6. ~~USB drive mode hardware test~~ **done 2026-07-10** — exercised on
   hardware as part of the full-device test pass (first run needed the
   re-enumeration fix now in `UsbDrive::beginMsc`).

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
- 2026-07-09: per-event sound vocabulary added (`src/sounds.cpp`); boot and
  shutdown chirps were tried and removed — power on/off is silent by choice.
- 2026-07-09: **USB drive mode added** (128 KB FAT12 partition on internal
  flash + TinyUSB MSC — see the "USB drive mode" section). Builds clean
  (RAM 10.4%, flash 57.8% of the new 668 KB app cap).
- 2026-07-10: **axis mappings determined** (mag `+Y-X+Z`, grav `+Y-X-Z` —
  commissioning item #1) and the remaining on-device features exercised in a
  full test pass: buzzer sound vocabulary, disco/WS2812 (the temporary LED
  self-test in setup() has been removed), snake, USB drive mode, filter
  retuning (`emaAlphaStable` 0.15, display deadband 0.10° replacing the V1
  anchor clamp), and the `+` sign on positive inclination.
- Still outstanding: **full on-device calibration** (embedded transform/
  centre data is still V1's — `MagErr` persists until then) and the **BLE
  phone run** (commissioning item #4).
