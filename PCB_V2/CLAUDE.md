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
2. OLED and buttons bring-up tests were still unrun on hardware at merge
   time (`../PCB_V2 test/` status table).
3. Accel motion threshold (`ACCEL_MOTION_THRESHOLD`, main.cpp) and the EMA
   alphas need field tuning — V1 gated these on the gyro, which V2 lacks.
4. BLE needs a phone run (SexyTopo / nRF Connect): 17-byte legs, ACK/seq
   bit, commands, Coded PHY on Android (iOS = 1 Mbps, normal).
5. SCA3300 runs in MODE_1 (±3 g) so disco shake detection (11 m/s²) doesn't
   clip; revisit MODE_4 (low noise) if shake detection is retired.

## Status

- 2026-07-07: initial merge complete — builds clean
  (RAM 8.7%, flash 45.2%). Not yet verified on hardware.
