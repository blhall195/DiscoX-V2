# Mr Zappy PCB V2 — single-MCU board bring-up

V2 of the cave survey device replaces the two-MCU design (Feather M4 CAN +
ItsyBitsy nRF52840) with **one Raytac MDBT50Q-U1MV2 module (nRF52840)** that
runs everything: sensors, display, laser, buttons, power management, and BLE.

This folder holds the per-IC hardware bring-up tests (in `bringup/`) and the
hardware documentation. The **merged production firmware now lives in
`../PCB_V2/`** (ported from `../Main board C++/` and `../DiscoX C++ BLE/`) —
this folder stays as the frozen per-IC test reference: come back here to
isolate a suspected hardware fault to a single IC.

## Folder structure

| Path | Purpose |
|------|---------|
| `bringup/` | **PlatformIO project** — per-IC hardware tests. One env per IC, shared drivers. This is where new IC tests go. |
| `bringup/include/pins_v2.h` | Canonical V2 pin definitions (all peripherals) — copied verbatim into `../PCB_V2/include/` |
| `bringup/src/drivers/` | Reusable drivers (copies shared with `../PCB_V2/src/drivers/`) |
| `bringup/src/tests/test_<ic>.cpp` | One standalone test program per IC |
| `hardware/NETLIST.md` | **Decoded netlist** — every net, module pin ↔ nRF GPIO table, power rails. Read this instead of re-parsing the JSON. |
| `hardware/PCB_V2-PCB_2026-07-06.json` | Raw EasyEDA PCB export (source of NETLIST.md) |
| `hardware/datasheets/` | IC datasheets (sca3300.pdf, ...) |

(The old `firmware/` subfolder — a stale copy of the V1 DiscoX code targeting
the wrong pin variant — was deleted when the production firmware moved to
`../PCB_V2/`.)

## How to flash and test (the board is on this PC via USB)

The module already has the **Adafruit nRF52840 UF2/DFU bootloader** burned
(done 2026-07-06 via SWD header H2). Day-to-day flashing is USB DFU:

```bash
cd "PCB_V2 test/bringup"
pio run -e sca3300_test -t upload    # build + flash (port auto-detected)
pio device monitor -b 115200         # watch output
```

- The board enumerates as **VID 239A** (Adafruit). The COM number drifts
  between flashes (COM103/104/105... seen so far) — always auto-detect or
  list with `pio device list`, never hard-code it.
- `board_upload.use_1200bps_touch = true` in platformio.ini makes upload
  reset a *running* app into the bootloader automatically. Without it upload
  fails with "Target is not in DFU mode" (and may still print SUCCESS — check
  for the `Device programmed.` line).
- **Upload fails with PermissionError if anything holds the COM port open**
  (a serial monitor, a stale python process). Close monitors before upload;
  on Windows check `Get-Process python*` for zombies.
- Test programs print their PASS/FAIL report at boot, which is easy to miss
  while USB re-enumerates. **Press any key in the serial monitor to re-run
  the full test sequence** — every test file implements this.
- To capture output programmatically: pyserial, find port by `vid == 0x239A`,
  115200 baud, write one byte to trigger a re-run, read ~15 s.
- Factory-fresh module (only if replaced): burn the Adafruit bootloader once
  via SWD header H2 with a J-Link (`upload_protocol = jlink`), which also
  installs SoftDevice S140 that the Arduino core links against.

## Adding a new IC test (the established pattern)

1. Driver → `bringup/src/drivers/<ic>.h/.cpp` (class style matches
   `../Main board C++/` — see rm3100.h; verify protocol against the
   datasheet in `hardware/datasheets/`, add it there if missing).
2. Test → `bringup/src/tests/test_<ic>.cpp` with: `[PASS]/[FAIL]` report()
   pattern, keypress re-run in loop(), then a live streaming mode.
3. Env → add to `bringup/platformio.ini`:
   `[env:<ic>_test]` + `build_src_filter = +<drivers/> +<tests/test_<ic>.cpp>`
4. Pins → already in `bringup/include/pins_v2.h`; cross-check against
   `hardware/NETLIST.md` if in doubt.
5. Flash, capture serial, record the result in the status table below.

## Board facts that matter for code

- **Board def**: `nrf52840_dk_adafruit` (pca10056 variant) — Arduino pin N
  maps 1:1 to nRF GPIO (P0.xx = xx, P1.yy = 32+yy). `pins_v2.h` relies on this.
- **I2C** (RM3100 0x20, MAX17048 0x36, OLED): SDA=P0.26, SCL=P0.05. These are
  NOT the variant defaults — call `Wire.setPins(PIN_I2C_SDA, PIN_I2C_SCL)`
  **before** `Wire.begin()`.
- **SCA3300 SPI**: dedicated `SPIClass(NRF_SPIM2, miso, sck, mosi)` instance
  (SPIM3 is the core's default `SPI`, SPIM0/1 overlap TWI/UART peripherals).
- **Power**: all peripheral rails (sensors, OLED, laser) are LDO-gated by
  `ENA` from the LTC2954 power-button controller. Drive `PIN_KILL` LOW to
  power off; `PIN_PB_INT` goes LOW on a power-button press. `PIN_PGOOD` is
  open-drain (INPUT_PULLUP).
- **Buttons** 1-4: active LOW, external header H3, use INPUT_PULLUP.
- **Buzzer**: two GPIOs push-pull (BUZZ1=P0.20, BUZZ2=P0.24), 4 kHz element —
  drive them in antiphase for max volume.
- **Laser UART**: Meskernel LDJ-100RED (not the V1 Egismos!) at 115200 8N1,
  JRT-family register protocol — driver `bringup/src/drivers/ldj100.h/.cpp`,
  datasheet `hardware/datasheets/LDJ-100RED.pdf`. nRF TX=P1.00, RX=P0.25:
  the netlist TX/RX net names are from the **laser's** perspective (verified
  on hardware). Module TXD is open drain — RX pin needs INPUT_PULLUP. Rail
  is ENA-gated, so the module's 2.5 s auto-baud window expires at board
  power-on; it sits at fixed 115200.
- **WS2812** DIN is on P0.31 (module pin 12), a "low-frequency-only" pad per
  Raytac — may degrade RF when BLE is active; watch for it.

## Gotchas / history

- **Do not use ItsyBitsy-based board defs or variants for V2 code.** The old
  `raytech_nrf52840.json` (deleted with the stale `firmware/` folder) extended
  the ItsyBitsy variant whose pin map contradicts the V2 netlist — e.g. its
  Wire pins were P0.16/P0.14, which on this PCB are the SCA3300's MISO/CSB.
  Both this project and `../PCB_V2/` use `nrf52840_dk_adafruit`.
- SCA3300 driver details (off-frame SPI protocol, CRC-8, start-up sequence)
  were verified against `hardware/datasheets/sca3300.pdf`, CRC against all
  18 datasheet example frames, then on real hardware.
- **`Uart::end()` hangs forever if the UART was never begun** (core busy-waits
  for TXSTOPPED/RXTO events that can't fire while the peripheral is disabled).
  Guard the first `Serial1.end()` — see `beginLaserUart()` in test_ldj100.cpp.
- LDJ-100 measurement failures come back as a **negative int16** status code
  in a 0xEE-headed frame (e.g. 0xFFFB = -5 = target out of range); the
  positive codes in datasheet table 6-1 use the same numbering
  (`LDJ100::statusText`). It reports -5 when aimed at nothing measurable.

## Bring-up status

| IC / feature | Env | Status |
|--------------|-----|--------|
| SCA3300 accelerometer | `sca3300_test` | ✅ **PASS 7/7 on hardware** (2026-07-06): WHOAMI 0x51, serial 2892520544B33, \|g\| = 0.9998, noise 1.7 mg RMS |
| RM3100 magnetometer | `rm3100_test` | ✅ **PASS 4/4 on hardware** (2026-07-06): I2C ACK 0x20, init CC 400, \|B\| = 39.4 uT, noise ~0.03 uT p-p at 10 Hz |
| MAX17048 battery gauge | `max17048_test` | ✅ **Hardware PASS** (2026-07-07): I2C ACK 0x36, no-reset begin(), voltage accurate (4.18 V on a near-full LiPo), charge-rate reads. ⚠️ **SOC reads high (~115%)** on this fresh cell — a ModelGauge accuracy matter, NOT a wiring fault (voltage is correct). `quickStart()` did NOT re-anchor it (press 'q' in the test), so it's the default voltage→SOC model, not a transient. TODO for production firmware: custom ModelGauge model or clamp-and-converge; V1 lesson (skip reset() to keep learned state) already baked into `drivers/max17048.h`. Note: gauge rail is ENA-gated, so it POR-resets each power-cycle (won't track SOC while off) |
| SH1107 OLED | `oled_test` | ✅ **Electrical PASS 2/2 on hardware** (2026-07-07): I2C ACK + `begin()` init, pattern cycle (all-on, border+X, checkerboard, text+shapes) running — **needs visual confirmation by eye**. ⚠️ **Address is 0x3C, not 0x3D** (this panel straps SA0 low; V1's SH1107 was 0x3D) — found via the built-in I2C scan, which also confirmed RM3100 0x20 + MAX17048 0x36 on the same bus. `setRotation(2)` for mounting; OLEDPOWER is ENA-gated (power the board via the button). Dark after begin() PASS = OLEDPOWER/CN2 ribbon; shifted/split = geometry/ribbon |
| Buttons 1-4 | `buttons_test` | Test written — boot pull-up self-test + live press/release mode; **needs hardware run** (H3 header, active LOW, INPUT_PULLUP) |
| Buzzer | `buzzer_test` | Electrical 1/1 PASS (2026-07-07): BUZZ1 P0.20 / BUZZ2 P0.24 idle LOW, antiphase driver + sweep/beep sequence running — **needs audible confirmation** (output-only; loudest near 4 kHz = OK, silent = BUZZ net / ENA rail / dead element) |
| WS2812B LED | `ws2812_test` | Electrical 2/2 PASS, colour cycle running (2026-07-06) — **needs visual confirmation** (output-only part; wrong colours = not NEO_GRB, dark = DIN/rail) |
| Laser (Meskernel LDJ-100RED) | `ldj100_test` | ✅ **PASS 6/6 on hardware** (2026-07-06): 115200 on TX=P1.00/RX=P0.25, HW 0xB338 SW 0x53FE, 3272 mV, laser on/off, 3.190 m single + continuous stream |
| Power mgmt (KILL/INT/PGOOD) | `power_test` | ✅ **PASS on hardware** (2026-07-07): self-test 2/2 (INT/KILL idle HIGH, PGOOD LOW on USB); power-button press → beep → KILL LOW → board off, confirmed by user. **INT pulses LOW (<1 s) per press — it does NOT stay low while held**, so shutdown is edge-triggered (armed LOW edge + 20 ms confirm), matching V1's INT-falling-edge interrupt. KILL kept high-Z until shutdown (external 10k pullup holds rail on); boot-hold guard arms only after INT first seen HIGH so the power-on press can't immediately power back off |
| BLE (SAP6 protocol) | `ble_test` | Test written, builds clean — SAP6 ported to `drivers/sap6_ble.*` + full DiscoX long-range recipe (event len 24, Coded PHY retry, per-conn +8 dBm) + new: 6 s supervision timeout, PHY fallback watchdog, RSSI status; buttons 1-3 send fake legs, 4 = status dump. **Needs phone run** (Android + nRF Connect for Coded PHY; iOS = 1 Mbps only). ⚠ Coded PHY needs the global Bluefruit `BLEConnection.cpp` patch (see DiscoX CLAUDE.md) — framework updates revert it silently (verified present 2026-07-07) |
| Production firmware merge | — | ✅ **Done 2026-07-07** — merged firmware lives in `../PCB_V2/` (builds clean; hardware verification per its CLAUDE.md) |
