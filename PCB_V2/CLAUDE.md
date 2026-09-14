# Mr Zappy PCB V2 — production firmware (single nRF52840)

Merged port of the V1 two-board system onto the Raytac MDBT50Q-U1MV2 module:
everything from the V1 main board (sensors, display, laser, survey logic,
calibration) plus the SAP6 BLE protocol from the V1 DiscoX BLE board now runs
in-process on one MCU. The V1 UART bridge (SERCOM1, DRDY handshake,
`COMPASS:`/`ALIVE`/`NAME:` lines, READY/ACK strings) is gone.

Hardware bring-up tests, the decoded netlist, and datasheets live in
**`../PCB_V2 test/`** — its CLAUDE.md has the per-IC verification status.
The V1 firmware this was ported from lives in the original repo:
https://github.com/blhall195/Mr_Zappy

## Build / flash / monitor

```bash
cd PCB_V2
pio run -t upload          # env pcb_v2 is the default — USB DFU, port auto-detected
pio device monitor -b 115200
```

- `pio run` also emits **`.pio/build/pcb_v2/firmware.uf2`** (post-build step
  `tools/make_uf2.py`) — the image to hand end users for a double-tap-reset
  drag-and-drop update. It covers only the application region
  (0x26000-0x8C200 as of 2026-09), so the FAT settings partition and the
  LittleFS store holding `config.json` survive the update untouched;
  new settings added by the update are then migrated in on first boot (see
  "Settings migration" below). Development flashing still uses USB DFU.
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

### First flash of a blank/new nRF52840 module (SWD bootloader burn)

A brand-new module has no bootloader, so it never enumerates over USB —
`pio run -t upload` (USB DFU) has nothing to talk to. The user's probe for
this is a **Raspberry Pi Pico running debugprobe firmware** (a CMSIS-DAP
SWD probe); they call it "the J-Link" but it isn't a SEGGER unit — it
enumerates as `VID_2E8A&PID_000C`, "Debugprobe on Pico (CMSIS-DAP)", not
`VID_1366`. Flashing tool is `pyocd` (installed at
`C:\Users\Brendans-PC\AppData\Local\Programs\Python\Python314\Scripts\pyocd.exe`,
not on PATH — call by full path or `where pyocd`).

1. Confirm the probe is connected and can see the target chip:
   ```
   pyocd list
   pyocd reset -t nrf52840 -v      # should print DP IDR / AHB-AP / "This appears to be an nRF52840..."
   ```
2. Burn the bootloader + SoftDevice (bundled combo hex, ships inside the
   PlatformIO framework package — matches `board = nrf52840_dk_adafruit`
   i.e. the `pca10056` variant):
   ```
   pyocd flash -t nrf52840 "C:\Users\Brendans-PC\.platformio\packages\framework-arduinoadafruitnrf52\bootloader\pca10056\pca10056_bootloader-0.6.2_s140_6.1.1.hex"
   pyocd reset -t nrf52840
   ```
3. SWD's job is now done — the SWD lines alone do **not** put the module on
   the USB bus. Plug the module's **own USB port** into the PC (separately
   from/instead of the probe). With no application present it boots
   straight into the bootloader and enumerates as VID 239A.
4. From here it's the normal path: `pio run -t upload` (USB DFU) flashes
   the actual `PCB_V2` application. Don't flash the app over SWD — USB DFU
   is the documented, tested path for every update after the initial
   bootloader burn.

This only applies to a blank chip. Re-flashing a board that already has the
bootloader (the normal case) should always go through USB DFU per the `u`
serial command / clean-bootloader-entry procedure — SWD skips that
storage-safe shutdown and risks corrupting LittleFS on a board with real
data on it.

## Architecture (what changed from V1)

Cooperative 10 ms super-loop in `src/main.cpp`, same poller structure as the
V1 main board. Modes (menu / calibration / snake) short-circuit the loop.

| Seam | V1 | V2 |
|------|----|----|
| BLE | UART bridge to DiscoX (`ble_manager` on SERCOM1) | `src/ble_manager.cpp` wraps `src/drivers/sap6_ble.*` in-process; same `BleCommand` dispatch in `pollBLECommands()`. Connection monitor + Coded-PHY dance live in `BleManager::update()`. Outbound legs go through the delivery drain — see "Reading delivery" below |
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
| Normal | take shot / wake laser | hold: disco toggle; short: splay shot | laser off (power saver; FIRE wakes it again). Disco on: Mario on/off | enter settings menu |
| Menu | select | up | down | select |
| Calibration | record point / F-B shot | finish F/B early; hold combos: save (FIRE+UP) / discard (UP) | undo last point / cancel capture (screen hint "B3:undo") | undo (alias of DOWN); any-button advance on intro screens |
| Snake | — | turn right | turn left | exit |

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

## Reading delivery (2026-09-14)

**Invariant: a reading is only removed from storage once the phone has ACKed
it.** Nothing else in the firmware may delete a pending reading on the strength
of having sent it.

Every measurement goes to `configMgr.appendPendingReading()` — there is no
longer a connected/disconnected fork in `handleMeasurementSuccess()`. The
pending store is the *only* send queue: `pollBLEDrain()` (main.cpp) hands SAP6
one leg at a time and advances the delivery cursor only after
`ble.pendingReadings() == 0`, which means queued *and* in-flight are both clear.

The cursor (`drainOffset_` in config_manager) is the byte offset of the oldest
undelivered line in `/pending.txt`; the cycle is `peekOldestPending()` → send →
(ACK) → `commitOldestPending()`, and the file is deleted by
`clearDeliveredPending()` only after the cursor is *verified* to have reached
EOF. It is RAM-only on purpose: a reboot mid-drain replays delivered legs rather
than risking undelivered ones — a duplicate is visible in the survey app, a
missing leg is not.

What this replaced, and why none of it should come back:

- Readings taken **while connected** were never persisted at all. The leg lived
  only in SAP6's RAM queue, so a dropout before the ACK lost it silently while
  the device showed it as sent. This was the field bug: readings taken, shown on
  the device, never arriving in SexyTopo.
- Reconnect ran a **blocking** flush that queued the whole file in one pass.
  Past the 20-slot queue the surplus was dropped with no error, ACKs could not
  be processed (the loop was blocked, so only one leg actually went out), and
  `/pending.txt` was then deleted regardless.
- `notify()`'s return was ignored, so a refused send (phone not re-subscribed
  yet) armed the 5 s ACK timer as though it had worked.

`ble.sendSurveyData()` now has exactly one caller, `pollBLEDrain()`. Keep it
that way — a second sender reintroduces the "sent but unrecorded" window.

Degraded path: if the RAM buffer cannot reach flash (unmounted, or the zombie
filesystem that mounts and reads but refuses every commit), the drain serves
readings straight from RAM rather than stranding them. That path is reachable
only when `syncPendingToFlash()` fails.

## Gotchas (inherited + new)

- **LDJ-100 signal quality is inverted from its own manual.** The manual says
  "the smaller the SQ value, the stronger the laser signal"; bench testing
  (2026-09-06) shows the **opposite** — a white surface up close reads a few
  hundred, a black/specular surface reads 4-6. SQ behaves like a return
  amplitude. The rejection gate in `LaserManager::measureValidated()`
  therefore rejects shots with SQ **below** `laser_sq_limit`. Do not
  "correct" that comparison to match the PDF — it would disable rejection of
  exactly the dark/specular targets it exists to catch. Note SQ also falls
  with distance, so a limit tuned up close will reject legitimate long shots;
  verify at survey range. Every shot logs `LZRSQ shot= mm= sq= st=` to serial
  for threshold calibration. Background: `discox-sq-rejection-brief.md`
  (note that brief repeats the manual's inverted claim).

- **Settings migration on firmware update.** `loadConfig()` compares the
  stored `/config.json` against `kExpectedKeys` (config_manager.cpp); if the
  firmware has gained settings since the file was written, boot logs
  `Firmware update added settings — migrating config` and re-saves, so new
  keys appear on the USB drive with their defaults and existing values are
  preserved. **`kExpectedKeys` must stay exactly in sync with the `doc[...]`
  keys in `saveConfig()`** — a key listed but never written makes every boot
  re-save the file (needless flash wear); a key written but not listed simply
  won't trigger migration.

- **FatFs caches sectors across the host's writes.** `importFiles()` force-
  remounts the FAT volume (`f_mount(nullptr,...)` then `mountOrFormat()`)
  before reading. Without this the host's edits are invisible behind stale
  cached FAT/directory sectors and imports fail with "CONFIG.JSON
  unreadable/too large" or silently import old content (fixed 2026-09-06).

- **`PIN_BUTTON1..4` collide with the pca10056 variant** (the DK's on-board
  buttons at pins 11/12/24/25). `config.h` therefore includes `<Arduino.h>`
  **before** `pins_v2.h`, and `pins_v2.h` `#undef`s the four names before
  redefining them to this board's pins (27/35/37/39). Do not reorder those
  includes — with `pins_v2.h` first, the variant's values win in whatever
  translation unit pulls `config.h` before any Arduino header (this silently
  made `button_manager.cpp` read the DK's buttons; fixed 2026-07-07).
- **I2C pins are not variant defaults**: `Wire.setPins(PIN_I2C_SDA, PIN_I2C_SCL)`
  runs before `Wire.begin()` in setup() — don't reorder.
- **Coded PHY needs the patched global Bluefruit library**: in
  `~/.platformio/packages/framework-arduinoadafruitnrf52/libraries/Bluefruit52Lib/src/BLEConnection.cpp`
  (~line 392), the `BLE_GAP_EVT_PHY_UPDATE_REQUEST` handler must answer with
  `ble_gap_phys_t phy = { BLE_GAP_PHY_CODED, BLE_GAP_PHY_CODED };` instead of
  `BLE_GAP_PHY_AUTO` (AUTO negotiates down to 1 Mbps). A platform/framework
  update reverts it silently — if a previously-Coded phone reports 1 Mbps,
  check there FIRST.
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
  I2C scan (pre-existing); **DOWN held at power-on = USB drive mode**;
  **DOWN+MENU held at power-on = storage factory reset** (confirm screen,
  hold FIRE 3 s to erase — formats LittleFS without reading the old
  metadata, the only route that recovers from corruption that hangs the
  filesystem code itself). Don't reassign any of these without updating
  the recovery docs. These button checks run BEFORE the first
  `configMgr.begin()` in setup() — keep that order (see next gotcha).
- **The loop task has a 4 KB stack — never put ≥1 KB of locals in code it
  runs** (setup/loop and everything they call). `CalibrationMode::
  saveCalibration()` had 2.5 KB of locals; at the deepest LittleFS write
  (the metrics rename) it overflowed, smashed littlefs's heap buffers,
  and littlefs committed CRC-valid garbage metadata to flash. Every boot
  then hung at the first filesystem write (the storage self-test) walking
  a fake 4 GB file — bricked device, 2026-07-22, recovered by dumping the
  FS region over a rescue firmware. Overflow detection is FreeRTOS method
  1 with a hook that continues in release builds, so nothing crashes at
  the moment of overflow. Big buffers in this kind of code go in
  `static`/.bss (see saveCalibration, initCalibration, config_manager,
  MenuManager::testCalSave). `arm-none-eabi-objdump -d` on the .o and
  check `sub sp` prologues if unsure.
- **Menu → Update Firmware needs USB already connected**: the bootloader's
  `enterUf2Dfu()` GPREGRET (0x57) path gives USB only 3 s to enumerate
  before falling back into the app, so main.cpp waits for
  `TinyUSBDevice.mounted()` before resetting (MENU cancels). The
  bootloader's no-timeout DFU branch is NOT reachable from software — its
  double-tap magic (`0x5A1AD5` → RAM `0x20007F7C`) is gated on
  `RESETREAS & RESETPIN`, and `NVIC_SystemReset()` doesn't set that (tried
  2026-07-15, bounced straight back into the app). If a fresh macOS
  "Allow accessory to connect?" prompt still outlasts the 3 s window, the
  only full fix is rebuilding the bootloader with a longer timeout.
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
   because a stored config.json otherwise shadows new defaults forever;
   `p` prints delivery status (undelivered count, in-flight flag, SAP6
   sent/acked/resend/failed-send counters) — the view for verifying the
   reading drain on the bench.
   Note the axes strings are ALSO stored inside saved calibrations
   (`/calibration.{bin,json}`) — they override config.h at load. Boot serial
   prints `Mag axes:`/`Grav axes:` showing what was actually loaded; if a
   stale stored calibration shadows the new strings, re-calibrate
   (`calibration_mode.cpp` constructs from `MAG_AXES`/`GRAV_AXES`) or delete
   the stored files.

   **Buzzer test console** (2026-07-15, `handleBuzzerTestLine` in
   `main.cpp`): lines starting with `>` are buffered until `\n` and
   dispatched as buzzer commands — `>TONE f ms`, `>SWEEP f0 f1 ms`,
   `>MELODY f:ms,f:ms,...` (f=0 is a rest), `>SOUND name` (click, shot,
   reading, leg, warning, error, snakestart, snakeeat, snakecrash), `>STOP`.
   Kept behind the `>` prefix so it can't collide with the single-char `r`/
   `s`/`c`/`f` commands above. Each command replies `OK`/`ERR ...` on
   `Serial`.
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
   bit, commands, Coded PHY on Android (iOS = 1 Mbps, normal). Since
   2026-09-14 this must also prove the delivery drain (see "Reading
   delivery"), none of which has run on hardware yet:
   - a leg taken while connected, with the phone force-dropped before it
     ACKs, survives and arrives on reconnect;
   - a backlog well over 20 legs drains completely (the old flush silently
     dropped everything past 20);
   - `p` on serial shows undelivered reaching 0 and `/pending.txt` gone;
   - dropping the link while sitting in the **menu** still lets the phone
     find the device again (advertising restart used to be skipped there).
5. SCA3300 runs in MODE_1 (±3 g) so disco shake detection (11 m/s²) doesn't
   clip; revisit MODE_4 (low noise) if shake detection is retired.
6. ~~USB drive mode hardware test~~ **done 2026-07-10** — exercised on
   hardware as part of the full-device test pass (first run needed the
   re-enumeration fix now in `UsbDrive::beginMsc`).

## Status

- 2026-09-14: **reading delivery reworked so no leg can be lost to a dropout**
  (see "Reading delivery"). Readings taken while connected were never
  persisted; the reconnect flush dropped everything past 20 queued legs and
  deleted `/pending.txt` before any ACK. Also fixed: advertising was never
  restarted after a disconnect in menu or snake mode (device invisible until
  you exited), bonds were wiped on every disconnect rather than once per
  power-on, and a refused `notify()` was counted as a send. Builds clean
  (RAM 15.9%, flash 62.5%); host tests pass. **Not yet run on hardware** —
  commissioning item #4.
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
