# Mr Zappy — Cave Survey Device (C++ Firmware)

Handheld cave surveying instrument measuring azimuth, inclination, and laser distance.

## ⚡ PCB V2 (active work as of July 2026)

The board has been redesigned around a single Raytac MDBT50Q-U1MV2 (nRF52840)
module that replaces both V1 MCUs. Two folders:

- **`PCB_V2/`** — the merged production firmware (V1 main board + DiscoX BLE
  ported in-process; the UART bridge is gone). Running verified on hardware
  as of 2026-07-10: sensor fusion, buttons, laser measurement, menu, sounds,
  disco, USB drive mode; commissioning still pending (full calibration,
  BLE phone test). **Read `PCB_V2/CLAUDE.md` first when working
  on V2** — build/flash instructions (`pio run -t upload` from `PCB_V2/`,
  USB DFU, VID 239A), V1→V2 seam table, button roles, gotchas,
  commissioning checklist.
- **`PCB_V2 test/`** — frozen per-IC hardware bring-up tests plus the decoded
  netlist (`hardware/NETLIST.md`) and datasheets. Its CLAUDE.md has the
  per-IC verification status and the pattern for adding IC tests. Come back
  here to isolate a suspected hardware fault to one IC.

## V1 Reference Implementation

V1 is a dual-MCU architecture (SAMD51 main board + nRF52840 BLE board communicating via UART).
It is the reference the V2 port came from — still the proven, calibrated baseline.

- **`V1/Main board C++/CLAUDE.md`** — full V1 main board detail: build/upload, pin map, I2C
  addresses, session log, module status table, known issues, build gotchas
  (SAMD51 / Feather M4 CAN, PlatformIO, `adafruit_feather_m4_can`)
- **`V1/DiscoX C++ BLE/CLAUDE.md`** — V1 BLE board detail: SAP6 BLE protocol impl, Coded PHY
  setup + library patch, UART state machine, pin assignments, build/flash
  (ItsyBitsy nRF52840)

## Data Flow

```
Sensors → SensorManager (calibrate → EMA → stability check)
       → DisplayManager (live compass/incline/distance)
       → BleManager → [V1: UART → DiscoX →] BLE GATT → Phone app
```

**Measurement**: Press MEASURE → laser fires → wait for stability (azimuth + inclination within tolerance for 3 consecutive samples) → freeze display → send via BLE → wait for ACK.

## Key Data Structures (`device_context.h`)

Shared concept across V1 and V2 (V2 ported the same structs):

- **SystemState**: `IDLE`, `TAKING_MEASUREMENT`, `MENU`
- **Readings**: azimuth, inclination, roll, distance, batteryLevel
- **Config**: tolerances (mag/grav/dip/stability/leg), EMA alpha, laser offset (0.14m), timeouts
- **DeviceContext**: central state — current state, readings, config, flags (laser, disco, BLE, quickShot)

## Calibration System (`mag_cal/`)

Same math in V1 and V2. Ported from Python; uses Eigen for linear algebra.

- **Ellipsoid fitting**: 56 points → hard-iron centre + soft-iron 3x3 transform
- **Alignment**: 24 points at 3 orientations → rotates transform to gravity reference
- **RBF non-linear correction**: Gaussian radial basis functions per magnetometer axis
- **Anomaly detection**: field strength ±2%, dip angle ±3° triggers warnings
- **F/B field check**: Post-calibration correction for residual hard-iron from calibration
  environment. User takes 3+ foresight/backsight pairs. Sinusoidal model `a·sin(θ) + b·cos(θ)`
  solved via least-squares, back-projected to adjust `mag.centre_`. Menu → Enter Calibration →
  Field Check (F/B).

Note: V2 axis mappings determined empirically 2026-07-10 (mag `"+Y-X+Z"`, accel `"+Y-X-Z"` —
raw-axis snapshots in three poses, see `PCB_V2/CLAUDE.md`); full on-device V2 calibration is
still pending, so the embedded V1 transform/centre data remains only roughly valid.

## SAP6 BLE GATT Protocol

Same in V1 and V2 (V1 implemented in `V1/DiscoX C++ BLE/`, V2 in-process in `PCB_V2/`):

- 17-byte leg data: `[seqBit][az float][inc float][roll float][dist float]`
- Sequence bit toggles 0x55/0x56 for reliable delivery with ACK/retry on 5s timeout
- Coded PHY (BLE Long Range, S=8) on Android; iOS falls back to 1 Mbps
- **Coded PHY requires a manual patch** to the Adafruit Bluefruit library — see
  `V1/DiscoX C++ BLE/CLAUDE.md` for the exact change; `PCB_V2/CLAUDE.md` notes the
  same gotcha applies there

## Current Status
- V1: calibration working, accurate to within 1 degree (commit 8682252)
- **PCB V2 merged firmware running on hardware** (2026-07-07, `PCB_V2/`) —
  first bring-up passed the core spine: live sensor fusion (RM3100 + SCA3300
  through the ported calibration pipeline), battery gauge, OLED (at 0x3C —
  corrected from V1's 0x3D), buttons, laser measurement, and menu all
  verified on the board. Measurements flag `anomaly: MagErr` because the
  calibration is still V1 placeholder data — expected, not a fault.
- 2026-07-09: V1's USB settings drive is back on V2 — a 128 KB FAT12
  partition carved from the nRF52840's internal flash, exposed over USB MSC
  (menu → Settings → USB Drive Mode, or hold DOWN at power-on). Settings,
  calibration, and unsent readings appear as editable files; LittleFS stays
  the authoritative store. Details in `PCB_V2/CLAUDE.md` → "USB drive mode".
- 2026-07-10: axis mappings determined (`MAG_AXES`/`GRAV_AXES`) and the
  remaining on-device features exercised in a full test pass — buzzer
  sounds, disco, snake, USB drive mode, filter retuning.
- V2 commissioning still pending: full on-device calibration (embedded
  transform/centre data is still V1's, so `MagErr` persists) and the BLE
  phone test (see `PCB_V2/CLAUDE.md` checklist and `PCB_V2 test/CLAUDE.md`
  status table)
