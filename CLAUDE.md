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

## Releasing — the release notes are customer-facing copy

Pushing a tag runs `.github/workflows/build.yml`: native tests, `./build.sh`,
then a GitHub release carrying `mrzappy-<tag>.uf2`. It then runs
`tools/publish_to_website.py`, which opens a pull request on
[blhall195/DiscoX-Cave-Survey-Device](https://github.com/blhall195/DiscoX-Cave-Survey-Device)
adding the new `.uf2`, rewriting the four version strings on `firmware.html`,
and **building the site's changelog from the release notes body**. Brendan
reviews that PR and merges; merging it deploys discox.co.uk.

So when you tag, **write the release notes for cavers, not for git**. Top-level
`-` bullets become the `<li>` items on the public firmware page. Say what was
wrong, what it meant in the field, and whether the user has to do anything
(re-flash, re-calibrate). The v2.0.2 entry on `firmware.html` is the standard.

`generate_release_notes: true` is still on as a backstop, but its output is a
list of commit titles. The script **detects that and refuses to publish it** —
you get the download links updated and an explicit warning on the PR that the
changelog was left alone, rather than commit messages appearing on the website.

Write the notes on the release before the workflow's "Read the release notes"
step runs, or edit the release afterwards and re-run the job.

The cross-repo PR needs the `WEBSITE_PR_TOKEN` secret (a fine-grained PAT scoped
to the website repo only, Contents + Pull requests write). Without it the
release still publishes and the workflow logs a warning.

Dry-run the script before changing it — it needs no token and writes nothing:

    python3 tools/publish_to_website.py --tag v2.0.2 --dry-run \
        --website-dir ../DiscoX-Cave-Survey-Device

Against an already-published tag that must print `4/4 version strings located`
and **no diff**: the script reproducing the live page byte-for-byte is the test
that its regexes still match the markup.

## V1 Reference Implementation

V1 is a dual-MCU architecture (SAMD51 main board + nRF52840 BLE board
communicating via UART) — the proven, calibrated baseline the V2 port came
from. Its code no longer lives in this repo: see the original V1 repository
at **https://github.com/blhall195/Mr_Zappy**.

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

Note: V2 axis mappings determined empirically 2026-07-10 (mag `"+Y-X+Z"`, grav `"+Y-X-Z"` —
raw-axis snapshots in three poses, see `PCB_V2/CLAUDE.md`). The grav string was corrected to
`"-Y-X+Z"` on 2026-09-19: `GRAV_AXES` maps to gravity (down), not to the accelerometer's raw
specific force, and reading each axis off in isolation left the accel frame 180° rolled
relative to the mag — which mirrored the azimuth (east read 270°) while leaving inclination
right. Full on-device V2 calibration is still pending, so the embedded V1 transform/centre
data remains only roughly valid.

## SAP6 BLE GATT Protocol

Same in V1 and V2 (V2 implements it in-process in `PCB_V2/`):

- 17-byte leg data: `[seqBit][az float][inc float][roll float][dist float]`
- Sequence bit toggles 0x55/0x56 for reliable delivery with ACK/retry on 5s timeout
- Coded PHY (BLE Long Range, S=8) on Android; iOS falls back to 1 Mbps
- **Coded PHY requires a manual patch** to the Adafruit Bluefruit library — see
  the "Coded PHY" gotcha in `PCB_V2/CLAUDE.md` for the exact change

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
