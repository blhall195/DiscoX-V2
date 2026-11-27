# DiscoX rangefinder — stripped-down PCB V2 firmware

The PCB V2 device cut back to a laser distance meter plus a small menu.
Same board, bootloader, pins, linker script and flash map as `../PCB_V2`;
nothing else from the survey firmware. Started 2026-11-27 (`RF v1.0.0`),
built clean (flash 22.5%, RAM 6.3%), **not yet run on hardware**.

`../PCB_V2/CLAUDE.md` stays the reference for the hardware, flashing,
bootloader and gotchas. This file covers only what differs.

## What it does

| | |
|---|---|
| FIRE | laser off → laser on (aim, inverted "FIRE to measure" bar); laser on → measure. After a good reading the laser goes off and the distance stays on screen until the next good reading (aiming doesn't clear it). Shown as `12.34m` (size 4 up to `9.99m`, size 3 above). Before the first reading it shows `0.00m`. No hint text while the laser is off |
| DOWN | laser off |
| UP | nothing |
| MENU | open the menu |
| Power button | hold (bar, 1.5 s) = power off. A tap shows "Hold to power off"; **there is no standby** |

Menu (FruityMenu, same buttons as the full firmware: UP/DOWN scroll,
FIRE/MENU select):

- Exit
- **Shutdown: N** — auto power-off after no button press (5 min - 2 hr)
- **Laser off: N** — beam timeout (30 s - 30 min)
- **Update** → No / Yes: reboots into the UF2 bootloader drive for a
  drag-and-drop `.uf2`, with the same wait for the PC link as the full
  firmware's Update Firmware (the bootloader only gives USB ~3 s)
- Power off

A shot is validated as a survey shot is (`measureValidated`, 3 shots, all
SQ ≥ 20, median reported) with the same error screens, **except there is
no agreement check between the shots**: `laserSpreadLimitMm` is 0 here
(the full firmware has 25 mm), so there is no SPREAD error (removed
2026-11-27 at Brendan's request). The SQ floor is what's left between a
dark target and a wrong reading, and the median only protects against one
bad shot out of three. The distance is from the back of the
device (+0.12 m) unless `measure_from_front` is set in config.json.

Also kept: hold-to-power-on on a cold boot, the USB plug-in pulse filter,
low-battery shutdown (off USB), battery icon with charging animation, a
minimal charging screen (cold plug-in on a 2026-10-18+ board), the serial
`u` command for clean bootloader entry (polled in every mode here, not just
the main screen) and `v` for the version and unit number
(`RF v1.0.0 #49`; set the number with the full firmware). No boot splash
and no measure-from label (removed 2026-11-27 at Brendan's request): after
the hold bar it goes straight to the distance screen.

Dropped: compass/inclinometer (RM3100, SCA3300 never initialised),
calibration, BLE (SoftDevice never enabled), reading history, pending
readings, leg detection, disco, snake, USB settings drive, storage repair
screens, charge limit (the charger runs to its own 4.2 V finish, i.e. the
full firmware's 100% setting) and standby.

## Sharing the device with the full firmware

Units can go back and forth between this and `../PCB_V2` by UF2 (either
firmware's Update item), and each must leave the other's data alone:

- **The image only covers 0x26000-0x49C00** (checked from the `.uf2`), far
  inside the shared app cap of 0xC5000, so the reading history ring, FAT
  partition and LittleFS are never written by flashing.
- **Settings live in the full firmware's `/config.json`** under its own key
  names (`auto_shutdown_timeout`, `laser_timeout`, read-only
  `screen_brightness` and `measure_from_front`), so a timeout set here is
  still set after going back. Saving is read-modify-write: every other key
  is carried over. If the file exists but can't be read or parsed, nothing
  is written (a two-key file would throw away the user's settings). With no
  file at all it creates one with just the two keys, and the full
  firmware's settings migration adds the rest on its next boot.
- **It never formats LittleFS.** A failed mount runs on defaults and menu
  changes aren't saved. A littlefs assert reboots (GPREGRET2 = 0xA7, the
  full firmware's mark), and the next boot runs without storage behind a
  "Storage error" notice. Repair is left to the full firmware, which has the
  factory reset and the FAT backup of settings and calibration.
- **GPREGRET2 = 0x5C at every boot**, the full firmware's APP_RAN_MARK, so
  a UF2 update from one to the other on USB is still a warm boot (no
  charging screen).
- Calibration, bonds, pending readings and history stay on the chip
  untouched; going back to the full firmware finds them as they were.

## Files

Copied unchanged from `../PCB_V2` (if one is fixed there, copy the fix
here): `include/{pins_v2,button_manager,fruity_menu,laser_manager,sounds,device_number,flash_layout}.h`,
`src/{button_manager,fruity_menu,sounds,device_number}.cpp`,
`src/laser_manager.cpp` (one change: `spreadLimitMm` 0 turns the agreement
check off — keep that line when copying a fix across),
`src/drivers/{buzzer,charger,ldj100,power}.*`, `src/drivers/max17048.h`,
`linker/`, `tools/make_uf2.py`.

New here: `src/main.cpp` (trimmed copies of the full firmware's boot hold,
power button, update and laser paths), `rf_display` (distance screen;
battery icon, power prompt and error screen copied from
`display_manager.cpp`), `rf_menu`, `settings` (config.json guest access),
`led`. `config.h` and `defaults.h` are trimmed copies: keep the pin
mapping and the `Defaults::laser*` values in step with `../PCB_V2`.

## Build / flash

```bash
cd "PCB_V2 rangefinder"
~/.platformio/penv/Scripts/pio.exe run             # also writes .pio/build/pcb_v2_rangefinder/firmware.uf2
~/.platformio/penv/Scripts/pio.exe run -t upload   # send 'u' over serial first
```

Not built by CI or `build.sh`, and the release pipeline and website never
see it.

## First hardware run (to do)

1. Cold power-on: hold bar, then straight to `0.00m` with nothing under it.
2. FIRE → beam on, inverted `FIRE to measure` bar; FIRE → reading against
   a tape (from back = +0.12 m), green LED + bleep, beam off. Dark target →
   error screen (LOW SIGNAL), beam stays on. FIRE to aim again: the last
   reading stays up until a new one replaces it. Dark rock at range: check
   readings against a tape, since there's no SPREAD check to refuse
   disagreeing shots.
3. DOWN turns the beam off; laser and shutdown timeouts fire (set 30 s
   laser / 5 min shutdown).
4. Menu: change both timeouts, power-cycle, check they held; flash the full
   firmware and check they're still set there and no other setting changed.
5. Update → Yes with USB attached: bootloader drive appears; drop the full
   firmware `.uf2` and back again. Without USB: "Plug device into PC",
   MENU cancels.
6. New board: plug in while off → charging screen; any button → rangefinder;
   unplug → off. Plug in while running → nothing but the icon.
7. Power button: tap → "Hold to power off"; hold → off.
