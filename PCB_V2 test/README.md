# PCB_V2

New single-MCU board revision: one Raytac MDBT50Q-U1MV2 (nRF52840) module
replaces the V1 Feather M4 + ItsyBitsy pair.

- `bringup/` — PlatformIO project with per-IC hardware tests (one env each):
  `pio run -e sca3300_test -t upload`, then serial monitor at 115200.
  Press any key in the monitor to re-run the PASS/FAIL test sequence.
- `firmware/` — production firmware (currently the V1 BLE-bridge code,
  pending rewrite for the single-MCU design)
- `hardware/` — netlist (raw JSON + decoded `NETLIST.md`) and datasheets

See `CLAUDE.md` in this folder for the full bring-up guide: flashing,
pin map, per-IC status, and how to add the next test.
