#pragma once

#include <Arduino.h>

class ConfigManager;

// USB-drive settings mode: exposes a 128 KB FAT12 partition carved out of the
// nRF52840's internal flash (see flash_layout.h) as a USB mass-storage drive,
// like V1's QSPI drive mode. LittleFS stays the authoritative store — the FAT
// partition is a staging area synced at explicit points:
//
//   entry:  LittleFS → FAT   (CONFIG.JSON, CALIBRATION.JSON, READINGS.CSV)
//   exit:   FAT → LittleFS   (validated; bad JSON is rejected, old data kept)
//
// The host owns the FAT volume while the drive is exposed — firmware never
// touches it in between. Exit-by-power-cycle is covered by the usb_import
// boot flag: the next normal boot runs the same validated import.
namespace UsbDrive {

// Register the mass-storage USB interface. Call as early as possible in
// setup() (before Serial chatter); forces a USB re-enumeration if the host
// had already mounted us, so no cable replug is needed.
void beginMsc();

// Mount the FAT partition, formatting it FAT12 (volume label MRZAPPY) if
// blank or corrupt. Returns false only if the flash itself misbehaves.
bool mountOrFormat();

// LittleFS → FAT: write CONFIG.JSON, CALIBRATION.JSON, READINGS.CSV and
// README.TXT from the current LittleFS contents. Call before setHostAccess.
bool exportFiles(ConfigManager &cfgMgr);

// Toggle host access ("media present"). While true the host owns the FAT
// volume: firmware-side writes are rejected in the MSC callback and no
// FatFs calls may be made. Turn off (and give Windows ~0.5 s to finish
// in-flight writes) before importFiles().
void setHostAccess(bool on);

// FAT → LittleFS: validate and import CONFIG.JSON and CALIBRATION.JSON.
// Invalid/missing files are skipped (old data kept). A successful
// calibration import also deletes /calibration.bin so the JSON wins at the
// next boot. Returns false if a file existed but failed validation.
bool importFiles(ConfigManager &cfgMgr);

} // namespace UsbDrive
