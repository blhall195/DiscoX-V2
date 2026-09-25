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
//
// Outside drive mode the partition also carries the settings/calibration
// backup (see below).
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

// ── Settings + calibration backup ──────────────────────────────────
// A second copy of /config.json and /calibration.json lives in a hidden
// BACKUP folder on the FAT partition. A LittleFS format (menu Reformat
// Storage, the DOWN+MENU boot reset, or the storage-damaged recovery screen)
// never touches that partition, so the next boot puts them back — only
// unsent readings are lost. ConfigManager calls backupWrite() after every
// successful save; syncBackups() runs once per boot.

enum class Backup : uint8_t { CONFIG, CALIBRATION };

// Mirror `data` into the backup copy. Skips the write when the stored copy
// already matches, so an unchanged save costs reads only. False while the
// host owns the volume (drive mode) or if the FAT partition misbehaves.
bool backupWrite(Backup which, const char *data, size_t len);

struct RestoreResult {
    bool config = false;
    bool calibration = false;
};

// Boot-time reconcile. For each file: a valid LittleFS copy refreshes the
// backup (seeds it on the first boot of this firmware); a missing or
// unparseable one is restored from a valid backup. Calibration counts as
// missing only when neither /calibration.json nor /calibration.bin exists.
RestoreResult syncBackups(ConfigManager &cfgMgr);

// Delete both backup copies (storage reset with UP held: "erase everything").
bool clearBackups();

} // namespace UsbDrive
