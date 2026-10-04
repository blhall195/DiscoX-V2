#include "usb_drive.h"
#include "config_manager.h"
#include "flash_layout.h"

#include <Adafruit_LittleFS.h>
#include <Adafruit_TinyUSB.h>
#include <ArduinoJson.h>
#include <InternalFileSystem.h>

// ff.h must precede diskio.h — diskio.h uses BYTE/UINT/LBA_t from ff.h
#include "ff.h"
#include "diskio.h"

// SoftDevice-safe internal-flash HAL from the core's InternalFileSytem
// library (same one LittleFS writes through) — gives erase/program with a
// 4 KB read-modify-write page cache, so 512-byte FAT sectors "just work".
#include "flash/flash_nrf5x.h"

using namespace Adafruit_LittleFS_Namespace;

// ═══════════════════════════════════════════════════════════════════
// ── Raw sector access (shared by FatFs diskio and USB MSC) ─────────
// ═══════════════════════════════════════════════════════════════════

// While the host owns the volume, firmware-side FatFs writes are a bug —
// and after host access ends, a straggling MSC write must not race the
// import running on the loop task (both funnel into the same page cache).
static volatile bool s_hostAccess = false;

static bool sectorRead(uint32_t sector, uint8_t *buf, uint32_t count) {
    if (sector + count > FlashLayout::SECTOR_COUNT) {
        return false;
    }
    return flash_nrf5x_read(buf, FlashLayout::FAT_START + sector * FlashLayout::SECTOR_SIZE,
                            count * FlashLayout::SECTOR_SIZE) >= 0;
}

static bool sectorWrite(uint32_t sector, const uint8_t *buf, uint32_t count) {
    if (sector + count > FlashLayout::SECTOR_COUNT) {
        return false;
    }
    return flash_nrf5x_write(FlashLayout::FAT_START + sector * FlashLayout::SECTOR_SIZE, buf,
                             count * FlashLayout::SECTOR_SIZE) >= 0;
}

// ── FatFs diskio glue (single volume, drive 0) ─────────────────────

extern "C" {

DSTATUS disk_status(BYTE) { return 0; }
DSTATUS disk_initialize(BYTE) { return 0; }

DRESULT disk_read(BYTE, BYTE *buff, LBA_t sector, UINT count) {
    return sectorRead(sector, buff, count) ? RES_OK : RES_ERROR;
}

DRESULT disk_write(BYTE, const BYTE *buff, LBA_t sector, UINT count) {
    return sectorWrite(sector, buff, count) ? RES_OK : RES_ERROR;
}

DRESULT disk_ioctl(BYTE, BYTE cmd, void *buff) {
    switch (cmd) {
    case CTRL_SYNC:
        flash_nrf5x_flush();
        return RES_OK;
    case GET_SECTOR_COUNT:
        *(LBA_t *)buff = FlashLayout::SECTOR_COUNT;
        return RES_OK;
    case GET_BLOCK_SIZE: // erase block, in sectors
        *(DWORD *)buff = FlashLayout::PAGE_SIZE / FlashLayout::SECTOR_SIZE;
        return RES_OK;
    default:
        return RES_PARERR;
    }
}

} // extern "C"

// ═══════════════════════════════════════════════════════════════════
// ── USB mass storage interface ─────────────────────────────────────
// ═══════════════════════════════════════════════════════════════════

static Adafruit_USBD_MSC s_usbMsc;

static int32_t msc_read_cb(uint32_t lba, void *buffer, uint32_t bufsize) {
    return sectorRead(lba, (uint8_t *)buffer, bufsize / FlashLayout::SECTOR_SIZE) ? (int32_t)bufsize : -1;
}

static int32_t msc_write_cb(uint32_t lba, uint8_t *buffer, uint32_t bufsize) {
    if (!s_hostAccess) {
        return -1; // media already "removed" — refuse straggling writes
    }
    return sectorWrite(lba, buffer, bufsize / FlashLayout::SECTOR_SIZE) ? (int32_t)bufsize : -1;
}

static void msc_flush_cb(void) { flash_nrf5x_flush(); }

namespace UsbDrive {

void beginMsc() {
    s_usbMsc.setID("Mr_Zappy", "Settings", "2.0");
    s_usbMsc.setReadWriteCallback(msc_read_cb, msc_write_cb, msc_flush_cb);
    s_usbMsc.setCapacity(FlashLayout::SECTOR_COUNT, FlashLayout::SECTOR_SIZE);
    s_usbMsc.setUnitReady(false); // media "inserted" only once files are staged
    s_usbMsc.begin();

    // USB attaches back in main(), before setup() — by now the host may have
    // read (or be mid-way through reading) the CDC-only descriptor. Force a
    // fresh enumeration that includes the MSC interface. Unconditional on
    // purpose: gating on mounted() is racy — first hardware test (2026-07-09)
    // hit the window where mounted() was still false yet the host had the
    // old descriptor, leaving the drive invisible until a cable replug.
    TinyUSBDevice.detach();
    delay(100); // long enough for the host to register the disconnect
    TinyUSBDevice.attach();
}

void setHostAccess(bool on) {
    s_hostAccess = on;
    s_usbMsc.setUnitReady(on);
}

// ═══════════════════════════════════════════════════════════════════
// ── FAT volume management ──────────────────────────────────────────
// ═══════════════════════════════════════════════════════════════════

static FATFS s_fatfs;
static bool s_fatMounted = false;

bool mountOrFormat() {
    if (s_fatMounted) {
        return true;
    }

    FRESULT rc = f_mount(&s_fatfs, "", 1);
    if (rc != FR_OK) {
        Serial.print(F("FAT mount failed (rc="));
        Serial.print(rc);
        Serial.println(F(") — formatting partition FAT12"));

        // 128 KB volume: FAT12, no partition table (SFD), 1 FAT copy,
        // 64 root entries — leaves ~124 KB of data clusters.
        MKFS_PARM parm = {};
        parm.fmt = FM_FAT | FM_SFD;
        parm.n_fat = 1;
        parm.n_root = 64;
        static uint8_t workBuf[FF_MAX_SS];
        rc = f_mkfs("", &parm, workBuf, sizeof(workBuf));
        if (rc != FR_OK) {
            Serial.print(F("  f_mkfs FAILED rc="));
            Serial.println(rc);
            return false;
        }
        rc = f_mount(&s_fatfs, "", 1);
        if (rc != FR_OK) {
            Serial.print(F("  remount FAILED rc="));
            Serial.println(rc);
            return false;
        }
        f_setlabel("MRZAPPY");
    }

    s_fatMounted = true;
    return true;
}

// ═══════════════════════════════════════════════════════════════════
// ── Export: LittleFS → FAT ─────────────────────────────────────────
// ═══════════════════════════════════════════════════════════════════

// Shared copy buffer — export/import run one file at a time from the loop
// task, never concurrently with host access.
static uint8_t s_xferBuf[512];

// Copy a LittleFS file onto the FAT volume (truncating). Missing source is
// not an error (nothing to export yet); optionalHeader (may be nullptr) is
// written before the file content.
static bool copyToFat(const char *lfsPath, const char *fatName, const char *optionalHeader) {
    File src(InternalFS);
    bool haveSrc = (lfsPath != nullptr) && src.open(lfsPath, FILE_O_READ);
    if (!haveSrc && !optionalHeader) {
        return true;
    }

    FIL dst;
    if (f_open(&dst, fatName, FA_WRITE | FA_CREATE_ALWAYS) != FR_OK) {
        if (haveSrc) {
            src.close();
        }
        Serial.print(F("  export open FAILED: "));
        Serial.println(fatName);
        return false;
    }

    bool ok = true;
    UINT written = 0;
    if (optionalHeader) {
        ok = (f_write(&dst, optionalHeader, strlen(optionalHeader), &written) == FR_OK) &&
             written == strlen(optionalHeader);
    }
    while (ok && haveSrc && src.available()) {
        int n = src.read(s_xferBuf, sizeof(s_xferBuf));
        if (n <= 0) {
            break;
        }
        ok = (f_write(&dst, s_xferBuf, (UINT)n, &written) == FR_OK) && written == (UINT)n;
    }

    if (haveSrc) {
        src.close();
    }
    f_close(&dst);
    if (!ok) {
        Serial.print(F("  export write FAILED: "));
        Serial.println(fatName);
    }
    return ok;
}

static const char README_TEXT[] = "Mr Zappy USB drive mode\r\n"
                                  "=======================\r\n"
                                  "\r\n"
                                  "CONFIG.JSON       device settings   - edit and save here\r\n"
                                  "CALIBRATION.JSON  sensor calibration - edit only if you know why\r\n"
                                  "READINGS.CSV      unsent survey legs (export only; edits ignored)\r\n"
                                  "\r\n"
                                  "CONFIG.JSON setting worth knowing:\r\n"
                                  "  standby_timeout  seconds a short power-button press stays in\r\n"
                                  "                   standby before switching off (0 = no standby)\r\n"
                                  "\r\n"
                                  "When done: eject the drive, then press MENU on the device (or just\r\n"
                                  "power-cycle it). Valid changes to CONFIG.JSON / CALIBRATION.JSON are\r\n"
                                  "imported; a file that fails to parse is rejected and the old data "
                                  "kept.\r\n"
                                  "\r\n"
                                  "Recovery: if this drive shows up corrupt, format it (FAT, 512-byte\r\n"
                                  "sectors) from the PC and power-cycle - the device restages the files.\r\n"
                                  "\r\n"
                                  "The hidden BACKUP folder is the device's own safety copy of its\r\n"
                                  "settings and calibration, restored automatically if the internal\r\n"
                                  "storage ever has to be reset. Leave it alone; formatting this drive\r\n"
                                  "deletes it until the next save or power-on recreates it.\r\n";

bool exportFiles(ConfigManager &cfgMgr) {
    if (!mountOrFormat()) {
        return false;
    }

    // Make sure everything queued in RAM is on LittleFS before staging
    cfgMgr.syncPendingToFlash();

    bool ok = true;
    ok &= copyToFat("/config.json", "CONFIG.JSON", nullptr);
    ok &= copyToFat("/calibration.json", "CALIBRATION.JSON", nullptr);
    ok &= copyToFat("/pending.txt", "READINGS.CSV", "azimuth_deg,inclination_deg,distance_m\r\n");
    ok &= copyToFat(nullptr, "README.TXT", README_TEXT);

    flash_nrf5x_flush();
    return ok;
}

// ═══════════════════════════════════════════════════════════════════
// ── Import: FAT → LittleFS (validated) ─────────────────────────────
// ═══════════════════════════════════════════════════════════════════

// Read a whole FAT file into buf (null-terminated). Returns bytes read,
// 0 if missing/empty, -1 if too large or unreadable.
static int readFatFile(const char *fatName, char *buf, size_t bufSize) {
    FIL f;
    if (f_open(&f, fatName, FA_READ) != FR_OK) {
        return 0;
    }
    FSIZE_t size = f_size(&f);
    if (size == 0 || size >= bufSize) {
        f_close(&f);
        return size == 0 ? 0 : -1;
    }
    UINT nRead = 0;
    FRESULT rc = f_read(&f, buf, (UINT)size, &nRead);
    f_close(&f);
    if (rc != FR_OK || nRead != size) {
        return -1;
    }
    buf[nRead] = '\0';
    return (int)nRead;
}

// Shared 2 KB parse buffer for import and backup restore — sized to match
// the boot-time parse buffers (config and calibration are both read into
// 2 KB); anything bigger would be rejected at boot anyway. backupWrite()'s
// compare path uses s_xferBuf, so s_fileBuf may be passed straight to it.
static char s_fileBuf[2048];

// Same acceptance test as the boot loaders: config must be a JSON object,
// calibration additionally needs the mag + grav sections.
static bool validJson(Backup which, const char *buf, size_t len) {
    JsonDocument doc;
    if (deserializeJson(doc, buf, len) != DeserializationError::Ok) {
        return false;
    }
    if (which == Backup::CONFIG) {
        return doc.is<JsonObject>();
    }
    return doc["mag"].is<JsonObject>() && doc["grav"].is<JsonObject>();
}

bool importFiles(ConfigManager &cfgMgr) {
    // The volume has been mounted since export, but the HOST has rewritten
    // the medium behind FatFs's back — its cached FAT/directory sectors are
    // stale, so reads of host-modified files see wrong sizes or fail with
    // FR_INT_ERR ("CONFIG.JSON unreadable"). Force a clean remount so every
    // sector is re-read from flash.
    f_mount(nullptr, "", 0);
    s_fatMounted = false;
    if (!mountOrFormat()) {
        return false;
    }

    char *fileBuf = s_fileBuf;
    bool allValid = true;

    // ── CONFIG.JSON ──
    int len = readFatFile("CONFIG.JSON", fileBuf, sizeof(s_fileBuf));
    if (len > 0) {
        if (validJson(Backup::CONFIG, fileBuf, (size_t)len)) {
            if (cfgMgr.saveConfigJsonRaw(fileBuf, (size_t)len)) {
                Serial.println(F("  imported CONFIG.JSON"));
            } else {
                allValid = false;
            }
        } else {
            Serial.println(F("  CONFIG.JSON invalid — keeping old settings"));
            allValid = false;
        }
    } else if (len < 0) {
        Serial.println(F("  CONFIG.JSON unreadable/too large — keeping old settings"));
        allValid = false;
    }

    // ── CALIBRATION.JSON ──
    len = readFatFile("CALIBRATION.JSON", fileBuf, sizeof(s_fileBuf));
    if (len > 0) {
        if (validJson(Backup::CALIBRATION, fileBuf, (size_t)len)) {
            if (cfgMgr.saveCalibrationJson(fileBuf, (size_t)len)) {
                // Binary would shadow the JSON at boot — force JSON to win
                cfgMgr.removeCalibrationBinary();
                Serial.println(F("  imported CALIBRATION.JSON"));
            } else {
                allValid = false;
            }
        } else {
            Serial.println(F("  CALIBRATION.JSON invalid — keeping old calibration"));
            allValid = false;
        }
    } else if (len < 0) {
        Serial.println(F("  CALIBRATION.JSON unreadable/too large — keeping old calibration"));
        allValid = false;
    }

    return allValid;
}

// ═══════════════════════════════════════════════════════════════════
// ── Settings + calibration backup ──────────────────────────────────
// ═══════════════════════════════════════════════════════════════════

// LittleFS on this chip does not survive a reset or a failed flash op
// mid-commit (the core's flash HAL ignores SoftDevice flash errors), and the
// only way out of a damaged filesystem is a format. These copies sit on the
// FAT partition, which a format never touches, so a storage reset costs the
// user their unsent readings rather than their calibration too (field
// incident 2026-09-25: half-finished calibration, filesystem assert at boot).
//
// FAT has no atomic replace, so a write goes to .TMP, is read back, and only
// then replaces the real copy; restore falls back to .TMP if the swap was
// interrupted.

static const char BACKUP_DIR[] = "BACKUP";

static const char *backupPath(Backup which) {
    return which == Backup::CONFIG ? "BACKUP/CONFIG.JSON" : "BACKUP/CALIB.JSON";
}

static const char *backupTmpPath(Backup which) {
    return which == Backup::CONFIG ? "BACKUP/CONFIG.TMP" : "BACKUP/CALIB.TMP";
}

static const char *backupName(Backup which) { return which == Backup::CONFIG ? "settings" : "calibration"; }

// True if the FAT file exists and holds exactly `data`.
static bool fatFileEquals(const char *path, const char *data, size_t len) {
    FIL f;
    if (f_open(&f, path, FA_READ) != FR_OK) {
        return false;
    }
    bool same = (f_size(&f) == len);
    size_t off = 0;
    while (same && off < len) {
        UINT chunk = (UINT)min(len - off, sizeof(s_xferBuf));
        UINT nRead = 0;
        if (f_read(&f, s_xferBuf, chunk, &nRead) != FR_OK || nRead != chunk) {
            same = false;
            break;
        }
        same = (memcmp(s_xferBuf, data + off, chunk) == 0);
        off += chunk;
    }
    f_close(&f);
    return same;
}

// Read a whole LittleFS file into buf (null-terminated). Same return
// convention as readFatFile: bytes read, 0 if missing/empty, -1 if too large.
static int readLfsFile(const char *path, char *buf, size_t bufSize) {
    File f(InternalFS);
    if (!f.open(path, FILE_O_READ)) {
        return 0;
    }
    size_t size = f.size();
    if (size == 0 || size >= bufSize) {
        f.close();
        return size == 0 ? 0 : -1;
    }
    int n = f.read(buf, size);
    f.close();
    if (n != (int)size) {
        return -1;
    }
    buf[n] = '\0';
    return n;
}

bool backupWrite(Backup which, const char *data, size_t len) {
    if (s_hostAccess || len == 0) {
        return false; // the host owns the volume in drive mode
    }
    if (!mountOrFormat()) {
        return false;
    }

    const char *path = backupPath(which);
    const char *tmp = backupTmpPath(which);
    if (fatFileEquals(path, data, len)) {
        return true; // unchanged — no flash wear
    }

    FRESULT rc = f_mkdir(BACKUP_DIR);
    if (rc == FR_OK) {
        f_chmod(BACKUP_DIR, AM_HID | AM_SYS, AM_HID | AM_SYS);
    } else if (rc != FR_EXIST) {
        Serial.print(F("  backup: mkdir FAILED rc="));
        Serial.println(rc);
        return false;
    }

    FIL f;
    bool ok = (f_open(&f, tmp, FA_WRITE | FA_CREATE_ALWAYS) == FR_OK);
    if (ok) {
        UINT written = 0;
        ok = (f_write(&f, data, (UINT)len, &written) == FR_OK) && written == len;
        ok = (f_close(&f) == FR_OK) && ok;
    }
    flash_nrf5x_flush();
    ok = ok && fatFileEquals(tmp, data, len); // read back before trusting it
    if (!ok) {
        f_unlink(tmp);
        Serial.print(F("  backup: write FAILED ("));
        Serial.print(backupName(which));
        Serial.println(F(")"));
        return false;
    }

    f_unlink(path); // FR_NO_FILE on the first write is fine
    if (f_rename(tmp, path) != FR_OK) {
        // The verified .TMP stays behind and restore knows to look for it
        Serial.println(F("  backup: rename FAILED"));
        flash_nrf5x_flush();
        return false;
    }
    flash_nrf5x_flush();
    Serial.print(F("  backup: "));
    Serial.print(backupName(which));
    Serial.println(F(" updated"));
    return true;
}

// Reconcile one file; true if it was restored from the backup.
static bool syncOne(ConfigManager &cfgMgr, Backup which) {
    const char *lfsPath = which == Backup::CONFIG ? "/config.json" : "/calibration.json";

    int len = readLfsFile(lfsPath, s_fileBuf, sizeof(s_fileBuf));
    if (len > 0 && validJson(which, s_fileBuf, (size_t)len)) {
        backupWrite(which, s_fileBuf, (size_t)len); // seed / refresh
        return false;
    }
    if (which == Backup::CALIBRATION && InternalFS.exists("/calibration.bin")) {
        return false; // boot loads the binary — nothing is missing
    }

    // Missing or unparseable on LittleFS — take the newest valid copy
    const char *candidates[] = {backupPath(which), backupTmpPath(which)};
    for (const char *path : candidates) {
        int n = readFatFile(path, s_fileBuf, sizeof(s_fileBuf));
        if (n <= 0 || !validJson(which, s_fileBuf, (size_t)n)) {
            continue;
        }
        bool ok = which == Backup::CONFIG ? cfgMgr.saveConfigJsonRaw(s_fileBuf, (size_t)n)
                                          : cfgMgr.saveCalibrationJson(s_fileBuf, (size_t)n);
        Serial.print(F("  Restored "));
        Serial.print(backupName(which));
        Serial.print(F(" from backup: "));
        Serial.println(ok ? F("OK") : F("FAILED"));
        return ok;
    }
    return false;
}

RestoreResult syncBackups(ConfigManager &cfgMgr) {
    RestoreResult r;
    if (!cfgMgr.isReady() || !mountOrFormat()) {
        return r;
    }
    r.config = syncOne(cfgMgr, Backup::CONFIG);
    r.calibration = syncOne(cfgMgr, Backup::CALIBRATION);
    return r;
}

bool clearBackups() {
    if (!mountOrFormat()) {
        return false;
    }
    f_unlink(backupPath(Backup::CONFIG));
    f_unlink(backupTmpPath(Backup::CONFIG));
    f_unlink(backupPath(Backup::CALIBRATION));
    f_unlink(backupTmpPath(Backup::CALIBRATION));
    flash_nrf5x_flush();
    Serial.println(F("  backup: cleared"));
    return true;
}

} // namespace UsbDrive
