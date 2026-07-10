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
    return sectorRead(lba, (uint8_t *)buffer, bufsize / FlashLayout::SECTOR_SIZE)
               ? (int32_t)bufsize
               : -1;
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

static const char README_TEXT[] =
    "Mr Zappy USB drive mode\r\n"
    "=======================\r\n"
    "\r\n"
    "CONFIG.JSON       device settings   - edit and save here\r\n"
    "CALIBRATION.JSON  sensor calibration - edit only if you know why\r\n"
    "READINGS.CSV      unsent survey legs (export only; edits ignored)\r\n"
    "\r\n"
    "When done: eject the drive, then press MENU on the device (or just\r\n"
    "power-cycle it). Valid changes to CONFIG.JSON / CALIBRATION.JSON are\r\n"
    "imported; a file that fails to parse is rejected and the old data kept.\r\n"
    "\r\n"
    "Recovery: if this drive shows up corrupt, format it (FAT, 512-byte\r\n"
    "sectors) from the PC and power-cycle - the device restages the files.\r\n";

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

bool importFiles(ConfigManager &cfgMgr) {
    if (!mountOrFormat()) {
        return false;
    }

    // Sized to match the boot-time parse buffers (config ≤1 KB read buffer,
    // calibration ≤2 KB) — anything bigger would be rejected at boot anyway.
    static char fileBuf[2048];
    bool allValid = true;

    // ── CONFIG.JSON ──
    int len = readFatFile("CONFIG.JSON", fileBuf, sizeof(fileBuf));
    if (len > 0) {
        JsonDocument doc;
        if (deserializeJson(doc, fileBuf, (size_t)len) == DeserializationError::Ok &&
            doc.is<JsonObject>()) {
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
    len = readFatFile("CALIBRATION.JSON", fileBuf, sizeof(fileBuf));
    if (len > 0) {
        JsonDocument doc;
        if (deserializeJson(doc, fileBuf, (size_t)len) == DeserializationError::Ok &&
            doc["mag"].is<JsonObject>() && doc["grav"].is<JsonObject>()) {
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

} // namespace UsbDrive
