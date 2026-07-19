#include "config_manager.h"
#include "config.h"
#include "mag_cal/calibration.h"

#include <Adafruit_LittleFS.h>
#include <ArduinoJson.h>
#include <InternalFileSystem.h>

using namespace Adafruit_LittleFS_Namespace;

// V2 backend: LittleFS on the nRF52840's internal flash (InternalFS).
// The V1 QSPI/FAT stack (Adafruit_SPIFlash + SdFat) is gone with the chip,
// and with it the Session-13 fragility workarounds — LittleFS is
// copy-on-write and power-loss-safe, and internal-flash writes are
// serialized through the SoftDevice. What stays from V1:
//   - the atomic temp-file + rename() write pattern (rename overwrites the
//     target atomically in littlefs), so an interrupted save never corrupts
//     the existing file;
//   - the RAM-buffered pending readings synced only when IDLE, so no flash
//     write ever lands inside the measurement loop (SoftDevice flash ops
//     can block for a few ms).
//
// Note: FILE_O_WRITE does not truncate an existing file — every fresh write
// removes the target first (same lesson as the DiscoX nvm_manager).

// Atomic byte-blob write: data → tmpPath, then rename over path.
static bool writeFileAtomicOnce(const char *path, const char *tmpPath, const uint8_t *data, size_t len) {
    InternalFS.remove(tmpPath); // FILE_O_WRITE won't truncate leftovers

    File file(InternalFS);
    if (!file.open(tmpPath, FILE_O_WRITE)) {
        Serial.println(F("  flash write FAILED (open)"));
        return false;
    }
    size_t written = file.write(data, len);
    file.close();

    if (written != len) {
        Serial.println(F("  flash write FAILED (short write)"));
        InternalFS.remove(tmpPath);
        return false;
    }

    // littlefs rename atomically replaces an existing target
    if (!InternalFS.rename(tmpPath, path)) {
        // Some littlefs builds refuse to rename onto an existing file —
        // fall back to remove-then-rename (tiny non-atomic window, but the
        // fully-written temp file is already safe on flash).
        InternalFS.remove(path);
        if (!InternalFS.rename(tmpPath, path)) {
            Serial.println(F("  flash write FAILED (rename)"));
            InternalFS.remove(tmpPath);
            return false;
        }
    }
    return true;
}

static bool writeFileAtomic(const char *path, const char *tmpPath, const uint8_t *data, size_t len) {
    // SoftDevice flash ops fail transiently under BLE radio load and the
    // core's flash HAL ignores the error event (see bleRadioQuiet in
    // config.h) — a short-spaced retry usually lands in a radio gap.
    for (int attempt = 0; attempt < 3; attempt++) {
        if (attempt > 0) {
            Serial.println(F("  flash write retrying..."));
            delay(75);
        }
        if (writeFileAtomicOnce(path, tmpPath, data, len)) {
            return true;
        }
    }
    return false;
}

// ── begin() ────────────────────────────────────────────────────────
bool ConfigManager::begin() {
    if (mounted_) {
        return true; // already initialized — idempotent
    }

    if (!InternalFS.begin()) {
        Serial.println(F("LittleFS mount FAILED — formatting..."));
        if (!InternalFS.format() || !InternalFS.begin()) {
            Serial.println(F("  Format/remount FAILED"));
            return false;
        }
        Serial.println(F("  Format successful — filesystem recovered"));
        reformatted_ = true;
    }

    // Ensure /flags directory exists
    if (!InternalFS.exists("/flags")) {
        InternalFS.mkdir("/flags");
    }

    mounted_ = true;
    Serial.println(F("Filesystem mounted OK (internal LittleFS)"));
    return true;
}

bool ConfigManager::reformat() {
    pendingBufCount_ = 0;
    if (!InternalFS.format()) {
        Serial.println(F("LittleFS format FAILED"));
        mounted_ = false;
        return false;
    }
    mounted_ = InternalFS.begin();
    if (mounted_ && !InternalFS.exists("/flags")) {
        InternalFS.mkdir("/flags");
    }
    reformatted_ = true;
    return mounted_;
}

// ── Settings persistence ───────────────────────────────────────────

bool ConfigManager::printConfig(Stream &out) {
    if (!mounted_) {
        return false;
    }
    File file(InternalFS);
    if (!file.open("/config.json", FILE_O_READ)) {
        return false;
    }
    char buf[64];
    int len;
    while ((len = file.read(buf, sizeof(buf))) > 0) {
        out.write(reinterpret_cast<const uint8_t *>(buf), len);
    }
    file.close();
    out.println();
    return true;
}

bool ConfigManager::loadConfig(Config &cfg) {
    if (!mounted_) {
        return false;
    }

    File file(InternalFS);
    if (!file.open("/config.json", FILE_O_READ)) {
        return false;
    }

    char buf[1024];
    int len = file.read(buf, sizeof(buf) - 1);
    file.close();
    if (len <= 0) {
        return false;
    }
    buf[len] = '\0';

    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, buf, (size_t)len);
    if (err) {
        Serial.print(F("Config parse error: "));
        Serial.println(err.c_str());
        return false;
    }

    cfg.magTolerance = doc["mag_tolerance"] | Defaults::magTolerance;
    cfg.gravTolerance = doc["grav_tolerance"] | Defaults::gravTolerance;
    cfg.dipTolerance = doc["dip_tolerance"] | Defaults::dipTolerance;
    cfg.anomalyDetection = doc["anomaly_detection"] | Defaults::anomalyDetection;
    cfg.stabilityTolerance = doc["stability_tolerance"] | Defaults::stabilityTolerance;
    cfg.stabilityBufferLength = doc["stability_buffer_length"] | (int)Defaults::stabilityBufferLength;
    cfg.emaAlphaStable = doc["ema_alpha_stable"] | Defaults::emaAlphaStable;
    cfg.emaAlphaMoving = doc["ema_alpha_moving"] | Defaults::emaAlphaMoving;
    cfg.legAngleTolerance = doc["leg_angle_tolerance"] | Defaults::legAngleTolerance;
    cfg.cartesianTolerance = doc["cartesian_tolerance"] | Defaults::cartesianTolerance;
    cfg.laserDistanceOffset = doc["laser_distance_offset"] | Defaults::laserDistanceOffset;
    cfg.calMagConsistency = doc["cal_mag_consistency"] | Defaults::calMagConsistency;
    cfg.calGravConsistency = doc["cal_grav_consistency"] | Defaults::calGravConsistency;
    cfg.calBufferLength = doc["cal_buffer_length"] | (int)Defaults::calBufferLength;
    cfg.calSettleMs = doc["cal_settle_ms"] | (int)Defaults::calSettleMs;
    cfg.calEmaAlpha = doc["cal_ema_alpha"] | Defaults::calEmaAlpha;
    cfg.calTimeoutMs = doc["cal_timeout_ms"] | (int)Defaults::calTimeoutMs;
    cfg.autoShutdownTimeout = doc["auto_shutdown_timeout"] | Defaults::autoShutdownTimeout;
    cfg.laserTimeout = doc["laser_timeout"] | Defaults::laserTimeout;
    cfg.laserWibble = doc["laser_wibble"] | Defaults::laserWibble;
    cfg.measureFromFront = doc["measure_from_front"] | Defaults::measureFromFront;
    cfg.screenBrightness = doc["screen_brightness"] | (int)Defaults::screenBrightness;
    cfg.splaysEnabled = doc["splays_enabled"] | Defaults::splaysEnabled;

    const char *rawName = doc["ble_name"] | Defaults::bleName;
    // Strip SAP6_ prefix if user included it — we always prepend it ourselves
    const char *userPart = (strncmp(rawName, "SAP6_", 5) == 0) ? rawName + 5 : rawName;
    snprintf(cfg.bleName, sizeof(cfg.bleName), "SAP6_%s", userPart);
    cfg.bleName[Defaults::bleNameMaxLen] = '\0';

    return true;
}

bool ConfigManager::saveConfig(const Config &cfg) {
    if (!mounted_) {
        return false;
    }

    JsonDocument doc;

    // ── Settings values ──
    doc["mag_tolerance"] = cfg.magTolerance;
    doc["grav_tolerance"] = cfg.gravTolerance;
    doc["dip_tolerance"] = cfg.dipTolerance;
    doc["anomaly_detection"] = cfg.anomalyDetection;
    doc["stability_tolerance"] = cfg.stabilityTolerance;
    doc["stability_buffer_length"] = (int)cfg.stabilityBufferLength;
    doc["ema_alpha_stable"] = cfg.emaAlphaStable;
    doc["ema_alpha_moving"] = cfg.emaAlphaMoving;
    doc["leg_angle_tolerance"] = cfg.legAngleTolerance;
    doc["cartesian_tolerance"] = cfg.cartesianTolerance;
    doc["laser_distance_offset"] = cfg.laserDistanceOffset;
    doc["cal_mag_consistency"] = cfg.calMagConsistency;
    doc["cal_grav_consistency"] = cfg.calGravConsistency;
    doc["cal_buffer_length"] = (int)cfg.calBufferLength;
    doc["cal_settle_ms"] = (int)cfg.calSettleMs;
    doc["cal_ema_alpha"] = cfg.calEmaAlpha;
    doc["cal_timeout_ms"] = (int)cfg.calTimeoutMs;
    doc["auto_shutdown_timeout"] = cfg.autoShutdownTimeout;
    doc["laser_timeout"] = cfg.laserTimeout;
    doc["laser_wibble"] = cfg.laserWibble;
    doc["measure_from_front"] = cfg.measureFromFront;
    doc["screen_brightness"] = (int)cfg.screenBrightness;
    doc["splays_enabled"] = cfg.splaysEnabled;
    // Save only the user portion — SAP6_ prefix is always auto-prepended on load
    const char *nameToSave = (strncmp(cfg.bleName, "SAP6_", 5) == 0) ? cfg.bleName + 5 : cfg.bleName;
    doc["ble_name"] = nameToSave;

    // Serialize to a RAM buffer first, then write atomically.
    char buf[1280];
    if (measureJsonPretty(doc) >= sizeof(buf)) {
        Serial.println(F("Config JSON too large for buffer"));
        return false;
    }
    size_t len = serializeJsonPretty(doc, buf, sizeof(buf));
    if (len == 0) {
        return false;
    }

    return writeFileAtomic("/config.json", "/cfg_tmp.json", reinterpret_cast<const uint8_t *>(buf), len);
}

bool ConfigManager::saveConfigJsonRaw(const char *json, size_t len) {
    if (!mounted_) {
        return false;
    }
    return writeFileAtomic("/config.json", "/cfg_tmp.json", reinterpret_cast<const uint8_t *>(json), len);
}

// ── Calibration data ───────────────────────────────────────────────

bool ConfigManager::loadCalibrationJson(char *buf, size_t bufSize, size_t &bytesRead) {
    if (!mounted_) {
        return false;
    }

    File file(InternalFS);
    if (!file.open("/calibration.json", FILE_O_READ)) {
        return false;
    }

    size_t fileSize = file.size();
    if (fileSize >= bufSize) {
        file.close();
        return false; // buffer too small
    }

    int n = file.read(buf, fileSize);
    file.close();
    if (n <= 0) {
        return false;
    }
    bytesRead = (size_t)n;
    buf[bytesRead] = '\0';
    return true;
}

bool ConfigManager::saveCalibrationJson(const char *json, size_t len) {
    if (!mounted_) {
        return false;
    }
    return writeFileAtomic("/calibration.json", "/cal_tmp.json", reinterpret_cast<const uint8_t *>(json),
                           len);
}

bool ConfigManager::loadCalibrationBinary(MagCal::CalibrationBinary &out) {
    if (!mounted_) {
        return false;
    }

    File file(InternalFS);
    if (!file.open("/calibration.bin", FILE_O_READ)) {
        return false;
    }

    size_t fileSize = file.size();
    if (fileSize != sizeof(MagCal::CalibrationBinary)) {
        file.close();
        return false;
    }

    int bytesRead = file.read(reinterpret_cast<uint8_t *>(&out), sizeof(out));
    file.close();
    return bytesRead == (int)sizeof(out);
}

bool ConfigManager::saveCalibrationBinary(const MagCal::CalibrationBinary &data) {
    if (!mounted_) {
        return false;
    }
    return writeFileAtomic("/calibration.bin", "/cal_tmp.bin", reinterpret_cast<const uint8_t *>(&data),
                           sizeof(data));
}

bool ConfigManager::testFileRoundTrip(const uint8_t *data, size_t len) {
    if (!mounted_) {
        return false;
    }
    if (!writeFileAtomic("/savetest.bin", "/svt_tmp", data, len)) {
        return false;
    }

    File f(InternalFS);
    if (!f.open("/savetest.bin", FILE_O_READ)) {
        return false;
    }
    bool ok = ((size_t)f.size() == len);
    uint8_t buf[64];
    size_t off = 0;
    while (ok && off < len) {
        size_t chunk = len - off;
        if (chunk > sizeof(buf)) {
            chunk = sizeof(buf);
        }
        int n = f.read(buf, (uint32_t)chunk);
        if (n <= 0) {
            ok = false;
            break;
        }
        ok = (memcmp(buf, data + off, (size_t)n) == 0);
        off += (size_t)n;
    }
    if (ok) {
        ok = (off == len);
    }
    f.close();
    InternalFS.remove("/savetest.bin");
    return ok;
}

bool ConfigManager::storageWriteTest() {
    if (!mounted_) {
        return false;
    }
    static const uint8_t probe[1] = {0x5A};
    if (!writeFileAtomic("/fs_selftest", "/fst_tmp", probe, sizeof(probe))) {
        return false;
    }
    return InternalFS.remove("/fs_selftest");
}

bool ConfigManager::removeCalibrationBinary() {
    if (!mounted_) {
        return false;
    }
    if (!InternalFS.exists("/calibration.bin")) {
        return true;
    }
    return InternalFS.remove("/calibration.bin");
}

// ── Calibration quality metrics ─────────────────────────────────────

bool ConfigManager::saveCalMetrics(const CalMetrics &m) {
    if (!mounted_) {
        return false;
    }
    return writeFileAtomic("/cal_metrics.bin", "/met_tmp.bin", reinterpret_cast<const uint8_t *>(&m),
                           sizeof(m));
}

bool ConfigManager::loadCalMetrics(CalMetrics &m) {
    if (!mounted_) {
        return false;
    }
    File file(InternalFS);
    if (!file.open("/cal_metrics.bin", FILE_O_READ)) {
        return false;
    }
    int bytesRead = file.read(reinterpret_cast<uint8_t *>(&m), sizeof(m));
    file.close();
    return bytesRead == (int)sizeof(m);
}

// ── Pending readings ───────────────────────────────────────────────

bool ConfigManager::appendPendingReading(float az, float inc, float dist) {
    if (pendingBufCount_ >= MAX_PENDING_BUF) {
        Serial.println(F("  Pending RAM buffer full — forcing sync"));
        syncPendingToFlash();
    }
    if (pendingBufCount_ < MAX_PENDING_BUF) {
        pendingBuf_[pendingBufCount_++] = {az, inc, dist};
        return true;
    }
    return false; // sync failed and buffer still full
}

bool ConfigManager::syncPendingToFlash() {
    if (pendingBufCount_ == 0) {
        return true;
    }
    if (!mounted_) {
        return false;
    }

    File file(InternalFS);
    if (!file.open("/pending.txt", FILE_O_WRITE)) {
        Serial.println(F("  syncPending: open FAILED"));
        return false;
    }
    file.seek(file.size()); // append

    char line[40];
    for (uint8_t i = 0; i < pendingBufCount_; i++) {
        int n = snprintf(line, sizeof(line), "%.1f,%.1f,%.2f\n", (double)pendingBuf_[i].az,
                         (double)pendingBuf_[i].inc, (double)pendingBuf_[i].dist);
        size_t written = file.write(reinterpret_cast<const uint8_t *>(line), (size_t)n);
        if (written != (size_t)n) {
            Serial.println(F("  syncPending: write FAILED"));
            file.close();
            return false;
        }
    }

    file.close();
    Serial.print(F("  syncPending: wrote "));
    Serial.print(pendingBufCount_);
    Serial.println(F(" readings to flash"));
    pendingBufCount_ = 0;
    return true;
}

uint16_t ConfigManager::countPendingReadings() {
    uint16_t count = pendingBufCount_;

    if (!mounted_) {
        return count;
    }

    File file(InternalFS);
    if (!file.open("/pending.txt", FILE_O_READ)) {
        return count;
    }

    while (file.available()) {
        if (file.read() == '\n') {
            count++;
        }
    }
    file.close();
    return count;
}

bool ConfigManager::flushPendingReadings(void (*callback)(float, float, float)) {
    // First flush any file-based readings
    if (mounted_) {
        File file(InternalFS);
        if (file.open("/pending.txt", FILE_O_READ)) {
            char line[48];
            uint8_t pos = 0;

            while (file.available()) {
                int c = file.read();
                if (c == '\n' || c == '\r') {
                    if (pos > 0) {
                        line[pos] = '\0';
                        // Parse with strtof (sscanf %f broken on newlib-nano)
                        char *p = line;
                        char *end;
                        float az = strtof(p, &end);
                        if (end != p && *end == ',') {
                            p = end + 1;
                            float inc = strtof(p, &end);
                            if (end != p && *end == ',') {
                                p = end + 1;
                                float dist = strtof(p, &end);
                                if (end != p) {
                                    callback(az, inc, dist);
                                }
                            }
                        }
                        pos = 0;
                    }
                } else if (pos < sizeof(line) - 1) {
                    line[pos++] = (char)c;
                }
            }
            file.close();
        }
    }

    // Then flush RAM buffer entries
    for (uint8_t i = 0; i < pendingBufCount_; i++) {
        callback(pendingBuf_[i].az, pendingBuf_[i].inc, pendingBuf_[i].dist);
    }
    pendingBufCount_ = 0;

    return true;
}

bool ConfigManager::clearPendingReadings() {
    pendingBufCount_ = 0;
    if (!mounted_) {
        return false;
    }
    return InternalFS.remove("/pending.txt");
}

// ── Flag files ─────────────────────────────────────────────────────

void ConfigManager::buildFlagPath(const char *name, char *path, size_t pathSize) {
    snprintf(path, pathSize, "/flags/%s", name);
}

bool ConfigManager::writeFlag(const char *name) {
    if (!mounted_) {
        return false;
    }

    char path[32];
    buildFlagPath(name, path, sizeof(path));

    if (!InternalFS.exists("/flags")) {
        InternalFS.mkdir("/flags");
    }

    InternalFS.remove(path); // FILE_O_WRITE won't truncate
    File file(InternalFS);
    if (!file.open(path, FILE_O_WRITE)) {
        Serial.println(F("  flag write FAILED"));
        return false;
    }
    file.write('1');
    file.close();
    return InternalFS.exists(path);
}

bool ConfigManager::hasFlag(const char *name) {
    if (!mounted_) {
        return false;
    }

    char path[32];
    buildFlagPath(name, path, sizeof(path));
    return InternalFS.exists(path);
}

bool ConfigManager::clearFlag(const char *name) {
    if (!mounted_) {
        return false;
    }

    char path[32];
    buildFlagPath(name, path, sizeof(path));
    return InternalFS.remove(path);
}
