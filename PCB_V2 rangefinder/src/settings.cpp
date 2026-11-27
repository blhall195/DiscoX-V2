#include "settings.h"

#include "defaults.h"
#include <Adafruit_LittleFS.h>
#include <Arduino.h>
#include <ArduinoJson.h>
#include <InternalFileSystem.h>

using namespace Adafruit_LittleFS_Namespace;

static bool mounted = false;

// static, not locals: the loop task has a 4 KB stack (see ../PCB_V2/CLAUDE.md)
static char fileBuf[2048];

static constexpr const char *CONFIG_PATH = "/config.json";
static constexpr const char *TMP_PATH = "/cfg_tmp.json"; // same temp name as the full firmware

namespace SettingsStore {

bool begin() {
    // No format on failure — see settings.h
    mounted = InternalFS.begin();
    Serial.print(F("Storage:     "));
    Serial.println(mounted ? F("OK") : F("not mounted - running on defaults"));
    return mounted;
}

bool available() { return mounted; }

// Whole file into fileBuf, NUL-terminated. False if missing or too large.
static bool readConfigFile(size_t &len) {
    len = 0;
    if (!mounted) {
        return false;
    }
    File file(InternalFS);
    if (!file.open(CONFIG_PATH, FILE_O_READ)) {
        return false;
    }
    size_t size = file.size();
    if (size == 0 || size >= sizeof(fileBuf)) {
        file.close();
        return false;
    }
    int n = file.read(fileBuf, size);
    file.close();
    if (n <= 0) {
        return false;
    }
    len = (size_t)n;
    fileBuf[len] = '\0';
    return true;
}

void load(Settings &s) {
    s.autoShutdownTimeout = Defaults::autoShutdownTimeout;
    s.laserTimeout = Defaults::laserTimeout;
    s.screenBrightness = Defaults::screenBrightness;
    s.measureFromFront = Defaults::measureFromFront;

    size_t len = 0;
    if (!readConfigFile(len)) {
        Serial.println(F("Settings:    no config.json - defaults"));
        return;
    }
    JsonDocument doc;
    if (deserializeJson(doc, fileBuf, len)) {
        Serial.println(F("Settings:    config.json unreadable - defaults"));
        return;
    }
    uint32_t shut = doc["auto_shutdown_timeout"] | Defaults::autoShutdownTimeout;
    uint32_t laser = doc["laser_timeout"] | Defaults::laserTimeout;
    int bright = doc["screen_brightness"] | (int)Defaults::screenBrightness;
    // Guard against a hand-edited file (USB drive mode in the full firmware)
    if (shut >= 60 && shut <= 24UL * 3600) {
        s.autoShutdownTimeout = shut;
    }
    if (laser >= 5 && laser <= 24UL * 3600) {
        s.laserTimeout = laser;
    }
    if (bright >= 1 && bright <= 255) {
        s.screenBrightness = (uint8_t)bright;
    }
    s.measureFromFront = doc["measure_from_front"] | Defaults::measureFromFront;

    Serial.print(F("Settings:    shutdown "));
    Serial.print(s.autoShutdownTimeout);
    Serial.print(F(" s, laser "));
    Serial.print(s.laserTimeout);
    Serial.print(F(" s, measure from "));
    Serial.println(s.measureFromFront ? F("front") : F("back"));
}

// data -> temp file, then rename over the target (the full firmware's
// writeFileAtomic, minus its BLE-contention retries: no radio here).
static bool writeAtomic(const char *data, size_t len) {
    InternalFS.remove(TMP_PATH); // FILE_O_WRITE does not truncate
    File file(InternalFS);
    if (!file.open(TMP_PATH, FILE_O_WRITE)) {
        return false;
    }
    size_t written = file.write(reinterpret_cast<const uint8_t *>(data), len);
    file.close();
    if (written != len) {
        InternalFS.remove(TMP_PATH);
        return false;
    }
    if (!InternalFS.rename(TMP_PATH, CONFIG_PATH)) {
        InternalFS.remove(CONFIG_PATH);
        if (!InternalFS.rename(TMP_PATH, CONFIG_PATH)) {
            InternalFS.remove(TMP_PATH);
            return false;
        }
    }
    return true;
}

bool save(const Settings &s) {
    if (!mounted) {
        return false;
    }
    JsonDocument doc;
    size_t len = 0;
    if (readConfigFile(len)) {
        if (deserializeJson(doc, fileBuf, len) || !doc.is<JsonObject>()) {
            Serial.println(F("Settings: config.json unreadable - not overwriting"));
            return false;
        }
    } else if (InternalFS.exists(CONFIG_PATH)) {
        // There but unreadable (too large, read error): writing a two-key
        // file over it would throw away the full firmware's settings
        Serial.println(F("Settings: config.json unreadable - not overwriting"));
        return false;
    } else {
        doc.to<JsonObject>(); // no file yet: the full firmware adds its keys on its next boot
    }
    doc["auto_shutdown_timeout"] = s.autoShutdownTimeout;
    doc["laser_timeout"] = s.laserTimeout;

    if (measureJsonPretty(doc) >= sizeof(fileBuf)) {
        Serial.println(F("Settings: config.json too large"));
        return false;
    }
    len = serializeJsonPretty(doc, fileBuf, sizeof(fileBuf));
    bool ok = len > 0 && writeAtomic(fileBuf, len);
    Serial.println(ok ? F("Settings: saved") : F("Settings: save FAILED"));
    return ok;
}

} // namespace SettingsStore
