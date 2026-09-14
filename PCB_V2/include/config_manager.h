#pragma once

#include "device_context.h"
#include <Arduino.h>

namespace MagCal {
struct CalibrationBinary;
}

// Boot-mode flag names
namespace Flags {
constexpr const char *CALIBRATION = "calibration";
constexpr const char *MENU = "menu";
constexpr const char *SNAKE = "snake";
constexpr const char *USB_DRIVE = "usb_drive";   // reboot into USB drive mode
constexpr const char *USB_IMPORT = "usb_import"; // FAT files may have host edits — import at boot
} // namespace Flags

// V2: persistence lives on the nRF52840's internal flash via LittleFS
// (InternalFileSystem) — the V1 QSPI chip does not exist on this board.
// Same public API and file set as V1 (minus the USB-MSC flash accessor).
class ConfigManager {
  public:
    // Mount LittleFS on internal flash (formats it on first ever boot).
    // Returns false if the mount fails even after a format (device runs
    // with defaults).
    bool begin();

    bool isReady() const { return mounted_; }

    // True if filesystem was (re)formatted this boot (data was lost)
    bool wasReformatted() const { return reformatted_; }

    // Recovery: erase everything (format LittleFS) and remount.
    // Used by the menu's "Reformat Storage" action; caller reboots after.
    bool reformat();

    // ── Settings persistence ────────────────────────────────────────

    // Load config from /config.json. Returns false if missing/corrupt.
    // On failure, cfg is left unchanged (caller uses defaults).
    bool loadConfig(Config &cfg);

    // Save config to /config.json. Returns false on write failure.
    bool saveConfig(const Config &cfg);

    // Write caller-provided JSON text straight to /config.json (atomic).
    // Used by the USB-drive import — caller must have validated the JSON.
    bool saveConfigJsonRaw(const char *json, size_t len);

    // True if the last successful loadConfig() found expected keys missing
    // from the stored file — i.e. this firmware added settings since the
    // file was written. Caller should saveConfig() to migrate the file.
    bool loadedConfigIncomplete() const { return loadMissing_; }

    // Print the stored /config.json verbatim to a stream (debug/commissioning).
    bool printConfig(Stream &out);

    // ── Calibration data ────────────────────────────────────────────

    // Load calibration JSON into caller-provided buffer.
    // Buffer is null-terminated on success. bytesRead excludes the null.
    bool loadCalibrationJson(char *buf, size_t bufSize, size_t &bytesRead);

    // Save calibration JSON string to /calibration.json.
    bool saveCalibrationJson(const char *json, size_t len);

    // Load calibration binary from /calibration.bin.
    bool loadCalibrationBinary(MagCal::CalibrationBinary &out);

    // Save calibration binary to /calibration.bin.
    bool saveCalibrationBinary(const MagCal::CalibrationBinary &data);

    // Delete /calibration.bin. The boot loader prefers binary over JSON, so
    // a USB-drive JSON import must remove the stale binary to take effect.
    bool removeCalibrationBinary();

    // ── Calibration quality metrics ───────────────────────────────────

    struct CalMetrics {
        float mag;
        float grav;
        float accuracy;
    };

    // Save calibration quality metrics to /cal_metrics.bin.
    bool saveCalMetrics(const CalMetrics &m);

    // Load calibration quality metrics from /cal_metrics.bin.
    bool loadCalMetrics(CalMetrics &m);

    // ── Pending readings (offline queue) ────────────────────────────

    // Buffer a reading in RAM (fast, no flash I/O).
    bool appendPendingReading(float az, float inc, float dist);

    // Write any RAM-buffered readings to flash. Call from main loop during
    // idle — internal-flash writes go through the SoftDevice and can block
    // for a few ms, which would disturb the measurement loop (and V1's QSPI
    // crashed outright mid-measurement, so the deferred design stays).
    // Also call from doShutdown() before pulling the KILL pin.
    bool syncPendingToFlash();

    // True if RAM buffer has unsaved readings.
    bool hasPendingToSync() const { return pendingBufCount_ > 0; }

    // Boot-time write self-test: create + rename + remove a tiny probe file.
    // A corrupt filesystem can mount and read fine while every commit fails
    // (seen 2026-07-19) — this catches that state at boot instead of at the
    // end of a calibration. Costs a few metadata commits per boot.
    bool storageWriteTest();

    // Write `len` bytes through the same atomic path as calibration saves,
    // read them back, verify byte-for-byte, then delete the scratch file.
    // Used by the menu's calibration save dry-run — never touches the real
    // /calibration.{json,bin}.
    bool testFileRoundTrip(const uint8_t *data, size_t len);

    // Count of readings still awaiting delivery: undelivered lines in
    // /pending.txt + whatever is still in the RAM buffer.
    uint16_t countPendingReadings();

    // ── Delivery cursor ─────────────────────────────────────────────
    // A reading leaves /pending.txt only once the phone has ACKed it, so a
    // dropout mid-flight can never lose one. The cursor is the byte offset of
    // the oldest undelivered line; the cycle is peek → send → (ACK) → commit.

    // True if anything is waiting to be delivered. Cheap — no flash I/O, so
    // the drain pump can call it every loop tick.
    bool hasUndelivered() const { return drainPending_ || pendingBufCount_ > 0; }

    // Arm the drain for readings already on flash at boot.
    void beginDrain();

    // Read the oldest undelivered reading without consuming it. False once
    // every line has been delivered (or there is no file).
    bool peekOldestPending(float &az, float &inc, float &dist);

    // Mark the last peeked reading as delivered. Only call once the phone has
    // acknowledged it — this is the point of no return. Returns false if there
    // was nothing outstanding (the store was cleared under us), so a stale
    // in-flight flag can never advance the cursor past a fresh reading.
    bool commitOldestPending();

    // Drop the fully-delivered file and rearm for new readings. Deliberately
    // leaves the RAM buffer alone: it may hold readings taken since the drain
    // started, which have NOT been delivered.
    bool clearDeliveredPending();

    // Delete /pending.txt, clear the RAM buffer and reset the cursor. This
    // discards undelivered data on purpose — menu "delete pending readings".
    bool clearPendingReadings();

    // ── Flag files (boot mode triggers) ─────────────────────────────

    // Create /flags/<name> (empty file).
    bool writeFlag(const char *name);

    // Check if /flags/<name> exists.
    bool hasFlag(const char *name);

    // Delete /flags/<name>.
    bool clearFlag(const char *name);

  private:
    bool mounted_ = false;
    bool loadMissing_ = false;
    bool reformatted_ = false; // set if LittleFS was formatted this boot

    // RAM buffer for pending readings (avoids flash writes mid-measurement)
    static const uint8_t MAX_PENDING_BUF = 20;
    struct PendingEntry {
        float az, inc, dist;
    };
    PendingEntry pendingBuf_[MAX_PENDING_BUF];
    uint8_t pendingBufCount_ = 0;

    // Delivery cursor into /pending.txt (see the drain API above). RAM-only by
    // choice: a reboot mid-drain replays already-delivered readings rather than
    // risking the loss of undelivered ones. A duplicate leg is visible in the
    // survey app and can be deleted; a missing one is not.
    uint32_t drainOffset_ = 0;     // start of the oldest undelivered line
    uint32_t drainNextOffset_ = 0; // just past the last line peeked
    bool drainPending_ = false;
    bool drainPeeked_ = false; // a peek is outstanding, so a commit is meaningful

    // Where the outstanding peek came from. Normally always FILE — the pump
    // syncs the RAM buffer first so the file is the single ordered source.
    // RAM is the degraded path for when flash cannot take the readings at all.
    enum class DrainSource : uint8_t { NONE, FILE, RAM };
    DrainSource drainSource_ = DrainSource::NONE;

    static bool parsePendingLine(char *line, float &az, float &inc, float &dist);
    bool peekFilePending(float &az, float &inc, float &dist);

    void buildFlagPath(const char *name, char *path, size_t pathSize);
};
