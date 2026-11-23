#pragma once

#include <stddef.h>
#include <stdint.h>

// ── Reading history ──────────────────────────────────────────────────
// Every shot the device takes, kept whether or not the phone received it, so
// a caver can scroll back through them on the screen (DOWN/UP in normal mode)
// or read them off the USB drive (HISTORY.CSV). Separate from the delivery
// queue (/pending.txt), which forgets a reading once the phone ACKs it.
//
// Storage is a raw ring of fixed 16-byte records over whole 4 KB flash pages
// (FlashLayout::HISTORY_*), not a LittleFS file: LittleFS has 28 KB in all,
// shared with settings, calibration and BLE bonds. Each record carries a
// lifetime sequence number and a CRC, so boot finds the newest by scanning,
// and anything else in the region (erased flash, an old firmware image's
// tail, a torn write) is simply not a record.
//
// Wear: the core's flash HAL rewrites a whole page per flush, so every sync
// erases one page. Spreading the ring over 8 pages gives each page about
// shots/8 erases, ~80,000 shots at the nRF52840's rated 10,000 cycles. The
// ring holds 2048 records; only the newest KEEP are shown or exported.
//
// The logic here is platform-free (unit-tested on the host); the flash
// access is behind History::Storage.

namespace History {

enum class Kind : uint8_t {
    SHOT = 0,  // a survey shot that didn't complete a leg
    SPLAY = 1, // quick shot (UP)
    LEG = 2,   // the shot that made 3 consistent readings
};

struct Entry {
    float az = 0.0f;   // degrees, 0..360
    float inc = 0.0f;  // degrees, -90..+90
    float dist = 0.0f; // metres
    Kind kind = Kind::SHOT;
};

// On-flash record. Stored to 0.01° and 1 mm: finer than the screen (0.1°,
// 1 cm) and the SAP6 floats are not needed back.
struct Record {
    uint32_t seq;     // 1, 2, 3... lifetime count; 0xFFFFFFFF = erased slot
    uint16_t azCenti; // 0..35999
    int16_t incCenti; // -9000..9000
    uint32_t distMm;
    uint8_t kind;
    uint8_t reserved; // 0xFF
    uint16_t crc;     // CRC-16/CCITT over the 14 bytes above
};
static_assert(sizeof(Record) == 16, "history record must stay 16 bytes (no padding)");

Record pack(const Entry &e, uint32_t seq);
bool unpack(const Record &r, Entry &out); // false if r is not a valid record
uint16_t crc16(const uint8_t *data, size_t len);

// Where the ring lives. Offsets are bytes from the start of the region.
// lock()/unlock() bracket every batch of calls; the device implementation
// takes the LittleFS mutex, because the flash page cache is shared with
// LittleFS (and BLE bond writes happen on another task).
class Storage {
  public:
    virtual ~Storage() = default;
    virtual void read(uint32_t off, void *dst, uint32_t len) = 0;
    virtual bool write(uint32_t off, const void *src, uint32_t len) = 0;
    virtual void flush() = 0;
    virtual void lock() {}
    virtual void unlock() {}
};

class Log {
  public:
    static constexpr uint32_t PAGE_SIZE = 4096;
    static constexpr uint32_t PER_PAGE = PAGE_SIZE / sizeof(Record); // 256
    static constexpr uint16_t KEEP = 500; // shown on screen / exported
    static constexpr uint8_t RAM_SLOTS = 20;

    Log(Storage &storage, uint32_t pages);

    // Scan the region for the newest record. Read-only, safe at any point
    // in setup (no LittleFS needed).
    void begin();

    // Buffer a reading in RAM (no flash I/O: called mid-measurement). A full
    // buffer is synced first.
    void add(const Entry &e);

    bool hasUnsynced() const { return ramCount_ > 0; }

    // Write buffered readings to flash. Call while idle, and before power-off.
    // False if a write failed; the batch is dropped either way so a broken
    // flash can't make the loop retry forever.
    bool sync();

    // Readings available to browse, newest first, at most KEEP. Includes
    // ones still in RAM.
    uint16_t count() const;

    // back = 0 is the newest reading. False past the end.
    bool get(uint16_t back, Entry &out) const;

    // Lifetime number of readings recorded (on flash + in RAM).
    uint32_t total() const { return newestSeq_ + ramCount_; }

  private:
    Storage &s_;
    uint32_t slots_;
    bool haveAny_ = false;
    uint32_t newestSeq_ = 0;  // seq of the newest record on flash
    uint32_t newestSlot_ = 0; // its slot
    uint32_t nextSlot_ = 0;   // where the next record goes

    Entry ram_[RAM_SLOTS];
    uint8_t ramCount_ = 0;

    bool readFlash(uint32_t k, Entry &out) const; // k = 0 newest on flash; no lock
    bool wipePage(uint32_t page);
};

} // namespace History
