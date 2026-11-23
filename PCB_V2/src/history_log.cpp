#include "history_log.h"

#include <math.h>
#include <string.h>

namespace History {

static constexpr uint32_t ERASED_SEQ = 0xFFFFFFFFu;
static constexpr size_t CRC_SPAN = offsetof(Record, crc);

uint16_t crc16(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t b = 0; b < 8; b++) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

Record pack(const Entry &e, uint32_t seq) {
    Record r;
    memset(&r, 0xFF, sizeof(r));
    r.seq = seq;

    long az = isfinite(e.az) ? lroundf(e.az * 100.0f) % 36000 : 0;
    if (az < 0) {
        az += 36000;
    }
    r.azCenti = (uint16_t)az;

    float inc = !isfinite(e.inc) ? 0.0f : e.inc > 90.0f ? 90.0f : (e.inc < -90.0f ? -90.0f : e.inc);
    r.incCenti = (int16_t)lroundf(inc * 100.0f);

    float mm = e.dist * 1000.0f;
    r.distMm = !(mm > 0.0f) ? 0u : (mm >= 4.0e9f ? 4000000000u : (uint32_t)lroundf(mm));

    r.kind = (uint8_t)e.kind;
    r.reserved = 0xFF;
    r.crc = crc16(reinterpret_cast<const uint8_t *>(&r), CRC_SPAN);
    return r;
}

bool unpack(const Record &r, Entry &out) {
    if (r.seq == ERASED_SEQ || r.seq == 0) {
        return false;
    }
    if (r.crc != crc16(reinterpret_cast<const uint8_t *>(&r), CRC_SPAN)) {
        return false;
    }
    if (r.kind > (uint8_t)Kind::LEG || r.azCenti >= 36000 || r.incCenti > 9000 || r.incCenti < -9000) {
        return false;
    }
    out.az = r.azCenti / 100.0f;
    out.inc = r.incCenti / 100.0f;
    out.dist = r.distMm / 1000.0f;
    out.kind = (Kind)r.kind;
    return true;
}

Log::Log(Storage &storage, uint32_t pages) : s_(storage), slots_(pages * PER_PAGE) {}

void Log::begin() {
    haveAny_ = false;
    newestSeq_ = 0;
    newestSlot_ = 0;

    s_.lock();
    for (uint32_t slot = 0; slot < slots_; slot++) {
        Record r;
        s_.read(slot * sizeof(Record), &r, sizeof(r));
        Entry e;
        if (unpack(r, e) && r.seq > newestSeq_) {
            newestSeq_ = r.seq;
            newestSlot_ = slot;
            haveAny_ = true;
        }
    }
    s_.unlock();

    nextSlot_ = haveAny_ ? (newestSlot_ + 1) % slots_ : 0;
}

void Log::add(const Entry &e) {
    if (ramCount_ >= RAM_SLOTS) {
        sync();
    }
    ram_[ramCount_++] = e;
}

// Fill a page with 0xFF through the page cache. Done as the ring enters a
// page, so the previous lap's records (or whatever was there before this
// firmware) can't sit after the new ones. Same cache session as the first
// record written into it, so still one erase.
bool Log::wipePage(uint32_t page) {
    static uint8_t ff[64];
    memset(ff, 0xFF, sizeof(ff));
    for (uint32_t off = 0; off < PAGE_SIZE; off += sizeof(ff)) {
        if (!s_.write(page * PAGE_SIZE + off, ff, sizeof(ff))) {
            return false;
        }
    }
    return true;
}

bool Log::sync() {
    if (ramCount_ == 0) {
        return true;
    }

    bool ok = true;
    s_.lock();
    for (uint8_t i = 0; i < ramCount_ && ok; i++) {
        uint32_t slot = nextSlot_;
        if (slot % PER_PAGE == 0) {
            ok = wipePage(slot / PER_PAGE);
        }
        if (ok) {
            Record r = pack(ram_[i], newestSeq_ + 1);
            ok = s_.write(slot * sizeof(Record), &r, sizeof(r));
        }
        if (ok) {
            newestSeq_++;
            newestSlot_ = slot;
            nextSlot_ = (slot + 1) % slots_;
            haveAny_ = true;
        }
    }
    s_.flush();
    s_.unlock();

    ramCount_ = 0;
    return ok;
}

bool Log::readFlash(uint32_t k, Entry &out) const {
    if (!haveAny_ || k >= slots_ || k >= newestSeq_) {
        return false;
    }
    uint32_t slot = (newestSlot_ + slots_ - k) % slots_;
    Record r;
    s_.read(slot * sizeof(Record), &r, sizeof(r));
    // The sequence must line up too: a slot holding an older lap's record,
    // or one wiped as the ring moved on, ends the history here.
    return unpack(r, out) && r.seq == newestSeq_ - k;
}

uint16_t Log::count() const {
    uint16_t n = ramCount_ < KEEP ? ramCount_ : KEEP;
    s_.lock();
    Entry e;
    for (uint32_t k = 0; n < KEEP && readFlash(k, e); k++) {
        n++;
    }
    s_.unlock();
    return n;
}

bool Log::get(uint16_t back, Entry &out) const {
    if (back >= KEEP) {
        return false;
    }
    if (back < ramCount_) {
        out = ram_[ramCount_ - 1 - back];
        return true;
    }
    s_.lock();
    bool ok = readFlash(back - ramCount_, out);
    s_.unlock();
    return ok;
}

} // namespace History
