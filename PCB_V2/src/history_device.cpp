#include "history.h"

#include "flash_layout.h"
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include <flash/flash_nrf5x.h>

namespace History {
namespace {

// Raw internal flash through the core's SoftDevice-safe HAL, the one LittleFS
// and the FAT partition use. Its single page cache is shared with them, so
// every batch holds the LittleFS mutex: the loop task is the only other
// writer we know of apart from Bluefruit saving bonds, which goes through
// LittleFS and so takes the same mutex.
class FlashStorage : public Storage {
  public:
    void read(uint32_t off, void *dst, uint32_t len) override {
        flash_nrf5x_read(dst, FlashLayout::HISTORY_START + off, len);
    }
    bool write(uint32_t off, const void *src, uint32_t len) override {
        if (off + len > FlashLayout::HISTORY_SIZE) {
            return false;
        }
        return flash_nrf5x_write(FlashLayout::HISTORY_START + off, src, len) == (int)len;
    }
    void flush() override { flash_nrf5x_flush(); }
    void lock() override { InternalFS._lockFS(); }
    void unlock() override { InternalFS._unlockFS(); }
};

FlashStorage s_storage;
Log s_log(s_storage, FlashLayout::HISTORY_SIZE / FlashLayout::PAGE_SIZE);

} // namespace

Log &device() { return s_log; }

} // namespace History
