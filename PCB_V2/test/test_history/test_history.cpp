// Reading history ring: record packing, boot scan, RAM buffering, wrap-around,
// and what happens with erased, foreign or damaged flash in the region.

#include "history_log.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <unity.h>

using namespace History;

static constexpr uint32_t PAGES = 8;
static constexpr uint32_t REGION = PAGES * Log::PAGE_SIZE;

// RAM stand-in for the flash region. write() is a plain copy: the real HAL's
// page cache erases and reprograms the page, which ends in the same bytes.
class FakeFlash : public Storage {
  public:
    uint8_t mem[REGION];
    uint32_t flushes = 0;
    bool outOfRange = false;
    int lockDepth = 0;

    FakeFlash() { erase(); }
    void erase() { memset(mem, 0xFF, sizeof(mem)); }

    void read(uint32_t off, void *dst, uint32_t len) override {
        if (off + len > REGION) {
            outOfRange = true;
            return;
        }
        memcpy(dst, mem + off, len);
    }
    bool write(uint32_t off, const void *src, uint32_t len) override {
        if (off + len > REGION) {
            outOfRange = true;
            return false;
        }
        memcpy(mem + off, src, len);
        return true;
    }
    void flush() override { flushes++; }
    void lock() override { lockDepth++; }
    void unlock() override { lockDepth--; }
};

static FakeFlash flash;

void setUp() { flash = FakeFlash(); }
void tearDown() {}

static Entry entry(float az, float inc, float dist, Kind k = Kind::SHOT) {
    Entry e;
    e.az = az;
    e.inc = inc;
    e.dist = dist;
    e.kind = k;
    return e;
}

// Reading i of a sequence, distinct enough to tell apart after a wrap
static Entry nth(uint32_t i) {
    return entry((float)(i % 360), (float)(i % 90) - 45.0f, 1.0f + (float)(i % 1000) / 100.0f,
                 (Kind)(i % 3));
}

static void assertEntry(const Entry &want, const Entry &got) {
    TEST_ASSERT_FLOAT_WITHIN(0.006f, want.az, got.az);
    TEST_ASSERT_FLOAT_WITHIN(0.006f, want.inc, got.inc);
    TEST_ASSERT_FLOAT_WITHIN(0.0006f, want.dist, got.dist);
    TEST_ASSERT_EQUAL((int)want.kind, (int)got.kind);
}

// ── Records ─────────────────────────────────────────────────────────

void test_pack_round_trip() {
    Entry in = entry(123.456f, -12.344f, 15.432f, Kind::LEG);
    Entry out;
    TEST_ASSERT_TRUE(unpack(pack(in, 7), out));
    assertEntry(in, out);
}

void test_pack_wraps_and_clamps() {
    Entry out;
    TEST_ASSERT_TRUE(unpack(pack(entry(359.999f, 0, 1), 1), out));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, out.az);
    TEST_ASSERT_TRUE(unpack(pack(entry(-10.0f, 95.0f, -2.0f), 1), out));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 350.0f, out.az);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 90.0f, out.inc);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, out.dist);
    TEST_ASSERT_TRUE(unpack(pack(entry(NAN, NAN, NAN), 1), out));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, out.az);
}

void test_unpack_rejects_erased_and_corrupt() {
    Record r;
    Entry out;
    memset(&r, 0xFF, sizeof(r));
    TEST_ASSERT_FALSE(unpack(r, out));

    r = pack(entry(10, 10, 10), 5);
    r.distMm ^= 1; // one bit flipped
    TEST_ASSERT_FALSE(unpack(r, out));

    r = pack(entry(10, 10, 10), 0); // seq 0 is never written
    TEST_ASSERT_FALSE(unpack(r, out));
}

// ── Log ─────────────────────────────────────────────────────────────

void test_empty_region() {
    Log log(flash, PAGES);
    log.begin();
    Entry e;
    TEST_ASSERT_EQUAL(0, log.count());
    TEST_ASSERT_FALSE(log.get(0, e));
    TEST_ASSERT_EQUAL(0, (int)log.total());
    TEST_ASSERT_TRUE(log.sync()); // nothing to do
    TEST_ASSERT_EQUAL(0, (int)flash.flushes);
}

void test_unsynced_readings_browse_newest_first() {
    Log log(flash, PAGES);
    log.begin();
    log.add(nth(1));
    log.add(nth(2));
    log.add(nth(3));
    TEST_ASSERT_TRUE(log.hasUnsynced());
    TEST_ASSERT_EQUAL(3, log.count());
    Entry e;
    TEST_ASSERT_TRUE(log.get(0, e));
    assertEntry(nth(3), e);
    TEST_ASSERT_TRUE(log.get(2, e));
    assertEntry(nth(1), e);
    TEST_ASSERT_FALSE(log.get(3, e));
    TEST_ASSERT_EQUAL(0, (int)flash.flushes); // RAM only until sync
}

void test_sync_survives_reboot() {
    {
        Log log(flash, PAGES);
        log.begin();
        for (uint32_t i = 1; i <= 5; i++) {
            log.add(nth(i));
        }
        TEST_ASSERT_TRUE(log.sync());
        TEST_ASSERT_FALSE(log.hasUnsynced());
        TEST_ASSERT_EQUAL(1, (int)flash.flushes); // one batch, one flush
    }
    Log again(flash, PAGES);
    again.begin();
    TEST_ASSERT_EQUAL(5, again.count());
    TEST_ASSERT_EQUAL(5, (int)again.total());
    Entry e;
    for (uint16_t back = 0; back < 5; back++) {
        TEST_ASSERT_TRUE(again.get(back, e));
        assertEntry(nth(5 - back), e);
    }
}

void test_mixed_ram_and_flash() {
    Log log(flash, PAGES);
    log.begin();
    log.add(nth(1));
    log.add(nth(2));
    log.sync();
    log.add(nth(3));
    TEST_ASSERT_EQUAL(3, log.count());
    Entry e;
    TEST_ASSERT_TRUE(log.get(0, e));
    assertEntry(nth(3), e);
    TEST_ASSERT_TRUE(log.get(1, e));
    assertEntry(nth(2), e);
    TEST_ASSERT_TRUE(log.get(2, e));
    assertEntry(nth(1), e);
}

void test_full_ram_buffer_syncs_itself() {
    Log log(flash, PAGES);
    log.begin();
    for (uint32_t i = 1; i <= Log::RAM_SLOTS + 5; i++) {
        log.add(nth(i));
    }
    TEST_ASSERT_EQUAL(1, (int)flash.flushes);
    TEST_ASSERT_EQUAL(Log::RAM_SLOTS + 5, log.count());
    Entry e;
    TEST_ASSERT_TRUE(log.get(0, e));
    assertEntry(nth(Log::RAM_SLOTS + 5), e);
    TEST_ASSERT_TRUE(log.get(Log::RAM_SLOTS + 4, e));
    assertEntry(nth(1), e);
}

// Several laps of the ring, one sync per reading as on the device; a reboot
// at every step must find the same newest reading.
void test_wraps_and_keeps_newest() {
    const uint32_t n = PAGES * Log::PER_PAGE * 3 + 77;
    Log log(flash, PAGES);
    log.begin();
    for (uint32_t i = 1; i <= n; i++) {
        log.add(nth(i));
        log.sync();
        if (i % 997 == 0) {
            Log check(flash, PAGES);
            check.begin();
            Entry e;
            TEST_ASSERT_TRUE(check.get(0, e));
            assertEntry(nth(i), e);
        }
    }
    TEST_ASSERT_FALSE(flash.outOfRange);
    TEST_ASSERT_EQUAL(0, flash.lockDepth);

    Log again(flash, PAGES);
    again.begin();
    TEST_ASSERT_EQUAL(Log::KEEP, again.count());
    TEST_ASSERT_EQUAL((int)n, (int)again.total());
    Entry e;
    for (uint16_t back = 0; back < Log::KEEP; back++) {
        TEST_ASSERT_TRUE(again.get(back, e));
        assertEntry(nth(n - back), e);
    }
    TEST_ASSERT_FALSE(again.get(Log::KEEP, e));
}

// Entering a page wipes it, so an old lap's records can't follow the new ones
void test_entering_a_page_wipes_the_old_lap() {
    Log log(flash, PAGES);
    log.begin();
    const uint32_t lap = PAGES * Log::PER_PAGE;
    for (uint32_t i = 1; i <= lap + 1; i++) { // ring full, then one into page 0
        log.add(nth(i));
        log.sync();
    }
    // Page 0 now holds only the newest record; the rest of it is erased
    Record r;
    memcpy(&r, flash.mem + sizeof(Record), sizeof(r));
    Entry e;
    TEST_ASSERT_FALSE(unpack(r, e));
    TEST_ASSERT_EQUAL(Log::KEEP, log.count());
}

// A region holding something else entirely (an old image's tail): no
// records, and recording starts cleanly over it.
void test_foreign_bytes_are_not_records() {
    srand(1234);
    for (uint32_t i = 0; i < REGION; i++) {
        flash.mem[i] = (uint8_t)rand();
    }
    Log log(flash, PAGES);
    log.begin();
    TEST_ASSERT_EQUAL(0, log.count());

    log.add(nth(1));
    log.add(nth(2));
    log.sync();
    Log again(flash, PAGES);
    again.begin();
    TEST_ASSERT_EQUAL(2, again.count());
    Entry e;
    TEST_ASSERT_TRUE(again.get(0, e));
    assertEntry(nth(2), e);
}

// Power lost between a page's erase and its program: that page's readings
// are gone and the history ends there, but everything newer still shows.
void test_lost_page_ends_the_history() {
    Log log(flash, PAGES);
    log.begin();
    const uint32_t n = Log::PER_PAGE * 3 + 10; // pages 0..3 in use
    for (uint32_t i = 1; i <= n; i++) {
        log.add(nth(i));
    }
    log.sync();
    memset(flash.mem + 1 * Log::PAGE_SIZE, 0xFF, Log::PAGE_SIZE); // page 1 erased

    Log again(flash, PAGES);
    again.begin();
    // newest page 3 (10) + page 2 (256) survive
    TEST_ASSERT_EQUAL(10 + Log::PER_PAGE, again.count());
    Entry e;
    TEST_ASSERT_TRUE(again.get(0, e));
    assertEntry(nth(n), e);
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_pack_round_trip);
    RUN_TEST(test_pack_wraps_and_clamps);
    RUN_TEST(test_unpack_rejects_erased_and_corrupt);
    RUN_TEST(test_empty_region);
    RUN_TEST(test_unsynced_readings_browse_newest_first);
    RUN_TEST(test_sync_survives_reboot);
    RUN_TEST(test_mixed_ram_and_flash);
    RUN_TEST(test_full_ram_buffer_syncs_itself);
    RUN_TEST(test_wraps_and_keeps_newest);
    RUN_TEST(test_entering_a_page_wipes_the_old_lap);
    RUN_TEST(test_foreign_bytes_are_not_records);
    RUN_TEST(test_lost_page_ends_the_history);
    return UNITY_END();
}
