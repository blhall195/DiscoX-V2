// Charge policy: states, the 80/90% hold and resume, the 4.1 V cap, and the
// old-board fallback (no CHG/CE wiring).

#include "charge_policy.h"
#include <math.h>
#include <unity.h>

using namespace ChargePolicy;

void setUp() {}
void tearDown() {}

static Inputs plugged(float soc, float volts, bool chgLow, uint8_t limit = 100) {
    Inputs in;
    in.usb = true;
    in.hasControl = true;
    in.chgLow = chgLow;
    in.gaugeOk = true;
    in.soc = soc;
    in.volts = volts;
    in.limit = limit;
    return in;
}

// ── Unplugged ───────────────────────────────────────────────────────

void test_unplugged_enables_charging_and_clears_hold() {
    Memory mem;
    mem.held = true;
    Inputs in = plugged(90.0f, 4.1f, false, 90);
    in.usb = false;
    Output out = step(mem, in);
    TEST_ASSERT_EQUAL(State::UNPLUGGED, out.state);
    TEST_ASSERT_FALSE(out.disableCharging);
    TEST_ASSERT_FALSE(mem.held);
}

// ── 100%: the charger decides ───────────────────────────────────────

void test_full_limit_never_disables() {
    Memory mem;
    Output out = step(mem, plugged(99.0f, 4.19f, true));
    TEST_ASSERT_EQUAL(State::CHARGING, out.state);
    TEST_ASSERT_FALSE(out.disableCharging);
}

void test_full_limit_charger_finished_is_complete() {
    Memory mem;
    Output out = step(mem, plugged(97.0f, 4.15f, false));
    TEST_ASSERT_EQUAL(State::COMPLETE, out.state);
    TEST_ASSERT_FALSE(out.disableCharging);
}

void test_charger_idle_low_down_is_paused() {
    Memory mem;
    Output out = step(mem, plugged(40.0f, 3.75f, false));
    TEST_ASSERT_EQUAL(State::PAUSED, out.state);
    TEST_ASSERT_FALSE(out.disableCharging);
}

// ── 80/90%: firmware hold ───────────────────────────────────────────

void test_limit_holds_at_soc() {
    Memory mem;
    Output out = step(mem, plugged(89.9f, 4.02f, true, 90));
    TEST_ASSERT_EQUAL(State::CHARGING, out.state);
    TEST_ASSERT_FALSE(out.disableCharging);
    out = step(mem, plugged(90.0f, 4.03f, true, 90));
    TEST_ASSERT_EQUAL(State::COMPLETE, out.state);
    TEST_ASSERT_TRUE(out.disableCharging);
    TEST_ASSERT_TRUE(mem.held);
}

void test_limit_voltage_cap_trips_before_soc() {
    Memory mem;
    Output out = step(mem, plugged(70.0f, 4.10f, true, 90)); // gauge reading low, cell already at the cap
    TEST_ASSERT_TRUE(out.disableCharging);
    TEST_ASSERT_EQUAL(State::COMPLETE, out.state);
}

void test_limit_stays_held_inside_hysteresis() {
    Memory mem;
    step(mem, plugged(90.0f, 4.03f, true, 90));
    // CHG is high while held — must still report complete, not paused
    Output out = step(mem, plugged(86.0f, 3.98f, false, 90));
    TEST_ASSERT_EQUAL(State::COMPLETE, out.state);
    TEST_ASSERT_TRUE(out.disableCharging);
}

void test_limit_resumes_below_gap() {
    Memory mem;
    step(mem, plugged(90.0f, 4.03f, true, 90));
    Output out = step(mem, plugged(84.9f, 3.97f, false, 90));
    TEST_ASSERT_EQUAL(State::CHARGING, out.state);
    TEST_ASSERT_FALSE(out.disableCharging);
    TEST_ASSERT_FALSE(mem.held);
}

void test_limit_no_resume_while_voltage_high() {
    Memory mem;
    step(mem, plugged(90.0f, 4.08f, true, 90));
    Output out = step(mem, plugged(80.0f, 4.06f, false, 90)); // gauge drifted, cell still high
    TEST_ASSERT_TRUE(out.disableCharging);
}

void test_raising_limit_to_100_releases_hold() {
    Memory mem;
    step(mem, plugged(80.0f, 3.98f, true, 80));
    TEST_ASSERT_TRUE(mem.held);
    Output out = step(mem, plugged(80.0f, 3.98f, false, 100));
    TEST_ASSERT_FALSE(out.disableCharging);
    TEST_ASSERT_FALSE(mem.held);
}

void test_limit_without_gauge_leaves_charger_alone() {
    Memory mem;
    Inputs in = plugged(0.0f, 0.0f, true, 80);
    in.gaugeOk = false;
    Output out = step(mem, in);
    TEST_ASSERT_FALSE(out.disableCharging);
    TEST_ASSERT_EQUAL(State::CHARGING, out.state);
}

// ── Old boards ──────────────────────────────────────────────────────

void test_old_board_never_disables_and_uses_gauge_for_complete() {
    Memory mem;
    Inputs in = plugged(95.0f, 4.15f, false, 80); // limit ignored, CHG not wired
    in.hasControl = false;
    Output out = step(mem, in);
    TEST_ASSERT_EQUAL(State::CHARGING, out.state);
    TEST_ASSERT_FALSE(out.disableCharging);
    in.soc = 99.2f;
    out = step(mem, in);
    TEST_ASSERT_EQUAL(State::COMPLETE, out.state);
    TEST_ASSERT_FALSE(out.disableCharging);
}

// ── Helpers ─────────────────────────────────────────────────────────

void test_sanitise_limit() {
    TEST_ASSERT_EQUAL_UINT8(80, sanitiseLimit(80));
    TEST_ASSERT_EQUAL_UINT8(90, sanitiseLimit(90));
    TEST_ASSERT_EQUAL_UINT8(100, sanitiseLimit(100));
    TEST_ASSERT_EQUAL_UINT8(100, sanitiseLimit(85));
    TEST_ASSERT_EQUAL_UINT8(100, sanitiseLimit(0));
    TEST_ASSERT_EQUAL_UINT8(100, sanitiseLimit(-5));
}

void test_led_colours() {
    TEST_ASSERT_EQUAL(Led::RED, ledFor(State::CHARGING, 79.9f));
    TEST_ASSERT_EQUAL(Led::ORANGE, ledFor(State::CHARGING, 80.0f));
    TEST_ASSERT_EQUAL(Led::GREEN, ledFor(State::COMPLETE, 80.0f));
    TEST_ASSERT_EQUAL(Led::ORANGE_PULSE, ledFor(State::PAUSED, 50.0f));
    TEST_ASSERT_EQUAL(Led::NONE, ledFor(State::UNPLUGGED, 50.0f));
}

void test_display_percent_clamps() {
    TEST_ASSERT_EQUAL_FLOAT(100.0f, displayPercent(103.4f));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, displayPercent(-1.0f));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, displayPercent(NAN));
    TEST_ASSERT_EQUAL_FLOAT(73.41f, displayPercent(73.41f));
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_unplugged_enables_charging_and_clears_hold);
    RUN_TEST(test_full_limit_never_disables);
    RUN_TEST(test_full_limit_charger_finished_is_complete);
    RUN_TEST(test_charger_idle_low_down_is_paused);
    RUN_TEST(test_limit_holds_at_soc);
    RUN_TEST(test_limit_voltage_cap_trips_before_soc);
    RUN_TEST(test_limit_stays_held_inside_hysteresis);
    RUN_TEST(test_limit_resumes_below_gap);
    RUN_TEST(test_limit_no_resume_while_voltage_high);
    RUN_TEST(test_raising_limit_to_100_releases_hold);
    RUN_TEST(test_limit_without_gauge_leaves_charger_alone);
    RUN_TEST(test_old_board_never_disables_and_uses_gauge_for_complete);
    RUN_TEST(test_sanitise_limit);
    RUN_TEST(test_led_colours);
    RUN_TEST(test_display_percent_clamps);
    return UNITY_END();
}
