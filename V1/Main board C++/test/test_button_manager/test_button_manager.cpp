#include "button_manager.h"
#include <unity.h>

void setUp() {
    testResetArduinoStubs();
}

void tearDown() {}

static void setButtonState(Button button, int level) {
    switch (button) {
    case Button::MEASURE:
        testSetDigitalRead(PIN_BTN_MEASURE, level);
        break;
    case Button::DISCO:
        testSetDigitalRead(PIN_BTN_DISCO, level);
        break;
    case Button::CALIB:
        testSetDigitalRead(PIN_BTN_CALIB, level);
        break;
    case Button::SHUTDOWN:
        testSetDigitalRead(PIN_BTN_SHUTDOWN, level);
        break;
    case Button::FIRE:
        testSetDigitalRead(PIN_BTN_FIRE, level);
        break;
    }
}

void test_begin_reads_initial_state() {
    ButtonManager buttons;
    setButtonState(Button::MEASURE, LOW);
    setButtonState(Button::DISCO, HIGH);
    setButtonState(Button::CALIB, HIGH);
    setButtonState(Button::SHUTDOWN, HIGH);
    setButtonState(Button::FIRE, HIGH);

    buttons.begin();

    TEST_ASSERT_TRUE(buttons.isPressed(Button::MEASURE));
    TEST_ASSERT_FALSE(buttons.isPressed(Button::DISCO));
    TEST_ASSERT_FALSE(buttons.wasPressed(Button::MEASURE));
    TEST_ASSERT_FALSE(buttons.wasReleased(Button::MEASURE));
}

void test_press_debounce_and_edge() {
    ButtonManager buttons;
    buttons.begin();

    testSetMillis(0);
    setButtonState(Button::MEASURE, HIGH);
    buttons.update();
    TEST_ASSERT_FALSE(buttons.isPressed(Button::MEASURE));
    TEST_ASSERT_FALSE(buttons.wasPressed(Button::MEASURE));

    testSetMillis(Timing::BUTTON_DEBOUNCE_MS - 1);
    setButtonState(Button::MEASURE, LOW);
    buttons.update();
    TEST_ASSERT_FALSE(buttons.isPressed(Button::MEASURE));
    TEST_ASSERT_FALSE(buttons.wasPressed(Button::MEASURE));

    testSetMillis(Timing::BUTTON_DEBOUNCE_MS);
    buttons.update();
    TEST_ASSERT_TRUE(buttons.isPressed(Button::MEASURE));
    TEST_ASSERT_TRUE(buttons.wasPressed(Button::MEASURE));
    TEST_ASSERT_FALSE(buttons.wasPressed(Button::MEASURE));
}

void test_release_debounce_and_edge() {
    ButtonManager buttons;
    setButtonState(Button::DISCO, LOW);
    buttons.begin();

    testSetMillis(0);
    buttons.update();
    TEST_ASSERT_TRUE(buttons.isPressed(Button::DISCO));

    testSetMillis(Timing::BUTTON_DEBOUNCE_MS - 1);
    setButtonState(Button::DISCO, HIGH);
    buttons.update();
    TEST_ASSERT_TRUE(buttons.isPressed(Button::DISCO));
    TEST_ASSERT_FALSE(buttons.wasReleased(Button::DISCO));

    testSetMillis(Timing::BUTTON_DEBOUNCE_MS);
    buttons.update();
    TEST_ASSERT_FALSE(buttons.isPressed(Button::DISCO));
    TEST_ASSERT_TRUE(buttons.wasReleased(Button::DISCO));
    TEST_ASSERT_FALSE(buttons.wasReleased(Button::DISCO));
}

void test_buttons_do_not_cross_talk() {
    ButtonManager buttons;
    buttons.begin();

    testSetMillis(0);
    setButtonState(Button::CALIB, LOW);
    setButtonState(Button::FIRE, HIGH);
    buttons.update();

    testSetMillis(Timing::BUTTON_DEBOUNCE_MS);
    buttons.update();

    TEST_ASSERT_TRUE(buttons.wasPressed(Button::CALIB));
    TEST_ASSERT_FALSE(buttons.wasPressed(Button::FIRE));
    TEST_ASSERT_TRUE(buttons.isPressed(Button::CALIB));
    TEST_ASSERT_FALSE(buttons.isPressed(Button::FIRE));
}

void test_button_names() {
    TEST_ASSERT_EQUAL_STRING("Measure", ButtonManager::name(Button::MEASURE));
    TEST_ASSERT_EQUAL_STRING("Disco", ButtonManager::name(Button::DISCO));
    TEST_ASSERT_EQUAL_STRING("Calib", ButtonManager::name(Button::CALIB));
    TEST_ASSERT_EQUAL_STRING("Shutdown", ButtonManager::name(Button::SHUTDOWN));
    TEST_ASSERT_EQUAL_STRING("Fire", ButtonManager::name(Button::FIRE));
}

int main() {
    UNITY_BEGIN();

    RUN_TEST(test_begin_reads_initial_state);
    RUN_TEST(test_press_debounce_and_edge);
    RUN_TEST(test_release_debounce_and_edge);
    RUN_TEST(test_buttons_do_not_cross_talk);
    RUN_TEST(test_button_names);

    return UNITY_END();
}