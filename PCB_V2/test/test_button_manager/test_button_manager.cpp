#include "button_manager.h"
#include <unity.h>

void setUp() {
    testResetArduinoStubs();
}

void tearDown() {}

static void setButtonState(Button button, int level) {
    switch (button) {
    case Button::FIRE:
        testSetDigitalRead(PIN_BTN_FIRE, level);
        break;
    case Button::UP_DISCO:
        testSetDigitalRead(PIN_BTN_UP_DISCO, level);
        break;
    case Button::DOWN:
        testSetDigitalRead(PIN_BTN_DOWN, level);
        break;
    case Button::MENU:
        testSetDigitalRead(PIN_BTN_MENU, level);
        break;
    }
}

void test_begin_reads_initial_state() {
    ButtonManager buttons;
    setButtonState(Button::FIRE, LOW);
    setButtonState(Button::UP_DISCO, HIGH);
    setButtonState(Button::DOWN, HIGH);
    setButtonState(Button::MENU, HIGH);

    buttons.begin();

    TEST_ASSERT_TRUE(buttons.isPressed(Button::FIRE));
    TEST_ASSERT_FALSE(buttons.isPressed(Button::UP_DISCO));
    TEST_ASSERT_FALSE(buttons.wasPressed(Button::FIRE));
    TEST_ASSERT_FALSE(buttons.wasReleased(Button::FIRE));
}

void test_press_debounce_and_edge() {
    ButtonManager buttons;
    buttons.begin();

    testSetMillis(0);
    setButtonState(Button::FIRE, HIGH);
    buttons.update();
    TEST_ASSERT_FALSE(buttons.isPressed(Button::FIRE));
    TEST_ASSERT_FALSE(buttons.wasPressed(Button::FIRE));

    testSetMillis(Timing::BUTTON_DEBOUNCE_MS - 1);
    setButtonState(Button::FIRE, LOW);
    buttons.update();
    TEST_ASSERT_FALSE(buttons.isPressed(Button::FIRE));
    TEST_ASSERT_FALSE(buttons.wasPressed(Button::FIRE));

    testSetMillis(Timing::BUTTON_DEBOUNCE_MS);
    buttons.update();
    TEST_ASSERT_TRUE(buttons.isPressed(Button::FIRE));
    TEST_ASSERT_TRUE(buttons.wasPressed(Button::FIRE));
    TEST_ASSERT_FALSE(buttons.wasPressed(Button::FIRE));
}

void test_release_debounce_and_edge() {
    ButtonManager buttons;
    setButtonState(Button::UP_DISCO, LOW);
    buttons.begin();

    testSetMillis(0);
    buttons.update();
    TEST_ASSERT_TRUE(buttons.isPressed(Button::UP_DISCO));

    testSetMillis(Timing::BUTTON_DEBOUNCE_MS - 1);
    setButtonState(Button::UP_DISCO, HIGH);
    buttons.update();
    TEST_ASSERT_TRUE(buttons.isPressed(Button::UP_DISCO));
    TEST_ASSERT_FALSE(buttons.wasReleased(Button::UP_DISCO));

    testSetMillis(Timing::BUTTON_DEBOUNCE_MS);
    buttons.update();
    TEST_ASSERT_FALSE(buttons.isPressed(Button::UP_DISCO));
    TEST_ASSERT_TRUE(buttons.wasReleased(Button::UP_DISCO));
    TEST_ASSERT_FALSE(buttons.wasReleased(Button::UP_DISCO));
}

void test_buttons_do_not_cross_talk() {
    ButtonManager buttons;
    buttons.begin();

    testSetMillis(0);
    setButtonState(Button::DOWN, LOW);
    setButtonState(Button::FIRE, HIGH);
    buttons.update();

    testSetMillis(Timing::BUTTON_DEBOUNCE_MS);
    buttons.update();

    TEST_ASSERT_TRUE(buttons.wasPressed(Button::DOWN));
    TEST_ASSERT_FALSE(buttons.wasPressed(Button::FIRE));
    TEST_ASSERT_TRUE(buttons.isPressed(Button::DOWN));
    TEST_ASSERT_FALSE(buttons.isPressed(Button::FIRE));
}

void test_button_names() {
    TEST_ASSERT_EQUAL_STRING("Fire", ButtonManager::name(Button::FIRE));
    TEST_ASSERT_EQUAL_STRING("Up/Disco", ButtonManager::name(Button::UP_DISCO));
    TEST_ASSERT_EQUAL_STRING("Down", ButtonManager::name(Button::DOWN));
    TEST_ASSERT_EQUAL_STRING("Menu", ButtonManager::name(Button::MENU));
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
