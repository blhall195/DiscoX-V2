#include "Arduino.h"

#include <array>

namespace {
std::array<int, 256> pinValues = [] {
    std::array<int, 256> values{};
    values.fill(HIGH);
    return values;
}();

uint32_t currentMillis = 0;
} // namespace

SerialStub Serial;

extern "C" uint32_t millis() { return currentMillis; }

extern "C" int digitalRead(uint8_t pin) { return pinValues[pin]; }

extern "C" void pinMode(uint8_t, uint8_t) {}

void testSetMillis(uint32_t value) { currentMillis = value; }

void testSetDigitalRead(uint8_t pin, int value) { pinValues[pin] = value; }

void testResetArduinoStubs() {
    currentMillis = 0;
    pinValues.fill(HIGH);
}