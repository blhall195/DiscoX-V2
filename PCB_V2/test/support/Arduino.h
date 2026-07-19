#pragma once

#include <stdint.h>

#ifndef LOW
#define LOW 0
#endif

#ifndef HIGH
#define HIGH 1
#endif

#ifndef INPUT
#define INPUT 0x0
#endif

#ifndef OUTPUT
#define OUTPUT 0x1
#endif

#ifndef INPUT_PULLUP
#define INPUT_PULLUP 0x2
#endif

// Pin constants as typed constants, NOT macros — real Arduino cores do the
// same, and macros here collide with identifiers inside Eigen (A0, A2, ...)
static const uint8_t A0 = 14;
static const uint8_t A1 = 15;
static const uint8_t A2 = 16;
static const uint8_t A3 = 17;
static const uint8_t A4 = 18;
static const uint8_t SCK = 19;

// Flash-string helper is a no-op on host
#ifndef F
#define F(x) (x)
#endif

extern "C" {
uint32_t millis();
int digitalRead(uint8_t pin);
void pinMode(uint8_t pin, uint8_t mode);
}

// Minimal no-op Serial so host builds can link code with debug prints
// (e.g. mag_cal/calibration.cpp)
class SerialStub {
  public:
    template <typename T> void print(const T &) {}
    template <typename T> void print(const T &, int) {}
    template <typename T> void println(const T &) {}
    template <typename T> void println(const T &, int) {}
    void println() {}
    void flush() {}
};
extern SerialStub Serial;

void testSetMillis(uint32_t value);
void testSetDigitalRead(uint8_t pin, int value);
void testResetArduinoStubs();