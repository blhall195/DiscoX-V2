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

#ifndef A0
#define A0 14
#endif

#ifndef A1
#define A1 15
#endif

#ifndef A2
#define A2 16
#endif

#ifndef A3
#define A3 17
#endif

#ifndef A4
#define A4 18
#endif

#ifndef SCK
#define SCK 19
#endif

extern "C" {
uint32_t millis();
int digitalRead(uint8_t pin);
void pinMode(uint8_t pin, uint8_t mode);
}

void testSetMillis(uint32_t value);
void testSetDigitalRead(uint8_t pin, int value);
void testResetArduinoStubs();