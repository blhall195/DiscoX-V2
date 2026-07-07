#include "buzzer.h"

void Buzzer::begin(uint8_t pinA, uint8_t pinB) {
    pinA_ = pinA;
    pinB_ = pinB;
    pinMode(pinA_, OUTPUT);
    pinMode(pinB_, OUTPUT);
    off();
}

void Buzzer::tone(uint32_t freqHz, uint32_t durationMs) {
    if (freqHz == 0 || pinA_ == 0xFF) {
        return;
    }
    // Each electrical period is two half cycles of opposite polarity, so a half
    // cycle lasts 1e6 / (2 * freq) microseconds.
    uint32_t halfPeriodUs = 500000UL / freqHz;
    if (halfPeriodUs == 0) {
        halfPeriodUs = 1;
    }
    uint32_t fullPeriods = ((uint64_t)durationMs * 1000ULL) / (2ULL * halfPeriodUs);

    for (uint32_t i = 0; i < fullPeriods; i++) {
        digitalWrite(pinA_, HIGH);
        digitalWrite(pinB_, LOW);
        delayMicroseconds(halfPeriodUs);
        digitalWrite(pinA_, LOW);
        digitalWrite(pinB_, HIGH);
        delayMicroseconds(halfPeriodUs);
    }
    off();
}

void Buzzer::off() {
    if (pinA_ == 0xFF) {
        return;
    }
    digitalWrite(pinA_, LOW);
    digitalWrite(pinB_, LOW);
}
