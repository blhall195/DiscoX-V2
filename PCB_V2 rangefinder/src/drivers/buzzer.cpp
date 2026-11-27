#include "buzzer.h"

void Buzzer::begin(uint8_t pinA, uint8_t pinB) {
    pinA_ = pinA;
    pinB_ = pinB;
    pinMode(pinA_, OUTPUT);
    pinMode(pinB_, OUTPUT);
    off();
}

void Buzzer::tone(uint32_t freqHz, uint32_t durationMs) {
    // Bit-bang needs the pads back as GPIO: digitalWrite is inert while PWM
    // drives them.
    releasePwm();
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

void Buzzer::sweep(uint32_t fromHz, uint32_t toHz, uint32_t durationMs) {
    // Bit-bang needs the pads back as GPIO: digitalWrite is inert while PWM
    // drives them.
    releasePwm();
    if (fromHz == 0 || toHz == 0 || pinA_ == 0xFF) {
        return;
    }
    uint32_t totalUs = durationMs * 1000UL;
    uint32_t start = micros();
    bool phase = false;
    uint32_t elapsed;
    while ((elapsed = micros() - start) < totalUs) {
        uint32_t freq = fromHz + (uint32_t)(((int64_t)toHz - (int64_t)fromHz) * elapsed / totalUs);
        uint32_t halfPeriodUs = 500000UL / freq;
        if (halfPeriodUs == 0) {
            halfPeriodUs = 1;
        }
        digitalWrite(pinA_, phase ? HIGH : LOW);
        digitalWrite(pinB_, phase ? LOW : HIGH);
        phase = !phase;
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

// ── Non-blocking tone via hardware PWM ──────────────────────────────
// PWM1: NeoPixel scans PWM0..PWM3 and takes the first instance that is
// disabled with no pins connected, so owning PWM1 with pins attached simply
// makes it skip to PWM0. The core's tone() is hard-coded to PWM2.
static constexpr uint32_t kPwmToken = 0x425A5A31; // 'BZZ1'
#define BUZZ_PWM HwPWMx[1]

// PWM clock after DIV16 is 1 MHz, so COUNTERTOP = 1e6 / freq. COUNTERTOP is
// 15-bit (max 32767), which floors the range at ~31 Hz — far below anything
// this element can reproduce, so every usable note fits.
static constexpr uint32_t kPwmClockHz = 1000000UL;

void Buzzer::startTone(uint32_t freqHz) {
    if (freqHz == 0 || pinA_ == 0xFF) {
        stopTone();
        return;
    }

    uint32_t top = kPwmClockHz / freqHz;
    if (top < 2) {
        top = 2; // ceiling ~500 kHz; nothing audible gets near it
    } else if (top > 32767) {
        stopTone(); // below the element's range — treat as a rest
        return;
    }

    if (!pwmActive_) {
        if (!BUZZ_PWM->takeOwnership(kPwmToken)) {
            return; // someone else holds it — stay silent rather than fight
        }
        BUZZ_PWM->setClockDiv(PWM_PRESCALER_PRESCALER_DIV_16);
        BUZZ_PWM->addPin(pinA_);
        BUZZ_PWM->addPin(pinB_);
        BUZZ_PWM->begin();
        pwmActive_ = true;
    }

    BUZZ_PWM->setMaxValue((uint16_t)top);
    // Same 50% duty on both pins, the second inverted: one pin is HIGH
    // exactly while the other is LOW, which is the antiphase drive the
    // two-GPIO wiring exists for.
    BUZZ_PWM->writePin(pinA_, (uint16_t)(top / 2), false);
    BUZZ_PWM->writePin(pinB_, (uint16_t)(top / 2), true);
}

void Buzzer::stopTone() {
    releasePwm();
    off();
}

void Buzzer::releasePwm() {
    if (!pwmActive_) {
        return;
    }
    BUZZ_PWM->stop();
    BUZZ_PWM->removePin(pinA_);
    BUZZ_PWM->removePin(pinB_);
    BUZZ_PWM->releaseOwnership(kPwmToken);
    pwmActive_ = false;
    // removePin leaves the pad as a plain GPIO but not necessarily driven.
    pinMode(pinA_, OUTPUT);
    pinMode(pinB_, OUTPUT);
}
