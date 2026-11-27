#include "power.h"

void PowerControl::begin(uint8_t killPin, uint8_t intPin) {
    killPin_ = killPin;
    intPin_ = intPin;

    // Keep KILL high-impedance: the external 10k pullup holds it HIGH so the
    // rail stays latched on. We only ever drive it LOW inside powerOff().
    pinMode(killPin_, INPUT);

    // INT has an external 10k pullup; the internal one is harmless insurance.
    pinMode(intPin_, INPUT_PULLUP);
}

bool PowerControl::buttonPressed() const { return digitalRead(intPin_) == LOW; }

void PowerControl::powerOff() {
    // Switch KILL from high-Z to a hard driven LOW.
    digitalWrite(killPin_, LOW);
    pinMode(killPin_, OUTPUT);
    digitalWrite(killPin_, LOW);
    // Power should collapse within the LTC2954's KILL debounce. Keep asserting
    // it in case USB is still feeding the nRF USB PHY and the core survives.
    while (true) {
        digitalWrite(killPin_, LOW);
    }
}
