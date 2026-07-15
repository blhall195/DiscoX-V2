#include "power.h"

void PowerControl::begin(uint8_t killPin, uint8_t intPin, int8_t pgoodPin) {
    killPin_ = killPin;
    intPin_ = intPin;
    pgoodPin_ = pgoodPin;

    // Keep KILL high-impedance: the external 10k pullup holds it HIGH so the
    // rail stays latched on. We only ever drive it LOW inside powerOff().
    pinMode(killPin_, INPUT);

    // INT has an external 10k pullup; the internal one is harmless insurance.
    pinMode(intPin_, INPUT_PULLUP);
    if (pgoodPin_ >= 0) {
        pinMode(pgoodPin_, INPUT_PULLUP); // open-drain output on the charger
    }
}

bool PowerControl::buttonPressed() const { return digitalRead(intPin_) == LOW; }

bool PowerControl::powerGood() const {
    if (pgoodPin_ < 0) {
        return false;
    }
    return digitalRead(pgoodPin_) == LOW; // open-drain: LOW = input power good
}

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
