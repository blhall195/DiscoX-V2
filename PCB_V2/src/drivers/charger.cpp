#include "charger.h"

#include <nrf_sdm.h>
#include <nrf_soc.h>

// Back to the reset state: input buffer disconnected, no pull. Used for the
// board strap after reading it (on a new board a pull-up left on would draw
// ~250 uA into the grounded pin) and for every charger pin on an old board,
// where they float.
static void disconnectPin(uint8_t pin) { nrf_gpio_cfg_default(g_ADigitalPinMap[pin]); }

void Charger::begin(uint8_t boardIdPin, uint8_t chgPin, uint8_t pgoodPin, uint8_t cePin) {
    chgPin_ = chgPin;
    pgoodPin_ = pgoodPin;
    cePin_ = cePin;

    // Board strap: GND on new boards (reads 0), unconnected on old (pulled to 1)
    pinMode(boardIdPin, INPUT_PULLUP);
    delayMicroseconds(100);
    hasControl_ = digitalRead(boardIdPin) == LOW;
    disconnectPin(boardIdPin);

    if (!hasControl_) {
        return; // old board: CHG/PGOOD/CE pads are not connected to anything
    }
    pinMode(chgPin_, INPUT_PULLUP);
    pinMode(pgoodPin_, INPUT_PULLUP);
    // Driven from here on, starting where R23 already holds it (charging on)
    digitalWrite(cePin_, LOW);
    pinMode(cePin_, OUTPUT);
    digitalWrite(cePin_, LOW);
    enabled_ = true;
}

bool Charger::usbPresent() {
    // POWER belongs to the SoftDevice once BLE is up: ask it, as TinyUSB does
    uint8_t sdEnabled = 0;
    (void)sd_softdevice_is_enabled(&sdEnabled);
    uint32_t status = 0;
    if (sdEnabled) {
        (void)sd_power_usbregstatus_get(&status);
    } else {
        status = NRF_POWER->USBREGSTATUS;
    }
    return status & POWER_USBREGSTATUS_VBUSDETECT_Msk;
}

bool Charger::chargingSignal() const { return hasControl_ && digitalRead(chgPin_) == LOW; }

bool Charger::powerGood() const { return hasControl_ && digitalRead(pgoodPin_) == LOW; }

void Charger::setChargingEnabled(bool enabled) {
    if (!hasControl_ || enabled == enabled_) {
        return;
    }
    enabled_ = enabled;
    digitalWrite(cePin_, enabled ? LOW : HIGH);
}
