#pragma once

#include <Arduino.h>

// BQ24074 charger status + control, and the board-revision strap.
//
// Boards from the 2026-10-18 PCB revision wire the charger's CHG, PGOOD and
// CE pins to the nRF and tie P1.02 to GND; older boards leave all four
// unconnected. begin() tells them apart, so one firmware build runs on both:
// on an old board every pin stays in its reset state (disconnected input),
// hasControl() is false and the charger keeps its default behaviour.
//
// CHG and PGOOD are open drain: they use the nRF's internal pull-ups, which
// only draw current while the charger pulls them low (USB present). Never
// pull them up to Bat+: that back-powers the nRF when its rail is off.
class Charger {
  public:
    /// Read the board strap and set the pins up. Call early in setup().
    void begin(uint8_t boardIdPin, uint8_t chgPin, uint8_t pgoodPin, uint8_t cePin);

    /// True on boards with CHG/PGOOD/CE wired to the MCU
    bool hasControl() const { return hasControl_; }

    /// VBUS present, from the nRF's own USB detector — works on every board
    static bool usbPresent();

    /// CHG LOW: the charger is charging. False on old boards.
    bool chargingSignal() const;

    /// PGOOD LOW: valid input power. False on old boards.
    bool powerGood() const;

    /// Charging on (CE low) or held off (CE high). No-op on old boards.
    void setChargingEnabled(bool enabled);
    bool chargingEnabled() const { return enabled_; }

  private:
    bool hasControl_ = false;
    bool enabled_ = true;
    uint8_t chgPin_ = 0xFF;
    uint8_t pgoodPin_ = 0xFF;
    uint8_t cePin_ = 0xFF;
};
