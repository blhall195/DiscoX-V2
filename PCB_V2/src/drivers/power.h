#pragma once

#include <Arduino.h>

// LTC2954-1 pushbutton power controller (U9) + BQ24074 charger PGOOD.
//
// The device is powered ON by hardware: pressing the button latches the
// LTC2954's ENA high, enabling every downstream 3V3 rail (the nRF core
// included — see NETLIST.md "Power rails"). While the board is running:
//
//   - INT (P0.06) is pulled LOW while the power button is held (10k pullup) —
//     the controller's way of telling the MCU "the user pressed the button".
//   - Driving KILL (P0.04) LOW tells the LTC2954 to drop ENA, which cuts ALL
//     rails and powers the board off.
//
// KILL has an external 10k pullup. We deliberately keep our GPIO
// high-impedance (INPUT) until the instant we want to shut down, so a stray
// LOW can never cut power mid-run.
class PowerControl {
  public:
    void begin(uint8_t killPin, uint8_t intPin, int8_t pgoodPin = -1);

    // True while the power button is held (INT asserted LOW).
    bool buttonPressed() const;

    // BQ24074 power-good: valid USB input present. Only meaningful if a PGOOD
    // pin was supplied to begin(). Open-drain, active LOW.
    bool powerGood() const;

    // Assert KILL LOW -> LTC2954 drops ENA -> all rails die. Does not return:
    // if power somehow lingers (e.g. USB still feeding the nRF's USB PHY) it
    // holds KILL low forever so nothing downstream powers back up.
    [[noreturn]] void powerOff();

  private:
    uint8_t killPin_ = 0xFF;
    uint8_t intPin_ = 0xFF;
    int8_t pgoodPin_ = -1;
};
