#pragma once

#include <Adafruit_MAX1704X.h>
#include <Wire.h>

// MAX17048 LiPo fuel gauge (U3, I2C 0x36) — senses BAT+_RAW at CN1.
//
// The stock Adafruit_MAX17048::begin() calls reset(), which wipes the IC's
// learned ModelGauge state and forces a voltage-only "first guess" every boot
// (SOC then reads only 0%/100% until it re-learns the cell). This subclass
// overrides begin() to SKIP reset(), so the gauge keeps tracking SOC across MCU
// reboots — the IC still POR-resets on its own if battery power is removed.
// Lifted from the V1 firmware's MAX17048_Persistent (Main board
// C++/src/main.cpp).
//
// Header-only on purpose: keeping this out of a drivers/*.cpp means the shared
// `+<drivers/>` build filter doesn't pull the Adafruit MAX1704X dependency into
// every other bring-up env — only the env that includes this header needs it.
class MAX17048_Persistent : public Adafruit_MAX17048 {
public:
  bool begin(TwoWire *wire = &Wire) {
    if (i2c_dev) {
      delete i2c_dev;
      delete status_reg;
    }
    i2c_dev = new Adafruit_I2CDevice(MAX17048_I2CADDR_DEFAULT, wire);
    if (!i2c_dev->begin()) {
      return false;
    }
    if (!isDeviceReady()) {
      return false;
    }
    status_reg = new Adafruit_BusIO_Register(i2c_dev, MAX1704X_STATUS_REG);
    // No reset() — preserve ModelGauge tracking state
    enableSleep(false);
    sleep(false);
    wake(); // exit hibernation if the IC entered it
    return true;
  }
};
