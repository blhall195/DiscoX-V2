#pragma once

#include <Arduino.h>
#include <SPI.h>

// Murata SCA3300-D01 3-axis accelerometer, SPI (mode 0, 32-bit frames).
//
// The sensor uses an off-frame protocol: the response to a command arrives
// in the *next* SPI frame. Every frame carries an 8-bit CRC and 2-bit return
// status. Datasheet: Murata Doc.No. 3165 Rev 2.
class SCA3300 {
  public:
    enum class Mode : uint8_t {
        MODE_1 = 0, // ±3 g,   2700 LSB/g, 70 Hz LPF (power-on default)
        MODE_2 = 1, // ±6 g,   1350 LSB/g, 70 Hz LPF
        MODE_3 = 2, // ±1.5 g, 5400 LSB/g, 70 Hz LPF
        MODE_4 = 3, // ±1.5 g, 5400 LSB/g, 10 Hz LPF (low noise)
    };

    enum class Error : uint8_t {
        NONE = 0,
        CRC,         // response CRC mismatch — bus/wiring problem
        STARTUP,     // RS = '00', sensor still starting up
        SENSOR_FLAG, // RS = '11', flag set in STATUS (read status() to see/clear)
        WHOAMI,      // WHOAMI mismatch — wrong/absent device
    };

    // Raw 16-bit accelerometer counts (2's complement)
    struct Reading {
        int16_t x, y, z;
    };

    // STATUS summary bits (Table 26)
    static constexpr uint16_t STATUS_DIGI1 = 1u << 9;
    static constexpr uint16_t STATUS_DIGI2 = 1u << 8;
    static constexpr uint16_t STATUS_CLK = 1u << 7;
    static constexpr uint16_t STATUS_SAT = 1u << 6;
    static constexpr uint16_t STATUS_TEMP_SAT = 1u << 5;
    static constexpr uint16_t STATUS_PWR = 1u << 4;
    static constexpr uint16_t STATUS_MEM = 1u << 3;
    static constexpr uint16_t STATUS_PD = 1u << 2;
    static constexpr uint16_t STATUS_MODE_CHANGE = 1u << 1;
    static constexpr uint16_t STATUS_PIN_CONTINUITY = 1u << 0;

    static constexpr uint8_t WHOAMI_VALUE = 0x51;

    // Full start-up sequence (Table 10): SW reset, set mode, wait for signal
    // path settling, clear status, verify RS = '01' and WHOAMI.
    // spi.begin() is called here; csPin is driven as an output.
    bool begin(SPIClass &spi, uint8_t csPin, Mode mode = Mode::MODE_1);

    // Acceleration in g (X, Y, Z read back-to-back in one chained sequence)
    bool readAcceleration(float &gx, float &gy, float &gz);
    bool readRaw(Reading &out);

    // Temperature in °C: -273 + raw / 18.9
    bool readTemperature(float &celsius);

    // Self-test output (STO register, raw counts). Monitor against the
    // datasheet threshold for the active mode (±800 LSB in mode 1,
    // ±400 in mode 2, ±1600 in modes 3/4).
    bool readSelfTest(int16_t &sto);

    // Reads and clears the STATUS summary register
    bool readStatus(uint16_t &status);

    uint8_t readWhoAmI();

    // Component serial number as printed on the lid, e.g. "1021704154B33".
    // buf must hold at least 14 chars.
    bool readSerialNumber(char *buf, size_t len);

    bool powerDown();
    bool wakeUp(); // restores the mode passed to begin()

    float sensitivity() const; // LSB/g for the active mode
    Mode mode() const { return mode_; }
    Error lastError() const { return lastError_; }
    uint8_t lastReturnStatus() const { return rs_; }

  private:
    // Table 14 — operations and their pre-computed SPI frames
    static constexpr uint32_t CMD_READ_ACC_X = 0x040000F7;
    static constexpr uint32_t CMD_READ_ACC_Y = 0x080000FD;
    static constexpr uint32_t CMD_READ_ACC_Z = 0x0C0000FB;
    static constexpr uint32_t CMD_READ_STO = 0x100000E9;
    static constexpr uint32_t CMD_READ_TEMP = 0x140000EF;
    static constexpr uint32_t CMD_READ_STATUS = 0x180000E5;
    static constexpr uint32_t CMD_MODE[4] = {0xB400001F, 0xB4000102, 0xB4000225, 0xB4000338};
    static constexpr uint32_t CMD_POWER_DOWN = 0xB400046B;
    static constexpr uint32_t CMD_WAKE_UP = 0xB400001F;
    static constexpr uint32_t CMD_SW_RESET = 0xB4002098;
    static constexpr uint32_t CMD_READ_WHOAMI = 0x40000091;
    static constexpr uint32_t CMD_READ_SERIAL1 = 0x640000A7;
    static constexpr uint32_t CMD_READ_SERIAL2 = 0x680000AD;
    static constexpr uint32_t CMD_SELECT_BANK_0 = 0xFC000073;
    static constexpr uint32_t CMD_SELECT_BANK_1 = 0xFC00016E;

    static constexpr uint8_t RS_STARTUP = 0x00;
    static constexpr uint8_t RS_NORMAL = 0x01;
    static constexpr uint8_t RS_ERROR = 0x03;

    static constexpr uint32_t TLH_US = 10; // min CSB high time between frames

    // One 32-bit frame exchange; returns the response to the *previous* command
    uint32_t transfer(uint32_t frame);
    // CRC + RS check on a response frame
    bool validate(uint32_t response);
    // Off-frame single register read: sends cmd twice, returns 16-bit data
    bool readRegister(uint32_t cmd, uint16_t &data);
    // Clear STATUS and confirm RS returns to '01' (start-up steps 6-8)
    bool clearStatus();

    static uint8_t crc8(uint32_t frame);

    SPIClass *spi_ = nullptr;
    uint8_t csPin_ = 0xFF;
    Mode mode_ = Mode::MODE_1;
    uint8_t rs_ = 0;
    Error lastError_ = Error::NONE;
    // Datasheet recommends 2-4 MHz for best noise performance (8 MHz max)
    SPISettings settings_{2000000, MSBFIRST, SPI_MODE0};
};
