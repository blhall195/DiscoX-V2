#pragma once

#include <Arduino.h>

// Meskernel LDJ-100 laser distance module — UART register protocol.
// Protocol per hardware/datasheets/LDJ-100RED.pdf (§13-15):
//
//   host -> module read : [0xAA][0x80|addr][reg16][ck]
//   host -> module write: [0xAA][addr][reg16][count16][count x word16][ck]
//   module -> host      : [0xAA][addr][reg16][count16][payload][ck]
//                         head 0xEE instead = error report, payload is a
//                         status code (see statusText())
//   ck = sum of every byte after the head, & 0xFF; count is in 16-bit words.
//
// Serial: 8N1, fixed default 115200 (auto-baud variants handshake with a
// 0x55 byte within 2.5 s of power-on). One 0x58 ('X') byte stops continuous
// measurement. Module TXD is open drain — give the host RX pin a pullup.
class LDJ100 {
  public:
    enum MeasureMode : uint16_t {
        SINGLE_AUTO = 0x0000,
        SINGLE_SLOW = 0x0001, // higher accuracy, up to ~4 s
        SINGLE_FAST = 0x0002,
        CONT_AUTO = 0x0004,
        CONT_SLOW = 0x0005,
        CONT_FAST = 0x0006,
    };

    struct Measurement {
        uint32_t distanceMm = 0;
        uint16_t signalQuality = 0; // lower = stronger laser signal
    };

    void begin(Stream &serial, uint8_t address = 0x00);

    // True if the module answers a status-register read.
    bool ping(uint32_t timeoutMs = 250);
    // Auto-baud variants reply with their 1-byte address after a 0x55 byte.
    bool autoBaudHandshake(uint8_t &moduleAddr, uint32_t timeoutMs = 200);

    bool readStatus(uint16_t &status, uint32_t timeoutMs = 300);
    bool readHardwareVersion(uint16_t &version, uint32_t timeoutMs = 300);
    bool readSoftwareVersion(uint16_t &version, uint32_t timeoutMs = 300);
    bool readSerialNumber(uint32_t &serialNo, uint32_t timeoutMs = 300);
    bool readInputVoltageMv(uint16_t &millivolts, uint32_t timeoutMs = 300);

    bool setLaser(bool on, uint32_t timeoutMs = 500);
    bool measure(Measurement &m, MeasureMode mode = SINGLE_AUTO, uint32_t timeoutMs = 5000);
    void startContinuous(MeasureMode mode = CONT_AUTO);
    void stopContinuous();
    // Wait for the next continuous-mode result frame. False on timeout or
    // module error — an error also sets lastStatus() non-zero.
    bool poll(Measurement &m, uint32_t timeoutMs = 300);

    // Status code from the most recent 0xEE error frame (0 = none).
    uint16_t lastStatus() const { return lastStatus_; }
    static const char *statusText(uint16_t status);

  private:
    static constexpr uint8_t MAX_WORDS = 4;

    struct Frame {
        uint8_t head = 0;
        uint8_t addr = 0;
        uint16_t reg = 0;
        uint8_t words = 0;
        uint16_t payload[MAX_WORDS] = {0};
    };

    void flushInput();
    bool readByte(uint8_t &b, uint32_t deadline);
    bool readFrame(Frame &f, uint32_t deadline);
    // Skips unrelated frames until `reg` arrives; error frames set
    // lastStatus_ and abort.
    bool waitForReg(uint16_t reg, Frame &f, uint32_t timeoutMs);
    void sendRead(uint16_t reg);
    void sendWrite(uint16_t reg, uint16_t value);
    bool readWord(uint16_t reg, uint16_t &value, uint32_t timeoutMs);

    Stream *serial_ = nullptr;
    uint8_t addr_ = 0x00;
    uint16_t lastStatus_ = 0;
};
